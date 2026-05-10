from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from makbets_pose.msg import MakbetsPose
import math
import matplotlib.pyplot as plt


class ControllerPath(Node):
    def __init__(self):
        super().__init__('controller_path')

        # --- Parámetros Sintonizados (Ajusta estos si sigue girando mucho) ---
        self.declare_parameter('kp_dist', 0.99)
        self.declare_parameter('kp_theta', 0.7)   # Bajado de 1.2 para evitar overshoot

        self.declare_parameter('ki_dist', 0.0)    # Empezamos en 0 para estabilidad
        self.declare_parameter('ki_theta', 0.000) # Valor muy pequeño

        self.declare_parameter('kd_dist', 0.06)
        self.declare_parameter('kd_theta', 0.1)   # Aumentado para "frenar" mejor el giro

        self.declare_parameter('theta_tolerance', 0.1)  # ~5.7 grados
        self.declare_parameter('goal_tolerance', 0.1)

        self.kp_dist = self.get_parameter('kp_dist').value
        self.ki_dist = self.get_parameter('ki_dist').value
        self.kd_dist = self.get_parameter('kd_dist').value
        self.kp_theta = self.get_parameter('kp_theta').value
        self.ki_theta = self.get_parameter('ki_theta').value
        self.kd_theta = self.get_parameter('kd_theta').value
        self.theta_tolerance = self.get_parameter('theta_tolerance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.current_x, self.current_y, self.current_theta = 0.0, 0.0, 0.0
        self.goal_x, self.goal_y = None, None
        self.v_limit, self.w_limit = 0.0, 0.0
        self.state = "STOP"

        # Variables PID
        self.prev_dist_error = 0.0
        self.prev_theta_error = 0.0
        self.integral_dist = 0.0
        self.integral_theta = 0.0

        # Subs/Pubs
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(MakbetsPose, '/goal', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.light_sub = self.create_subscription(String, '/detected_color', self.light_callback, 10)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.x_hist, self.y_hist = [], []

        # --- Variables para debugging ---
        self._log_counter = 0
        self._log_period = 10        # Status cada 10 ciclos = 0.5s a 20Hz
        self._prev_mode = None       # "GIRO" o "AVANCE"
        self._goal_reached_logged = False

        self.get_logger().info('ControllerPath iniciado. Esperando semaforo VERDE para arrancar.')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        self.x_hist.append(self.current_x)
        self.y_hist.append(self.current_y)

    def goal_callback(self, msg):
        if len(msg.x) > 0:
            new_goal_x = msg.x[0]
            new_goal_y = msg.y[0]

            # Detectar goal nuevo con tolerancia (1 mm)
            is_new_goal = (
                self.goal_x is None
                or not math.isclose(self.goal_x, new_goal_x, abs_tol=1e-3)
                or not math.isclose(self.goal_y, new_goal_y, abs_tol=1e-3)
            )

            if is_new_goal:
                self.integral_dist = 0.0
                self.integral_theta = 0.0
                self.prev_dist_error = 0.0
                self.prev_theta_error = 0.0
                self._goal_reached_logged = False
                self.get_logger().info(
                    f'Nuevo goal: ({new_goal_x:.2f}, {new_goal_y:.2f}) | '
                    f'v_max={msg.linear_speed[0]:.2f} w_max={msg.angular_speed[0]:.2f}'
                )

            self.goal_x = new_goal_x
            self.goal_y = new_goal_y
            self.v_limit = msg.linear_speed[0]
            self.w_limit = msg.angular_speed[0]

    def light_callback(self, msg):
        color = msg.data
        if color == "NONE":
            return

        new_state = self.state
        if color == "ROJO":
            new_state = "STOP"
        elif color == "AMARILLO":
            new_state = "SLOW"
        elif color == "VERDE":
            new_state = "GO"

        # Solo logguea si realmente cambio el estado
        if new_state != self.state:
            self.get_logger().info(f'Semaforo: {color} -> estado={new_state}')
            self.state = new_state

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        if self.goal_x is None:
            return

        # Factor de velocidad según el semáforo
        if self.state == "STOP":
            factor = 0.0
        elif self.state == "SLOW":
            factor = 0.3
        elif self.state == "GO":
            factor = 1.0
        else:
            factor = 0.0

        v_max = self.v_limit * factor
        w_max = self.w_limit * factor

        # En STOP publicamos cero y salimos (evita acumular integral)
        if self.state == "STOP":
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - self.current_theta)

        # --- PID ÁNGULO ---
        # Anti-Windup: solo integra cuando el error es chico
        if abs(theta_error) < 0.5:
            self.integral_theta += theta_error * self.dt

        derivative_theta = (theta_error - self.prev_theta_error) / self.dt
        w_out = (self.kp_theta * theta_error) \
              + (self.ki_theta * self.integral_theta) \
              + (self.kd_theta * derivative_theta)

        # --- PID DISTANCIA ---
        self.integral_dist += dist_error * self.dt
        derivative_dist = (dist_error - self.prev_dist_error) / self.dt
        v_out = (self.kp_dist * dist_error) \
              + (self.ki_dist * self.integral_dist) \
              + (self.kd_dist * derivative_dist)

        cmd = Twist()
        if dist_error < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.integral_dist, self.integral_theta = 0.0, 0.0

            if not self._goal_reached_logged:
                self.get_logger().info(
                    f'GOAL ALCANZADO en ({self.current_x:.2f}, {self.current_y:.2f}) | '
                    f'error final={dist_error:.3f}m'
                )
                self._goal_reached_logged = True
                self._prev_mode = None
        else:
            # Detectar modo y loggear transiciones
            current_mode = "GIRO" if abs(theta_error) > self.theta_tolerance else "AVANCE"
            if current_mode != self._prev_mode:
                self.get_logger().info(
                    f'Modo: {current_mode} (theta_err={math.degrees(theta_error):.1f} deg)'
                )
                self._prev_mode = current_mode

            if abs(theta_error) > self.theta_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = max(-w_max, min(w_max, w_out))
            else:
                cmd.linear.x = max(0.0, min(v_max, v_out))
                cmd.angular.z = max(-w_max, min(w_max, w_out))

        # --- Status periódico (cada 0.5s) ---
        self._log_counter += 1
        if self._log_counter % self._log_period == 0:
            self.get_logger().info(
                f'[{self.state}] pos=({self.current_x:.2f},{self.current_y:.2f}) '
                f'goal=({self.goal_x:.2f},{self.goal_y:.2f}) | '
                f'dist={dist_error:.2f}m theta_err={math.degrees(theta_error):.1f}deg | '
                f'cmd: v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}'
            )

        self.cmd_pub.publish(cmd)
        self.prev_dist_error = dist_error
        self.prev_theta_error = theta_error


def main(args=None):
    rclpy.init(args=args)
    node = ControllerPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()