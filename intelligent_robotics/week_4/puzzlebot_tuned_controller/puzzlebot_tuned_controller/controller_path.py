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
        self.declare_parameter('kp_dist', 0.7)
        self.declare_parameter('kp_theta', 0.8) # Bajado de 1.2 para evitar overshoot
        
        self.declare_parameter('ki_dist', 0.0)  # Empezamos en 0 para estabilidad
        self.declare_parameter('ki_theta', 0.001) # Valor muy pequeño
        
        self.declare_parameter('kd_dist', 0.05)
        self.declare_parameter('kd_theta', 0.1) # Aumentado para "frenar" mejor el giro

        self.declare_parameter('theta_tolerance', 0.1) # Más estricto (aprox 5.7 grados)
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

        # Variables PID
        self.prev_dist_error = 0.0
        self.prev_theta_error = 0.0
        self.integral_dist = 0.0
        self.integral_theta = 0.0
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(MakbetsPose, '/goal', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.x_hist, self.y_hist = [], []

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
            # Reiniciar TODO al recibir nuevo punto
            if self.goal_x != msg.x[0] or self.goal_y != msg.y[0]:
                self.integral_dist = 0.0
                self.integral_theta = 0.0
                self.prev_dist_error = 0.0
                self.prev_theta_error = 0.0
            
            self.goal_x = msg.x[0]
            self.goal_y = msg.y[0]
            self.v_limit = msg.linear_speed[0]
            self.w_limit = msg.angular_speed[0]

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        if self.goal_x is None: return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - self.current_theta)

        # --- PID ÁNGULO ---
        # Anti-Windup: Solo integra si el error es pequeño pero no nulo
        if abs(theta_error) < 0.5:
            self.integral_theta += theta_error * self.dt
        
        # Reset de integral si cruzamos el cero (evita oscilación)
        if (self.prev_theta_error > 0 and theta_error < 0) or (self.prev_theta_error < 0 and theta_error > 0):
            self.integral_theta = 0.0

        derivative_theta = (theta_error - self.prev_theta_error) / self.dt
        w_out = (self.kp_theta * theta_error) + (self.ki_theta * self.integral_theta) + (self.kd_theta * derivative_theta)

        # --- PID DISTANCIA ---
        self.integral_dist += dist_error * self.dt
        derivative_dist = (dist_error - self.prev_dist_error) / self.dt
        v_out = (self.kp_dist * dist_error) + (self.ki_dist * self.integral_dist) + (self.kd_dist * derivative_dist)

        cmd = Twist()
        if dist_error < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.integral_dist, self.integral_theta = 0.0, 0.0
        else:
            # Lógica de estados: Giro o Avance
            if abs(theta_error) > self.theta_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = max(-self.w_limit, min(self.w_limit, w_out))
            else:
                # Mientras avanza, sigue corrigiendo ángulo suavemente
                cmd.linear.x = max(0.0, min(self.v_limit, v_out))
                cmd.angular.z = max(-self.w_limit, min(self.w_limit, w_out))

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