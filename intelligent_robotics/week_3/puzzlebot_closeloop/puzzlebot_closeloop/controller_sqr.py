import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math


class SquareController(Node):
    def __init__(self):
        super().__init__('controller_sqr')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_period', 0.1)

        self.declare_parameter('kp_dist', 0.8)
        self.declare_parameter('kp_theta', 2.0)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.7)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('theta_tolerance', 0.15)

        #goals
        self.declare_parameter('goals', [
            2.0, 0.0,
            2.0, 2.0,
            0.0, 2.0,
            0.0, 0.0
        ])

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_period = float(self.get_parameter('control_period').value)

        self.kp_dist = float(self.get_parameter('kp_dist').value)
        self.kp_theta = float(self.get_parameter('kp_theta').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.theta_tolerance = float(self.get_parameter('theta_tolerance').value)

        raw_goals = self.get_parameter('goals').value

        if len(raw_goals) % 2 != 0:
            self.get_logger().error(
                'El parámetro "goals" debe tener número par de elementos.'
            )
            raise ValueError('Goals inválidos')

        self.goals = [
            (float(raw_goals[i]), float(raw_goals[i + 1]))
            for i in range(0, len(raw_goals), 2)
        ]

        #estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_received = False

        self.current_goal_index = 0
        self.path_done = False
        self.x_hist = []
        self.y_hist = []

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(f'Controller iniciado con {len(self.goals)} goals')
        self.get_logger().info(f'Suscrito a {self.odom_topic}')
        self.get_logger().info(f'Publicando en {self.cmd_vel_topic}')

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Trayectoria en Tiempo Real")
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.grid(True) 
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.x_hist.append(self.x)
        self.y_hist.append(self.y)

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.theta = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.pose_received = True

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def saturate(self, value, limit):
        return max(-limit, min(limit, value))
    
    def control_loop(self):
        self.get_logger().info('Entrando a control_loop')

        if not self.pose_received:
            self.get_logger().info('No se ha recibido pose todavía')
            return

        if self.path_done:
            self.get_logger().info('Path terminado')
            self.cmd_vel_pub.publish(Twist())
            return

        if self.current_goal_index >= len(self.goals):
            self.path_done = True
            self.get_logger().info('Todos los goals completados')
            self.cmd_vel_pub.publish(Twist())
            return

        goal_x, goal_y = self.goals[self.current_goal_index]

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance_error = math.sqrt(dx ** 2 + dy ** 2)
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - self.theta)

        if self.x_hist: 
            self.line.set_data(self.x_hist, self.y_hist)
            self.ax.relim()
            self.ax.autoscale_view()
            
            self.fig.canvas.flush_events()
            plt.pause(0.001)

        if distance_error < self.goal_tolerance:
            self.get_logger().info(
                f'Goal {self.current_goal_index + 1}/{len(self.goals)} alcanzado'
            )
            self.current_goal_index += 1
            self.cmd_vel_pub.publish(Twist())
            return

        cmd = Twist()

        if abs(theta_error) > self.theta_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.saturate(self.kp_theta * theta_error, self.w_max)
            self.get_logger().info(
                f'Solo girando -> v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}'
            )
        else:
            cmd.linear.x = self.saturate(self.kp_dist * distance_error, self.v_max)
            cmd.angular.z = self.saturate(self.kp_theta * theta_error, self.w_max)
            self.get_logger().info(
                f'Avanzando -> v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}'
            )

        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Comando publicado en /cmd_vel')
        self.get_logger().info(
            f"\n> Posición: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}"
            f"\n> Error: dist={distance_error:.3f}, angle={theta_error:.3f}"
            f"\n> Comandos: v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}"
            f"\n----------------------------------------"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SquareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()