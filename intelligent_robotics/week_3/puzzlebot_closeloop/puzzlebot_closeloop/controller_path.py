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

        #parametros del control
        self.declare_parameter('kp_dist', 0.9)
        self.declare_parameter('kp_theta', 0.3)
        self.declare_parameter('theta_tolerance', 0.15)
        self.declare_parameter('goal_tolerance', 0.1)

        self.kp_dist = self.get_parameter('kp_dist').value
        self.kp_theta = self.get_parameter('kp_theta').value
        self.theta_tolerance = self.get_parameter('theta_tolerance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.goal_x = None
        self.goal_y = None
        self.v_limit = 0.0
        self.w_limit = 0.0

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        

        self.goal_sub = self.create_subscription(MakbetsPose, '/goal', self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.x_hist, self.y_hist = [], []
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        self.ax.grid(True)
        self.ax.set_title("Seguimiento de Trayectoria Makbets")

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
            self.goal_x = msg.x[0]
            self.goal_y = msg.y[0]
            self.v_limit = msg.linear_speed[0]
            self.w_limit = msg.angular_speed[0]

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        if self.goal_x is None:
            return

        #calculo de error
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)
        
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - self.current_theta)

        cmd = Twist()

        if abs(theta_error) > self.theta_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.w_limit, min(self.w_limit, self.kp_theta * theta_error))
        else:
            v_out = self.kp_dist * dist_error
            cmd.linear.x = max(0.0, min(self.v_limit, v_out))
            cmd.angular.z = max(-self.w_limit, min(self.w_limit, self.kp_theta * theta_error))

        if dist_error < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.update_plot()

    def update_plot(self):
        if self.x_hist:
            self.line.set_data(self.x_hist, self.y_hist)
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.flush_events()
            plt.pause(0.001)

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