import rclpy
import numpy as np
import signal

from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DeadReckoning(Node):

    def __init__(self):
        super().__init__('puzzlebot_odometry')

        self.r = 0.05            # Radio de la rueda 
        self.L = 0.19            # Distancia entre ruedas 
        self.odom_topic = '/odom'
        self.encR_topic = '/VelocityEncR'
        self.encL_topic = '/VelocityEncL'
        self.Ts = 0.02           # Tiempo de muestreo 

        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.wr = 0.0
        self.wl = 0.0
        self.encR_received = False
        self.encL_received = False
        self.waiting_logged = False

        # Perfil QoS para Best Effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.encR_sub = self.create_subscription(
            Float32, 
            self.encR_topic, 
            self.encR_callback, 
            qos_profile
        )

        self.encL_sub = self.create_subscription(
            Float32, 
            self.encL_topic, 
            self.encL_callback, 
            qos_profile
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(self.Ts, self.run)

        self.get_logger().info("Localisation Node Started (Static Mode).")
        self.get_logger().info(f"Subscribing to {self.encR_topic} and {self.encL_topic}")

    def encR_callback(self, msg):
        self.wr = float(msg.data)
        self.encR_received = True

    def encL_callback(self, msg):
        self.wl = float(msg.data)
        self.encL_received = True

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def run(self):
        self.waiting_logged = False

        # Modelo cinemático diferencial
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L

        # Integración de Euler
        self.x += v * np.cos(self.theta) * self.Ts
        self.y += v * np.sin(self.theta) * self.Ts
        self.theta += w * self.Ts
        self.theta = self.normalize_angle(self.theta)

        qx, qy, qz, qw = self.yaw_to_quaternion(self.theta)
        current_time = self.get_clock().now().to_msg()

        # Mensaje de Odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = float(qx)
        odom_msg.pose.pose.orientation.y = float(qy)
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Shutting down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()