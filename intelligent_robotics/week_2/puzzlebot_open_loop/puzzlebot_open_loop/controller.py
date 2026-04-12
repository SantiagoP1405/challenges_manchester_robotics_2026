import rclpy
from rclpy.node import Node
import numpy as np
from makbets_pose.msg import MakbetsPose
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.pose_sub = self.create_subscription(MakbetsPose, '/pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Controller initialized, waiting for pose...')

        self.pose_received = False
        self.current_point = 0
        # States: 0=rotate, 1=pause, 2=forward, 3=pause, 4=done
        self.state = 0
        self.state_start_time = None

        self.distances = []
        self.angles = []
        self.linear_times = []
        self.angular_times = []

        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        if self.pose_received:
            return
        self.pose = msg
        num_points = len(msg.x)

        prev_x, prev_y = 0.0, 0.0
        prev_angle = 0.0  # ← AGREGA ESTO

        for i in range(num_points):
            dx = msg.x[i] - prev_x
            dy = msg.y[i] - prev_y
            self.distances.append(np.sqrt(dx**2 + dy**2))

            abs_angle = np.arctan2(dy, dx)
            delta = abs_angle - prev_angle
            delta = (delta + np.pi) % (2 * np.pi) - np.pi  # normalizar [-π, π]
            self.angles.append(delta)  # ← ahora guarda el delta, no el absoluto

            prev_x, prev_y = msg.x[i], msg.y[i]
            prev_angle = abs_angle  # ← AGREGA ESTO

        print(self.distances)
        print(self.angles)
        # Calculate times for each segment
        if msg.mode == 0:
            for i in range(num_points):
                self.linear_times.append(self.distances[i] / msg.linear_speed[i])
                self.angular_times.append(abs(self.angles[i]) / msg.angular_speed[i])
                
        elif msg.mode == 1:
            self.linear_speeds = []
            self.angular_speeds = []
            for i in range(num_points):
                if self.distances[i] > 1e-6:
                    lin_speed = self.distances[i] / msg.path_time[i]
                    ang_speed = abs(self.angles[i]) / msg.path_time[i]

                    # Clamp speeds
                    if lin_speed > 0.3:
                        self.get_logger().warning(f'Point {i} unreacheable: linear speed {lin_speed:.3f} clamped to 0.3')
                        lin_speed = 0.3
                    if ang_speed > 0.72:
                        self.get_logger().warning(f'Point {i} unreacheable: angular speed {ang_speed:.3f} clamped to 0.72')
                        ang_speed = 0.72

                    self.linear_speeds.append(lin_speed)
                    self.angular_speeds.append(ang_speed)
                    self.linear_times.append(self.distances[i] / lin_speed)
                    self.angular_times.append(abs(self.angles[i]) / ang_speed)
                else:
                    self.linear_speeds.append(0.0)
                    self.angular_speeds.append(0.0)
                    self.linear_times.append(0.0)
                    self.angular_times.append(0.0)

        self.pose_received = True
        self.state = 0
        self.state_start_time = self.get_clock().now()
        #self.get_logger().info(f'Received {num_points} waypoints. Starting trajectory.')

    def timer_callback(self):
        if not self.pose_received:
            return

        cmd = Twist()
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        i = self.current_point

        if self.state == 0:  # Rotate toward point
            ang_speed = self.pose.angular_speed[i] if self.pose.mode == 0 else self.angular_speeds[i]
            direction = 1.0 if self.angles[i] >= 0 else -1. 
            cmd.angular.z = ang_speed * direction
            #self.get_logger().info(f'Point {i}: Rotating... {elapsed:.2f}/{self.angular_times[i]:.2f}s')
            if elapsed >= self.angular_times[i]:    
                self.state = 1
                self.state_start_time = now

        elif self.state == 1:  # Pause after rotation
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if elapsed >= 0.5:
                self.state = 2
                self.state_start_time = now

        elif self.state == 2:  # Move forward
            lin_speed = self.pose.linear_speed[i] if self.pose.mode == 0 else self.linear_speeds[i]
            cmd.linear.x = lin_speed
            #self.get_logger().info(f'Point {i}: Moving forward... {elapsed:.2f}/{self.linear_times[i]:.2f}s')
            if elapsed >= self.linear_times[i]:
                self.state = 3
                self.state_start_time = now

        elif self.state == 3:  # Pause after forward
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if elapsed >= 0.5:
                self.current_point += 1
                if self.current_point >= len(self.distances):
                    self.state = 4
                    #self.get_logger().info('Trajectory complete!')
                else:
                    self.state = 0
                    self.state_start_time = now
                    #self.get_logger().info(f'Moving to point {self.current_point}')

        elif self.state == 4:  # Done
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)
        


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()