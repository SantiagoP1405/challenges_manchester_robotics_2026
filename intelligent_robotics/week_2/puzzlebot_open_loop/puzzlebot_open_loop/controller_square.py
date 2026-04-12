# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


#Class Definition
class ControllerSquare(Node):
    def __init__(self):
        super().__init__('contoller_square')

        self.wait_for_ros_time()
        self.declare_parameter('linear_speed', 0.3)  # m/s
        self.declare_parameter('angular_speed', 0.7) # rad/s
        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Time-based control variables
        self.state = 0  # 0: forward, 1: stop, 2: rotate
        self.state_start_time = self.get_clock().now()

        # Define speeds
        self.linear_speed = self.get_parameter('linear_speed').value  # m/s
        self.angular_speed = self.get_parameter('angular_speed').value  # rad/s

        # Define durations (seconds)
        self.forward_time = (2.0 * (0.95)) / self.linear_speed   # Time to move 2m
        self.rotate_time = ((np.pi/2)*0.895) / self.angular_speed  # Time to rotate 180 deg

        # Timer to update state machine
        self.timer_period = 0.2  # 10 Hz control loop
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Open loop controller initialized!')
        
    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        self.get_logger().info(f"Start: {self.state_start_time.nanoseconds * 1e-9}, NOW: {now.nanoseconds * 1e-9:.2f}s")
        self.get_logger().info(f"State: {self.state}, Elapsed: {elapsed_time:.2f}s")

        cmd = Twist()

        if self.state == 0:
            # Move forward
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward...')
            if elapsed_time >= self.forward_time:
                self.state = 1
                self.state_start_time = now
                self.get_logger().info('Finished moving forward. Stopping...')
        
        elif self.state == 1:
            # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Stopped.')
            if elapsed_time >= 0.5:  # Stop for 0.5 seconds before rotating
                self.state = 2
                self.state_start_time = now
                self.get_logger().info('Starting rotation...')


        elif self.state == 2:
            # Rotate 180 degrees
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Rotating 90 degrees...')
            if elapsed_time >= self.rotate_time:
                self.state = 0
                self.state_start_time = now
                self.get_logger().info('Finished rotation. Moving forward...')

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)


    # Wrap to Pi function
    # def wrap_to_Pi(self,theta):
    #     result = np.fmod((theta+np.pi),(2*np.pi))
    #     if (result<0):
    #         result += 2 * np.pi
    #     return result - np.pi

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active! Start time: {now.nanoseconds * 1e-9:.2f}s')

    def parameters_callback(self, params):
        for param in params:
            #linear speed parameter check
            if param.name == "linear_speed":
                #check if it is negative
                if (param.value <= 0.0):
                    self.get_logger().warn("Invalid linear_speed! It must be positive.")
                    return SetParametersResult(successful=False, reason="linear_speed must be positive")
                elif (param.value > 0.3):
                    self.get_logger().warn("Invalid linear_speed! It cannot be greater than 0.3 m/s.")
                    return SetParametersResult(successful=False, reason="linear_speed cannot be greater than 0.3 m/s")
                else:
                    self.linear_speed = param.value  
                    self.get_logger().info(f"linear_speed updated to {self.linear_speed}")

            #angular speed parameter check
            if param.name == "angular_speed":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid angular_speed! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="angular_speed cannot be negative")
                elif (param.value > 0.7):
                    self.get_logger().warn("Invalid angular_speed! It cannot be greater than 0.7 rad/s.")
                    return SetParametersResult(successful=False, reason="angular_speed cannot be greater than 0.7 rad/s")
                else:
                    self.angular_speed = param.value  
                    self.get_logger().info(f"angular_speed updated to {self.angular_speed}")
        return SetParametersResult(successful=True)

#Main
def main(args=None):
    rclpy.init(args=args)
    node = ControllerSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()

#Execute Node
if __name__ == '__main__':
    main()
