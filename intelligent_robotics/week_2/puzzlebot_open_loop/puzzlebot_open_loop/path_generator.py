import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
#from geometry_msgs.msg import Pose, Twist
from makbets_pose.msg import MakbetsPose
#from rcl_interfaces.msg import SetParametersResults 

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        # self.declare_parameter('pointx', 0.0)  # coordinate x
        # self.declare_parameter('pointy', 0.0)  # coordinate y
        # self.declare_parameter('linear_speed', 0.0)  # m/s
        # self.declare_parameter('angular_speed', 0.0) # rad/s
        # self.declare_parameter('path_time', 0.0)  # time s
        #self.declare_parameter('mode', 0)  # execution mode: 0 for receiving coordinates and speeds, 1 for receiving coordinates and time
        self.path_pub = self.create_publisher(MakbetsPose, '/pose', 10)
        #self.timer_period = 0.2  # seconds
        self.mode = int(input('Enter execution mode (0 for coordinates and speeds, 1 for coordinates and time): '))
        self.pointx = []
        self.pointy = []
        self.point_linear_speed = []
        self.point_angular_speed = []
        self.point_time = []
        if (self.mode == 0):
            for i in range(4):
                x = float(input('Enter x coordinate: '))
                y = float(input('Enter y coordinate: '))
                linear_speed = float(input('Enter linear speed: '))
                while (linear_speed <= 0.005 or linear_speed > 0.3):
                    self.get_logger().warning('Linear speed must be in the range (0.005, 0.3]. Please enter a valid value.')
                    linear_speed = float(input('Enter linear speed: '))
                angular_speed = float(input('Enter angular speed: '))
                while (angular_speed <= 0.005 or angular_speed > 0.72):
                    self.get_logger().warning('Angular speed must be in the range (0.005, 0.72]. Please enter a valid value.')
                    angular_speed = float(input('Enter angular speed: '))
                self.pointx.append(x)
                self.pointy.append(y)
                self.point_linear_speed.append(linear_speed)
                self.point_angular_speed.append(angular_speed)

        elif (self.mode == 1):
            for i in range(4):
                x = float(input('Enter x coordinate: '))
                y = float(input('Enter y coordinate: '))
                path_time = float(input('Enter path time: '))
                self.pointx.append(x)
                self.pointy.append(y)
                self.point_time.append(path_time)
        
        else:
            self.get_logger().error('Invalid mode selected. Please restart the program and select a valid mode (0 or 1).')
            self.mode = int(input('Enter execution mode (0 for coordinates and speeds, 1 for coordinates and time): '))

        self.timer_callback()
        self.get_logger().info('Path generator initialized')

    def timer_callback(self):
        pose = MakbetsPose()
        pose.x = self.pointx
        pose.y = self.pointy
        pose.mode = self.mode
        if (self.mode == 0):
            pose.linear_speed = self.point_linear_speed
            pose.angular_speed = self.point_angular_speed
            pose.path_time = []
        elif (self.mode == 1):
            pose.path_time = self.point_time
            pose.linear_speed = []
            pose.angular_speed = []
        self.path_pub.publish(pose)
        self.get_logger().info('Path published')

#Main
def main():
    rclpy.init()
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#Execute Node
if __name__ == '__main__':
    main()
    