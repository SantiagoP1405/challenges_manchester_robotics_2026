import rclpy
from rclpy.node import Node
import numpy as np
from makbets_pose.msg import MakbetsPose


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        self.path_pub = self.create_publisher(MakbetsPose, '/pose', 10)

        # Declare parameters with defaults
        self.declare_parameter('mode', 0)
        self.declare_parameter('points_x', [0.0])
        self.declare_parameter('points_y', [0.0])
        self.declare_parameter('linear_speeds', [0.2])
        self.declare_parameter('angular_speeds', [0.5])
        self.declare_parameter('path_times', [5.0])

        # Read parameters
        self.mode = self.get_parameter('mode').value
        self.pointx = list(self.get_parameter('points_x').value)
        self.pointy = list(self.get_parameter('points_y').value)

        # Validate that x and y have the same length
        if len(self.pointx) != len(self.pointy):
            self.get_logger().error('points_x and points_y must have the same length!')
            return

        num_points = len(self.pointx)

        if self.mode == 0:
            self.point_linear_speed = list(self.get_parameter('linear_speeds').value)
            self.point_angular_speed = list(self.get_parameter('angular_speeds').value)

            # Validate lengths
            if len(self.point_linear_speed) != num_points or len(self.point_angular_speed) != num_points:
                self.get_logger().error('linear_speeds and angular_speeds must have the same length as points!')
                return

            # Validate speed ranges
            for i, ls in enumerate(self.point_linear_speed):
                if ls <= 0.005 or ls > 0.3:
                    self.get_logger().error(f'Point {i}: linear_speed {ls} out of range (0.005, 0.3]')
                    return

            for i, as_ in enumerate(self.point_angular_speed):
                if as_ <= 0.005 or as_ > 0.72:
                    self.get_logger().error(f'Point {i}: angular_speed {as_} out of range (0.005, 0.72]')
                    return

            self.point_time = []

        elif self.mode == 1:
            self.point_time = list(self.get_parameter('path_times').value)

            if len(self.point_time) != num_points:
                self.get_logger().error('path_times must have the same length as points!')
                return

            self.point_linear_speed = []
            self.point_angular_speed = []

        else:
            self.get_logger().error(f'Invalid mode: {self.mode}. Must be 0 or 1.')
            return

        self.get_logger().info(f'Mode: {self.mode}')
        self.get_logger().info(f'Points X: {self.pointx}')
        self.get_logger().info(f'Points Y: {self.pointy}')

        # Publish once after a short delay to let subscribers connect
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.published = False
        self.get_logger().info('Path generator initialized, waiting to publish...')

    def timer_callback(self):
        if self.published:
            return

        pose = MakbetsPose()
        pose.x = self.pointx
        pose.y = self.pointy
        pose.mode = self.mode

        if self.mode == 0:
            pose.linear_speed = self.point_linear_speed
            pose.angular_speed = self.point_angular_speed
            pose.path_time = []
        elif self.mode == 1:
            pose.path_time = self.point_time
            pose.linear_speed = []
            pose.angular_speed = []

        self.path_pub.publish(pose)
        self.published = True
        self.get_logger().info('Path published successfully!')


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
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