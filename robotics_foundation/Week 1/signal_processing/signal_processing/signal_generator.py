import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SignalGenerator(Node):

    def __init__(self):
        super().__init__('signal_generator')
        # --- Publisher ---
        self.sine_signal_publisher = self.create_publisher(Float32, '/signal', 10)
        self.time_publisher = self.create_publisher(Float32, '/time', 10)

        # --- Timer ---
        self.start_time = self.get_clock().now()

        # Timer -> 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('---- Signal Generator Initiated :) ----')

    def timer_callback(self):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Sign signal
        signal = math.sin(current_time)
        
        # Messages for the topics
        signal_msg = Float32()
        time_msg = Float32()

        signal_msg.data = float(signal)
        time_msg.data = float(current_time)

        self.sine_signal_publisher.publish(signal_msg) # Publish to /signal topic
        self.time_publisher.publish(time_msg) #Publish to /time topic

        # Print in terminal
        self.get_logger().info(f't={current_time:.2f}  signal={signal:.3f}')

def main():
    rclpy.init()
    node = SignalGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()