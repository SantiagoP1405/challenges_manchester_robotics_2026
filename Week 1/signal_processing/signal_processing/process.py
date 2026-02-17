import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SignalProcess(Node):
    def __init__(self):
        super().__init__('process')

        # --- Subscriber ---
        self.sine_signal_subscriber = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.time_subscriber = self.create_subscription(Float32, '/time', self.time_callback, 10)

        self.phase = math.pi / 4.0
        
        self.processed_signal_publisher = self.create_publisher(Float32, '/proc_signal', 10)

        self.signal = None
        self.time = None

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('---- Signal Processor Initiated :) ----')
    
    def signal_callback(self, msg):
        self.signal = msg.data

    def time_callback(self, msg):
        self.time = msg.data
    
    def timer_callback(self):
        if self.time is None:
            return

        # Phase
        shifted = math.sin(self.time + self.phase)

        # Offset and amplitude reduction
        result = 0.5 * (shifted + 1.0)

        out = Float32()
        out.data = float(result)

        self.processed_signal_publisher.publish(out)

        self.get_logger().info(
            f't={self.time:.2f}  proc_signal={result:.3f}'
        )

def main():
    rclpy.init()
    node = SignalProcess()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()