# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('input_node')
        
        # Parameters for the input signal
        self.declare_parameter('amplitude', 17 * ((2*np.pi) / 60)) # Convert 17 RPM to rad/s. 
        # This is the maximum speed of the motor, so it ensures that the input signal is within the system's capabilities.
        self.declare_parameter('omega', 0.3) # Frequency of the input signal (rad/s). This value is chosen to be low enough 
        # to allow the system to respond, but high enough to test the dynamics of the system.
        self.declare_parameter('timer_period', 0.02) # Timer period for publishing the input signal (s). 
        # This value is chosen to be small enough to provide a smooth signal,

        self.declare_parameter('signal_type', 'sine')  # New parameter for signal type
        # This parameter allows switching between different types of input signals.
        
        # Retrieve input signal parameters
        self.amplitude = self.get_parameter('amplitude').value
        self.omega  = self.get_parameter('omega').value
        self.timer_period = self.get_parameter('timer_period').value
        self.signal_type = self.get_parameter('signal_type').value

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10) 
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("SetPoint Node Started \U0001F680")

    # Timer Callback: Generate and Publish Input Signal
    def timer_cb(self):
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        # Generate sine wave signal
        if self.signal_type == 'sine':
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        #Generate square signal
        elif self.signal_type == 'square':
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))
        # Generate step signal
        elif self.signal_type == 'step':
            self.signal_msg.data = self.amplitude if elapsed_time > 1.0 else 0.0
        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)
    
    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "amplitude":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid amplitude! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="amplitude cannot be negative")
                else:
                    self.amplitude = param.value  # Update internal variable
                    self.get_logger().info(f"amplitude updated to {self.amplitude}")
            #system gain parameter check
            if param.name == "omega":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid omega! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="omega cannot be negative")
                else:
                    self.omega = param.value  # Update internal variable
                    self.get_logger().info(f"omega updated to {self.omega}")
            #system gain parameter check
            if param.name == "timer_period":
                #check if it is negative
                if (param.value <= 0.0):
                    self.get_logger().warn("Invalid timer_period! It must be positive.")
                    return SetParametersResult(successful=False, reason="timer_period must be positive")
                else:
                    self.timer_period = param.value  # Update internal variable
                    self.timer.cancel()  # Cancel the existing timer
                    self.timer = self.create_timer(self.timer_period, self.timer_cb)  # Create a new timer with the updated period
                    self.get_logger().info(f"timer_period updated to {self.timer_period}")
            #system gain parameter check
            if param.name == "signal_type":
                #check if it is valid
                if (param.value not in ['sine', 'square', 'step']):
                    self.get_logger().warn("Invalid signal_type! It must be 'sine', 'square', or 'step'.")
                    return SetParametersResult(successful=False, reason="signal_type must be 'sine', 'square', or 'step'")
                else:
                    self.signal_type = param.value  # Update internal variable
                    self.get_logger().info(f"signal_type updated to {self.signal_type}")
        return SetParametersResult(successful=True)
    

#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
