import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('Ts', 0.005) # Sampling 4 times faster than the input signal (0.02s) to ensure good performance

        # Parameters for the PID controller. Ziegler-Nichols tuning method was used to get initial values, then manually tuned for better performance.
        # Though the controller structure corresponds to a PID, the derivative term is not used in this implementation, as it did not improve 
        # performance and added noise sensitivity.
        self.declare_parameter('kp', 0.0874503393459739)
        self.declare_parameter('ki', 2.54986519821649)
        self.declare_parameter('kd', 0.0)

        self.Ts = self.get_parameter('Ts').value
        # Create a subscriber to the set point and motor speed
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        #Publisher for the control signal
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input', 10)
        # Subscriber for the motor speed feedback
        self.motor_w_sub = self.create_subscription(Float32, 'motor_w', self.motor_w_callback, 10)
        # Timer for the control loop, update at the same rate as the sampling time
        self.timer = self.create_timer(self.Ts, self.timer_callback)

        # Initialize variables
        self.set_point = 0.0
        self.motor_w = 0.0
        self.debug_motor_w = 0.0
        self.deathzone = 0.4

        self.rpm_max = 17.0 # Maximum speed of the motor in RPM, used for normalizing the set point and feedback to the range [0,1]
        self.w_max = self.rpm_max * ((2*np.pi) / 60) # Convert RPM to rad/s for internal calculations, as the controller works with normalized values.
        
        # Controller parameters
        # Proportional gain
        self.kp = self.get_parameter('kp').value
        # Integral gain
        self.ki = self.get_parameter('ki').value
        # Derivative gain
        self.kd = self.get_parameter('kd').value
        
        # PID state
        self.error_prev = 0.0       # e(k-1)
        self.error_int = 0.0        # sum(e) for integral term

        self.get_logger().info("Controller Node Started")

        #Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    # Received set point callback
    def set_point_callback(self, msg):
        self.set_point = msg.data / self.w_max

    # Received motor speed callback
    def motor_w_callback(self, msg):
        self.debug_motor_w = msg.data # Variable for debugging purposes, to see the speed in rad/s.
        self.motor_w = msg.data / self.w_max # Convert to normalized value

    def timer_callback(self):
        # Compute the control error between the desired set point and the measured motor speed
        error = self.set_point - self.motor_w
        # Print debug information: set point, measured speed, and current error
        self.get_logger().info(f"SP: {self.set_point:.2f}, W: {self.debug_motor_w:.2f}, E: {error:.2f}")
        # Compute the derivative of the error (rate of change)
        error_der = (error - self.error_prev) / self.Ts
        # Store the current error to use in the next iteration
        self.error_prev = error
        # Compute the preliminary PID output (before updating the integral term)
        u = self.kp * error + self.ki * self.error_int + self.kd * error_der

        # Anti-windup mechanism:
        # The integral term of a PID controller accumulates the error over time.
        # When the controller output exceeds the actuator limits (here [-1, 1]),
        # the signal is saturated and the actuator cannot apply the additional control effort.
        #
        # If the integrator continues accumulating error during saturation,
        # the integral term can grow excessively (windup).
        # This typically causes large overshoot, slow recovery, and oscillations once the
        # system re-enters the controllable region.
        #
        # To prevent this, the integrator is updated only when the control signal is not
        # saturated. In this way, the integral term grows only when the actuator is able
        # to respond to the controller output.
        if abs(u) < 1.0:
            self.error_int += error * self.Ts

        # Recalculate the control signal with the updated integral term
        u = self.kp * error + self.ki * self.error_int + self.kd * error_der

        # Saturate the control signal to the valid range [-1, 1] 
        u = np.clip(u, -1.0, 1.0)
        
        # Deadzone compensation:
        # Map the control signal from [-1,1] to [-1,-0.4] U [0.4,1]
        # to avoid the motor deadzone around zero, since the 
        if abs(u) < 0.01:
            u = 0.0
        elif u > 0.0:
            u = 0.4 + (u * 0.6)
        elif u < 0.0:
            u = -0.4 + (u * 0.6)

        # Create the ROS message for the control signal
        msg = Float32()
         # Assign the computed control value
        msg.data = float(u)
        # Publish the control signal to the motor input topic
        self.motor_input_pub.publish(msg)

    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "Ts":
                #check if it is negative
                if (param.value <= 0.0):
                    self.get_logger().warn("Invalid Ts! It must be positive.")
                    return SetParametersResult(successful=False, reason="Ts must be positive")
                else:
                    self.Ts = param.value  
                    self.get_logger().info(f"Ts updated to {self.Ts}")
                
            #system gain parameter check
            if param.name == "kp":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp = param.value  
                    self.get_logger().info(f"kp updated to {self.kp}")
            #system gain parameter check
            if param.name == "ki":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki = param.value
                    self.get_logger().info(f"ki updated to {self.ki}")
            #system gain parameter check
            if param.name == "kd":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd = param.value 
                    self.get_logger().info(f"kd updated to {self.kd}")

        return SetParametersResult(successful=True)

#Main
def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#Execute Node
if __name__ == '__main__':
    main()