import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('t_s', 0.02)

        # Parameters for the PID controller
        self.declare_parameter('kp', 0.0371615460917942)
        self.declare_parameter('ki', 0.829038652294856)
        self.declare_parameter('kd', 0.0)

        self.Ts = self.get_parameter('t_s').value
        # Create a subscriber to the set point and motor speed
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.motor_output_sub = self.create_subscription(Float32, 'motor_output_y', self.motor_output_y_callback, 10)

        self.timer = self.create_timer(self.Ts, self.timer_callback)

        # Initialize variables
        self.set_point = 0.0
        self.motor_output = 0.0
        
        # Controller parameters (for a simple P controller)
        # Proportional gain
        self.kp = self.get_parameter('kp').value
        # Integral gain
        self.ki = self.get_parameter('ki').value
        # Derivative gain
        self.kd = self.get_parameter('kd').value
        
        # Estados del PID
        self.error_prev = 0.0       # e(k-1)
        self.error_int = 0.0        # sumatoria Σ e(n)

        #self.uk = None
        self.get_logger().info("Controller Node Started")
        #Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    def set_point_callback(self, msg):
        self.set_point = msg.data

    def motor_output_y_callback(self, msg):
        self.motor_output = msg.data

    def timer_callback(self):
        # error actual e(k)
        error = self.set_point - self.motor_output

        # INTEGRAL: sumatoria e(n)
        self.error_int += error * self.Ts   # ≈ Ts * Σ e(n)

        # DERIVADA: (e(k) - e(k-1)) / Ts
        error_der = (error - self.error_prev) / self.Ts

        # PID discreto
        u = self.kp * error + self.ki * self.error_int + self.kd * error_der

        # guardar error para la próxima iteración
        self.error_prev = error

        # Publicar
        msg = Float32()
        msg.data = float(u)
        self.motor_input_pub.publish(msg)

    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "t_s":
                #check if it is negative
                if (param.value <= 0.0):
                    self.get_logger().warn("Invalid t_s! It must be positive.")
                    return SetParametersResult(successful=False, reason="t_s must be positive")
                else:
                    self.Ts = param.value  # Update internal variable
                    self.get_logger().info(f"t_s updated to {self.Ts}")
                
            #system gain parameter check
            if param.name == "kp":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp = param.value  # Update internal variable
                    self.get_logger().info(f"kp updated to {self.kp}")
            #system gain parameter check
            if param.name == "ki":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki = param.value  # Update internal variable
                    self.get_logger().info(f"ki updated to {self.ki}")
            #system gain parameter check
            if param.name == "kd":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd = param.value  # Update internal variable
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
