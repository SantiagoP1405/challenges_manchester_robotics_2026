import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('Ts', 0.01)

        # Parameters for the PID controller
        self.declare_parameter('kp', 0.559290436564823)
        self.declare_parameter('ki', 0.954858253910344)
        self.declare_parameter('kd', 0.0)

        self.Ts = self.get_parameter('Ts').value
        # Create a subscriber to the set point and motor speed
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input', 10)
        self.motor_w_sub = self.create_subscription(Float32, 'motor_w', self.motor_w_callback, 10)

        self.timer = self.create_timer(self.Ts, self.timer_callback)

        # Initialize variables
        self.set_point = 0.0
        self.motor_w = 0.0
        self.deathzone = 0.3
        self.rpm_max = 22.0
        self.w_max = 22 * ((2*np.pi) / 60)
        
        # Controller parameters
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
        self.set_point = msg.data / self.w_max

    def motor_w_callback(self, msg):
        self.motor_w = msg.data / self.w_max

    def timer_callback(self):
        error = self.set_point - self.motor_w
        self.get_logger().info(f"SP: {self.set_point:.2f}, W: {self.motor_w:.2f}, E: {error:.2f}")

        error_der = (error - self.error_prev) / self.Ts
        self.error_prev = error

        # PID sin integrar todavía
        u = self.kp * error + self.ki * self.error_int + self.kd * error_der

        # # Anti-windup: solo integra si la salida NO está saturada
        # if abs(u) < 1.0:
        #     self.error_int += error * self.Ts

        # Recalcular con el integrador actualizado
        u = self.kp * error + self.ki * self.error_int + self.kd * error_der


        # Deadzone: solo aplica si hay señal
        # if abs(u) > 0.01:
        #     u = np.sign(u) * (0.3 + abs(u) * 0.7)

        u = np.clip(u, -1.0, 1.0)
        # Deadzone: mapea [-1,-0.3] U [0.3,1] evitando la zona muerta
        if abs(u) < 0.01:
            u = 0.0
        elif u > 0.0:
            u = 0.3 + (u * 0.7)
        elif u < 0.0:
            u = -0.3 + (u * 0.7)

        

        msg = Float32()
        msg.data = float(u)
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