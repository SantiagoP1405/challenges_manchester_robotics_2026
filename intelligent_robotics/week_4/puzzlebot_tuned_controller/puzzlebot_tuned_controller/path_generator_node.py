import rclpy
from rclpy.node import Node
import math
from makbets_pose.msg import MakbetsPose
from nav_msgs.msg import Odometry

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')

        # 1. Parámetros: Definimos 4 puntos por defecto (un cuadrado de 1m)
        self.declare_parameter('points_x', [1.0, 1.0, 0.0, 0.0])
        self.declare_parameter('points_y', [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('w_max', 0.5)
        self.declare_parameter('arrival_tolerance', 0.15) # Tolerancia para pasar al siguiente punto

        self.pointx = self.get_parameter('points_x').value
        self.pointy = self.get_parameter('points_y').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.tolerance = self.get_parameter('arrival_tolerance').value

        # Estado de la misión
        self.current_point_idx = 0
        self.mission_completed = False
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Pub/Sub
        self.goal_pub = self.create_publisher(MakbetsPose, '/goal', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer para revisar el estado del progreso (10Hz)
        self.timer = self.create_timer(0.1, self.check_progress)
        
        self.get_logger().info(f'Generador de trayectoria listo. Siguiendo {len(self.pointx)} puntos.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def check_progress(self):
        if self.mission_completed:
            return

        # Obtener el objetivo actual
        gx = self.pointx[self.current_point_idx]
        gy = self.pointy[self.current_point_idx]

        # Calcular distancia al objetivo actual
        dist = math.sqrt((gx - self.robot_x)**2 + (gy - self.robot_y)**2)

        if dist < self.tolerance:
            self.get_logger().info(f'¡Punto {self.current_point_idx + 1} alcanzado!')
            self.current_point_idx += 1
            
            if self.current_point_idx >= len(self.pointx):
                self.get_logger().info('¡Misión completada! Todos los puntos alcanzados.')
                self.mission_completed = True
                return

        # Publicar el objetivo actual para el controlador PID
        msg = MakbetsPose()
        msg.x = [float(gx)]
        msg.y = [float(gy)]
        msg.linear_speed = [float(self.v_max)]
        msg.angular_speed = [float(self.w_max)]
        msg.mode = 0 # El controlador usará modo posición
        
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()