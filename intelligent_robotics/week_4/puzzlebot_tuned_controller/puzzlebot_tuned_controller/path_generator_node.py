import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from makbets_pose.msg import MakbetsPose
from rcl_interfaces.msg import SetParametersResult

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        
        self.declare_parameter('modo', 1)
        self.declare_parameter('puntos_manuales', [2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0])

        self.goal_tolerance = 0.1
        self.path = []
        self.current_goal_idx = 0
        self.path_completed = True
        self.current_x = 0.0
        self.current_y = 0.0
        self.pose_received = False
        self._SAFE_V_MAX = 0.3  
        self._SAFE_W_MAX = 0.7   

        self.goal_pub = self.create_publisher(MakbetsPose, '/goal', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.configurar_trayectoria()
        self.timer = self.create_timer(0.5, self.control_rutas)
        
        self.get_logger().info("Nodo listo. Cambia el 'modo' en rqt_reconfigure.")

    def actualizar_path_desde_lista(self, lista):
        if len(lista) % 2 != 0:
            self.get_logger().error("La lista de puntos debe tener un número par de elementos.")
            return False
        self.path = [(float(lista[i]), float(lista[i+1])) for i in range(0, len(lista), 2)]
        return True

    def configurar_trayectoria(self):
        modo = self.get_parameter('modo').value
        
        if modo == 1:
            raw_puntos = self.get_parameter('puntos_manuales').value
            if not self.actualizar_path_desde_lista(raw_puntos):
                return
            self.get_logger().info("Modo 1: Puntos manuales.")
        elif modo == 2:
            self.path = [(0.0, 0.0), (2.0, 0.0), (1.0, 1.5), (0.0, 0.0)]
            self.get_logger().info("Modo 2: Triángulo.")
        elif modo == 3:
            self.path = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0), (1.0, -1.0), (0.0, 0.0)]
            self.get_logger().info("Modo 3: Rombo.")
        elif modo == 4:
            self.path = [(0.0, 0.0), (2.0, 0.0), (2.5, 1.5), (1.0, 2.5), (-0.5, 1.5), (0.0, 0.0)]
            self.get_logger().info("Modo 4: Pentágono.")
        else:
            return

        self.current_goal_idx = 0
        self.path_completed = False
        self.verificar_viabilidad()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'modo' or param.name == 'puntos_manuales':
                self.get_logger().info(f"Actualizando a {param.name}...")
                self.executor.create_task(self.configurar_trayectoria)                
        return SetParametersResult(successful=True)

    def verificar_viabilidad(self):
        for i in range(len(self.path)):
            if i == 0:
                dist = math.sqrt(self.path[i][0]**2 + self.path[i][1]**2)
            else:
                dist = math.sqrt((self.path[i][0]-self.path[i-1][0])**2 + (self.path[i][1]-self.path[i-1][1])**2)

            if dist > 3.0:
                self.get_logger().error(f"¡Ruta muy larga! Tramo {i+1}: {dist:.2f}m")
                self.path_completed = True

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.pose_received = True

    def control_rutas(self):
        if not self.pose_received or self.path_completed or not self.path:
            return

        gx, gy = self.path[self.current_goal_idx]

        dx, dy = gx - self.current_x, gy - self.current_y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < self.goal_tolerance:
            self.current_goal_idx += 1
            if self.current_goal_idx >= len(self.path):
                self.get_logger().info("Ruta completada.")
                self.path_completed = True
            return

        msg = MakbetsPose()
        msg.x, msg.y = [float(gx), float(gx)], [float(gy), float(gy)]
        msg.linear_speed = [self._SAFE_V_MAX, self._SAFE_V_MAX]
        msg.angular_speed = [self._SAFE_W_MAX, self._SAFE_W_MAX]
        msg.mode = 0 
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