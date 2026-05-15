#!/usr/bin/env python3
"""
pure_pursuit_lane_controller.py  (con máquina de estados de semáforo)

PC-side: suscribe topics de visión de la Jetson y el color detectado,
publica /cmd_vel.

Topics de entrada:
  /lane/lookahead_point  geometry_msgs/Point   (coords de imagen)
  /lane/visible          std_msgs/Bool
  /detected_color        std_msgs/String        ← NUEVO: "ROJO","AMARILLO","VERDE","NONE"

Topic de salida:
  /cmd_vel               geometry_msgs/Twist

Estados:
  STOP   → factor=0.0  (velocidad cero)
  SLOW   → factor=0.3  (v y ω escalados al 30%)
  GO     → factor=1.0  (operación normal)

Convenciones (REP-103):
  linear.x  > 0  → adelante
  angular.z > 0  → giro a la izquierda (CCW)
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, String


class PurePursuitLaneController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_lane_controller')

        # ====================== Parámetros ======================
        self.declare_parameter('frame_width',  640)
        self.declare_parameter('frame_height', 480)

        self.declare_parameter('v_nominal', 0.12)
        self.declare_parameter('v_min',     0.05)

        self.declare_parameter('k_omega',   1.8)
        self.declare_parameter('omega_max', 1.8)

        # cos(alpha)^slow_exp: slow_exp=1 suave, slow_exp=2 frena más en curvas
        self.declare_parameter('slow_exp', 1.5)

        self.declare_parameter('lost_timeout_sec', 0.4)
        self.declare_parameter('enabled', True)
        self.declare_parameter('control_rate_hz', 30.0)

        # Factor de velocidad para estado SLOW (0.0–1.0)
        self.declare_parameter('slow_factor', 0.3)

        self._reload_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        # ====================== Estado interno ======================
        self.lookahead       = None
        self.visible         = False
        self.last_visible_time = self.get_clock().now()
        self.last_cmd        = (0.0, 0.0)

        # Máquina de estados de semáforo
        self.traffic_state   = "STOP"   # arranca detenido hasta recibir VERDE

        # ====================== Subs / Pubs ======================
        self.create_subscription(
            Point,  '/lane/lookahead_point', self._lookahead_cb, 10)
        self.create_subscription(
            Bool,   '/lane/visible',         self._visible_cb,   10)
        self.create_subscription(
            String, '/detected_color',       self._color_cb,     10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.dt     = 1.0 / self.control_rate_hz
        self.timer  = self.create_timer(self.dt, self._control_loop)

        self._tick      = 0
        self._log_every = max(1, int(self.control_rate_hz))    # ~1 Hz

        self.get_logger().info(
            f"Pure pursuit + semáforo listo. "
            f"v_nominal={self.v_nominal:.2f} m/s, "
            f"k_omega={self.k_omega:.2f}, "
            f"enabled={self.enabled}. "
            f"Esperando VERDE para arrancar."
        )

    # ------------------------------------------------------------------ #
    #  Parámetros dinámicos
    # ------------------------------------------------------------------ #
    def _reload_params(self):
        self.frame_w         = self.get_parameter('frame_width').value
        self.frame_h         = self.get_parameter('frame_height').value
        self.v_nominal       = self.get_parameter('v_nominal').value
        self.v_min           = self.get_parameter('v_min').value
        self.k_omega         = self.get_parameter('k_omega').value
        self.omega_max       = self.get_parameter('omega_max').value
        self.slow_exp        = self.get_parameter('slow_exp').value
        self.lost_timeout    = self.get_parameter('lost_timeout_sec').value
        self.enabled         = self.get_parameter('enabled').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value
        self.slow_factor     = self.get_parameter('slow_factor').value

    def _on_param_change(self, params):
        for p in params:
            self.get_logger().info(f"Param {p.name} → {p.value}")
        self._reload_params()
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #
    #  Callbacks
    # ------------------------------------------------------------------ #
    def _lookahead_cb(self, msg: Point):
        self.lookahead = (msg.x, msg.y)

    def _visible_cb(self, msg: Bool):
        self.visible = bool(msg.data)
        if self.visible:
            self.last_visible_time = self.get_clock().now()

    def _color_cb(self, msg: String):
        """Máquina de estados de semáforo."""
        color = msg.data
        if color == "NONE":
            return                          # sin detección: mantén estado actual

        mapping = {"ROJO": "STOP", "AMARILLO": "SLOW", "VERDE": "GO"}
        new_state = mapping.get(color)
        if new_state is None:
            self.get_logger().warn(f"Color desconocido recibido: '{color}'")
            return

        if new_state != self.traffic_state:
            self.get_logger().info(
                f"Semáforo: {color} → estado={new_state}"
            )
            self.traffic_state = new_state

    # ------------------------------------------------------------------ #
    #  Lazo de control
    # ------------------------------------------------------------------ #
    def _control_loop(self):

        # 1) Enable global
        if not self.enabled:
            self._publish(0.0, 0.0)
            return

        # 2) Semáforo: STOP → parar de inmediato
        if self.traffic_state == "STOP":
            if self.last_cmd != (0.0, 0.0):
                self.get_logger().info("Semáforo ROJO → deteniendo robot.")
            self._publish(0.0, 0.0)
            return

        # 3) Failsafe por línea perdida
        if not self._line_is_fresh():
            if self.last_cmd != (0.0, 0.0):
                self.get_logger().warn(
                    "Línea perdida más allá del timeout → parando"
                )
            self._publish(0.0, 0.0)
            return

        # 4) Necesitamos al menos un punto de lookahead
        if self.lookahead is None:
            self._publish(0.0, 0.0)
            return

        # 5) Geometría pure pursuit (image-space)
        la_x, la_y = self.lookahead
        dx_px = la_x - (self.frame_w / 2.0)
        dy_px = self.frame_h - la_y          # > 0 si el target está adelante

        # [CALIB] En v2: mapear (dx_px, dy_px) → (dx_m, dy_m) con homografía.

        if dy_px <= 1.0:
            dy_px = 1.0

        # 6) Ángulo al lookahead. alpha > 0 → target a la DERECHA.
        alpha = math.atan2(dx_px, dy_px)

        # 7) Pure pursuit: ω = -K·alpha
        omega = -self.k_omega * alpha

        # 8) Velocidad: frena en curvas
        slowdown = max(0.0, math.cos(alpha)) ** self.slow_exp
        v = max(self.v_min, self.v_nominal * slowdown)

        # 9) Clamps base
        omega = max(-self.omega_max, min(self.omega_max, omega))
        v     = max(0.0,             min(self.v_nominal, v))

        # 10) Semáforo SLOW: escala v y ω
        if self.traffic_state == "SLOW":
            v     *= self.slow_factor
            omega *= self.slow_factor
            # Asegura que v no baje del v_min solo si sigue habiendo movimiento
            if v > 0.0:
                v = max(self.v_min * self.slow_factor, v)

        self._publish(v, omega)

        # 11) Log periódico
        self._tick += 1
        if self._tick % self._log_every == 0:
            self.get_logger().info(
                f"[{self.traffic_state}] "
                f"alpha={math.degrees(alpha):+6.1f}°  "
                f"dx={dx_px:+6.1f}px  dy={dy_px:6.1f}px  "
                f"→  v={v:.3f} m/s  ω={omega:+.2f} rad/s"
            )

    # ------------------------------------------------------------------ #
    #  Helpers
    # ------------------------------------------------------------------ #
    def _line_is_fresh(self) -> bool:
        if self.visible:
            return True
        elapsed = (
            self.get_clock().now() - self.last_visible_time
        ).nanoseconds / 1e9
        return elapsed <= self.lost_timeout

    def _publish(self, v: float, omega: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)
        self.last_cmd = (v, omega)

    def destroy_node(self):
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitLaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()