
"""
vision_pipeline_node.py

Nodo de visión unificado que corre en la Jetson. Lee la cámara CSI UNA SOLA
VEZ por frame y ejecuta ambas detecciones (semáforo + línea) sobre el mismo
frame, cada una en su propia ROI.

Esto evita pagar dos veces el costo de captura, de serialización de
sensor_msgs/Image y de paso por DDS, manteniendo cada detector aislado como
clase pura (testeable independientemente).

Publica los topics que ya consumen los controladores existentes:
  /detected_color           std_msgs/String              "ROJO"/"AMARILLO"/"VERDE"/"NONE"
  /lane/state               std_msgs/Float32MultiArray   [lat_err_px, heading_rad,
                                                           la_x, la_y, visible_flag]
  /lane/lookahead_point     geometry_msgs/Point          (en coords de imagen)
  /lane/visible             std_msgs/Bool
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool
from geometry_msgs.msg import Point

import cv2
import numpy as np


# =====================================================================
#                          CÁMARA (compartida)
# =====================================================================

def gstreamer_pipeline(sensor_id=0, sensor_mode=3,
                       capture_width=1640, capture_height=1232,
                       display_width=640, display_height=480,
                       framerate=30, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, "
        f"height=(int){capture_height}, format=(string)NV12, "
        f"framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, "
        f"format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! "
        f"appsink drop=true max-buffers=1"
    )


# =====================================================================
#                       DETECTOR DE SEMÁFORO
# =====================================================================

# Rangos HSV (calibrados)
VERDE_BAJO    = np.array([73, 131, 180]);  VERDE_ALTO    = np.array([94, 255, 255])
AMARILLO_BAJO = np.array([22,   0, 222]);  AMARILLO_ALTO = np.array([155, 102, 255])
ROJO_BAJO_1   = np.array([  0,  60, 175]);  ROJO_ALTO_1 = np.array([ 15, 200, 255])
ROJO_BAJO_2   = np.array([165,  60, 175]);  ROJO_ALTO_2 = np.array([179, 200, 255])

# Umbrales de área a ~25 cm (calibrados con calibrate_traffic_light_area.py)
AREA_UMBRAL_VERDE    = 90
AREA_UMBRAL_AMARILLO = 208
AREA_UMBRAL_ROJO     = 201

# Filtros de forma para AMARILLO (circular)
AREA_MIN_AMARILLO = 100
AREA_MAX_AMARILLO = 15000
CIRCULARIDAD_MIN  = 0.6
ASPECT_RATIO_MIN  = 0.7
ASPECT_RATIO_MAX  = 1.4

# ROI poligonal del semáforo (del roi_picker, orden BL,BR,TR,TL)
LIGHT_ROI_VERTICES_FRAC = [
    (0.0172, 0.4396),
    (0.2531, 0.4375),
    (0.2437, 0.1396),
    (0.0187, 0.1167),
]


def _get_light_roi_polygon_and_bbox(frame_shape,
                                    vertices_frac=LIGHT_ROI_VERTICES_FRAC):
    h, w = frame_shape[:2]
    polygon = np.array(
        [(int(fx * w), int(fy * h)) for fx, fy in vertices_frac],
        dtype=np.int32,
    )
    xs, ys = polygon[:, 0], polygon[:, 1]
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
    return polygon, bbox


def _encontrar_circulo_amarillo(mask):
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    mejor = None
    mejor_area = 0
    for c in contornos:
        area = cv2.contourArea(c)
        if area < AREA_MIN_AMARILLO or area > AREA_MAX_AMARILLO:
            continue
        perimetro = cv2.arcLength(c, True)
        if perimetro == 0:
            continue
        circ = 4 * np.pi * area / (perimetro * perimetro)
        if circ < CIRCULARIDAD_MIN:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h == 0:
            continue
        aspect = w / h
        if aspect < ASPECT_RATIO_MIN or aspect > ASPECT_RATIO_MAX:
            continue
        if area > mejor_area:
            (cx, cy), radio = cv2.minEnclosingCircle(c)
            mejor = (area, (int(cx), int(cy)), int(radio))
            mejor_area = area
    return mejor


class TrafficLightDetector:
    """
    Detecta el color del semáforo dentro de una ROI poligonal.
    Aplica umbrales de área para que solo dispare a la distancia objetivo.
    """

    def __init__(self):
        self.kernel = np.ones((5, 5), np.uint8)

    def detect(self, frame):
        """
        Devuelve dict con:
          color:           "ROJO" | "AMARILLO" | "VERDE" | None
          area_verde:      int  (área cruda, sin umbral)
          area_amarillo:   int  (área del mejor círculo amarillo)
          area_rojo:       int
          info_amarillo:   (area, (cx, cy), radio) | None   (coords de la ROI)
          polygon:         np.ndarray (N, 2)  en coords del frame
          bbox:            (x1, y1, x2, y2)
        """
        polygon, (x1, y1, x2, y2) = _get_light_roi_polygon_and_bbox(frame.shape)
        roi = frame[y1:y2, x1:x2]

        # Máscara del polígono en coords locales
        poly_local = polygon - np.array([x1, y1])
        roi_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        cv2.fillPoly(roi_mask, [poly_local], 255)

        blur = cv2.GaussianBlur(roi, (7, 7), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        mask_v  = cv2.inRange(hsv, VERDE_BAJO,    VERDE_ALTO)
        mask_a  = cv2.inRange(hsv, AMARILLO_BAJO, AMARILLO_ALTO)
        mask_r1 = cv2.inRange(hsv, ROJO_BAJO_1,   ROJO_ALTO_1)
        mask_r2 = cv2.inRange(hsv, ROJO_BAJO_2,   ROJO_ALTO_2)
        mask_r  = cv2.bitwise_or(mask_r1, mask_r2)

        mask_v = cv2.bitwise_and(mask_v, mask_v, mask=roi_mask)
        mask_r = cv2.bitwise_and(mask_r, mask_r, mask=roi_mask)
        mask_a = cv2.bitwise_and(mask_a, mask_a, mask=roi_mask)

        mask_v = cv2.morphologyEx(mask_v, cv2.MORPH_OPEN,  self.kernel)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN,  self.kernel)
        mask_a = cv2.morphologyEx(mask_a, cv2.MORPH_OPEN,  self.kernel)
        mask_a = cv2.morphologyEx(mask_a, cv2.MORPH_CLOSE, self.kernel)

        area_verde = cv2.countNonZero(mask_v)
        area_rojo  = cv2.countNonZero(mask_r)
        info_amarillo = _encontrar_circulo_amarillo(mask_a)
        area_amarillo = info_amarillo[0] if info_amarillo is not None else 0

        areas_validas = {
            "VERDE":    area_verde    if area_verde    >= AREA_UMBRAL_VERDE    else 0,
            "AMARILLO": area_amarillo if area_amarillo >= AREA_UMBRAL_AMARILLO else 0,
            "ROJO":     area_rojo     if area_rojo     >= AREA_UMBRAL_ROJO     else 0,
        }

        color = max(areas_validas, key=areas_validas.get)
        if areas_validas[color] == 0:
            color = None

        return {
            'color':         color,
            'area_verde':    area_verde,
            'area_amarillo': area_amarillo,
            'area_rojo':     area_rojo,
            'info_amarillo': info_amarillo,
            'polygon':       polygon,
            'bbox':          (x1, y1, x2, y2),
        }


# =====================================================================
#                            LANE FOLLOWER
# =====================================================================

# ROI vertical angosta centrada (parte baja del frame)
ROI_HALF_W_FRAC = 0.18
ROI_TOP_FRAC    = 0.60
ROI_BOTTOM_FRAC = 1.00

# Slicing
N_SLICES             = 6
SLICE_HALF_H         = 4
MIN_PIXELS_PER_SLICE = 200

# Preprocesado
GAUSSIAN_KSIZE   = (5, 5)
THRESHOLD_VALUE  = 100
MORPH_KSIZE      = (3, 3)
MORPH_OPEN_ITER  = 1
MORPH_CLOSE_ITER = 2

# Failsafe
LOST_FRAMES_TIMEOUT = 5


class LaneFollower:
    """
    Detecta la línea central en una ROI vertical angosta usando slicing
    horizontal + image moments por slice.

    Convenciones de signo:
      lateral_error_px > 0  →  línea a la DERECHA del centro del frame
      heading_rad      > 0  →  línea se abre hacia la DERECHA al alejarse
    """

    def __init__(self):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_KSIZE)
        self.lost_count = 0
        self.last_result = None

    def _preprocess(self, roi_bgr):
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, GAUSSIAN_KSIZE, 0)
        _, binary = cv2.threshold(blur, THRESHOLD_VALUE, 255,
                                  cv2.THRESH_BINARY_INV)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,
                                  self.kernel, iterations=MORPH_OPEN_ITER)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,
                                  self.kernel, iterations=MORPH_CLOSE_ITER)
        return binary

    def compute(self, frame):
        h, w = frame.shape[:2]
        x1 = w // 2 - int(w * ROI_HALF_W_FRAC)
        x2 = w // 2 + int(w * ROI_HALF_W_FRAC)
        y1 = int(h * ROI_TOP_FRAC)
        y2 = int(h * ROI_BOTTOM_FRAC)
        roi = frame[y1:y2, x1:x2]

        mask = self._preprocess(roi)
        roi_h = mask.shape[0]

        ys_local = np.linspace(roi_h - 1, 0, num=N_SLICES, dtype=int)

        points = []
        for y in ys_local:
            y_lo = max(0, y - SLICE_HALF_H)
            y_hi = min(roi_h, y + SLICE_HALF_H + 1)
            band = mask[y_lo:y_hi, :]
            M = cv2.moments(band, binaryImage=True)
            if M["m00"] >= MIN_PIXELS_PER_SLICE:
                cx_local = M["m10"] / M["m00"]
                cx_frame = x1 + cx_local
                cy_frame = y1 + y
                points.append((float(cx_frame), float(cy_frame)))

        result = {
            'visible':          False,
            'lateral_error_px': 0.0,
            'heading_rad':      0.0,
            'points':           points,
            'roi_bounds':       (x1, y1, x2, y2),
        }

        if not points:
            self.lost_count += 1
            if (self.last_result is not None
                    and self.lost_count < LOST_FRAMES_TIMEOUT):
                stale = dict(self.last_result)
                stale['roi_bounds'] = (x1, y1, x2, y2)
                return stale
            return result

        self.lost_count = 0
        result['visible']          = True
        result['lateral_error_px'] = points[0][0] - (w / 2.0)
        if len(points) >= 2:
            dx = points[-1][0] - points[0][0]
            dy = points[-1][1] - points[0][1]
            result['heading_rad'] = float(math.atan2(dx, -dy))

        self.last_result = result
        return result


# =====================================================================
#                            NODO ROS
# =====================================================================

class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline')

        # Parámetros
        self.declare_parameter('debug_view', True)
        self.debug_view = (self.get_parameter('debug_view')
                           .get_parameter_value().bool_value)

        # Cámara
        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara CSI")
            raise RuntimeError("CSI camera not available")

        # Detectores (clases puras)
        self.light = TrafficLightDetector()
        self.lane  = LaneFollower()

        # Publishers (mismos topics que ya consumen los controladores)
        self.color_pub   = self.create_publisher(String,             '/detected_color',       10)
        self.state_pub   = self.create_publisher(Float32MultiArray,  '/lane/state',           10)
        self.target_pub  = self.create_publisher(Point,              '/lane/lookahead_point', 10)
        self.visible_pub = self.create_publisher(Bool,               '/lane/visible',         10)

        # Timer
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.frame_count   = 0
        self.ultimo_color  = None

        self.get_logger().info(
            "Vision pipeline iniciado: semáforo + lane sobre la misma cámara"
        )

    # ----------------- Bucle principal -----------------
    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Ambas detecciones sobre el MISMO frame
        light_r = self.light.detect(frame)
        lane_r  = self.lane.compute(frame)

        # Publicar
        self._publish_light(light_r)
        self._publish_lane(lane_r, frame.shape[1])

        # Log de cambio de color (event-based)
        if light_r['color'] != self.ultimo_color:
            if light_r['color']:
                area_dict = {
                    "VERDE":    light_r['area_verde'],
                    "AMARILLO": light_r['area_amarillo'],
                    "ROJO":     light_r['area_rojo'],
                }
                self.get_logger().info(
                    f"Semáforo {light_r['color']} a distancia objetivo "
                    f"(area={area_dict[light_r['color']]} px)"
                )
            else:
                self.get_logger().info("Sin semáforo a distancia objetivo")
            self.ultimo_color = light_r['color']

        # Log periódico combinado (cada ~1 s)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self._log_combined_status(light_r, lane_r)

        # Debug view
        if self.debug_view:
            self._draw_debug(frame, light_r, lane_r)
            cv2.waitKey(1)

    # ----------------- Publishers -----------------
    def _publish_light(self, light_r):
        msg = String()
        msg.data = light_r['color'] if light_r['color'] else "NONE"
        self.color_pub.publish(msg)

    def _publish_lane(self, lane_r, frame_width):
        # /lane/visible
        m = Bool()
        m.data = bool(lane_r['visible'])
        self.visible_pub.publish(m)

        # Lookahead = slice válido más lejano (o centro si nada visible)
        if lane_r['points']:
            la_x, la_y = lane_r['points'][-1]
        else:
            la_x, la_y = frame_width / 2.0, 0.0

        # /lane/state
        state = Float32MultiArray()
        state.data = [
            float(lane_r['lateral_error_px']),
            float(lane_r['heading_rad']),
            float(la_x),
            float(la_y),
            1.0 if lane_r['visible'] else 0.0,
        ]
        self.state_pub.publish(state)

        # /lane/lookahead_point
        pt = Point()
        pt.x = float(la_x)
        pt.y = float(la_y)
        pt.z = 0.0
        self.target_pub.publish(pt)

    # ----------------- Logging -----------------
    def _log_combined_status(self, light_r, lane_r):
        light_str = light_r['color'] if light_r['color'] else "----"
        if lane_r['visible']:
            lane_str = (
                f"err={lane_r['lateral_error_px']:+6.1f}px  "
                f"hdg={math.degrees(lane_r['heading_rad']):+5.1f}°  "
                f"pts={len(lane_r['points'])}/{N_SLICES}"
            )
        else:
            lane_str = "LOST"
        self.get_logger().info(f"semaforo={light_str:<8}  lane: {lane_str}")

    # ----------------- Debug view -----------------
    def _draw_debug(self, frame, light_r, lane_r):
        vis = frame.copy()
        h, w = vis.shape[:2]

        # ---- Lane ----
        lx1, ly1, lx2, ly2 = lane_r['roi_bounds']
        cv2.rectangle(vis, (lx1, ly1), (lx2, ly2), (0, 255, 0), 2)
        cv2.line(vis, (w // 2, 0), (w // 2, h), (180, 180, 180), 1)

        for i, (px, py) in enumerate(lane_r['points']):
            c = (0, 0, 255) if i == 0 else (0, 255, 255)
            cv2.circle(vis, (int(px), int(py)), 5, c, -1)

        if len(lane_r['points']) >= 2:
            pts = np.array([(int(p[0]), int(p[1])) for p in lane_r['points']],
                           dtype=np.int32)
            cv2.polylines(vis, [pts], False, (255, 0, 255), 2)

        # ---- Traffic light ----
        cv2.polylines(vis, [light_r['polygon']], isClosed=True,
                      color=(255, 255, 0), thickness=2)

        tx1, ty1, _, _ = light_r['bbox']
        info_a = light_r['info_amarillo']

        if light_r['color'] == "AMARILLO" and info_a is not None:
            _, (cx, cy), radio = info_a
            cx_full = cx + tx1
            cy_full = cy + ty1
            cv2.circle(vis, (cx_full, cy_full), radio, (0, 255, 255), 2)
            cv2.circle(vis, (cx_full, cy_full), 3,     (0, 255, 255), -1)
            cv2.putText(vis, "AMARILLO",
                        (cx_full - 30, cy_full - radio - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        elif light_r['color']:
            label_color = {"VERDE": (0, 255, 0),
                           "ROJO":  (0, 0, 255)}.get(light_r['color'],
                                                     (0, 255, 0))
            cv2.putText(vis, light_r['color'], (tx1, ty1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, label_color, 2)

        # ---- HUD ----
        light_str  = light_r['color'] if light_r['color'] else "NONE"
        lane_state = "VISIBLE" if lane_r['visible'] else "LOST"
        hud_lines = [
            f"SEMAFORO: {light_str}   "
            f"V:{light_r['area_verde']:4d}  "
            f"A:{light_r['area_amarillo']:4d}  "
            f"R:{light_r['area_rojo']:4d}",
            f"LANE: {lane_state}   "
            f"err={lane_r['lateral_error_px']:+6.1f}px   "
            f"hdg={math.degrees(lane_r['heading_rad']):+5.1f}°",
        ]
        for i, line in enumerate(hud_lines):
            y_text = 25 + i * 25
            cv2.putText(vis, line, (10, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
            cv2.putText(vis, line, (10, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        cv2.imshow("Vision Pipeline", vis)

    # ----------------- Limpieza -----------------
    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()