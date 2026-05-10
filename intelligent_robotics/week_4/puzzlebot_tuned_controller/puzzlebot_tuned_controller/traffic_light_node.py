#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np


# === Rangos HSV ===
VERDE_BAJO    = np.array([40,  70,  70]);  VERDE_ALTO    = np.array([85, 255, 255])
AMARILLO_BAJO = np.array([16,  73, 139]);  AMARILLO_ALTO = np.array([40, 164, 255])

# Rojo: dos rangos por el wrap-around del HSV
ROJO_BAJO_1   = np.array([  0,  80, 180]);  ROJO_ALTO_1   = np.array([ 10, 255, 255])
ROJO_BAJO_2   = np.array([170,  80, 180]);  ROJO_ALTO_2   = np.array([179, 255, 255])

# === Umbrales de área calibrados a ~25 cm ===
AREA_UMBRAL_VERDE    = 1650   # min observado: 1711
AREA_UMBRAL_AMARILLO = 1180   # min observado: 1236
AREA_UMBRAL_ROJO     = 1380   # min observado: 1418

# Filtros de forma para AMARILLO solamente
AREA_MIN_AMARILLO     = 200
AREA_MAX_AMARILLO     = 15000
CIRCULARIDAD_MIN      = 0.6
ASPECT_RATIO_MIN      = 0.7
ASPECT_RATIO_MAX      = 1.4

# === ROI central ===
ROI_W_FRAC = 0.2
ROI_H_FRAC = 0.6


def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720,
                       display_width=640, display_height=360, framerate=30, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1"
    )


def get_roi_bounds(frame_shape, w_frac=ROI_W_FRAC, h_frac=ROI_H_FRAC):
    h, w = frame_shape[:2]
    rw, rh = int(w * w_frac), int(h * h_frac)
    x1 = (w - rw) // 2
    y1 = (h - rh) // 2
    return x1, y1, x1 + rw, y1 + rh


def encontrar_circulo_amarillo(mask):
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara CSI")
            raise RuntimeError("CSI camera not available")

        self.color_pub = self.create_publisher(String, '/detected_color', 10)
        self.kernel = np.ones((5, 5), np.uint8)
        self.ultimo_color = None
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        x1, y1, x2, y2 = get_roi_bounds(frame.shape)
        roi = frame[y1:y2, x1:x2]

        blur = cv2.GaussianBlur(roi, (7, 7), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # === Máscaras HSV ===
        mask_v = cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO)
        mask_a = cv2.inRange(hsv, AMARILLO_BAJO, AMARILLO_ALTO)
        mask_r1 = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1)
        mask_r2 = cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
        mask_r  = cv2.bitwise_or(mask_r1, mask_r2)

        mask_v = cv2.morphologyEx(mask_v, cv2.MORPH_OPEN, self.kernel)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, self.kernel)
        mask_a = cv2.morphologyEx(mask_a, cv2.MORPH_OPEN,  self.kernel)
        mask_a = cv2.morphologyEx(mask_a, cv2.MORPH_CLOSE, self.kernel)

        # === Áreas ===
        area_verde = cv2.countNonZero(mask_v)
        area_rojo  = cv2.countNonZero(mask_r)
        info_amarillo = encontrar_circulo_amarillo(mask_a)
        area_amarillo = info_amarillo[0] if info_amarillo is not None else 0

        # === Aplicar UMBRALES DE DISTANCIA (~25 cm) ===
        # Solo se considera detectado si el área supera el umbral calibrado.
        # Si el coche está más lejos, el área es menor y no publica el color.
        areas_validas = {
            "VERDE":    area_verde    if area_verde    >= AREA_UMBRAL_VERDE    else 0,
            "AMARILLO": area_amarillo if area_amarillo >= AREA_UMBRAL_AMARILLO else 0,
            "ROJO":     area_rojo     if area_rojo     >= AREA_UMBRAL_ROJO     else 0,
        }

        color = max(areas_validas, key=areas_validas.get)
        if areas_validas[color] == 0:
            color = None

        # === Publica ===
        msg = String()
        msg.data = color if color else "NONE"
        self.color_pub.publish(msg)

        if color != self.ultimo_color:
            if color:
                self.get_logger().info(
                    f"Semáforo {color} a distancia objetivo "
                    f"(area={areas_validas[color]} px)"
                )
            else:
                self.get_logger().info("Sin semáforo a distancia objetivo")
            self.ultimo_color = color

        # === Visualización ===
        vista = frame.copy()
        cv2.rectangle(vista, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Texto con áreas crudas (sin umbral) para depurar
        cv2.putText(vista, f"V:{area_verde}  A:{area_amarillo}  R:{area_rojo}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if color == "AMARILLO" and info_amarillo is not None:
            _, (cx, cy), radio = info_amarillo
            cx_full = cx + x1
            cy_full = cy + y1
            cv2.circle(vista, (cx_full, cy_full), radio, (0, 255, 255), 2)
            cv2.circle(vista, (cx_full, cy_full), 3, (0, 255, 255), -1)
            cv2.putText(vista, color, (cx_full - 30, cy_full - radio - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        elif color:
            cv2.putText(vista, color, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("CSI", vista)
        cv2.imshow("Mask VERDE",    mask_v)
        cv2.imshow("Mask AMARILLO", mask_a)
        cv2.imshow("Mask ROJO",     mask_r)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()