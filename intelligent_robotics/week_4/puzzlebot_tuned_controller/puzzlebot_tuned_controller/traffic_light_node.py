#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np


# Rangos HSV
VERDE_BAJO    = np.array([40,  70,  70]);  VERDE_ALTO    = np.array([85, 255, 255])
AMARILLO_BAJO = np.array([18,  40, 200]);  AMARILLO_ALTO = np.array([35, 255, 255])
ROJO_BAJO     = np.array([ 0, 120, 120]);  ROJO_ALTO     = np.array([18, 255, 255])
AREA_MIN = 500

# ROI central
ROI_W_FRAC = 0.5
ROI_H_FRAC = 0.5


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

        # 30 Hz para igualar el framerate del pipeline
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        x1, y1, x2, y2 = get_roi_bounds(frame.shape)
        roi = frame[y1:y2, x1:x2]

        blur = cv2.GaussianBlur(roi, (7, 7), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        mask_v = cv2.inRange(hsv, VERDE_BAJO,    VERDE_ALTO)
        mask_a = cv2.inRange(hsv, AMARILLO_BAJO, AMARILLO_ALTO)
        mask_r = cv2.inRange(hsv, ROJO_BAJO,     ROJO_ALTO)

        mask_v = cv2.morphologyEx(mask_v, cv2.MORPH_OPEN, self.kernel)
        mask_a = cv2.morphologyEx(mask_a, cv2.MORPH_OPEN, self.kernel)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, self.kernel)

        areas = {
            "VERDE":    cv2.countNonZero(mask_v),
            "AMARILLO": cv2.countNonZero(mask_a),
            "ROJO":     cv2.countNonZero(mask_r),
        }
        color = max(areas, key=areas.get)
        if areas[color] < AREA_MIN:
            color = None

        msg = String()
        msg.data = color if color else "NONE"
        self.color_pub.publish(msg)

        if color != self.ultimo_color:
            if color:
                self.get_logger().info(f"Color detectado: {color} ({areas[color]} px)")
            else:
                self.get_logger().info("Sin color detectado")
            self.ultimo_color = color

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
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