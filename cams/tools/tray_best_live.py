#!/usr/bin/env python3
"""One-shot: grab /camera/rgb, run the captain's tray model, save annotated."""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

MODEL = sys.argv[1] if len(sys.argv) > 1 else '/root/weights/tray_best.pt'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/tray_best_view.png'
IMGSZ = int(sys.argv[3]) if len(sys.argv) > 3 else 960


class Grab(Node):
    def __init__(self):
        super().__init__('tray_model_probe')
        self.rgb = None
        self.create_subscription(Image, '/camera/rgb', self._rgb,
                                 qos_profile_sensor_data)

    def _rgb(self, m):
        self.rgb = m


def main():
    rclpy.init()
    n = Grab()
    end = time.time() + 15
    while rclpy.ok() and n.rgb is None and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.5)
    if n.rgb is None:
        print('NO FRAME')
        sys.exit(1)
    m = n.rgb
    arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
    rgb = arr[:, :m.width * 3].copy().reshape(m.height, m.width, 3)
    bgr = rgb[:, :, ::-1].copy()

    from ultralytics import YOLO
    model = YOLO(MODEL)
    res = model.predict(bgr, imgsz=IMGSZ, conf=0.25, verbose=False)
    r = res[0]
    for b in r.boxes:
        cls = r.names[int(b.cls)]
        xy = [round(float(v)) for v in b.xyxy[0].tolist()]
        print(f'DET {cls} conf={float(b.conf):.3f} bbox={xy}')
    if len(r.boxes) == 0:
        print('DET none')

    import cv2
    cv2.imwrite(OUT, r.plot())
    print(f'SAVED {OUT}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
