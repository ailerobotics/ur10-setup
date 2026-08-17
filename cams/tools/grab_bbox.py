#!/usr/bin/env python3
"""Wait for a tray detection, then save the RGB frame with the bbox drawn."""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/detection.png'
WAIT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0


class Grab(Node):
    def __init__(self):
        super().__init__('bbox_grab')
        self.rgb = None
        self.bbox = None
        self.create_subscription(Image, '/camera/rgb', self._rgb,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/perception/tray_bbox',
                                 self._bbox, 10)

    def _rgb(self, m):
        self.rgb = m

    def _bbox(self, m):
        self.bbox = list(m.data)


def main():
    rclpy.init()
    n = Grab()
    end = time.time() + WAIT_S
    while rclpy.ok() and time.time() < end and (
            n.rgb is None or n.bbox is None):
        rclpy.spin_once(n, timeout_sec=0.5)

    if n.bbox is None:
        print('NO DETECTION within %.0fs' % WAIT_S)
        sys.exit(1)

    m = n.rgb
    arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
    rgb = arr[:, :m.width * 3].copy().reshape(m.height, m.width, 3)
    u1, v1, u2, v2 = [int(round(v)) for v in n.bbox[:4]]
    u1, u2 = max(0, min(u1, m.width - 1)), max(0, min(u2, m.width - 1))
    v1, v2 = max(0, min(v1, m.height - 1)), max(0, min(v2, m.height - 1))
    for t in range(4):  # 4px border
        rgb[max(0, v1 - t), u1:u2] = (57, 255, 20)
        rgb[min(m.height - 1, v2 + t), u1:u2] = (57, 255, 20)
        rgb[v1:v2, max(0, u1 - t)] = (57, 255, 20)
        rgb[v1:v2, min(m.width - 1, u2 + t)] = (57, 255, 20)

    try:
        import cv2
        cv2.imwrite(OUT, rgb[:, :, ::-1])
    except ImportError:
        from PIL import Image as PImage
        PImage.fromarray(rgb).save(OUT)
    print(f'DETECTION bbox=({u1},{v1})-({u2},{v2}) SAVED {OUT}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
