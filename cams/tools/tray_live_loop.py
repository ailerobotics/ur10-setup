#!/usr/bin/env python3
"""
Continuous bench view of the captain's trained tray model.

Subscribes /camera/rgb, runs the latest tray_perception weights (~4 Hz),
publishes annotated frames on /detect_new/annotated for RViz, and logs
detections at most every 2 s. Zero shelfbot code touched: this is a
scratch harness beside the pipeline, not part of it.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

MODEL = sys.argv[1] if len(sys.argv) > 1 else '/root/weights/tray_best_vm.pt'
IMGSZ = 960
PERIOD_S = 0.25          # ~4 Hz inference
LOG_EVERY_S = 2.0


class LiveAnnotator(Node):
    def __init__(self, model):
        super().__init__('tray_live_annotator')
        self._model = model
        self._last_run = 0.0
        self._last_log = 0.0
        self._pub = self.create_publisher(
            Image, '/detect_new/annotated', qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/rgb', self._on_rgb, qos_profile_sensor_data)

    def _on_rgb(self, m):
        now = time.time()
        if now - self._last_run < PERIOD_S:
            return
        self._last_run = now
        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
        rgb = arr[:, :m.width * 3].copy().reshape(m.height, m.width, 3)
        res = self._model.predict(rgb[:, :, ::-1], imgsz=IMGSZ, conf=0.25,
                                  verbose=False)
        r = res[0]
        if len(r.boxes) and now - self._last_log > LOG_EVERY_S:
            self._last_log = now
            dets = ', '.join(
                f'{r.names[int(b.cls)]} {float(b.conf):.2f}' for b in r.boxes)
            self.get_logger().info(f'DET {dets}')
        out = r.plot()[:, :, ::-1]          # BGR -> RGB
        msg = Image()
        msg.header = m.header
        msg.height, msg.width = out.shape[0], out.shape[1]
        msg.encoding = 'rgb8'
        msg.step = out.shape[1] * 3
        msg.data = np.ascontiguousarray(out).tobytes()
        self._pub.publish(msg)


def main():
    from ultralytics import YOLO
    model = YOLO(MODEL)
    rclpy.init()
    node = LiveAnnotator(model)
    node.get_logger().info(f'live annotator up: {MODEL} @ imgsz {IMGSZ}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
