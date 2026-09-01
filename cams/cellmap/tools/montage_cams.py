#!/usr/bin/env python3
"""
Show several camera colour feeds side by side in ONE OpenCV window.

Why not RViz Image displays: on this headless-ish display the image panels
dock behind the Displays panel and silently show nothing — the exact
"No Image" problem hit repeatedly during this project. A single cv2 window
is always on top of its own content and cannot hide a feed behind a tab.

Every tile is labelled and shows its live rate. A camera that is not
publishing gets a visible NO SIGNAL tile rather than being silently dropped,
so a missing feed is obvious instead of looking like a smaller montage.

  python3 montage_cams.py --topics /camera/rgb:wrist_D435i \
      /cam415a/camera/color/image_raw:D415_024122061936 \
      /cam415b/camera/color/image_raw:D415_021422061772
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TILE_W, TILE_H = 640, 480


class Feed:
    def __init__(self, node, topic, label):
        self.topic, self.label = topic, label
        self.img = None
        self.stamps = []
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        node.create_subscription(Image, topic, self._cb, q)

    def _cb(self, m):
        if m.encoding not in ('rgb8', 'bgr8'):
            return
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        self.img = a[:, :, ::-1] if m.encoding == 'rgb8' else a
        t = time.time()
        self.stamps.append(t)
        self.stamps = [s for s in self.stamps if t - s < 2.0]

    def hz(self):
        return len(self.stamps) / 2.0 if len(self.stamps) > 1 else 0.0

    def tile(self):
        live = self.img is not None and self.hz() > 0.5
        if live:
            t = cv2.resize(self.img, (TILE_W, TILE_H))
        else:
            t = np.full((TILE_H, TILE_W, 3), 40, np.uint8)
            cv2.putText(t, 'NO SIGNAL', (TILE_W // 2 - 150, TILE_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (60, 60, 220), 3)
        t = t.copy()
        cv2.rectangle(t, (0, 0), (TILE_W - 1, 34), (0, 0, 0), -1)
        col = (120, 240, 120) if live else (80, 80, 230)
        cv2.putText(t, f'{self.label}   {self.hz():4.1f} Hz', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2)
        cv2.rectangle(t, (0, 0), (TILE_W - 1, TILE_H - 1), (90, 90, 90), 2)
        return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topics', nargs='+', required=True, help='topic:label ...')
    ap.add_argument('--title', default='cell cameras')
    ap.add_argument('--cols', type=int, default=0, help='0 = auto')
    a = ap.parse_args()
    rclpy.init()
    n = Node('montage_cams')
    feeds = []
    for spec in a.topics:
        topic, _, label = spec.partition(':')
        feeds.append(Feed(n, topic, label or topic))
    cols = a.cols or (3 if len(feeds) >= 3 else len(feeds))
    rows = (len(feeds) + cols - 1) // cols
    cv2.namedWindow(a.title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(a.title, min(TILE_W * cols, 1900), TILE_H * rows + 40)
    print(f'montage: {len(feeds)} feeds, {rows}x{cols}. Close the window or Ctrl-C to stop.',
          flush=True)
    try:
        while rclpy.ok():
            # Drain the queues rather than taking one message per redraw,
            # otherwise the displayed Hz is the montage's own loop rate and
            # understates a healthy 30 Hz camera as ~13 Hz.
            for _ in range(12):
                rclpy.spin_once(n, timeout_sec=0.001)
            tiles = [f.tile() for f in feeds]
            while len(tiles) < rows * cols:
                tiles.append(np.full((TILE_H, TILE_W, 3), 25, np.uint8))
            grid = np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)])
            cv2.imshow(a.title, grid)
            if cv2.waitKey(30) & 0xFF in (27, ord('q')):
                break
            if cv2.getWindowProperty(a.title, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
