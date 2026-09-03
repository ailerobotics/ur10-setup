#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
One window, all three rig color feeds side by side — the captain-facing "are the
cameras live?" view.

Why this exists (2026-08-31): RViz Image displays each become a dockable panel,
and with three of them RViz tabs them behind the Displays panel on load — so the
feeds are there but hidden, and there is no headless way to un-tab them (no
window-manager tooling, no reusable RViz dock-state blob). This montage sidesteps
all of that: a single OpenCV highgui window with the three colour streams tiled,
labelled, and refreshed live. RViz stays up alongside it for 3D / robot context.

Run (domain 0, same as the rig):
  ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 DISPLAY=:0 \
    python3 cams/tools/rig_feed_montage.py
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# (role, topic, label). Order = left-to-right in the montage.
FEEDS = [
    # wrist: the camera is mounted upside down; show the upright twin (rot180).
    ('wrist', '/wrist/color_upright/image_raw', 'WRIST  D435i 018322070619 (upright)'),
    ('shelf', '/shelf/color/image_raw', 'SHELF  D415 021422061772'),
    ('table', '/table/color/image_raw', 'TABLE  D415 025522060953'),
]
TILE_W = 620           # per-feed display width; height follows the aspect ratio
WIN = 'shelfbot rig — 3 camera feeds (wrist | shelf | table)'


def img_to_bgr(msg):
    """sensor_msgs/Image (rgb8/bgr8) -> HxWx3 uint8 BGR, or None if unhandled."""
    h, w = msg.height, msg.width
    if msg.encoding not in ('rgb8', 'bgr8') or h == 0 or w == 0:
        return None
    arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(h, w, 3)
    return arr[:, :, ::-1].copy() if msg.encoding == 'rgb8' else arr.copy()


class Montage(Node):
    def __init__(self):
        super().__init__('rig_feed_montage')
        self.frames = {role: None for role, _, _ in FEEDS}
        self.counts = {role: 0 for role, _, _ in FEEDS}
        for role, topic, _ in FEEDS:
            self.create_subscription(
                Image, topic,
                lambda m, r=role: self._on(r, m), qos_profile_sensor_data)
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, TILE_W * len(FEEDS), int(TILE_W * 9 / 16) + 34)
        cv2.moveWindow(WIN, 8, 40)          # top-left, clear of the taskbar
        self.create_timer(1.0 / 20, self._render)

    def _on(self, role, msg):
        bgr = img_to_bgr(msg)
        if bgr is not None:
            self.frames[role] = bgr
            self.counts[role] += 1

    def _tile(self, role, label):
        tile_h = int(TILE_W * 9 / 16)
        img = self.frames[role]
        if img is None:
            panel = np.full((tile_h, TILE_W, 3), 40, np.uint8)
            cv2.putText(panel, 'waiting for frames...', (20, tile_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 160, 255), 2)
        else:
            panel = cv2.resize(img, (TILE_W, tile_h))
        bar = np.full((34, TILE_W, 3), 22, np.uint8)
        live = self.counts[role] > 0
        cv2.circle(bar, (16, 17), 7, (0, 200, 0) if live else (0, 0, 200), -1)
        cv2.putText(bar, f'{label}   [{self.counts[role]} frames]', (32, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1)
        return np.vstack([bar, panel])

    def _render(self):
        montage = np.hstack([self._tile(r, lbl) for r, _, lbl in FEEDS])
        cv2.imshow(WIN, montage)
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):   # Esc / q to quit
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Montage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
