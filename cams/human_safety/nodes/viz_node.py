#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Owner-facing viewer: one OpenCV window with every camera feed (boxes,
keypoints, track/zone-coloured overlays, distances) plus a top-down map of the
cell (robot, reach envelope, SLOW/STOP rings, tracks, current state banner).

Runs on the SYSTEM python (cv2 + rclpy, no torch). Reads ``$DISPLAY`` from the
environment (do NOT assume :0 — the owner works over SSH X-forwarding, e.g.
DISPLAY=localhost:10.0). Redraw is capped (viz_rate) to stay light over X11.

This is a monitor only; it publishes nothing and touches no robot.
"""
import argparse
import json
import math
import os
import pathlib
import sys
import time

import numpy as np

_PKG_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import cv2  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from human_safety.config import load_config  # noqa: E402
from human_safety.imgcodec import image_to_ndarray  # noqa: E402

ZONE_BGR = {"CLEAR": (80, 200, 80), "SLOW": (0, 200, 255), "STOP": (0, 0, 255)}
COCO_SKELETON = [(5, 7), (7, 9), (6, 8), (8, 10), (11, 13), (13, 15),
                 (12, 14), (14, 16), (5, 6), (11, 12), (5, 11), (6, 12)]


class VizNode(Node):
    def __init__(self, cfg_path, headless=False):
        super().__init__("human_safety_viz")
        self.cfg = load_config(cfg_path)
        self.headless = headless
        self.roles = [c.role for c in self.cfg.cameras]
        self._img = {}       # role -> (wall, rgb)
        self._dets = {}      # role -> (wall, list[dict])
        self._state = None
        self._tracks = []
        self.tile_h = 360
        self.map_size = 480
        self.map_range = 4.0   # metres shown from base outward

        for c in self.cfg.cameras:
            self.create_subscription(Image, c.color_topic,
                                     self._img_cb(c.role), qos_profile_sensor_data)
            self.create_subscription(String, f"/human_safety/detections/{c.role}",
                                     self._det_cb(c.role), 10)
        self.create_subscription(String, "/human_safety/state", self._state_cb, 10)
        self.create_subscription(String, "/human_safety/tracks", self._tracks_cb, 10)

        if not headless:
            disp = os.environ.get("DISPLAY", "(unset)")
            self.get_logger().info(f"viewer on DISPLAY={disp}")
            cv2.namedWindow("human_safety", cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / max(1e-3, self.cfg.viz_rate), self._redraw)
        self._last_saved = 0.0
        self.save_path = None

    # -- callbacks ---------------------------------------------------------- #
    def _img_cb(self, role):
        def cb(msg):
            try:
                self._img[role] = (time.time(), image_to_ndarray(msg))
            except Exception:  # noqa: BLE001
                pass
        return cb

    def _det_cb(self, role):
        def cb(msg):
            try:
                self._dets[role] = (time.time(), json.loads(msg.data)["detections"])
            except Exception:  # noqa: BLE001
                pass
        return cb

    def _state_cb(self, msg):
        try:
            self._state = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            pass

    def _tracks_cb(self, msg):
        try:
            self._tracks = json.loads(msg.data).get("tracks", [])
        except Exception:  # noqa: BLE001
            pass

    # -- rendering ---------------------------------------------------------- #
    def _redraw(self):
        tiles = [self._render_tile(r) for r in self.roles]
        montage = np.hstack(tiles) if tiles else np.zeros(
            (self.tile_h, 640, 3), np.uint8)
        top = self._render_map()
        # match heights
        h = max(montage.shape[0], top.shape[0])
        montage = _pad_to_h(montage, h)
        top = _pad_to_h(top, h)
        frame = np.hstack([montage, top])
        frame = self._banner(frame)
        if self.save_path and time.time() - self._last_saved > 1.0:
            cv2.imwrite(self.save_path, frame)
            self._last_saved = time.time()
        if not self.headless:
            cv2.imshow("human_safety", frame)
            cv2.waitKey(1)

    def _render_tile(self, role):
        now = time.time()
        entry = self._img.get(role)
        if entry is None:
            tile = np.zeros((self.tile_h, 480, 3), np.uint8)
            cv2.putText(tile, f"{role}: no feed", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return tile
        wall, rgb = entry
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        h0, w0 = bgr.shape[:2]
        scale = self.tile_h / h0
        bgr = cv2.resize(bgr, (int(w0 * scale), self.tile_h))
        age = now - wall
        dets = self._dets.get(role, (0, []))[1]
        reach = self.cfg.safety.reach_radius
        for d in dets:
            self._draw_det(bgr, d, scale, reach)
        live = (0, 255, 0) if age < 0.5 else (0, 0, 255)
        cv2.circle(bgr, (14, 14), 7, live, -1)
        cv2.putText(bgr, f"{role}  age {age:.2f}s  {len(dets)} ppl", (28, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        return bgr

    def _draw_det(self, bgr, d, scale, reach):
        bbox = d.get("bbox")
        method = d.get("method", "presence")
        col = {"depth": (0, 255, 0), "monocular": (0, 200, 255),
               "presence": (200, 120, 255)}.get(method, (200, 200, 200))
        if bbox:
            x0, y0, x1, y1 = [int(v * scale) for v in bbox]
            cv2.rectangle(bgr, (x0, y0), (x1, y1), col, 2)
            label = method
            pos = d.get("position")
            if pos is not None:
                dist = max(0.0, math.hypot(pos[0], pos[1]) - reach)
                label = f"{method} {dist:.2f}m"
            cv2.putText(bgr, label, (x0, max(12, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        kp = d.get("kp")
        if kp:
            pts = [(int(x * scale), int(y * scale), c) for x, y, c in kp]
            for a, b in COCO_SKELETON:
                if a < len(pts) and b < len(pts) and pts[a][2] > 0.3 and pts[b][2] > 0.3:
                    cv2.line(bgr, pts[a][:2], pts[b][:2], (255, 200, 0), 1)
            for x, y, c in pts:
                if c > 0.3:
                    cv2.circle(bgr, (x, y), 2, (255, 255, 0), -1)

    def _render_map(self):
        s = self.map_size
        img = np.full((s, s, 3), 30, np.uint8)
        cx = cy = s // 2
        ppm = (s / 2 - 20) / self.map_range   # pixels per metre

        def to_px(x, y):
            # base +x (forward) -> up, +y (left) -> left
            return int(cx - y * ppm), int(cy - x * ppm)

        # zone rings
        p = self.cfg.safety
        for radius, col in [(p.reach_radius, (90, 90, 90)),
                            (p.reach_radius + p.slow_radius, (0, 200, 255)),
                            (p.reach_radius + p.stop_radius, (0, 0, 255))]:
            cv2.circle(img, (cx, cy), int(radius * ppm), col, 1)
        # robot base
        cv2.circle(img, (cx, cy), 6, (255, 255, 255), -1)
        cv2.putText(img, "UR10", (cx + 8, cy + 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255, 255, 255), 1)
        # tracks
        for t in self._tracks:
            pos = t["position"]
            px, py = to_px(pos[0], pos[1])
            conf = t.get("confirmed", False)
            col = (255, 120, 60) if conf else (150, 90, 60)
            cv2.circle(img, (px, py), 8, col, -1 if conf else 1)
            cv2.putText(img, f"id{t['id']}", (px + 8, py),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            v = t.get("velocity")
            if v:
                cv2.arrowedLine(img, (px, py),
                                to_px(pos[0] + v[0] * 0.5, pos[1] + v[1] * 0.5),
                                (255, 255, 255), 1, tipLength=0.3)
        cv2.putText(img, "top-down (base_link)", (8, s - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        return img

    def _banner(self, frame):
        st = self._state
        h = 46
        bar = np.full((h, frame.shape[1], 3), 40, np.uint8)
        if st:
            zone = st.get("zone", "?")
            col = ZONE_BGR.get(zone, (200, 200, 200))
            dist = st.get("nearest_distance")
            dtxt = "none" if dist is None else f"{dist:.2f} m"
            lat = ""
            if st.get("evidence_stamp"):
                lat = f"  latency {time.time() - st['evidence_stamp']:.2f}s"
            hb = st.get("heartbeat_seq")
            if hb is not None:
                lat += f"  hb{hb}"
            txt = f"{zone}   nearest {dtxt}{lat}"
            cv2.rectangle(bar, (0, 0), (14, h), col, -1)
            cv2.putText(bar, txt, (24, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
            reasons = st.get("reasons", [])
            if reasons:
                cv2.putText(bar, reasons[0][:80], (24, 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        else:
            cv2.putText(bar, "waiting for /human_safety/state ...", (24, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return np.vstack([bar, frame])


def _pad_to_h(img, h):
    if img.shape[0] >= h:
        return img
    pad = np.zeros((h - img.shape[0], img.shape[1], 3), np.uint8)
    return np.vstack([img, pad])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--headless", action="store_true",
                    help="do not open a window; use with --save to write frames")
    ap.add_argument("--save", default=None, help="periodically write the view here")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = VizNode(args.config, headless=args.headless)
    node.save_path = args.save
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not args.headless:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
