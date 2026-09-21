#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Record the running human_safety graph against the Isaac scene.

Subscribes the REAL graph's published outputs — ``/human_safety/state``,
``/human_safety/tracks``, ``/human_safety/detections/<role>`` — plus the
simulated colour images, and writes:

  * a combined JSONL log (one line per state / detection message), and
  * a handful of annotated colour frames (the detector's boxes + keypoints drawn
    on the simulated image) for the report.

It NEVER touches the robot and changes nothing in the decision logic. Runs in the
human-safety venv (rclpy via --system-site-packages, plus numpy + Pillow +
human_safety.imgcodec for the dependency-free image decode).
"""
import argparse
import json
import os
import pathlib
import sys
import time

import numpy as np

_HS = pathlib.Path(__file__).resolve().parents[2] / "cams" / "human_safety"
if str(_HS) not in sys.path:
    sys.path.insert(0, str(_HS))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from human_safety.config import load_config  # noqa: E402
from human_safety.imgcodec import image_to_ndarray  # noqa: E402

# COCO keypoint skeleton (pairs of indices) for a readable overlay.
_SKELETON = [(5, 7), (7, 9), (6, 8), (8, 10), (11, 13), (13, 15), (12, 14),
             (14, 16), (5, 6), (11, 12), (5, 11), (6, 12), (0, 5), (0, 6)]


class Recorder(Node):
    def __init__(self, cfg_path, roles, out_path, frames_dir, frame_period):
        super().__init__("isaac_human_recorder")
        self.cfg = load_config(cfg_path)
        self.roles = roles
        self.out = open(out_path, "w")
        self.frames_dir = frames_dir
        os.makedirs(frames_dir, exist_ok=True)
        self.frame_period = frame_period
        self._latest_img = {}       # role -> (recv, ndarray)
        self._latest_dets = {}      # role -> (recv, [det dicts w/ bbox+kp])
        self._last_saved = {}       # role -> wall time
        self._saved_count = 0
        self._last_zone = None
        self.n_state = 0
        self.n_det = 0

        self.create_subscription(String, "/human_safety/state",
                                 self._on_state, 10)
        for role in roles:
            cam = self.cfg.camera(role)
            self.create_subscription(
                String, f"/human_safety/detections/{role}",
                self._make_det_cb(role), 10)
            self.create_subscription(
                Image, cam.color_topic, self._make_img_cb(role),
                qos_profile_sensor_data)
        self.create_timer(0.5, self._maybe_save_frames)
        self.get_logger().info(
            f"recorder up; roles={roles}; log={out_path}; frames={frames_dir}")

    # -- subscriptions ---------------------------------------------------- #
    def _on_state(self, msg):
        try:
            x = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        rec = {
            "kind": "state",
            "recv": time.time(),
            "t": x.get("stamp"),
            "evidence_stamp": x.get("evidence_stamp"),
            "heartbeat_seq": x.get("heartbeat_seq"),
            "zone": x.get("zone"),
            "distance_status": x.get("distance_status"),
            "nearest_distance": x.get("nearest_distance"),
            "nearest_distance_base": x.get("nearest_distance_base"),
            # DIAGNOSTIC body-centre distances (what the zone used before the
            # nearest-point fix), so an offline old-vs-new before/after needs no
            # second sim run (see compare_fix.py).
            "nearest_distance_centre": x.get("nearest_distance_centre"),
            "nearest_distance_base_centre": x.get("nearest_distance_base_centre"),
            "nearest_track_id": x.get("nearest_track_id"),
            "any_person_visible": x.get("any_person_visible"),
            # Per-track position AND nearest credible point, plus the per-camera
            # health block, so the safety state machine can be re-run offline on
            # the exact live inputs with old (centre) vs new (nearest) logic.
            "tracks": [{"id": t.get("id"), "position": t.get("position"),
                        "nearest_point": t.get("nearest_point"),
                        "confirmed": t.get("confirmed", True)}
                       for t in x.get("tracks", [])],
            "cameras": x.get("cameras", []),
            "reasons": x.get("reasons", []),
        }
        self.out.write(json.dumps(rec) + "\n")
        self.out.flush()
        self.n_state += 1
        self._last_zone = x.get("zone")

    def _make_det_cb(self, role):
        def cb(msg):
            try:
                x = json.loads(msg.data)
            except Exception:  # noqa: BLE001
                return
            dets = x.get("detections", [])
            positioned = any(d.get("position") is not None for d in dets)
            rec = {
                "kind": "det", "camera": role, "recv": time.time(),
                "t": x.get("stamp"), "n_persons": len(dets),
                "positioned": positioned,
                "scores": [d.get("score") for d in dets],
                # per-camera 3-D positions (base_link) + nearest points, so the
                # analysis can split accuracy by camera and compare the median
                # body-centre against the nearest credible point.
                "positions": [d.get("position") for d in dets],
                "nearest_points": [d.get("nearest_point") for d in dets],
                "methods": [d.get("method") for d in dets],
            }
            self.out.write(json.dumps(rec) + "\n")
            self.out.flush()
            self.n_det += 1
            self._latest_dets[role] = (time.time(), dets)
        return cb

    def _make_img_cb(self, role):
        def cb(msg):
            try:
                arr = image_to_ndarray(msg)
            except Exception:  # noqa: BLE001
                return
            self._latest_img[role] = (time.time(), arr)
        return cb

    # -- annotated frame saving ------------------------------------------ #
    def _maybe_save_frames(self):
        now = time.time()
        force = self._last_zone is not None and self._last_zone == "STOP"
        for role in self.roles:
            img = self._latest_img.get(role)
            if img is None:
                continue
            last = self._last_saved.get(role, 0.0)
            due = (now - last) >= self.frame_period
            if not due and not (force and (now - last) >= 1.0):
                continue
            dets = self._latest_dets.get(role)
            self._save_annotated(role, img[1],
                                 dets[1] if dets else [], now)
            self._last_saved[role] = now

    def _save_annotated(self, role, rgb, dets, now):
        try:
            from PIL import Image as PImage, ImageDraw
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"PIL unavailable, cannot annotate: {e}")
            return
        im = PImage.fromarray(np.ascontiguousarray(rgb.astype(np.uint8)))
        d = ImageDraw.Draw(im)
        for det in dets:
            bbox = det.get("bbox")
            if bbox:
                d.rectangle([bbox[0], bbox[1], bbox[2], bbox[3]],
                            outline=(0, 255, 0), width=3)
                sc = det.get("score")
                zlabel = f"person {sc:.2f}" if sc is not None else "person"
                d.text((bbox[0] + 2, bbox[1] + 2), zlabel, fill=(0, 255, 0))
            kp = det.get("kp")
            if kp:
                for (x, y, *_r) in kp:
                    d.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(255, 80, 80))
                for (i, j) in _SKELETON:
                    if i < len(kp) and j < len(kp):
                        d.line([kp[i][0], kp[i][1], kp[j][0], kp[j][1]],
                               fill=(80, 160, 255), width=2)
        banner = f"{role}  zone={self._last_zone}  persons={len(dets)}"
        d.rectangle([0, 0, im.width, 18], fill=(0, 0, 0))
        d.text((3, 3), banner, fill=(255, 255, 255))
        fname = os.path.join(
            self.frames_dir, f"{role}_{int(now * 1000)}_{self._last_zone}.png")
        im.save(fname)
        self._saved_count += 1


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--cams", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--seconds", type=float, default=200.0)
    ap.add_argument("--frame-period", type=float, default=4.0)
    args, ros_args = ap.parse_known_args(argv)
    roles = [r.strip() for r in args.cams.split(",") if r.strip()]

    rclpy.init(args=ros_args)
    node = Recorder(args.config, roles, args.out, args.frames_dir, args.frame_period)
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"recorder done: {node.n_state} state msgs, {node.n_det} detection "
            f"msgs, {node._saved_count} frames saved")
        node.out.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
