#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fusion + safety node.

Subscribes every camera's detections and status, fuses positioned detections
into base-frame tracks, evaluates the zone / hysteresis / dwell / health state
machine, and publishes:

  /human_safety/tracks   std_msgs/String            (JSON track list)
  /human_safety/state    std_msgs/String            (JSON SafetyState)
  /human_safety/markers  visualization_msgs/MarkerArray  (RViz: tracks + zones)

Health is judged here (fail toward STOP): a camera whose status stopped arriving
is missing; a frame older than the budget is stale; a positioned camera that
stopped reporting people while one was close forces STOP (via the safety layer).
This node NEVER commands the robot — it only publishes (hard rule).
"""
import argparse
import json
import pathlib
import sys
import time

# Package importable without install.
_PKG_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402
from geometry_msgs.msg import Point  # noqa: E402

from human_safety.config import load_config  # noqa: E402
from human_safety.fusion import Fusion, gather_evidence  # noqa: E402
from human_safety.json_util import (detection_to_dict, detections_from_json,  # noqa: E402
                                    health_to_dict, state_to_json, track_to_dict)
from human_safety.messages import CameraHealth  # noqa: E402
from human_safety.safety import SafetyEvaluator  # noqa: E402
from human_safety.tracker import MultiTargetTracker  # noqa: E402


class FusionNode(Node):
    def __init__(self, cfg_path, record_path=None, cams=None):
        super().__init__("human_safety_fusion")
        self.cfg = load_config(cfg_path)
        self._record = open(record_path, "w") if record_path else None
        self.base_frame = self.cfg.base_frame
        self.fusion = Fusion(MultiTargetTracker(
            sigma_meas=0.15, n_confirm=3, max_confirmed_coast_time=2.5))
        self.evaluator = SafetyEvaluator(self.cfg.safety)
        self._heartbeat = 0

        # role -> latest (recv_wall, list[Detection3D], image_stamp)
        self._dets = {}
        # role -> latest (recv_wall, status dict)
        self._status = {}
        # Expected cameras = the ones actually being brought up (launch cams:=),
        # intersected with the config. A configured-but-not-launched camera must
        # NOT be treated as a crashed detector (permanent STOP).
        cfg_roles = [c.role for c in self.cfg.cameras]
        self._roles = ([r for r in cams if r in cfg_roles] if cams else cfg_roles)
        self._status_timeout = 2.0   # no status for this long -> detector down

        for role in self._roles:
            self.create_subscription(
                String, f"/human_safety/detections/{role}",
                self._make_det_cb(role), 10)
            self.create_subscription(
                String, f"/human_safety/status/{role}",
                self._make_status_cb(role), 10)

        self.pub_state = self.create_publisher(String, "/human_safety/state", 10)
        self.pub_tracks = self.create_publisher(String, "/human_safety/tracks", 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_safety/markers", 10)

        self.create_timer(1.0 / max(1e-3, self.cfg.fusion_rate), self._cycle)
        self.get_logger().info(
            f"fusion up; cameras={self._roles}; PROVISIONAL zones "
            f"stop={self.cfg.safety.stop_radius} slow={self.cfg.safety.slow_radius}")

    def _make_det_cb(self, role):
        def cb(msg):
            try:
                x = json.loads(msg.data)
                dets = detections_from_json(msg.data)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"[{role}] bad detections json: {e}")
                return
            self._dets[role] = (time.time(), dets, x.get("stamp"))
        return cb

    def _make_status_cb(self, role):
        def cb(msg):
            try:
                self._status[role] = (time.time(), json.loads(msg.data))
            except Exception:  # noqa: BLE001
                pass
        return cb

    def _cycle(self):
        now_wall = time.time()
        # Gather the freshest detections within one budget and derive the
        # STOP-ring evidence stamp / latency (fault 5) from the positioned ones
        # only, using the OLDEST contributing stamp so a camera streaming fresh
        # empty frames can never make older person evidence look fresh; per-camera
        # staleness is reported separately (report §7 #2).
        all_dets, evidence_stamp, latency = gather_evidence(
            self._dets, now_wall, self.cfg.safety.frame_age_budget)

        tracks, presence_cams = self.fusion.step(all_dets, now_wall)
        health = self._assess_health(now_wall)
        any_presence = bool(presence_cams)

        state = self.evaluator.evaluate(
            tracks, health, now_wall, any_presence=any_presence,
            source_stamp=evidence_stamp, latency=latency)
        self._heartbeat += 1
        state.heartbeat_seq = self._heartbeat

        self.pub_state.publish(String(data=state_to_json(state)))
        self.pub_tracks.publish(String(data=json.dumps(
            {"stamp": now_wall, "tracks": [track_to_dict(t) for t in tracks]})))
        self.pub_markers.publish(self._markers(state))

        if self._record is not None:
            # Faithful replay input: the exact detections/health/clock this cycle
            # consumed, so replay.py reproduces this state deterministically.
            self._record.write(json.dumps({
                "now": now_wall,
                "detections": [detection_to_dict(d) for d in all_dets],
                "health": [health_to_dict(h) for h in health],
                "any_presence": any_presence,
                "zone": state.zone.value,
            }) + "\n")
            self._record.flush()

    def _assess_health(self, now_wall):
        out = []
        budget = self.cfg.safety.frame_age_budget
        for role in self._roles:
            st = self._status.get(role)
            if st is None or now_wall - st[0] > self._status_timeout:
                out.append(CameraHealth(
                    camera=role, present_in_graph=False, detector_alive=False,
                    fresh=False, note="no status (node/camera down)"))
                continue
            _, sd = st
            img_age = sd.get("last_image_age")
            # Never seen a frame, or last frame older than the budget -> stale.
            fresh = img_age is not None and img_age <= budget
            out.append(CameraHealth(
                camera=role, present_in_graph=True, detector_alive=True,
                frame_age=(float("inf") if img_age is None else float(img_age)),
                positioned=bool(sd.get("positioned", False)),
                fresh=bool(fresh),
                note="" if fresh else "stale/no frame"))
        return out

    # -- RViz markers ------------------------------------------------------- #
    def _markers(self, state):
        arr = MarkerArray()
        # Zone rings (SLOW, STOP) as line strips on the floor.
        p = self.cfg.safety
        for i, (radius, rgba) in enumerate([
                (p.reach_radius + p.slow_radius, (1.0, 0.8, 0.0, 0.6)),
                (p.reach_radius + p.stop_radius, (1.0, 0.1, 0.1, 0.8))]):
            arr.markers.append(self._ring(f"zone", i, radius, rgba))
        # Tracks as spheres + id text.
        for t in state.tracks:
            arr.markers.append(self._sphere(t))
            arr.markers.append(self._text(t))
        return arr

    def _ring(self, ns, mid, radius, rgba, n=64):
        import math
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        for k in range(n + 1):
            a = 2 * math.pi * k / n
            m.points.append(Point(x=radius * math.cos(a),
                                  y=radius * math.sin(a), z=0.01))
        return m

    def _sphere(self, t):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns = "tracks"
        m.id = int(t.id)
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(t.position[0]), float(t.position[1]), float(t.position[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        conf = 1.0 if t.confirmed else 0.4
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.4, 1.0, conf
        return m

    def _text(self, t):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns = "track_ids"
        m.id = int(t.id)
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(t.position[0]), float(t.position[1]), float(t.position[2]) + 0.4
        m.pose.orientation.w = 1.0
        m.scale.z = 0.2
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f"id{t.id}"
        return m


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--record", default=None,
                    help="write a replayable JSONL log of every fusion cycle")
    ap.add_argument("--cams", default=None,
                    help="comma list of cameras actually launched; a "
                         "configured-but-not-launched camera is not expected "
                         "(so it never forces a spurious STOP)")
    args, ros_args = ap.parse_known_args(argv)
    cams = ([c.strip() for c in args.cams.split(",") if c.strip()]
            if args.cams else None)
    rclpy.init(args=ros_args)
    node = FusionNode(args.config, record_path=args.record, cams=cams)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._record is not None:
            node._record.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
