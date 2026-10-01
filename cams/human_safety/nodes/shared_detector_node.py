#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Shared (batched) person-detector node — the efficient alternative to one
``detector_node.py`` per camera.

ONE detector instance serves every launched camera. On each scheduling tick the
node asks the pure :class:`~human_safety.scheduler.FrameScheduler` which cameras
actually need a full detection (see that module for the safety invariants), then
runs those frames through the detector in a **single batched forward pass** and
lifts each person with the shared :func:`~human_safety.lifting.lift_detection`.

It keeps the per-camera ROS shape the rest of the graph relies on — it still
publishes ``/human_safety/detections/<role>`` and ``/human_safety/status/<role>``
for every camera — so ``fusion_node.py`` is unchanged. Per-camera freshness is
driven by each camera's last FULL detection, so a camera the scheduler stops
detecting ages out to stale and the existing health rules degrade the safety
state toward SLOW then STOP (fail toward STOP); skipping never fabricates a fresh
empty observation.

Near-robot priority is read advisorily from the fusion node's published
``/human_safety/state`` (zone / nearest distance): when a person is near the
robot every camera runs at the busy rate. If that state is missing the node
simply falls back to the motion trigger and the hard T_max bound, both of which
are self-contained here — the priority can only ever make it detect MORE.

Runs in the human-safety venv (torch); decodes images without cv_bridge.
"""
import argparse
import json
import pathlib
import sys
import time

import numpy as np

_PKG_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
import tf2_ros  # noqa: E402

from human_safety import geometry as geo  # noqa: E402
from human_safety.config import load_config  # noqa: E402
from human_safety.detector import build_detector  # noqa: E402
from human_safety.frameselect import change_score, downscale_gray  # noqa: E402
from human_safety.imgcodec import (depth_to_metres, image_to_ndarray,  # noqa: E402
                                   intrinsics_from_caminfo)
from human_safety.json_util import frame_to_json  # noqa: E402
from human_safety.lifting import lift_detection  # noqa: E402
from human_safety.scheduler import CameraTick, FrameScheduler  # noqa: E402


class _Cam:
    """Per-camera mutable state held by the shared node."""

    def __init__(self, cfg_cam):
        self.cfg = cfg_cam
        self.role = cfg_cam.role
        self.color_info = None
        self.depth_info = None
        self.frame = None            # (stamp, rgb)  latest colour frame
        self.depth = None            # (stamp, metres)
        self.last_tick_stamp = None  # colour stamp last offered to the scheduler
        self.last_full_detect = None  # wall time of last FULL detection
        self.last_image_stamp = None  # colour stamp of last FULL detection
        self.prev_thumb = None       # thumbnail at the last FULL detection
        self.positioned_recent = False
        self.proc_times = []


class SharedDetectorNode(Node):
    def __init__(self, cfg_path, roles, device_override=None):
        super().__init__("human_safety_shared_detector")
        self.cfg = load_config(cfg_path)
        self.fp = self.cfg.frame_processing
        self.base_frame = self.cfg.base_frame
        self.sched = FrameScheduler(self.fp.scheduler)

        cfg_roles = [c.role for c in self.cfg.cameras]
        self.roles = [r for r in roles if r in cfg_roles] if roles else cfg_roles
        self.cams = {r: _Cam(self.cfg.camera(r)) for r in self.roles}

        device = device_override or self.cfg.device
        self.get_logger().info(
            f"loading shared detector '{self.cfg.detector}' on {device} "
            f"(fp16={self.fp.fp16}) for {self.roles} ...")
        self.detector = build_detector(
            self.cfg.detector, device=device, score_thresh=self.cfg.score_thresh,
            fp16=self.fp.fp16)
        self.detector.warmup()
        self.get_logger().info(f"shared detector ready ({self.detector.license})")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_det = {}
        self.pub_status = {}
        for r in self.roles:
            self.pub_det[r] = self.create_publisher(
                String, f"/human_safety/detections/{r}", 10)
            self.pub_status[r] = self.create_publisher(
                String, f"/human_safety/status/{r}", 10)
            self._subscribe(self.cams[r])

        # Advisory near-robot signal from the fusion node's state.
        self._near_ring = False
        self.create_subscription(
            String, "/human_safety/state", self._on_state, 10)

        self.create_timer(1.0 / max(1e-3, self.fp.schedule_rate), self._tick)
        self.create_timer(0.5, self._publish_status)

    # -- subscriptions ------------------------------------------------------ #
    def _subscribe(self, cam):
        c = cam.cfg
        self.create_subscription(CameraInfo, c.info_topic,
                                 lambda m, k=cam: setattr(k, "color_info", m),
                                 qos_profile_sensor_data)
        self.create_subscription(
            Image, c.color_topic,
            lambda m, k=cam: self._on_color(k, m), qos_profile_sensor_data)
        if c.has_depth and c.depth_topic:
            self.create_subscription(
                Image, c.depth_topic,
                lambda m, k=cam: self._on_depth(k, m), qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, c.depth_info_topic,
                lambda m, k=cam: setattr(k, "depth_info", m),
                qos_profile_sensor_data)

    def _on_color(self, cam, msg):
        try:
            rgb = image_to_ndarray(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"[{cam.role}] color decode failed: {e}")
            return
        cam.frame = (_stamp_sec(msg.header.stamp), rgb)

    def _on_depth(self, cam, msg):
        try:
            metres = depth_to_metres(image_to_ndarray(msg), msg.encoding)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"[{cam.role}] depth decode failed: {e}")
            return
        cam.depth = (_stamp_sec(msg.header.stamp), metres)

    def _on_state(self, msg):
        try:
            x = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        zone = x.get("zone")
        near = zone in ("SLOW", "STOP")
        d = x.get("nearest_distance")
        if d is not None:
            margin = self.fp.scheduler.near_margin
            near = near or (d <= self.cfg.safety.slow_radius + margin)
        self._near_ring = bool(near)

    # -- scheduling tick ---------------------------------------------------- #
    def _tick(self):
        now = time.time()
        ticks, thumbs = [], {}
        for r in self.roles:
            cam = self.cams[r]
            have_new = (cam.frame is not None
                        and cam.frame[0] != cam.last_tick_stamp)
            score = None
            if have_new:
                thumb = downscale_gray(cam.frame[1], self.fp.scheduler.downscale)
                thumbs[r] = thumb
                score = change_score(cam.prev_thumb, thumb)
                cam.last_tick_stamp = cam.frame[0]
            ticks.append(CameraTick(
                role=r, have_new_frame=have_new, change_score=score,
                last_full_detect_time=cam.last_full_detect))

        decisions = self.sched.decide(now, ticks, near_ring=self._near_ring)
        to_detect = [d.role for d in decisions if d.detect]
        # Cap the batch; the leftover cameras simply wait for the next tick (the
        # T_max bound still guarantees each gets looked at in time).
        to_detect = to_detect[: self.fp.scheduler.max_batch]
        if not to_detect:
            return

        rgbs = [self.cams[r].frame[1] for r in to_detect]
        t0 = time.time()
        batch = self.detector.detect_batch(rgbs)
        dt = time.time() - t0
        for r, dets2d in zip(to_detect, batch):
            cam = self.cams[r]
            stamp = cam.frame[0]
            T = self._lookup_pose(cam, stamp)
            Kc = (intrinsics_from_caminfo(cam.color_info)
                  if cam.color_info is not None else None)
            if Kc is None:
                continue                     # no intrinsics yet; skip this frame
            dets3d = [self._lift(cam, d2d, T, Kc, stamp) for d2d in dets2d]
            cam.positioned_recent = any(d.positioned for d in dets3d)
            cam.last_full_detect = now
            cam.last_image_stamp = stamp
            cam.prev_thumb = thumbs.get(r, cam.prev_thumb)
            cam.proc_times.append(dt / len(to_detect))
            cam.proc_times = cam.proc_times[-30:]
            self.pub_det[r].publish(
                String(data=frame_to_json(r, stamp, dets3d, dets2d)))

    # -- lifting ------------------------------------------------------------ #
    def _lookup_pose(self, cam, stamp):
        if cam.cfg.presence_only:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, cam.cfg.optical_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return geo.pose_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def _lift(self, cam, d2d, T, Kc, stamp):
        dmetres = cam.depth[1] if cam.depth is not None else None
        Kd = (intrinsics_from_caminfo(cam.depth_info)
              if cam.depth_info is not None else None)
        return lift_detection(
            d2d, T, Kc, camera=cam.role, stamp=stamp,
            has_depth=cam.cfg.has_depth, depth_metres=dmetres, Kd=Kd,
            min_range=cam.cfg.min_range, max_range=cam.cfg.max_range,
            ground_z=self.cfg.ground_z, height_prior=self.cfg.height_prior)

    # -- status (per camera, keeps the per-camera health shape) ------------- #
    def _publish_status(self):
        now = time.time()
        gpu_mb = getattr(self.detector, "gpu_mem_mb", lambda: 0.0)()
        for r in self.roles:
            cam = self.cams[r]
            age = (None if cam.last_full_detect is None
                   else now - cam.last_full_detect)
            rate = 0.0
            if cam.proc_times:
                rate = 1.0 / max(1e-6, float(np.mean(cam.proc_times)))
            self.pub_status[r].publish(String(data=json.dumps({
                "camera": r,
                "detector_alive": True,
                "last_image_stamp": cam.last_image_stamp,
                "last_image_age": age,         # wall seconds since last FULL detection
                "infer_hz": round(rate, 2),
                "positioned": cam.positioned_recent,
                "gpu_mem_mb": round(gpu_mb, 1),
                "shared": True,
            })))


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--cams", default=None,
                    help="comma list of camera roles to serve")
    ap.add_argument("--device", default=None)
    args, ros_args = ap.parse_known_args(argv)
    roles = ([c.strip() for c in args.cams.split(",") if c.strip()]
             if args.cams else None)
    rclpy.init(args=ros_args)
    node = SharedDetectorNode(args.config, roles, args.device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
