#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Per-camera person detector + 3D lifting node.

One instance per camera role. Subscribes the colour image (+ CameraInfo) and,
when the camera has depth, the aligned depth image (+ CameraInfo). Runs the
pluggable detector, lifts each person to the robot base frame, and publishes:

  /human_safety/detections/<role>   std_msgs/String  (JSON, per processed frame)
  /human_safety/status/<role>       std_msgs/String  (JSON, on a timer)

Lifting policy per person, per frame:
  * base<-optical TF available AND depth valid  -> DEPTH   (centre + nearest pt)
  * base<-optical TF available, no/failed depth -> MONOCULAR (ground/height prior)
  * base<-optical TF NOT available              -> PRESENCE only (flagged)

The TF requirement is what makes the wrist (needs live robot joint states, driver
is down) and an uncalibrated D415 (no static base<-optical TF) contribute
presence only, exactly as specified — with no camera-specific code.

Runs in the human-safety venv (torch). Uses a dependency-free image codec, not
cv_bridge, to stay clear of the venv/system NumPy-ABI boundary.
"""
import argparse
import pathlib
import sys
import time

import numpy as np

# Make the package importable without an install / colcon build.
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
from human_safety.imgcodec import (depth_to_metres, image_to_ndarray,  # noqa: E402
                                   intrinsics_from_caminfo)
from human_safety.json_util import frame_to_json  # noqa: E402
from human_safety.lifting import lift_detection  # noqa: E402


class DetectorNode(Node):
    def __init__(self, cfg_path, role, device_override=None):
        super().__init__(f"human_safety_detector_{role}")
        self.cfg = load_config(cfg_path)
        self.cam = self.cfg.camera(role)
        self.role = role
        self.base_frame = self.cfg.base_frame

        device = device_override or self.cfg.device
        self.get_logger().info(
            f"[{role}] loading detector '{self.cfg.detector}' on {device} ...")
        self.detector = build_detector(
            self.cfg.detector, device=device, score_thresh=self.cfg.score_thresh)
        self.detector.warmup()
        self.get_logger().info(
            f"[{role}] detector ready ({self.detector.license})")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._color_info = None
        self._depth = None          # (stamp_sec, metres ndarray)
        self._depth_info = None
        self._last_proc = 0.0
        self._min_period = 1.0 / max(1e-3, self.cam.max_rate)
        self._last_image_stamp = None
        self._last_image_wall = None
        self._proc_times = []
        self._positioned_recent = False

        self.pub_det = self.create_publisher(
            String, f"/human_safety/detections/{role}", 10)
        self.pub_status = self.create_publisher(
            String, f"/human_safety/status/{role}", 10)

        self.create_subscription(CameraInfo, self.cam.info_topic,
                                 self._on_color_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.cam.color_topic,
                                 self._on_color, qos_profile_sensor_data)
        if self.cam.has_depth and self.cam.depth_topic:
            self.create_subscription(Image, self.cam.depth_topic,
                                     self._on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, self.cam.depth_info_topic,
                                     self._on_depth_info, qos_profile_sensor_data)
        self.create_timer(0.5, self._publish_status)

    # -- subscriptions ------------------------------------------------------ #
    def _on_color_info(self, msg):
        self._color_info = msg

    def _on_depth_info(self, msg):
        self._depth_info = msg

    def _on_depth(self, msg):
        try:
            arr = image_to_ndarray(msg)
            metres = depth_to_metres(arr, msg.encoding)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"[{self.role}] depth decode failed: {e}")
            return
        self._depth = (_stamp_sec(msg.header.stamp), metres)

    def _on_color(self, msg):
        now = time.time()
        if now - self._last_proc < self._min_period:
            return   # rate cap; one slow/dead camera never gates the others
        self._last_proc = now
        if self._color_info is None:
            return
        try:
            rgb = image_to_ndarray(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"[{self.role}] color decode failed: {e}")
            return
        stamp = _stamp_sec(msg.header.stamp)
        self._last_image_stamp = stamp
        self._last_image_wall = now

        t0 = time.time()
        dets2d = self.detector.detect(rgb)
        T = self._lookup_pose(msg.header.stamp)
        dets3d = [self._lift(d, T) for d in dets2d]
        self._proc_times.append(time.time() - t0)
        self._proc_times = self._proc_times[-30:]
        self._positioned_recent = any(d.positioned for d in dets3d)

        self.pub_det.publish(
            String(data=frame_to_json(self.role, stamp, dets3d, dets2d)))

    # -- lifting ------------------------------------------------------------ #
    def _lookup_pose(self, stamp):
        """base_frame <- optical_frame at the image stamp, or None."""
        if self.cam.presence_only:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.cam.optical_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return geo.pose_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def _lift(self, d2d, T):
        stamp = self._last_image_stamp or 0.0
        Kc = intrinsics_from_caminfo(self._color_info)
        dmetres = self._depth[1] if self._depth is not None else None
        Kd = (intrinsics_from_caminfo(self._depth_info)
              if self._depth_info is not None else None)
        return lift_detection(
            d2d, T, Kc, camera=self.role, stamp=stamp,
            has_depth=self.cam.has_depth, depth_metres=dmetres, Kd=Kd,
            min_range=self.cam.min_range, max_range=self.cam.max_range,
            ground_z=self.cfg.ground_z, height_prior=self.cfg.height_prior)

    # -- status ------------------------------------------------------------- #
    def _publish_status(self):
        import json
        now = time.time()
        image_age = (None if self._last_image_wall is None
                     else now - self._last_image_wall)
        rate = 0.0
        if len(self._proc_times) >= 2 and self._last_image_wall:
            rate = 1.0 / max(1e-6, np.mean(self._proc_times))
        gpu_mb = getattr(self.detector, "gpu_mem_mb", lambda: 0.0)()
        self.pub_status.publish(String(data=json.dumps({
            "camera": self.role,
            "detector_alive": True,
            "last_image_stamp": self._last_image_stamp,
            "last_image_age": image_age,           # wall seconds since last frame
            "infer_hz": round(rate, 2),
            "positioned": self._positioned_recent,
            "gpu_mem_mb": round(gpu_mb, 1),
        })))


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--role", required=True)
    ap.add_argument("--device", default=None)
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = DetectorNode(args.config, args.role, args.device)
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
