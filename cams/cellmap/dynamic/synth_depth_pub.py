#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Publish SYNTHETIC depth (+ CameraInfo + a static TF) for a fixed camera, so
nvblox's dynamic layer can be exercised with NO robot and NO live camera.

Runs INSIDE the planner-image container on the task's own ROS domain. Renders a
pinhole depth image (16UC1, millimetres — the cell's ``/camera/depth/image_mm``
convention) of a static backdrop plus obstacles that appear and disappear on a
timeline, from a fixed camera pose. The pose is a PROVISIONAL simulation
extrinsic (``extrinsics_guard.py`` mode ``sim``); it is not a survey and must not
be used to place a real obstacle relative to the arm.

Ground truth (which obstacles are present, and where) is logged to stderr and
mirrored on ``/fm_dyn/ground_truth`` (std_msgs/String JSON) so a proof can line
up ESDF changes with the moment an obstacle appeared or left.

Depth-only realism: obstacles here are generic occupied volumes. A person is just
another volume — which is exactly how the ``dynamic`` mapper treats one without a
segmentation model, and exactly the "person as a routing obstacle, never the
stop" principle (routing_gate.py).

  ros2 run ... OR: python3 synth_depth_pub.py --timeline default --rate 10
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from synth_scene import (  # noqa: E402
    Box, Intrinsics, Scene, look_at, matrix_to_quaternion_xyzw,
    render_depth_mm, render_mask)

# Obstacle timelines: label -> (min_corner, max_corner, t_appear_s, t_leave_s).
# A "box" (0.3 m cube) then a "person" (0.5 x 0.4 x 1.7 m column). Windows are
# disjoint so a proof can watch each appear and each decay out independently.
TIMELINES = {
    "default": {
        "box":    ((-0.15, 0.55, 0.0), (0.15, 0.85, 0.30), 6.0, 16.0),
        "person": ((0.35, 0.50, 0.0), (0.85, 0.90, 1.70), 22.0, 34.0),
    },
    "box_only": {
        "box": ((-0.15, 0.55, 0.0), (0.15, 0.85, 0.30), 6.0, 16.0),
    },
    "person_only": {
        "person": ((0.35, 0.50, 0.0), (0.85, 0.90, 1.70), 6.0, 20.0),
    },
}


class SynthDepthPub(Node):
    def __init__(self, args):
        super().__init__("fm_dyn_synth_depth_pub")
        self.intr = Intrinsics.d415_like(args.width, args.height)
        self.frame = args.frame
        self.rate = args.rate
        self.manual = args.manual
        self.manual_boxes = {}          # label -> (min, max) currently present
        self.timeline = {} if args.manual else TIMELINES[args.timeline]
        # Fixed camera pose: PROVISIONAL sim extrinsic (not a survey). Configurable
        # so a proof can aim the fixed view at the region it exercises.
        self.base_T_opt = look_at(eye=tuple(args.eye), target=tuple(args.target))
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub_depth = self.create_publisher(Image, args.depth_topic, qos)
        self.pub_info = self.create_publisher(CameraInfo, args.info_topic, qos)
        self.pub_gt = self.create_publisher(String, "/fm_dyn/ground_truth", 10)
        self.mask_labels = set(args.mask_labels or [])
        self.pub_mask = None
        self.pub_mask_info = None
        if args.publish_mask:
            self.pub_mask = self.create_publisher(Image, args.mask_topic, qos)
            self.pub_mask_info = self.create_publisher(CameraInfo, args.mask_info_topic, qos)
        # Runtime obstacle control (used by the route proof): add/remove a box at
        # a computed base_link location via a JSON String on /fm_dyn/set_obstacle.
        self.create_subscription(String, "/fm_dyn/set_obstacle", self._on_set_obstacle, 10)
        self.tf = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"synthetic depth on {args.depth_topic} @ {self.rate} Hz, frame "
            f"{self.frame}, timeline '{args.timeline}' — PROVISIONAL sim pose")

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = self.frame
        T = self.base_T_opt
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        qx, qy, qz, qw = matrix_to_quaternion_xyzw(T)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)

    def _on_set_obstacle(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        label = d.get("label", "box")
        if d.get("present"):
            self.manual_boxes[label] = (tuple(d["min"]), tuple(d["max"]))
            self.get_logger().info(f"set_obstacle: {label} present at {d['min']}..{d['max']}")
        else:
            self.manual_boxes.pop(label, None)
            self.get_logger().info(f"set_obstacle: {label} removed")

    def _active_boxes(self, t_s):
        boxes, present = [], {}
        if self.manual:
            for label, (mn, mx) in self.manual_boxes.items():
                present[label] = {"present": True, "min": list(mn), "max": list(mx)}
                boxes.append(Box(mn, mx, label))
            return boxes, present
        for label, (mn, mx, ta, tl) in self.timeline.items():
            on = ta <= t_s < tl
            present[label] = {"present": on, "min": mn, "max": mx}
            if on:
                boxes.append(Box(mn, mx, label))
        return boxes, present

    def _camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame
        info.width = self.intr.width
        info.height = self.intr.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx, fy, cx, cy = self.intr.fx, self.intr.fy, self.intr.cx, self.intr.cy
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _tick(self):
        now = self.get_clock().now()
        t_s = (now - self.t0).nanoseconds * 1e-9
        boxes, present = self._active_boxes(t_s)
        scene = Scene(boxes=boxes)
        depth_mm = render_depth_mm(self.intr, self.base_T_opt, scene)

        stamp = now.to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame
        img.height, img.width = depth_mm.shape
        img.encoding = "16UC1"
        img.is_bigendian = 0
        img.step = img.width * 2
        img.data = depth_mm.tobytes()
        self.pub_depth.publish(img)
        info = self._camera_info(stamp)
        self.pub_info.publish(info)

        if self.pub_mask is not None:
            mask = render_mask(self.intr, self.base_T_opt, scene, self.mask_labels)
            m = Image()
            m.header.stamp = stamp
            m.header.frame_id = self.frame
            m.height, m.width = mask.shape
            m.encoding = "mono8"
            m.is_bigendian = 0
            m.step = m.width
            m.data = mask.tobytes()
            self.pub_mask.publish(m)
            self.pub_mask_info.publish(info)

        gt = String()
        gt.data = json.dumps({"t_s": round(t_s, 3), "obstacles": present})
        self.pub_gt.publish(gt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--depth-topic", default="/fm_dyn/depth/image_mm")
    ap.add_argument("--info-topic", default="/fm_dyn/depth/camera_info")
    ap.add_argument("--frame", default="fm_dyn_sim_camera_optical_frame")
    ap.add_argument("--timeline", default="default", choices=list(TIMELINES))
    ap.add_argument("--manual", action="store_true",
                    help="ignore the timeline; obstacles are controlled at runtime "
                         "via /fm_dyn/set_obstacle (used by the route proof)")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--publish-mask", action="store_true",
                    help="also publish a synthetic segmentation mask (stands in "
                         "for a people-seg network) for human_with_static_* modes")
    ap.add_argument("--mask-topic", default="/fm_dyn/mask/image")
    ap.add_argument("--mask-info-topic", default="/fm_dyn/mask/camera_info")
    ap.add_argument("--mask-labels", nargs="*", default=["person"],
                    help="obstacle labels to mark as foreground in the mask")
    ap.add_argument("--eye", type=float, nargs=3, default=[0.0, -1.6, 1.2],
                    help="fixed camera position in base_link (provisional sim pose)")
    ap.add_argument("--target", type=float, nargs=3, default=[0.0, 0.4, 0.2],
                    help="fixed camera look-at point in base_link")
    args = ap.parse_args()
    rclpy.init()
    node = SynthDepthPub(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
