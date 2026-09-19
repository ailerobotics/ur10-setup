#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Publish base_link -> camera static TFs for the dynamic layer, honouring the
calibration guard. This is the REAL-camera / bag-replay path (the synthetic
publisher publishes its own provisional TF, so this node is not needed in sim).

It loads provisional_extrinsics.yaml via extrinsics_guard and publishes a static
TF ONLY for entries that resolve: a ``sim`` provisional pose, or a ``real`` camera
with a passing calibration file. A real camera with no valid calibration is
REFUSED (no TF), and logged — exactly how ``cams/human_safety`` treats an
uncalibrated camera, so live depth from an unlocalised camera can never be
integrated into a map the planner trusts.

  python3 publish_extrinsics.py --poses provisional_extrinsics.yaml
"""
from __future__ import annotations

import argparse
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from extrinsics_guard import load_extrinsics  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "provisional_extrinsics.yaml"))
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("fm_dyn_extrinsics")
    good, rejected = load_extrinsics(args.poses)
    for role, reason in rejected:
        node.get_logger().warn(f"camera '{role}' PRESENCE-ONLY (no TF): {reason}")
    tfs = []
    br = StaticTransformBroadcaster(node)
    for e in good:
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = e.parent
        t.child_frame_id = e.child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            (float(v) for v in e.translation)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = (float(v) for v in e.rotation)
        tfs.append(t)
        tag = "PROVISIONAL-SIM" if e.provisional else "calibrated"
        node.get_logger().info(f"camera '{e.role}' ({tag}) TF {e.parent}->{e.child} "
                               f"from {e.source}")
    if not tfs:
        node.get_logger().error("no valid camera extrinsics — every fixed camera is "
                                "presence-only; refusing to publish any TF")
    else:
        br.sendTransform(tfs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
