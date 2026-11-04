#!/usr/bin/env python3
"""Thin ROS live mode for the pre-contact alignment check.

DOCUMENTED + DRY-TESTED ONLY in this task -- it is NEVER run against the cell
here (cell powered off; hard rule). It subscribes (read-only) to the calibrated
wrist colour image + CameraInfo (+ depth for shelf, + /joint_states for table),
runs the SAME offline pipeline as align_check.py, and writes the verdict JSON.
It publishes nothing and commands nothing: no driver, controller, dashboard,
freedrive, or service call, and it does NOT start any camera driver (it only
subscribes to an already-running one). It runs on a task-only ROS_DOMAIN_ID
(params.yaml live.ros_domain_id, default 76 -- never 0 or 42).

The ROS-free core is process_frame(): the node is a thin shell that fills its
arguments from the latest messages. --dry-run exercises process_frame on a
file (no rclpy, no DDS) so the wiring is provable offline.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from fork_demo.align_check import align_check, calib, geometry, verdict_io
    from fork_demo.align_check.fk_ur10 import JOINT_ORDER
else:
    from . import align_check, calib, geometry, verdict_io
    from .fk_ur10 import JOINT_ORDER


def process_frame(side, bgr, *, K=None, depth=None, joints=None, params=None,
                  out_verdict=None, image_stamp="live"):
    """ROS-free core: run the pipeline on one frame, optionally write the verdict.

    Returns (verdict_or_None, annotated_bgr, info). Identical maths to the CLI.
    """
    params = params or align_check.load_params()
    if side == "shelf":
        verdict, ann, info = align_check.align_shelf(bgr, params, depth=depth, K=K)
    else:
        t_base_optical = None
        if joints is not None:
            t_base_optical = align_check.base_optical_from_joints(list(joints))
        verdict, ann, info = align_check.align_table(bgr, params, K=K, t_base_optical=t_base_optical)
    if verdict is not None:
        verdict.image_stamp = image_stamp
        if out_verdict:
            verdict_io.write_verdict(verdict, out_verdict)
    return verdict, ann, info


def run_live(side, out_verdict, params=None, timeout_s=10.0):  # pragma: no cover - needs ROS + cell
    """Collect one frame off the calibrated topics and write a verdict. Never
    run in this task. Imports rclpy lazily so the module loads without ROS."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, JointState

    params = params or align_check.load_params()
    live = params["live"]
    # Enforce the task-only domain; refuse the cell domains.
    dom = int(live["ros_domain_id"])
    if dom in (0, 42):
        raise SystemExit(f"refusing ROS_DOMAIN_ID {dom} (cell domain); use a task-only domain")
    os.environ["ROS_DOMAIN_ID"] = str(dom)
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", live.get("fastdds_builtin_transports", "UDPv4"))

    img_topic = live["shelf_image_topic"] if side == "shelf" else live["table_image_topic"]
    info_topic = live["shelf_info_topic"] if side == "shelf" else live["table_info_topic"]

    rclpy.init()
    node = Node("align_check_live")
    state = {"bgr": None, "K": None, "depth": None, "joints": None}

    def _decode(msg):
        # cv_bridge-free: raw rgb8/bgr8 -> ndarray (cams/human_safety pattern).
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        return arr[:, :, ::-1].copy() if msg.encoding == "rgb8" else arr.copy()

    node.create_subscription(Image, img_topic, lambda m: state.update(bgr=_decode(m)), 1)
    node.create_subscription(CameraInfo, info_topic,
                             lambda m: state.update(K=calib.K_from_camera_info({"k": list(m.k)})), 1)
    if side == "shelf":
        node.create_subscription(Image, live["shelf_depth_topic"],
                                 lambda m: state.update(depth=_decode_depth(m)), 1)
    else:
        node.create_subscription(
            JointState, live["joint_states_topic"],
            lambda m: state.update(joints=_order_joints(m)), 1)

    import time
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
        need = state["bgr"] is not None and state["K"] is not None and (
            side == "shelf" or state["joints"] is not None)
        if need:
            break
    v, _, _ = process_frame(side, state["bgr"], K=state["K"], depth=state["depth"],
                            joints=state["joints"], params=params, out_verdict=out_verdict,
                            image_stamp=f"live-{side}")
    node.destroy_node()
    rclpy.shutdown()
    return v


def _decode_depth(msg):  # pragma: no cover - needs ROS
    d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(float) / 1000.0
    return d


def _order_joints(msg):  # pragma: no cover - needs ROS
    name_to_pos = dict(zip(msg.name, msg.position))
    return [name_to_pos[j] for j in JOINT_ORDER]


def main(argv=None):
    ap = argparse.ArgumentParser(description="Live pre-contact alignment check (read-only, writes verdict).")
    ap.add_argument("--side", required=True, choices=["shelf", "table"])
    ap.add_argument("--verdict", required=True, help="verdict JSON output path")
    ap.add_argument("--dry-run", action="store_true",
                    help="ROS-free wiring test: run process_frame on --image instead of subscribing")
    ap.add_argument("--image", default=None, help="image for --dry-run")
    ap.add_argument("--depth", default=None)
    ap.add_argument("--wrist-pose", default=None, help="table --dry-run: joint yaml")
    args = ap.parse_args(argv)

    if args.dry_run:
        import cv2
        import yaml
        if not args.image:
            print("--dry-run needs --image", file=sys.stderr)
            return 2
        bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
        depth = None
        if args.depth:
            depth = align_check._load_depth(args.depth)
        joints = None
        if args.wrist_pose:
            joints = align_check._load_joints(args.wrist_pose)
        v, _, info = process_frame(args.side, bgr, depth=depth, joints=joints,
                                   out_verdict=args.verdict, image_stamp=f"dryrun-{args.side}")
        print(f"[dry-run] process_frame ok; verdict={'written' if v else 'none (detection-only)'} -> {args.verdict}")
        return 0

    print("LIVE mode subscribes to the cell cameras -- do NOT run against a powered cell in this task.")
    return run_live(args.side, args.verdict)


if __name__ == "__main__":
    raise SystemExit(main())
