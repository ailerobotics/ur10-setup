#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Quick re-check that a fixed camera has NOT moved since it was calibrated
(human-safety-s1 §6 / Phase 2, the "guard against a moved camera" item).

It re-detects the mapped AprilTags live and asks: do they still reproject where
the STORED base_link->camera pose says they should? A camera that has been nudged
makes the reprojection error blow past the threshold. On failure it stamps the
calibration file's ``recheck`` block with ``status: fail``; the camera side
(``human_safety`` pose loader) treats a failed or stale recheck as UNCALIBRATED
and drops that camera back to presence-only.

The geometry is ``staticcam.recheck_pose`` (unit-tested); this is the thin
ROS/RealSense shell.

  ROS_DOMAIN_ID=80 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 recheck_static_camera.py \
      --calib calib/base_to_camera_025522060953_2026-09-19.yaml \
      --image /table/color/image_raw --info /table/color/camera_info

Offline (a folder of colour PNGs + intrinsics, for tests / reproducibility):
  python3 recheck_static_camera.py --calib <file> --from-images shots/ --intrinsics intr.yaml
"""
import argparse
import datetime as dt
import glob
import os
import signal
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import staticcam as sc  # noqa: E402

DEFAULT_TAGMAP = os.path.join(HERE, "tags", "tag_world_poses.yaml")


def load_calib(path):
    d = yaml.safe_load(open(path))
    e = d["base_link_to_camera_optical_frame"]
    T = sc.T_from_xyz_quat(e["xyz"], e["quaternion_xyzw"])
    return d, T


def stamp_recheck(path, d, res, serial_confirmed):
    d["recheck"] = {
        "when": dt.datetime.now().isoformat(timespec="seconds"),
        "status": "pass" if (res.passed and serial_confirmed is not False) else "fail",
        "reproj_px_max": res.reproj_px_max if np.isfinite(res.reproj_px_max) else None,
        "tags_seen": res.tags_seen,
        "threshold_px": res.threshold_px,
        "serial_confirmed": serial_confirmed,
        "reason": res.reason,
    }
    with open(path, "w") as f:
        yaml.safe_dump(d, f, sort_keys=False)


def _report(res, serial_confirmed, path):
    ok = res.passed and serial_confirmed is not False
    print(f"\n{'='*60}")
    print(f"  tags seen    : {res.tags_seen}")
    print(f"  reproj max   : "
          f"{res.reproj_px_max:.2f} px (threshold {res.threshold_px})"
          if np.isfinite(res.reproj_px_max) else "  reproj max   : n/a")
    if serial_confirmed is not None:
        print(f"  serial match : {serial_confirmed}")
    print(f"  RESULT: {'PASS - camera has not moved' if ok else 'FAIL - '+res.reason}")
    print(f"  stamped recheck -> {path}\n{'='*60}")


def run_offline(a, tagmap, calib_d, T):
    import cv2
    intr = yaml.safe_load(open(a.intrinsics))
    K = np.array(intr["camera_matrix"], float).reshape(3, 3)
    D = np.array(intr.get("dist_coeffs", np.zeros(5)), float).ravel()
    files = sorted(glob.glob(os.path.join(a.from_images, "*.png")) +
                   glob.glob(os.path.join(a.from_images, "*.jpg")))
    best = None
    for fp in files:
        dets = sc.detect_tags(cv2.cvtColor(cv2.imread(fp), cv2.COLOR_BGR2GRAY))
        obs = [sc.TagObs(t, c) for t, c in dets.items() if t in tagmap]
        if not obs:
            continue
        res = sc.recheck_pose(T, obs, tagmap, K, D, threshold_px=a.threshold)
        if best is None or res.reproj_px_max < best.reproj_px_max:
            best = res
    if best is None:
        print("no known tag detected in any image")
        return 3
    stamp_recheck(a.calib, calib_d, best, None)
    _report(best, None, a.calib)
    return 0 if best.passed else 2


def run_ros(a, tagmap, calib_d, T):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    class Recheck(Node):
        def __init__(self):
            super().__init__("recheck_static_camera")
            self.K = self.D = None
            self.best = None
            self.t_last = 0.0
            q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CameraInfo, a.info, self._info, 5)
            self.create_subscription(Image, a.image, self._image, q)

        def _info(self, m):
            if self.K is None:
                self.K = np.array(m.k, float).reshape(3, 3)
                self.D = np.array(m.d, float)

        def _image(self, m):
            if self.K is None or m.encoding not in ("rgb8", "bgr8"):
                return
            now = time.time()
            if now - self.t_last < 1.0 / 4.0:
                return
            self.t_last = now
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if m.encoding == "rgb8"
                                else cv2.COLOR_BGR2GRAY)
            dets = sc.detect_tags(gray)
            obs = [sc.TagObs(t, c) for t, c in dets.items() if t in tagmap]
            if not obs:
                return
            res = sc.recheck_pose(T, obs, tagmap, self.K, self.D, threshold_px=a.threshold)
            if self.best is None or res.reproj_px_max < self.best.reproj_px_max:
                self.best = res

    rclpy.init()
    n = Recheck()
    print(f"recheck: stored pose {os.path.basename(a.calib)} on {a.image}; "
          f"threshold {a.threshold} px; <= {a.seconds}s.")
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    t0 = time.time()
    while (rclpy.ok() and not stop["now"] and time.time() - t0 < a.seconds
           and (n.best is None or time.time() - t0 < a.min_seconds)):
        rclpy.spin_once(n, timeout_sec=0.05)
    serial_ok = None
    if a.expected_serial:
        serial_ok = (a.expected_serial == calib_d.get("serial"))
    if n.best is None:
        print("no known tag detected — cannot confirm the camera position; "
              "treating as FAIL (fails toward uncalibrated).")
        from types import SimpleNamespace
        n.best = SimpleNamespace(passed=False, reproj_px_max=float("inf"),
                                 tags_seen=[], threshold_px=a.threshold,
                                 reason="no known tag detected")
    stamp_recheck(a.calib, calib_d, n.best, serial_ok)
    _report(n.best, serial_ok, a.calib)
    code = 0 if (n.best.passed and serial_ok is not False) else 2
    try:
        n.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    os._exit(code)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", required=True, help="calibration yaml to re-check")
    ap.add_argument("--image", default="")
    ap.add_argument("--info", default="")
    ap.add_argument("--tagmap", default=DEFAULT_TAGMAP)
    ap.add_argument("--threshold", type=float, default=5.0, help="reproj px to still call PASS")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--min-seconds", type=float, default=3.0,
                    help="keep looking at least this long for a better view")
    ap.add_argument("--expected-serial", default="",
                    help="fail if the calib file's serial differs (identity guard)")
    ap.add_argument("--from-images", default="")
    ap.add_argument("--intrinsics", default="")
    a = ap.parse_args()

    tagmap = sc.load_tag_map(a.tagmap)
    calib_d, T = load_calib(a.calib)
    if a.from_images:
        if not a.intrinsics:
            sys.exit("--from-images needs --intrinsics")
        return run_offline(a, tagmap, calib_d, T)
    if not (a.image and a.info):
        sys.exit("live mode needs --image and --info (or use --from-images)")
    return run_ros(a, tagmap, calib_d, T)


if __name__ == "__main__":
    sys.exit(main())
