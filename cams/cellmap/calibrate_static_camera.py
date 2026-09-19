#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Calibrate a FIXED camera into base_link by AprilTag PnP, and write the result
in the repo's calibration format (``calib/base_to_camera_<serial>_<date>.yaml``).

This is the net-new static-camera solver from human-safety-s1 §6 / Phase 2. It
finds the cell's printed 36h11 tags (``tags/tag_world_poses.yaml``, known
base_link poses) in a fixed camera's COLOUR image, solves the camera pose by PnP
with that camera's colour intrinsics, composes with the known base->tag pose to
get base_link -> camera_optical, averages many frames, validates against the
<5 px reprojection / <1 cm depth target, and writes the extrinsic keyed BY SERIAL.

The geometry lives in ``staticcam.py`` (ROS-free, unit-tested); this file is the
thin ROS/RealSense capture shell.

Works for a colour-only camera (no depth): omit --depth. For a RealSense the
colour intrinsics arrive on --info (factory values from the driver); for an
ordinary webcam, run its one-time intrinsics with
``calibrate_webcam_intrinsics.py`` first and pass --intrinsics.

USAGE (camera already publishing on this ROS domain; e.g. via rig_multicam):
  ROS_DOMAIN_ID=80 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 calibrate_static_camera.py \
      --serial 025522060953 --role table \
      --image /table/color/image_raw --info /table/color/camera_info \
      --depth /table/aligned_depth/image --depth-info /table/aligned_depth/camera_info \
      --optical-frame table_color_optical_frame --views 60 --seconds 30

OFFLINE (a folder of colour PNGs + an intrinsics yaml — webcam / reproducibility):
  python3 calibrate_static_camera.py --from-images shots/ --intrinsics webcam_intr.yaml \
      --serial WEBCAM01 --role entrance --optical-frame entrance_color_optical_frame
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
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import staticcam as sc  # noqa: E402

DEFAULT_TAGMAP = os.path.join(HERE, "tags", "tag_world_poses.yaml")
CALIB_DIR = os.path.join(HERE, "calib")


def obliquity_deg(T_base_cam, tagmap, tag_id):
    """Angle between the camera optical axis and the tag normal (0 = looking
    straight down the tag normal = fronto-parallel = weakest for planar PnP)."""
    fwd = T_base_cam[:3, 2]                       # optical +z in base
    n = tagmap[tag_id].T_base_tag[:3, 2]          # tag normal in base
    return float(np.degrees(np.arccos(min(1.0, abs(fwd @ n)))))


def accept_view(sol, a, tagmap):
    """Gate a single-view solution. Returns (ok, reason)."""
    if sol.reproj_px_max > a.max_reproj:
        return False, f"reproj {sol.reproj_px_max:.2f} > {a.max_reproj} px"
    for tid, rng in sol.range_m_per_tag.items():
        if not (a.min_range <= rng <= a.max_range):
            return False, f"tag{tid} range {rng:.2f} outside {a.min_range}-{a.max_range} m"
    return True, ""


def build_result(a, cal, tagmap, depth_checks, obliq):
    xyz, quat = sc.xyz_quat_from_T(cal.T_base_cam)
    reproj_ok = cal.reproj_px_max <= a.max_reproj
    depth_ok = all(abs(d["error_cm"]) <= a.max_depth_cm for d in depth_checks) \
        if depth_checks else None
    # Gate on the tag-separation FIXTURE check (well constrained), not on the
    # per-tag pose disagreement (single static-view single-tag solves are
    # depth-weak, so their disagreement is pessimistic; kept as a diagnostic).
    fixture_ok = (cal.fixture_sep_err_mm is None) or \
        (cal.fixture_sep_err_mm <= a.max_fixture_mm)
    passed = bool(reproj_ok and fixture_ok and (depth_ok in (None, True)))
    res = {
        "calibrated": bool(passed),
        "when": dt.datetime.now().isoformat(timespec="seconds"),
        "method": "static-camera AprilTag PnP into base_link "
                  "(calibrate_static_camera.py; joint SQPNP over mapped tag corners)",
        "serial": a.serial,
        "role_hint": a.role,
        "source": {
            "image_topic": a.image, "info_topic": a.info,
            "depth_topic": a.depth or None,
            "tag_map": os.path.basename(a.tagmap),
            "dictionary": "apriltag_36h11",
        },
        "tags_used": cal.tags_used,
        "base_link_to_camera_optical_frame": {
            "parent": "base_link", "child": a.optical_frame,
            "xyz": xyz, "quaternion_xyzw": quat,
            "rpy_deg": [float(v) for v in
                        R.from_quat(quat).as_euler("xyz", degrees=True)],
        },
        "accuracy": {
            "n_views": cal.n_views,
            "reproj_px_mean": cal.reproj_px_mean,
            "reproj_px_max": cal.reproj_px_max,
            "view_spread_mm": cal.view_spread_mm,
            "view_spread_deg": cal.view_spread_deg,
            "fixture_sep_err_mm": cal.fixture_sep_err_mm,
            "cross_tag_mm": cal.cross_tag_mm,
            "cross_tag_deg": cal.cross_tag_deg,
            "obliquity_deg_per_tag": obliq,
            "depth_check": depth_checks or None,
        },
        "targets": {
            "reproj_px": a.max_reproj, "depth_cm": a.max_depth_cm,
            "fixture_sep_mm": a.max_fixture_mm,
        },
        "verdict": {
            "reproj_pass": bool(reproj_ok),
            "fixture_pass": bool(fixture_ok),
            "depth_pass": depth_ok,
            "pass": passed,
        },
        # Filled by recheck_static_camera.py; the camera side treats a stale or
        # failed recheck as uncalibrated (guard against a moved camera).
        "recheck": None,
        "notes": [
            "Coplanar tags on one plane give a systematic depth bias that shrinks "
            "with viewing obliquity; oblique views + the depth cross-check bound it.",
            "Absolute accuracy also inherits the tag-map budget (wrist hand-eye "
            "~8 mm), see tags/tag_world_poses.yaml provenance.",
        ],
    }
    return res, passed


def write_result(res, a):
    os.makedirs(CALIB_DIR, exist_ok=True)
    out = a.out or os.path.join(
        CALIB_DIR, f"base_to_camera_{a.serial}_{dt.date.today().isoformat()}.yaml")
    with open(out, "w") as f:
        yaml.safe_dump(res, f, sort_keys=False)
    return out


# --------------------------------------------------------------- offline images
def run_offline(a, tagmap):
    import cv2
    intr = yaml.safe_load(open(a.intrinsics))
    K = np.array(intr["camera_matrix"], float).reshape(3, 3)
    D = np.array(intr.get("dist_coeffs", np.zeros(5)), float).ravel()
    files = sorted(glob.glob(os.path.join(a.from_images, "*.png")) +
                   glob.glob(os.path.join(a.from_images, "*.jpg")))
    if not files:
        sys.exit(f"no images in {a.from_images}")
    views, obliq, rejected = [], {}, 0
    for fp in files:
        img = cv2.imread(fp)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dets = sc.detect_tags(gray)
        obs = [sc.TagObs(t, c) for t, c in dets.items() if t in tagmap]
        if not obs:
            rejected += 1
            continue
        # drop tags touching the border (corner refinement unreliable there)
        h_img, w_img = gray.shape[:2]
        obs = [o for o in obs if not _at_border(o.corners_px, w_img, h_img, a.margin)]
        if not obs:
            rejected += 1
            continue
        try:
            sol = sc.solve_camera_pose(obs, tagmap, K, D)
        except ValueError:
            rejected += 1
            continue
        ok, why = accept_view(sol, a, tagmap)
        if not ok:
            rejected += 1
            print(f"  reject {os.path.basename(fp)}: {why}")
            continue
        views.append(sol)
        for tid in sol.tag_ids:
            obliq.setdefault(tid, obliquity_deg(sol.T_base_cam, tagmap, tid))
    if not views:
        sys.exit(f"no acceptable views ({rejected} rejected)")
    cal = sc.average_views(views)
    res, passed = build_result(a, cal, tagmap, [], obliq)
    out = write_result(res, a)
    _report(cal, res, out, passed)
    return 0 if passed else 2


def _report(cal, res, out, passed):
    print(f"\n{'='*64}\n{cal.n_views} views | tags {cal.tags_used}")
    print(f"  reprojection : mean {cal.reproj_px_mean:.2f} / max "
          f"{cal.reproj_px_max:.2f} px  (target {res['targets']['reproj_px']})")
    print(f"  view spread  : {cal.view_spread_mm:.1f} mm / "
          f"{cal.view_spread_deg:.3f} deg")
    if cal.fixture_sep_err_mm is not None:
        print(f"  fixture sep  : {cal.fixture_sep_err_mm:.1f} mm error "
              f"(target {res['targets']['fixture_sep_mm']} mm)")
    if cal.cross_tag_mm is not None:
        print(f"  cross-tag*   : {cal.cross_tag_mm:.1f} mm / {cal.cross_tag_deg:.2f} deg "
              f"(informational; single-tag static solves are depth-weak)")
    for d in (res["accuracy"]["depth_check"] or []):
        print(f"  depth tag{d['tag']:>2}: pnp {d['pnp_m']:.3f} m vs measured "
              f"{d['measured_m']:.3f} m -> {d['error_cm']:+.1f} cm "
              f"(target {res['targets']['depth_cm']})")
    print(f"  VERDICT: {'PASS' if passed else 'DIAGNOSE'}\n  wrote {out}\n{'='*64}")


# ------------------------------------------------------------------- ROS live
def run_ros(a, tagmap):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    class Cal(Node):
        def __init__(self):
            super().__init__("calibrate_static_camera")
            self.K = self.D = None
            self.w = self.h = 0
            self.depth = None
            self.depth_scale = a.depth_scale
            self.views, self.obliq, self.n_rej = [], {}, 0
            self.why = {}
            self.last_annot = None
            q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CameraInfo, a.info, self._info, 5)
            self.create_subscription(Image, a.image, self._image, q)
            if a.depth:
                self.create_subscription(Image, a.depth, self._depth, q)
            self.t_last = 0.0

        def _info(self, m):
            if self.K is None:
                self.K = np.array(m.k, float).reshape(3, 3)
                self.D = np.array(m.d, float)
                self.w, self.h = m.width, m.height
                print(f"camera_info {m.width}x{m.height} frame {m.header.frame_id} "
                      f"fx {self.K[0,0]:.1f}")

        def _depth(self, m):
            if m.encoding in ("16UC1", "mono16"):
                self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            elif m.encoding == "32FC1":
                self.depth = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)

        def _bump(self, why):
            self.n_rej += 1
            self.why[why] = self.why.get(why, 0) + 1

        def _image(self, m):
            if self.K is None or m.encoding not in ("rgb8", "bgr8"):
                return
            now = time.time()
            if now - self.t_last < 1.0 / a.max_fps:
                return
            self.t_last = now
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if m.encoding == "rgb8"
                                else cv2.COLOR_BGR2GRAY)
            dets = sc.detect_tags(gray)
            obs = [sc.TagObs(t, c) for t, c in dets.items() if t in tagmap]
            if not obs:
                self._bump("no known tag in view")
                return
            # drop tags touching the border (corner refinement unreliable there)
            obs = [o for o in obs if not _at_border(o.corners_px, self.w, self.h, a.margin)]
            if not obs:
                self._bump("tag at image border")
                return
            try:
                sol = sc.solve_camera_pose(obs, tagmap, self.K, self.D)
            except ValueError as e:
                self._bump(str(e))
                return
            ok, why = accept_view(sol, a, tagmap)
            if not ok:
                self._bump(why)
                return
            self.views.append(sol)
            for tid in sol.tag_ids:
                self.obliq.setdefault(tid, obliquity_deg(sol.T_base_cam, tagmap, tid))
            self.last_annot = _annotate(img.copy(), dets, sol, tagmap, self.K, self.D,
                                        m.encoding)

        def depth_check(self, cal, tagmap):
            checks = []
            if self.depth is None:
                return checks
            for tid in cal.tags_used:
                # tag centre pixel in the colour image = project its base pose
                T_cam_base = sc.invert_T(cal.T_base_cam)
                c = sc.project_points(T_cam_base,
                                      tagmap[tid].T_base_tag[:3, 3][None, :],
                                      self.K, self.D)[0]
                u, v = int(round(c[0])), int(round(c[1]))
                if not (0 <= u < self.w and 0 <= v < self.h):
                    continue
                win = self.depth[max(0, v-2):v+3, max(0, u-2):u+3].astype(float)
                win = win[win > 0]
                if win.size == 0:
                    continue
                measured = float(np.median(win)) * self.depth_scale
                # PnP optical-axis depth (z) of the tag centre in camera frame
                pnp_z = float((T_cam_base @ np.append(
                    tagmap[tid].T_base_tag[:3, 3], 1.0))[2])
                checks.append({"tag": int(tid), "pnp_m": pnp_z,
                               "measured_m": measured,
                               "error_cm": (pnp_z - measured) * 100.0})
            return checks

    rclpy.init()
    n = Cal()
    print(f"static-camera calibrate: serial {a.serial} role {a.role}; tags "
          f"{sorted(tagmap)} on {a.image}; need {a.views} views, <= {a.seconds}s.")
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    t0 = time.time()
    while (rclpy.ok() and not stop["now"] and len(n.views) < a.views
           and time.time() - t0 < a.seconds):
        rclpy.spin_once(n, timeout_sec=0.05)
        if int((time.time() - t0) * 2) % 2 == 0:
            top = sorted(n.why.items(), key=lambda kv: -kv[1])[:1]
            sys.stdout.write(f"\r  views {len(n.views)}/{a.views}  rejected "
                             f"{n.n_rej}" + (f" ({top[0][0]})" if top else "") + "   ")
            sys.stdout.flush()
    print()
    if not n.views:
        print(f"NO acceptable views ({n.n_rej} rejected). "
              "Is a known tag (id 0/1) visible to this camera? If not, this camera "
              "stays presence-only — place a tag at a mapped base pose first.")
        _os_exit(3, n)
    cal = sc.average_views(n.views)
    depth_checks = n.depth_check(cal, tagmap)
    res, passed = build_result(a, cal, tagmap, depth_checks, n.obliq)
    out = write_result(res, a)
    if n.last_annot is not None:
        import cv2 as _cv
        png = out.replace(".yaml", ".png")
        _cv.imwrite(png, n.last_annot)
        print(f"  annotated frame -> {png}")
    _report(cal, res, out, passed)
    _os_exit(0 if passed else 2, n)


def _at_border(corners, w, h, margin):
    c = np.asarray(corners)
    return bool((c[:, 0] < margin).any() or (c[:, 1] < margin).any() or
                (c[:, 0] > w - margin).any() or (c[:, 1] > h - margin).any())


def _annotate(img, dets, sol, tagmap, K, D, encoding):
    import cv2
    if encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    T_cam_base = sc.invert_T(sol.T_base_cam)
    for tid, c in dets.items():
        col = (0, 255, 0) if tid in tagmap else (0, 128, 255)
        cv2.polylines(img, [c.astype(int)], True, col, 2)
        cv2.putText(img, str(tid), tuple(c[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        if tid in tagmap:
            pred = sc.project_points(
                T_cam_base, sc.tag_corners_base(tagmap[tid].T_base_tag,
                                                tagmap[tid].size), K, D)
            for p in pred:
                cv2.circle(img, tuple(p.astype(int)), 3, (0, 0, 255), -1)
    cv2.putText(img, f"reproj max {sol.reproj_px_max:.2f}px", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return img


def _os_exit(code, node):
    # tf/rclpy shutdown can hang the CLI after results are safely on disk;
    # os._exit like validate_tag1.py.
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    os._exit(code)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", required=True, help="camera serial (the identity anchor)")
    ap.add_argument("--role", default="", help="provisional role label (hint only)")
    ap.add_argument("--optical-frame", required=True,
                    help="child TF frame the camera side looks up (e.g. table_color_optical_frame)")
    ap.add_argument("--image", default="")
    ap.add_argument("--info", default="")
    ap.add_argument("--depth", default="", help="aligned depth topic (RealSense; omit for colour-only)")
    ap.add_argument("--depth-info", default="")
    ap.add_argument("--depth-scale", type=float, default=1e-3,
                    help="raw depth unit -> metres (RealSense 16UC1 = 1e-3)")
    ap.add_argument("--tagmap", default=DEFAULT_TAGMAP)
    ap.add_argument("--views", type=int, default=60, help="accepted views to collect")
    ap.add_argument("--seconds", type=float, default=45.0, help="max capture seconds")
    ap.add_argument("--max-fps", type=float, default=6.0)
    ap.add_argument("--max-reproj", type=float, default=5.0, help="px (human-safety-s1 target)")
    ap.add_argument("--max-depth-cm", type=float, default=1.0, help="cm (target)")
    ap.add_argument("--max-fixture-mm", type=float, default=10.0,
                    help="max tag-to-tag separation error (mm)")
    ap.add_argument("--min-range", type=float, default=0.3)
    ap.add_argument("--max-range", type=float, default=6.0)
    ap.add_argument("--margin", type=int, default=20, help="px from border to reject a tag")
    ap.add_argument("--from-images", default="", help="offline: folder of colour PNGs")
    ap.add_argument("--intrinsics", default="", help="offline/webcam intrinsics yaml")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    tagmap = sc.load_tag_map(a.tagmap)
    if a.from_images:
        if not a.intrinsics:
            sys.exit("--from-images needs --intrinsics")
        return run_offline(a, tagmap)
    if not (a.image and a.info):
        sys.exit("live mode needs --image and --info (or use --from-images)")
    return run_ros(a, tagmap)


if __name__ == "__main__":
    sys.exit(main())
