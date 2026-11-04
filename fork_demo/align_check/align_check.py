#!/usr/bin/env python3
"""Pre-contact alignment check for the UR10 fork pick/place -- CLI + library.

Before the tines touch anything, answer two questions:
  * SHELF: will the two tines enter the dock channels (lateral offset + yaw)?
  * TABLE: will the set-down seat in the table dock (dx, dy, dyaw vs the tag)?

Everything here is OFFLINE and PLAN-ONLY: it reads an image (and optional depth
/ pose), prints a go/no-go with a never-applied correction proposal, and writes
the verdict JSON the sibling contact-guard lane's pre-contact gate reads. It
NEVER commands the robot, a controller, a driver, or a camera driver. The thin
ROS live mode is a separate module (live_node.py), documented + dry-tested only.

Usage:
  align_check.py --side shelf --image wrist.png [--depth d.npy] [--out ann.png] [--verdict v.json]
  align_check.py --side table --image table.png [--intrinsics info.yaml]
                 [--wrist-pose joints.yaml] [--out ann.png] [--verdict v.json]
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

try:
    import cv2
    import yaml
except ImportError as e:  # pragma: no cover
    print(f"missing dependency: {e}", file=sys.stderr)
    raise

# Support running as a module (python -m fork_demo.align_check.align_check)
# and as a script (python fork_demo/align_check/align_check.py).
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from fork_demo.align_check import calib, geometry, shelf_detect, tag_detect, tolerance, verdict_io
    from fork_demo.align_check.fk_ur10 import UR10Kinematics, JOINT_ORDER
    from fork_demo.align_check.geometry import TineGeometry
else:
    from . import calib, geometry, shelf_detect, tag_detect, tolerance, verdict_io
    from .fk_ur10 import UR10Kinematics, JOINT_ORDER
    from .geometry import TineGeometry

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARAMS = os.path.join(HERE, "params.yaml")


def load_params(path: str = DEFAULT_PARAMS) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def tine_geometry(params: dict) -> TineGeometry:
    t = params["tines"]
    return TineGeometry(
        spacing_m=t["spacing_m"], width_m=t["width_m"], thickness_m=t["thickness_m"],
        center_y_m=t["center_y_m"], root_z_m=t["root_z_m"], tip_z_m=t["tip_z_m"],
    )


# --------------------------------------------------------------------------
# Shelf pipeline
# --------------------------------------------------------------------------
def _sample_depth(depth: np.ndarray, uv, win: int = 7) -> float | None:
    """Median valid depth (metres) in a window around uv. depth in metres."""
    if depth is None:
        return None
    h, w = depth.shape[:2]
    u, v = int(round(uv[0])), int(round(uv[1]))
    if not (0 <= u < w and 0 <= v < h):
        return None
    r = win // 2
    patch = depth[max(0, v - r): v + r + 1, max(0, u - r): u + r + 1].astype(float)
    vals = patch[np.isfinite(patch) & (patch > 0)]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def align_shelf(bgr, params, *, depth=None, handeye_path=None, K=None):
    """Returns (verdict, annotated_bgr, info)."""
    tg = tine_geometry(params)
    sp = params["shelf"]
    t_tool0_optical, t_optical_tool0, hmeta = calib.load_handeye(
        handeye_path or calib.DEFAULT_HANDEYE
    )
    K = calib.K_from_params(params) if K is None else K
    det = shelf_detect.detect_strips(bgr)
    assumptions = [
        f"tine geometry from forklift_S kit (spacing {tg.spacing_m*1000:.0f} mm, "
        f"width {tg.width_m*1000:.0f} mm) -- captain to confirm vs physical forks",
        f"hand-eye: {os.path.basename(hmeta['source'])} ({hmeta.get('calibrated')})",
    ]
    if not det.ok:
        verdict = tolerance.shelf_verdict(
            {"left": 0.0, "right": 0.0}, 0.0,
            channel_width_m=sp["channel_width_m"], tine_width_m=sp["tine_width_m"],
            safety_factor=sp["safety_factor"], yaw_tolerance_deg=sp["yaw_tolerance_deg"],
            assumptions=assumptions + [f"DETECTION FAILED: found {len(det.strips)} yellow strip(s), expected 2"],
        )
        verdict.ok = False
        return verdict, shelf_detect.annotate(bgr, det), {"detection": det}

    # Back-project each strip centre to tool0 (depth if available, else plane).
    plane = sp["default_plane_depth_m"]
    pts_tool0 = {}
    used_depth = {}
    for side, s in zip(("left", "right"), det.strips):
        d = _sample_depth(depth, (s.cx, s.cy))
        z = d if d is not None else plane
        used_depth[side] = ("measured" if d is not None else "assumed_plane", round(z, 4))
        p_opt = geometry.backproject_pixel(K, (s.cx, s.cy), z)
        pts_tool0[side] = geometry.transform_points(t_tool0_optical, p_opt)[0]

    lateral = {}
    for side in ("left", "right"):
        tine_x = -tg.spacing_m / 2.0 if side == "left" else tg.spacing_m / 2.0
        lateral[side] = float((pts_tool0[side][0] - tine_x) * 1000.0)  # + channel right of tine

    # Yaw about tool Y (vertical): channel-to-channel vector vs tool X axis.
    # +yaw => the dock is rotated about tool Y by +yaw (right channel further
    # from the camera than the left). See synth.render_shelf_scene sign check.
    v = pts_tool0["right"] - pts_tool0["left"]
    yaw_deg = float(np.degrees(np.arctan2(-v[2], v[0])))

    if any(x[0] == "assumed_plane" for x in used_depth.values()):
        assumptions.append(
            f"no wrist depth: strips assumed on fronto-parallel plane at optical Z={plane} m "
            "(lateral mm scales with this depth -- supply --depth for the true value)"
        )
    assumptions.append("nominal D435i colour K (params.yaml); live mode overrides from CameraInfo")

    proj = geometry.project_tines(K, t_optical_tool0, tg)
    verdict = tolerance.shelf_verdict(
        lateral, yaw_deg,
        channel_width_m=sp["channel_width_m"], tine_width_m=sp["tine_width_m"],
        safety_factor=sp["safety_factor"], yaw_tolerance_deg=sp["yaw_tolerance_deg"],
        assumptions=assumptions,
        extra_detail={
            "strip_px": {s_side: [round(s.cx, 1), round(s.cy, 1)]
                         for s_side, s in zip(("left", "right"), det.strips)},
            "depth_source": {k: v for k, v in used_depth.items()},
            "channel_axis_tool0_m": [round(float(x), 4) for x in v],
        },
    )
    ann = shelf_detect.annotate(bgr, det, projected=proj)
    return verdict, ann, {"detection": det, "projected": proj, "pts_tool0": pts_tool0}


# --------------------------------------------------------------------------
# Table pipeline
# --------------------------------------------------------------------------
def base_optical_from_joints(joints, handeye_path=None, urdf_path=None):
    """T_base_optical = FK(joints) @ T_tool0_optical."""
    t_tool0_optical, _, _ = calib.load_handeye(handeye_path or calib.DEFAULT_HANDEYE)
    kin = UR10Kinematics.from_urdf(urdf_path) if urdf_path else UR10Kinematics.from_urdf()
    t_base_tool0 = kin.fk_tool0(joints)
    return t_base_tool0 @ t_tool0_optical


def align_table(bgr, params, *, K=None, t_base_optical=None, taught=None):
    """Returns (verdict_or_None, annotated_bgr, info).

    Full dx/dy/dyaw verdict requires t_base_optical (live: FK(joints) @ hand-eye)
    AND a taught place pose with yaw. Offline on a guide photo (neither known) the
    function still detects + annotates the tag and returns a detection-only info;
    verdict is None in that case.
    """
    tp = params["table"]
    det = tag_detect.detect_tag(bgr, tag_id=tp["tag_id"])
    if not det.found:
        return None, tag_detect.annotate(bgr, det), {"detection": det, "reason": "tag not found"}

    K = calib.K_from_params(params) if K is None else K
    pose = tag_detect.solve_tag_pose(det, K, tp["tag_size_m"])
    ann = tag_detect.annotate(bgr, det, k=K, size_m=tp["tag_size_m"], pose=pose)
    info = {"detection": det, "pose": pose}

    if t_base_optical is None:
        return None, ann, info  # detection-only (offline guide photo)

    t_cam_tag = tag_detect.tag_matrix_in_camera(pose)
    t_base_tag = t_base_optical @ t_cam_tag

    taught = taught or {}
    tx, ty = taught.get("xy", tp["taught_place_xy_m"])
    tyaw = taught.get("yaw_deg", tp["taught_place_yaw_deg"])
    dx_mm, dy_mm, dyaw, tag_xy, tag_yaw, target_xy = geometry.compare_place_to_tag(
        t_base_tag, (tx, ty), tyaw, tp["tag_to_place_offset_xy_m"]
    )

    assumptions = [
        f"tag {tp['tag_size_m']*1000:.0f} mm 36h11 id {tp['tag_id']}; TOP arrow == tag +Y (print convention)",
        f"tag_to_place offset {tp['tag_to_place_offset_xy_m']} m (ASSUMPTION: tag centred on dock slot)",
        f"reproj RMS {pose.reproj_px:.2f} px, tag distance {pose.distance_m:.3f} m",
    ]
    if tyaw is None:
        assumptions.append("taught place yaw unknown offline; dyaw=0 placeholder (live reads tcp_pose)")
    verdict = tolerance.table_verdict(
        dx_mm, dy_mm, dyaw,
        channel_width_m=tp["dock_slot_width_m"], tine_width_m=tp["tray_width_m"],
        safety_factor=tp["safety_factor"], dyaw_tolerance_deg=tp["dyaw_tolerance_deg"],
        assumptions=assumptions,
        extra_detail={
            "tag_xy_base_m": [round(float(x), 4) for x in tag_xy],
            "tag_yaw_base_deg": round(tag_yaw, 3),
            "target_xy_base_m": [round(float(x), 4) for x in target_xy],
            "taught_place_xy_m": [round(tx, 4), round(ty, 4)],
        },
    )
    info["t_base_tag"] = t_base_tag
    return verdict, ann, info


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def _load_depth(path):
    if path is None:
        return None
    if path.endswith(".npy"):
        d = np.load(path).astype(float)
    else:
        d = cv2.imread(path, cv2.IMREAD_UNCHANGED).astype(float)
    # Heuristic: integer depths are millimetres -> metres.
    if np.nanmax(d) > 100:
        d = d / 1000.0
    return d


def _load_joints(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    if isinstance(data, dict) and "joints" in data:
        data = data["joints"]
    if isinstance(data, dict):
        return [data[j] for j in JOINT_ORDER]
    return list(data)


def _print_verdict(v, side):
    print("=" * 68)
    print(f"PRE-CONTACT ALIGNMENT CHECK -- {side.upper()}  (PLAN-ONLY, no motion)")
    print("=" * 68)
    print(f"  VERDICT: {'GO' if v.ok else 'NO-GO'}")
    if side == "shelf":
        print(f"  lateral offset per tine (mm): {v.detail.get('lateral_per_tine_mm')}")
        print(f"  mean lateral dx = {v.dx_mm:+.2f} mm   yaw dyaw = {v.dyaw_deg:+.2f} deg")
    else:
        print(f"  dx = {v.dx_mm:+.2f} mm   dy = {v.dy_mm:+.2f} mm   dyaw = {v.dyaw_deg:+.2f} deg")
    print(f"  tolerance: +/-{v.tolerance_mm:.2f} mm, +/-{v.tolerance_deg:.2f} deg")
    print(f"  margin:    {v.margin_mm:+.2f} mm, {v.margin_deg:+.2f} deg  (negative => over tolerance)")
    print("  CORRECTION PROPOSAL (PRINT ONLY -- never applied automatically):")
    print(f"    dx={v.correction_dx_mm:+.2f} mm  dy={v.correction_dy_mm:+.2f} mm  "
          f"dyaw={v.correction_dyaw_deg:+.2f} deg")
    if v.assumptions:
        print("  assumptions:")
        for a in v.assumptions:
            print(f"    - {a}")
    print("=" * 68)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Pre-contact fork alignment check (offline, plan-only).")
    ap.add_argument("--side", required=True, choices=["shelf", "table"])
    ap.add_argument("--image", required=True, help="colour image (wrist for shelf; table/wrist for table)")
    ap.add_argument("--depth", default=None, help="shelf: aligned depth (.npy or 16-bit png), metres or mm")
    ap.add_argument("--intrinsics", default=None, help="table: CameraInfo yaml with 'k' (else nominal K)")
    ap.add_argument("--wrist-pose", default=None, help="table live: joint-state yaml -> FK -> base frame")
    ap.add_argument("--params", default=DEFAULT_PARAMS)
    ap.add_argument("--out", default=None, help="write annotated image here")
    ap.add_argument("--verdict", default=None, help="write verdict JSON here (contact-guard contract)")
    ap.add_argument("--image-stamp", default=None, help="stamp to record in the verdict")
    args = ap.parse_args(argv)

    params = load_params(args.params)
    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"cannot read image: {args.image}", file=sys.stderr)
        return 2
    stamp = args.image_stamp or os.path.basename(args.image)

    if args.side == "shelf":
        depth = _load_depth(args.depth)
        verdict, ann, _ = align_shelf(bgr, params, depth=depth)
        verdict.image_stamp = stamp
        _print_verdict(verdict, "shelf")
    else:
        K = None
        if args.intrinsics:
            with open(args.intrinsics) as f:
                K = calib.K_from_camera_info(yaml.safe_load(f))
        t_base_optical = None
        if args.wrist_pose:
            t_base_optical = base_optical_from_joints(_load_joints(args.wrist_pose))
        verdict, ann, info = align_table(bgr, params, K=K, t_base_optical=t_base_optical)
        if verdict is None:
            print("TABLE: tag detection only (no wrist pose / taught yaw for a base-frame verdict).")
            if "pose" in info:
                p = info["pose"]
                print(f"  tag id {params['table']['tag_id']} found; reproj RMS {p.reproj_px:.2f} px, "
                      f"distance {p.distance_m:.3f} m (K assumption-dependent)")
            else:
                print(f"  {info.get('reason')}")
        else:
            verdict.image_stamp = stamp
            _print_verdict(verdict, "table")

    if args.out is not None and ann is not None:
        cv2.imwrite(args.out, ann)
        print(f"wrote annotated image: {args.out}")
    if args.verdict is not None and verdict is not None:
        contract = verdict_io.write_verdict(verdict, args.verdict)
        print(f"wrote verdict JSON: {args.verdict}")
        print(json.dumps(contract, indent=2, sort_keys=True))
    return 0 if (verdict is not None and verdict.ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
