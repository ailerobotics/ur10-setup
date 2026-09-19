#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Derive the full 6-DoF base_link pose of each printed cell AprilTag from the
TRUSTED wrist hand-eye calibration, and write it as the tag world-pose map the
static-camera calibrator (``calibrate_static_camera.py``) treats as ground truth.

WHY THIS EXISTS. To calibrate a *fixed* camera into base_link by AprilTag PnP we
must know where the tags are in base_link. The repo already carries that
knowledge implicitly and to mm accuracy: the wrist D435i hand-eye solve
(``calib/tool0_to_camera_2026-09-03.yaml``, cross-tag agreement 8.4 mm / 1.06 deg,
independently validated to 1.87 mm scale / 0.68 mm table-plane) plus the raw
per-view samples it was fitted from (``calib/samples-2026-09-03-tag*.yaml``, each
carrying ``T_base_tool`` and ``T_cam_tag`` per view). Carrying every view through

    T_base_tag = T_base_tool @ T_tool0_opticalframe @ T_cam_tag

and averaging (translation median, rotation = Markley quaternion mean) yields the
tag's fixed base_link pose. The per-view spread is reported as the internal
consistency; the ABSOLUTE accuracy is inherited from the wrist calibration budget
(quoted in the output and the calibration report), not from the spread.

The validate tool (``validate_tag1.py``) computes the same tag positions as a
by-product but only records position + surface normal; the static-camera solver
needs the FULL orientation (to place the tag's four corners in base_link), so this
tool derives and persists it once, with provenance.

  python3 tools/derive_tag_world_poses.py               # writes tags/tag_world_poses.yaml
  python3 tools/derive_tag_world_poses.py --check       # print, do not write
"""
import argparse
import datetime as dt
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
CELLMAP = os.path.dirname(HERE)
DEFAULT_CALIB = os.path.join(CELLMAP, "calib", "tool0_to_camera_2026-09-03.yaml")
DEFAULT_SAMPLES = [
    os.path.join(CELLMAP, "calib", "samples-2026-09-03-tag0.yaml"),
    os.path.join(CELLMAP, "calib", "samples-2026-09-03-tag1.yaml"),
]
DEFAULT_OUT = os.path.join(CELLMAP, "tags", "tag_world_poses.yaml")


def quat_mean(quats):
    """Markley's quaternion mean (dominant eigenvector of sum(q q^T)); component-
    wise averaging is not rotation-averaging. Same method as tools/combine_tags.py."""
    Q = np.array([q if q[3] >= 0 else -q for q in quats], dtype=float)
    M = sum(np.outer(q, q) for q in Q)
    w, v = np.linalg.eigh(M)
    q = v[:, int(np.argmax(w))]
    return q / np.linalg.norm(q)


def T_from_xyz_quat(xyz, quat_xyzw):
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = xyz
    return T


def derive_one(samples_path, T_tool_opt):
    d = yaml.safe_load(open(samples_path))
    ps, qs, rngs = [], [], []
    for s in d["samples"]:
        T_base_tool = np.array(s["T_base_tool"], float)
        T_cam_tag = np.array(s["T_cam_tag"], float)
        T_base_tag = T_base_tool @ T_tool_opt @ T_cam_tag
        ps.append(T_base_tag[:3, 3])
        qs.append(R.from_matrix(T_base_tag[:3, :3]).as_quat())
        rngs.append(float(np.linalg.norm(T_cam_tag[:3, 3])))
    P = np.array(ps)
    pos = np.median(P, axis=0)
    spread = np.linalg.norm(P - pos, axis=1) * 1e3
    q = quat_mean(qs)
    return {
        "tag_id": int(d["tag_id"]),
        "tag_size_m": float(d["tag_size_m"]),
        "samples": len(ps),
        "position": [float(v) for v in pos],
        "quaternion_xyzw": [float(v) for v in q],
        "normal": [float(v) for v in R.from_quat(q).as_matrix()[:, 2]],
        "consistency_pos_med_mm": float(np.median(spread)),
        "consistency_pos_max_mm": float(spread.max()),
        "range_m": [float(min(rngs)), float(max(rngs))],
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=DEFAULT_CALIB,
                    help="wrist hand-eye result (for tool0->optical extrinsic)")
    ap.add_argument("--samples", nargs="+", default=DEFAULT_SAMPLES,
                    help="per-tag hand-eye sample yamls")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--check", action="store_true", help="print, do not write")
    a = ap.parse_args()

    calib = yaml.safe_load(open(a.calib))
    opt = calib["tool0_to_camera_color_optical_frame"]
    T_tool_opt = T_from_xyz_quat(opt["xyz"], opt["quaternion_xyzw"])

    tags = {}
    for sp in a.samples:
        rec = derive_one(sp, T_tool_opt)
        tags[rec["tag_id"]] = rec
        print(f'tag{rec["tag_id"]}: n={rec["samples"]} '
              f'pos={np.round(rec["position"], 4)} '
              f'normal={np.round(rec["normal"], 3)} '
              f'consistency med={rec["consistency_pos_med_mm"]:.1f}mm '
              f'max={rec["consistency_pos_max_mm"]:.1f}mm')

    # A tag-to-tag spacing that reproduces the validated scale (622 mm tape,
    # 1.87 mm error on 2026-08-27) is the independent sanity check on this map.
    if len(tags) >= 2:
        ids = sorted(tags)
        p0 = np.array(tags[ids[0]]["position"])
        p1 = np.array(tags[ids[1]]["position"])
        print(f"tag{ids[0]}-tag{ids[1]} spacing: "
              f"{np.linalg.norm(p0 - p1) * 1e3:.1f} mm")

    out = {
        "frame_id": "base_link",
        "derived": dt.datetime.now().isoformat(timespec="seconds"),
        "dictionary": "apriltag_36h11",
        "provenance": {
            "method": "T_base_tag = T_base_tool @ T_tool0_optical @ T_cam_tag, "
                      "per wrist hand-eye view; translation median, rotation "
                      "Markley quaternion mean",
            "wrist_calib": os.path.basename(a.calib),
            "samples": [os.path.basename(s) for s in a.samples],
            "absolute_accuracy_note":
                "positions inherit the wrist hand-eye budget (cross-tag 8.4 mm / "
                "1.06 deg; validated 1.87 mm scale, 0.68 mm table plane). The "
                "per-tag consistency figures are the INTERNAL fit spread, not the "
                "absolute error.",
            "placement_note":
                "tags were re-placed between 2026-08-27 and 2026-09-03; these are "
                "the 2026-09-03 positions (they match human-safety-s1 report tag1 "
                "(-0.506,-1.058,-0.057)). Re-derive if the tags are moved again.",
        },
        "tags": {int(k): tags[k] for k in sorted(tags)},
    }
    if a.check:
        print("\n--check: not writing")
        return 0
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
