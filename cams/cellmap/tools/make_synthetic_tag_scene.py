#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Render synthetic colour images of the cell's real AprilTags as seen by a fixed
camera at a KNOWN base_link pose, by warping the printed tag PNG assets
(tags/apriltag36h11-id*-120mm.png) through the projective geometry.

Used to (a) unit-test the full detect->solve->average->write path of
calibrate_static_camera.py offline with ground truth, and (b) generate the
worked example in the calibration report when no live camera has a mapped tag in
view. The camera pose it renders from is the ground truth the offline solve must
recover.

  python3 tools/make_synthetic_tag_scene.py --out /tmp/scene --n 12
"""
import argparse
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import staticcam as sc  # noqa: E402

TAGMAP = os.path.join(HERE, "tags", "tag_world_poses.yaml")
TAG_PNG = {0: os.path.join(HERE, "tags", "apriltag36h11-id0-120mm.png"),
           1: os.path.join(HERE, "tags", "apriltag36h11-id1-120mm.png")}


def look_at(eye, target, up=(0, 0, 1)):
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = eye
    return T


def _tag_core(tid):
    """Crop the printable-page PNG to the tag (with a quiet-zone margin) and return
    (crop_bgr, src_corners) where src_corners are the tag's OUTER BLACK BORDER
    corners in crop coordinates, found by detecting the tag in the PNG itself — so
    they are exactly the corners cv2.aruco returns and the 120 mm physical extent
    maps to them. (The naive black-pixel bbox is contaminated by page crop marks.)"""
    tag = cv2.imread(TAG_PNG[tid])
    if tag is None:
        raise SystemExit(f"missing tag asset {TAG_PNG[tid]}")
    det = sc.detect_tags(cv2.cvtColor(tag, cv2.COLOR_BGR2GRAY))
    if tid not in det:
        raise SystemExit(f"could not detect tag {tid} in its own PNG")
    c = det[tid]
    x0, y0 = c[:, 0].min(), c[:, 1].min()
    x1, y1 = c[:, 0].max(), c[:, 1].max()
    mx, my = 0.3 * (x1 - x0), 0.3 * (y1 - y0)     # quiet-zone margin
    cx0, cy0 = int(max(0, x0 - mx)), int(max(0, y0 - my))
    cx1, cy1 = int(min(tag.shape[1], x1 + mx)), int(min(tag.shape[0], y1 + my))
    crop = tag[cy0:cy1, cx0:cx1].copy()
    return crop, (c - np.array([cx0, cy0], float)).astype(np.float32)


def render(T_base_cam, K, W, H, tagmap, noise_px=0.0, seed=0):
    rng = np.random.default_rng(seed)
    canvas = np.full((H, W, 3), 210, np.uint8)   # light-grey background
    T_cam_base = sc.invert_T(T_base_cam)
    # far tags first so nearer ones overwrite (rough painter's order by range)
    order = sorted(tagmap, key=lambda t: -np.linalg.norm(
        T_base_cam[:3, 3] - tagmap[t].T_base_tag[:3, 3]))
    for tid in order:
        tw = tagmap[tid]
        img_pts = sc.project_points(T_cam_base, sc.tag_corners_base(tw.T_base_tag, tw.size),
                                    K).astype(np.float32)
        if noise_px:
            img_pts = img_pts + rng.normal(0, noise_px, img_pts.shape).astype(np.float32)
        crop, src = _tag_core(tid)
        # map the PNG tag's detected black-border corners onto the projected 120 mm
        # corners; warping the whole crop carries the white quiet zone with it.
        Hmat = cv2.getPerspectiveTransform(src, img_pts)
        ph, pw = crop.shape[:2]
        warp = cv2.warpPerspective(crop, Hmat, (W, H))
        mask = cv2.warpPerspective(np.full((ph, pw), 255, np.uint8), Hmat, (W, H))
        canvas[mask > 0] = warp[mask > 0]
    return canvas


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output folder")
    ap.add_argument("--n", type=int, default=12, help="frames (temporal noise reps)")
    ap.add_argument("--noise-px", type=float, default=0.3)
    ap.add_argument("--eye", default="-0.2,0.55,0.85", help="camera xyz in base_link")
    ap.add_argument("--target", default="-0.2,-1.058,-0.057")
    ap.add_argument("--fx", type=float, default=615.0)
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    a = ap.parse_args()

    eye = [float(v) for v in a.eye.split(",")]
    target = [float(v) for v in a.target.split(",")]
    K = np.array([[a.fx, 0, a.width/2], [0, a.fx, a.height/2], [0, 0, 1.0]])
    tagmap = sc.load_tag_map(TAGMAP)
    T = look_at(eye, target)

    os.makedirs(a.out, exist_ok=True)
    for i in range(a.n):
        img = render(T, K, a.width, a.height, tagmap, noise_px=a.noise_px, seed=i)
        cv2.imwrite(os.path.join(a.out, f"frame_{i:03d}.png"), img)
    xyz, quat = sc.xyz_quat_from_T(T)
    gt = {"ground_truth_base_link_to_camera": {
              "xyz": xyz, "quaternion_xyzw": quat,
              "rpy_deg": [float(v) for v in R.from_quat(quat).as_euler("xyz", degrees=True)]},
          "camera_matrix": K.reshape(-1).tolist(), "dist_coeffs": [0.0]*5,
          "width": a.width, "height": a.height}
    with open(os.path.join(a.out, "ground_truth.yaml"), "w") as f:
        yaml.safe_dump(gt, f, sort_keys=False)
    with open(os.path.join(a.out, "intrinsics.yaml"), "w") as f:
        yaml.safe_dump({"camera_matrix": K.reshape(-1).tolist(),
                        "dist_coeffs": [0.0]*5,
                        "width": a.width, "height": a.height,
                        "source": "synthetic"}, f, sort_keys=False)
    print(f"wrote {a.n} frames + ground_truth.yaml + intrinsics.yaml to {a.out}")
    print(f"GT base_link->camera xyz {np.round(xyz,4)}")


if __name__ == "__main__":
    sys.exit(main())
