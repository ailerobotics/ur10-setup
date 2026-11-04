#!/usr/bin/env python3
"""Generate the offline proof artifacts for the alignment check.

Produces, into --out-dir (default the firstmate data dir, NOT git):
  * shelf_detector_guidephoto.png  -- REAL yellow-strip detection on the guide photo
  * table_tag_guidephoto.png       -- REAL AprilTag id1 detection on the guide photo
  * shelf_synthetic_recovery.png   -- synthetic wrist scene, injected vs recovered
  * table_synthetic_recovery.png   -- synthetic look-down scene, injected vs recovered
  * *_verdict_*.json               -- example verdict files (the contact-guard contract)

Copy the small (<2 MB) annotated PNGs that belong in the PR with --repo-proof.
Nothing here commands the robot or a camera.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from fork_demo.align_check import align_check, geometry, shelf_detect, tag_detect, verdict_io  # noqa: E402
from fork_demo.align_check import synth  # noqa: E402
from fork_demo.align_check.geometry import invert_transform  # noqa: E402

GUIDES = "/home/manishsaini/firstmate/data/ur10-fork-pick-real-v1/guides"
DEFAULT_OUT = "/home/manishsaini/firstmate/data/ur10-pick-align-check-v1/proof"


def _caption(img, lines, org=(20, 40)):
    y = org[1]
    for ln in lines:
        cv2.putText(img, ln, (org[0], y), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(img, ln, (org[0], y), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
        y += 44
    return img


def _downscale(img, w=1200):
    if img.shape[1] <= w:
        return img
    h = int(img.shape[0] * w / img.shape[1])
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


def _save_photo(img, out_dir, repo_proof, name, quality=85):
    """Write a photographic annotation as JPG (keeps the PR copy < 2 MB)."""
    enc = [cv2.IMWRITE_JPEG_QUALITY, quality]
    cv2.imwrite(os.path.join(out_dir, name), img, enc)
    if repo_proof:
        cv2.imwrite(os.path.join(repo_proof, name), img, enc)


def shelf_guidephoto(out_dir, repo_proof):
    bgr = cv2.imread(f"{GUIDES}/shelf-yellow-tape-guides.jpg")
    det = shelf_detect.detect_strips(bgr)
    ann = shelf_detect.annotate(bgr, det)  # detector only -- no projected tines on a phone photo
    ann = _downscale(ann)
    _caption(ann, [
        "SHELF detector on the real guide photo (phone, NOT the wrist camera)",
        f"two yellow strips found: {'YES' if det.ok else 'NO'}; rail angle {det.rail_angle_deg:+.1f} deg",
        "centres mark each channel; mm verdict needs the calibrated wrist frame",
    ])
    _save_photo(ann, out_dir, repo_proof, "shelf_detector_guidephoto.jpg")
    return det


def table_guidephoto(out_dir, repo_proof):
    bgr = cv2.imread(f"{GUIDES}/table-apriltag-id1-dock.jpg")
    det = tag_detect.detect_tag(bgr, 1)
    ann = tag_detect.annotate(bgr, det)
    ann = _downscale(ann)
    _caption(ann, [
        "TABLE detector on the real guide photo (phone, NOT a calibrated camera)",
        f"AprilTag 36h11 id 1 found: {'YES' if det.found else 'NO'}; "
        f"mean side {det.mean_side_px*ann.shape[1]/bgr.shape[1]:.0f} px",
        "dx/dy/dyaw vs taught needs the wrist/table pose -> see synthetic proof",
    ])
    _save_photo(ann, out_dir, repo_proof, "table_tag_guidephoto.jpg")
    return det


def shelf_synthetic(out_dir, repo_proof):
    params = align_check.load_params()
    tg = align_check.tine_geometry(params)
    K = geometry.make_K(915, 915, 640, 360)
    panels = []
    rows = []
    for lat, yaw in [(0.0, 0.0), (12.0, 0.0), (-6.0, 4.0)]:
        bgr, depth, gt = synth.render_shelf_scene(tg, lateral_err_mm=lat, yaw_deg=yaw, plane_z_m=0.5, K=K)
        v, ann, info = align_check.align_shelf(bgr, params, depth=depth, K=K)
        _caption(ann, [
            f"inject lateral {lat:+.1f} mm, yaw {yaw:+.1f} deg",
            f"recover dx {v.dx_mm:+.2f} mm, dyaw {v.dyaw_deg:+.2f} deg  [{'GO' if v.ok else 'NO-GO'}]",
        ])
        panels.append(ann)
        rows.append((lat, yaw, v.dx_mm, v.dyaw_deg, v.ok))
    montage = np.vstack(panels)
    p = os.path.join(out_dir, "shelf_synthetic_recovery.png")
    cv2.imwrite(p, _downscale(montage, 900))
    if repo_proof:
        cv2.imwrite(os.path.join(repo_proof, "shelf_synthetic_recovery.png"), _downscale(montage, 900))
    # Example verdict files (GO and NO-GO).
    for tag, lat, yaw in [("go", 0.0, 0.0), ("nogo", 12.0, 0.0)]:
        bgr, depth, _ = synth.render_shelf_scene(tg, lateral_err_mm=lat, yaw_deg=yaw, plane_z_m=0.5, K=K)
        v, _, _ = align_check.align_shelf(bgr, params, depth=depth, K=K)
        v.image_stamp = f"synthetic-shelf-{tag}"
        verdict_io.write_verdict(v, os.path.join(out_dir, f"shelf_verdict_{tag}.json"))
        if repo_proof:
            verdict_io.write_verdict(v, os.path.join(repo_proof, f"shelf_verdict_{tag}.json"))
    return rows


def table_synthetic(out_dir, repo_proof):
    params = align_check.load_params()
    K = geometry.make_K(915, 915, 640, 360)
    place_xy = [-0.5165, -1.2173]
    t_base_opt = synth.look_down_camera(place_xy, height_m=0.6, yaw_deg=0.0)
    rows = []
    panels = []
    for tag_yaw, inj in [(15.0, (0, 0, 0)), (15.0, (5, -3, 2)), (-10.0, (-4, 6, -1.5))]:
        th = np.radians(tag_yaw)
        r = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
        t_base_tag = np.eye(4)
        t_base_tag[:3, :3] = r
        t_base_tag[:3, 3] = [place_xy[0], place_xy[1], 0.05]
        t_opt_tag = invert_transform(t_base_opt) @ t_base_tag
        bgr = synth.render_tag_at(t_opt_tag, K=K)
        det = tag_detect.detect_tag(bgr, 1)
        t_bt = t_base_opt @ tag_detect.tag_matrix_in_camera(tag_detect.solve_tag_pose(det, K, 0.12))
        tag_xy = t_bt[:2, 3]
        tag_yaw_rec = geometry.yaw_about_z_deg(t_bt[:3, :3])
        taught = {"xy": [tag_xy[0] + inj[0] / 1000.0, tag_xy[1] + inj[1] / 1000.0],
                  "yaw_deg": tag_yaw_rec + inj[2]}
        v, ann, _ = align_check.align_table(bgr, params, K=K, t_base_optical=t_base_opt, taught=taught)
        _caption(ann, [
            f"look-down wrist; inject dx {inj[0]:+d} dy {inj[1]:+d} mm, dyaw {inj[2]:+.1f} deg",
            f"recover dx {v.dx_mm:+.2f} dy {v.dy_mm:+.2f} mm, dyaw {v.dyaw_deg:+.2f} deg  "
            f"[{'GO' if v.ok else 'NO-GO'}]",
        ])
        panels.append(ann)
        rows.append((inj, v.dx_mm, v.dy_mm, v.dyaw_deg, v.ok))
        if inj == (0, 0, 0):
            v.image_stamp = "synthetic-table-go"
            verdict_io.write_verdict(v, os.path.join(out_dir, "table_verdict_go.json"))
            if repo_proof:
                verdict_io.write_verdict(v, os.path.join(repo_proof, "table_verdict_go.json"))
        if inj == (5, -3, 2):
            v.image_stamp = "synthetic-table-nogo"
            verdict_io.write_verdict(v, os.path.join(out_dir, "table_verdict_nogo.json"))
            if repo_proof:
                verdict_io.write_verdict(v, os.path.join(repo_proof, "table_verdict_nogo.json"))
    montage = np.vstack(panels)
    cv2.imwrite(os.path.join(out_dir, "table_synthetic_recovery.png"), _downscale(montage, 900))
    if repo_proof:
        cv2.imwrite(os.path.join(repo_proof, "table_synthetic_recovery.png"), _downscale(montage, 900))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=DEFAULT_OUT)
    ap.add_argument("--repo-proof", default=None, help="also copy small PNGs/JSONs into this repo dir")
    args = ap.parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    if args.repo_proof:
        os.makedirs(args.repo_proof, exist_ok=True)

    sd = shelf_guidephoto(args.out_dir, args.repo_proof)
    td = table_guidephoto(args.out_dir, args.repo_proof)
    sr = shelf_synthetic(args.out_dir, args.repo_proof)
    tr = table_synthetic(args.out_dir, args.repo_proof)

    print("SHELF guide-photo detection:", "2 strips" if sd.ok else "FAILED",
          f"rail {sd.rail_angle_deg:+.1f} deg")
    print("TABLE guide-photo detection:", "id1 found" if td.found else "FAILED")
    print("SHELF synthetic (inject lat,yaw -> recover dx,dyaw, ok):")
    for lat, yaw, dx, dyaw, ok in sr:
        print(f"   ({lat:+.1f},{yaw:+.1f}) -> ({dx:+.2f},{dyaw:+.2f}) {'GO' if ok else 'NO-GO'}")
    print("TABLE synthetic (inject dx,dy,dyaw -> recover, ok):")
    for inj, dx, dy, dyaw, ok in tr:
        print(f"   {inj} -> ({dx:+.2f},{dy:+.2f},{dyaw:+.2f}) {'GO' if ok else 'NO-GO'}")
    print("artifacts in", args.out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
