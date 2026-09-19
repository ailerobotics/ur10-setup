#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""One-time intrinsics calibration for an ORDINARY colour camera (a UVC webcam)
so it can be calibrated into base_link by ``calibrate_static_camera.py``.

A RealSense publishes factory colour intrinsics on its ``camera_info`` topic, so
it needs no this step. An ordinary webcam does not — PnP against the cell tags is
only trustworthy once the webcam's focal length, principal point and lens
distortion are known (human-safety-s1 §6, "a colour-only camera still needs its
intrinsics"). This is the standard OpenCV chessboard calibration; it writes an
intrinsics yaml in the format ``calibrate_static_camera.py --intrinsics`` reads.

Print an N x M INNER-corner chessboard (default 9x6), measure a square edge in
metres, and capture ~15-20 views at varied distances and tilts.

  # capture live from a V4L device (shows a window; SPACE grabs, q finishes):
  python3 calibrate_webcam_intrinsics.py --device 0 --cols 9 --rows 6 --square 0.025 \
      --out webcam_intr.yaml

  # or from a folder of chessboard photos:
  python3 calibrate_webcam_intrinsics.py --from-images shots/ --cols 9 --rows 6 \
      --square 0.025 --out webcam_intr.yaml
"""
import argparse
import datetime as dt
import glob
import os
import sys

import cv2
import numpy as np
import yaml


def object_grid(cols, rows, square):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * square


def find_corners(gray, cols, rows):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)


def calibrate(objpoints, imgpoints, size):
    rms, K, D, _, _ = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
    return rms, K, D


def write_intrinsics(path, K, D, size, rms, n, source):
    out = {
        "camera_matrix": [float(v) for v in np.asarray(K).reshape(-1)],
        "dist_coeffs": [float(v) for v in np.asarray(D).reshape(-1)],
        "width": int(size[0]), "height": int(size[1]),
        "rms_reproj_px": float(rms), "n_views": int(n),
        "source": source,
        "when": dt.datetime.now().isoformat(timespec="seconds"),
    }
    with open(path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"\nRMS reprojection {rms:.3f} px over {n} views  (aim < 0.5 px)")
    print(f"fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}")
    print(f"wrote {path}")


def from_images(a):
    objp = object_grid(a.cols, a.rows, a.square)
    objpoints, imgpoints, size = [], [], None
    files = sorted(glob.glob(os.path.join(a.from_images, "*.png")) +
                   glob.glob(os.path.join(a.from_images, "*.jpg")))
    if not files:
        sys.exit(f"no images in {a.from_images}")
    for fp in files:
        img = cv2.imread(fp)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        c = find_corners(gray, a.cols, a.rows)
        if c is None:
            print(f"  no chessboard: {os.path.basename(fp)}")
            continue
        objpoints.append(objp)
        imgpoints.append(c)
    if len(objpoints) < 5:
        sys.exit(f"only {len(objpoints)} usable views; need >= 5 (ideally 15-20)")
    rms, K, D = calibrate(objpoints, imgpoints, size)
    write_intrinsics(a.out, K, D, size, rms, len(objpoints),
                     f"webcam chessboard {a.cols}x{a.rows}@{a.square}m from {a.from_images}")
    return 0


def from_device(a):
    objp = object_grid(a.cols, a.rows, a.square)
    cap = cv2.VideoCapture(a.device)
    if not cap.isOpened():
        sys.exit(f"cannot open device {a.device}")
    objpoints, imgpoints, size = [], [], None
    print("SPACE: grab a view when the chessboard is found (green). q: finish.")
    try:
        while True:
            ok, img = cap.read()
            if not ok:
                break
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            size = (gray.shape[1], gray.shape[0])
            c = find_corners(gray, a.cols, a.rows)
            disp = img.copy()
            if c is not None:
                cv2.drawChessboardCorners(disp, (a.cols, a.rows), c, True)
            cv2.putText(disp, f"grabbed {len(objpoints)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("webcam intrinsics", disp)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" ") and c is not None:
                objpoints.append(objp)
                imgpoints.append(c)
                print(f"  grabbed {len(objpoints)}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
    if len(objpoints) < 5:
        sys.exit(f"only {len(objpoints)} views; need >= 5 (ideally 15-20)")
    rms, K, D = calibrate(objpoints, imgpoints, size)
    write_intrinsics(a.out, K, D, size, rms, len(objpoints),
                     f"webcam chessboard {a.cols}x{a.rows}@{a.square}m device {a.device}")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    ap.add_argument("--square", type=float, default=0.025, help="square edge (m)")
    ap.add_argument("--device", type=int, default=None, help="V4L device index")
    ap.add_argument("--from-images", default="", help="folder of chessboard photos")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    if a.from_images:
        return from_images(a)
    if a.device is not None:
        return from_device(a)
    sys.exit("give --device N (live) or --from-images DIR")


if __name__ == "__main__":
    sys.exit(main())
