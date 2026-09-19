# SPDX-License-Identifier: Apache-2.0
"""Offline test for the webcam intrinsics calibrator
(cams/cellmap/calibrate_webcam_intrinsics.py): render a chessboard through a KNOWN
pinhole camera from several poses (physically consistent projection, no
distortion), then confirm calibrateCamera recovers that intrinsic. No camera.
"""
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))     # cams/cellmap
import calibrate_webcam_intrinsics as wi  # noqa: E402

COLS, ROWS, SQUARE = 9, 6, 0.025             # inner corners, square edge (m)
W, H = 1280, 720
K_TRUE = np.array([[1000.0, 0, 640.0], [0, 1000.0, 360.0], [0, 0, 1.0]])


def _board_bitmap(scale=80):
    """A (ROWS+1)x(COLS+1)-square board bitmap with a one-square white quiet
    border, plus the object-plane coords of its outer rectangle corners."""
    nx, ny = COLS + 1, ROWS + 1
    img = np.zeros((ny * scale, nx * scale), np.uint8)
    for i in range(ny):
        for j in range(nx):
            if (i + j) % 2 == 0:
                img[i*scale:(i+1)*scale, j*scale:(j+1)*scale] = 255
    img = cv2.copyMakeBorder(img, scale, scale, scale, scale,
                             cv2.BORDER_CONSTANT, value=255)
    # object-plane extent that the bitmap (incl. border) spans, in metres:
    # inner corners are at 1..COLS * SQUARE; squares are SQUARE wide; the bitmap
    # spans from -SQUARE (quiet+first square origin) to (nx)*SQUARE + SQUARE.
    x0, y0 = -1.0 * SQUARE, -1.0 * SQUARE
    x1, y1 = (nx) * SQUARE, (ny) * SQUARE
    obj_corners = np.array([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]],
                           np.float32)
    return img, obj_corners


def _render(board, obj_corners, rvec, tvec):
    img_pts, _ = cv2.projectPoints(obj_corners, rvec, tvec, K_TRUE, np.zeros(5))
    img_pts = img_pts.reshape(-1, 2).astype(np.float32)
    bh, bw = board.shape
    src = np.array([[0, 0], [bw-1, 0], [bw-1, bh-1], [0, bh-1]], np.float32)
    Hm = cv2.getPerspectiveTransform(src, img_pts)
    warp = cv2.warpPerspective(board, Hm, (W, H), borderValue=200)
    return cv2.cvtColor(warp, cv2.COLOR_GRAY2BGR)


def test_recovers_known_intrinsics(tmp_path):
    board, obj_corners = _board_bitmap()
    out_dir = str(tmp_path / "shots")
    os.makedirs(out_dir)
    # a spread of poses: board ~0.5-0.8 m away, tilted various ways
    poses = [
        ((0.0, 0.0, 0.0), (-0.10, -0.06, 0.55)),
        ((0.3, 0.0, 0.0), (-0.10, -0.06, 0.60)),
        ((0.0, 0.3, 0.0), (-0.12, -0.06, 0.60)),
        ((-0.3, 0.0, 0.0), (-0.10, -0.05, 0.65)),
        ((0.0, -0.3, 0.0), (-0.08, -0.06, 0.65)),
        ((0.25, 0.25, 0.0), (-0.11, -0.06, 0.62)),
        ((-0.25, 0.2, 0.05), (-0.10, -0.07, 0.58)),
        ((0.2, -0.2, -0.05), (-0.09, -0.05, 0.60)),
        ((0.15, 0.15, 0.1), (-0.12, -0.05, 0.70)),
        ((-0.2, -0.15, 0.0), (-0.08, -0.07, 0.68)),
    ]
    n = 0
    for rv, tv in poses:
        img = _render(board, obj_corners, np.array(rv, float), np.array(tv, float))
        # only keep frames whose chessboard is actually detectable
        if wi.find_corners(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), COLS, ROWS) is not None:
            cv2.imwrite(os.path.join(out_dir, f"v{n:02d}.png"), img)
            n += 1
    assert n >= 6, f"only {n} renders were detectable"

    out = str(tmp_path / "intr.yaml")
    args = type("A", (), dict(cols=COLS, rows=ROWS, square=SQUARE,
                              from_images=out_dir, out=out))()
    rc = wi.from_images(args)
    assert rc == 0

    import yaml
    d = yaml.safe_load(open(out))
    K = np.array(d["camera_matrix"]).reshape(3, 3)
    assert d["rms_reproj_px"] < 1.0
    assert abs(K[0, 0] - K_TRUE[0, 0]) < 25      # fx within 2.5%
    assert abs(K[1, 1] - K_TRUE[1, 1]) < 25      # fy
    assert abs(K[0, 2] - K_TRUE[0, 2]) < 25      # cx
    assert abs(K[1, 2] - K_TRUE[1, 2]) < 25      # cy


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
