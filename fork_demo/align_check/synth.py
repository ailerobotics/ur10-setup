"""Synthetic wrist-view scene generator -- ground-truth proof of the geometry.

The real guide photos are phone snaps, not the calibrated wrist camera, so they
can only prove the *detectors*. To prove the full shelf pipeline end-to-end we
render a wrist-resolution image with the two yellow channel strips placed at
KNOWN tool0 channel positions (a chosen lateral error + yaw), using the real
hand-eye and the nominal K, plus a matching depth map. align_shelf() must then
recover the injected lateral error and yaw -- see tests/test_geometry.py and
tools/make_proof.py.

This is a test/proof fixture generator, not part of the live path.
"""

from __future__ import annotations

import cv2
import numpy as np

from . import calib, geometry
from .geometry import TineGeometry


import cv2.aruco as _aruco

from .geometry import pose_to_matrix


def _rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def render_tag_at(T_opt_tag, *, tag_id=1, tag_size_m=0.12, K=None, width=1280, height=720):
    """Render a 36h11 tag at an explicit optical-frame pose T_opt_tag (4x4).

    The printed face (+Z) is drawn toward the camera; the marker bitmap is warped
    onto the exactly-projected tag corners, so detect_tag + solve_tag_pose recover
    T_opt_tag. Mild optical blur mimics a real lens (keeps sub-pixel refinement
    stable). Returns bgr.
    """
    if K is None:
        K = geometry.make_K(915.0, 915.0, width / 2.0, height / 2.0)
    T_opt_tag = np.asarray(T_opt_tag, dtype=float)
    h = tag_size_m / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)  # TL,TR,BR,BL
    uv, _ = geometry.project_points(K, np.eye(4), geometry.transform_points(T_opt_tag, obj))

    dict_ = _aruco.Dictionary_get(_aruco.DICT_APRILTAG_36h11)
    side = 400
    marker = _aruco.drawMarker(dict_, tag_id, side)
    quiet = side // 2
    canvas_tag = np.full((side + 2 * quiet, side + 2 * quiet), 255, np.uint8)
    canvas_tag[quiet:quiet + side, quiet:quiet + side] = marker
    s = side + 2 * quiet
    src = np.array([[quiet, quiet], [quiet + side, quiet],
                    [quiet + side, quiet + side], [quiet, quiet + side]], dtype=np.float32)
    Hmat, _ = cv2.findHomography(src, uv.astype(np.float32))
    bgr = np.full((height, width, 3), 150, np.uint8)
    tag_bgr = cv2.cvtColor(canvas_tag, cv2.COLOR_GRAY2BGR)
    warped = cv2.warpPerspective(tag_bgr, Hmat, (width, height), flags=cv2.INTER_NEAREST,
                                 borderMode=cv2.BORDER_TRANSPARENT)
    full = np.full((s, s, 3), 255, np.uint8)
    mask = cv2.warpPerspective(full, Hmat, (width, height), flags=cv2.INTER_NEAREST)[:, :, 0]
    bgr[mask > 0] = warped[mask > 0]
    bgr = cv2.GaussianBlur(bgr, (0, 0), 1.0)
    return bgr


def render_tag_scene(
    *,
    tag_id: int = 1,
    tag_size_m: float = 0.12,
    distance_m: float = 0.5,
    tag_yaw_deg: float = 0.0,
    tag_offset_xy_m=(0.0, 0.0),
    K: np.ndarray | None = None,
    width: int = 1280,
    height: int = 720,
):
    """Camera-frame proof: render a tag facing the lens at a known optical pose.

    Returns (bgr, T_optical_tag). The printed face points back at the camera
    (tag +Z -> optical -Z via Rx(pi)); yaw is about the tag normal.
    """
    _rx_pi = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
    T_opt_tag = np.eye(4)
    T_opt_tag[:3, :3] = _rot_z(np.radians(tag_yaw_deg)) @ _rx_pi
    T_opt_tag[:3, 3] = [tag_offset_xy_m[0], tag_offset_xy_m[1], distance_m]
    bgr = render_tag_at(T_opt_tag, tag_id=tag_id, tag_size_m=tag_size_m, K=K,
                        width=width, height=height)
    return bgr, T_opt_tag


def look_down_camera(base_xy, height_m=0.6, yaw_deg=0.0):
    """A synthetic T_base_optical for a wrist camera looking straight down at
    base_xy from height_m. Optical +Z points down (-base Z); optical +X along
    base +X rotated by yaw. Physically consistent for the table place proof.
    """
    yaw = np.radians(yaw_deg)
    cz, sz = np.cos(yaw), np.sin(yaw)
    # optical axes in base: Z_opt = -Zbase (down); X_opt = base X (rot yaw);
    # Y_opt = Z x X.
    x_opt = np.array([cz, sz, 0.0])
    z_opt = np.array([0.0, 0.0, -1.0])
    y_opt = np.cross(z_opt, x_opt)
    R = np.column_stack([x_opt, y_opt, z_opt])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [base_xy[0], base_xy[1], height_m]
    return T


def render_shelf_scene(
    tines: TineGeometry,
    *,
    lateral_err_mm: float = 0.0,
    yaw_deg: float = 0.0,
    plane_z_m: float = 0.5,
    handeye_path: str | None = None,
    K: np.ndarray | None = None,
    width: int = 1280,
    height: int = 720,
    strip_half_m: float = 0.02,
):
    """Render (bgr, depth_m, ground_truth).

    The two channel centres start at the ideal tine positions (tool X = +/- spacing/2,
    Y = center_y, Z = plane_z), are shifted laterally by ``lateral_err_mm`` (both
    channels, +X) and the pair is yawed by ``yaw_deg`` about tool Y through the dock
    centre. Strips are drawn as filled yellow quads; a depth map carries each strip's
    true optical depth so align_shelf recovers the offset exactly.
    """
    t_tool0_optical, t_optical_tool0, _ = calib.load_handeye(handeye_path or calib.DEFAULT_HANDEYE)
    if K is None:
        K = geometry.make_K(915.0, 915.0, width / 2.0, height / 2.0)

    cy = tines.center_y_m
    centre = np.array([0.0, cy, plane_z_m])
    err = lateral_err_mm / 1000.0
    ry = _rot_y(np.radians(yaw_deg))
    channels = {}
    for side in ("left", "right"):
        x = -tines.spacing_m / 2.0 if side == "left" else tines.spacing_m / 2.0
        p = np.array([x + err, cy, plane_z_m])
        channels[side] = centre + ry @ (p - centre)

    bgr = np.full((height, width, 3), 40, np.uint8)  # dark background
    depth = np.zeros((height, width), np.float32)
    for side, c in channels.items():
        # Build a small square quad in tool0 around the channel centre, in the
        # plane spanned by tool X and tool Z (fronto-ish), project its corners.
        corners_tool0 = [
            c + ry @ np.array([dx, 0.0, dz])
            for dx, dz in [(-strip_half_m, -strip_half_m), (strip_half_m, -strip_half_m),
                           (strip_half_m, strip_half_m), (-strip_half_m, strip_half_m)]
        ]
        uv, _ = geometry.project_points(K, t_optical_tool0, np.array(corners_tool0))
        poly = uv.astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(bgr, [poly], (40, 200, 230))  # BGR yellow
        # Depth: fill the strip's bounding region with its optical depth.
        z_opt = float(geometry.transform_points(t_optical_tool0, c)[0][2])
        mask = np.zeros((height, width), np.uint8)
        cv2.fillPoly(mask, [poly], 255)
        depth[mask > 0] = z_opt

    gt = {
        "lateral_err_mm": lateral_err_mm,
        "yaw_deg": yaw_deg,
        "channels_tool0": {k: v.tolist() for k, v in channels.items()},
    }
    return bgr, depth, gt
