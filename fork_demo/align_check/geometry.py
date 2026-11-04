"""Pure projection / pose geometry for the pre-contact alignment check.

No ROS, no OpenCV, no I/O -- just numpy so the maths is unit-testable offline.

Frames
------
* tool0            : UR10 flange (ISO), gripper frame == tool0 (identity).
* optical          : wrist_color_upright_optical_frame (hand-eye child).
* base_link        : robot base.
* image            : pixels (u right, v down), pinhole K.

The two forks are rigid in tool0: centre-lines at tool X = +/- spacing/2,
tool Y = center_y, running tool Z from root_z to tip_z. The wrist camera is
rigid in tool0 too (hand-eye tool0->optical), so the tines' *image* positions
are a constant of (hand-eye, K) and do not depend on the arm pose.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------
# Pose helpers
# --------------------------------------------------------------------------
def quat_xyzw_to_matrix(q) -> np.ndarray:
    """Unit quaternion (x, y, z, w) -> 3x3 rotation."""
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_to_matrix(xyz, quat_xyzw) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = quat_xyzw_to_matrix(quat_xyzw)
    m[:3, 3] = xyz
    return m


def invert_transform(t: np.ndarray) -> np.ndarray:
    r = t[:3, :3]
    p = t[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = r.T
    inv[:3, 3] = -r.T @ p
    return inv


def transform_points(t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply 4x4 transform to an (N,3) array of points -> (N,3)."""
    pts = np.atleast_2d(np.asarray(pts, dtype=float))
    h = np.hstack([pts, np.ones((pts.shape[0], 1))])
    return (t @ h.T).T[:, :3]


def rotation_angle_deg(r: np.ndarray) -> float:
    """Geodesic angle of a 3x3 rotation, degrees."""
    c = (np.trace(r) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return float(np.degrees(np.arccos(c)))


def yaw_about_z_deg(r: np.ndarray) -> float:
    """Yaw (rotation about Z) extracted from a 3x3 rotation, degrees."""
    return float(np.degrees(np.arctan2(r[1, 0], r[0, 0])))


# --------------------------------------------------------------------------
# Pinhole projection
# --------------------------------------------------------------------------
def make_K(fx, fy, cx, cy) -> np.ndarray:
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)


def project_points(k: np.ndarray, t_cam_from_world: np.ndarray, pts_world: np.ndarray):
    """Project world points into pixels.

    Returns (uv, valid) where uv is (N,2) and valid marks points in front of
    the camera (optical +Z). Points behind the camera are returned but invalid.
    """
    pts_cam = transform_points(t_cam_from_world, pts_world)
    z = pts_cam[:, 2]
    valid = z > 1e-6
    zc = np.where(valid, z, 1.0)
    x = pts_cam[:, 0] / zc
    y = pts_cam[:, 1] / zc
    u = k[0, 0] * x + k[0, 2]
    v = k[1, 1] * y + k[1, 2]
    return np.stack([u, v], axis=1), valid


# --------------------------------------------------------------------------
# Fork tine geometry (in tool0)
# --------------------------------------------------------------------------
@dataclass
class TineGeometry:
    spacing_m: float = 0.12     # between the two tine centre-lines (tool X)
    width_m: float = 0.04       # lateral width of each tine (tool X)
    thickness_m: float = 0.02   # vertical thickness (tool Y)
    center_y_m: float = -0.038  # tine centre-line on tool Y
    root_z_m: float = 0.047     # tine root along tool Z
    tip_z_m: float = 0.427      # tine tip along tool Z

    def centreline_points(self, side: str, n: int = 2) -> np.ndarray:
        """Sample points along one tine centre-line in tool0 (N,3).

        side: 'left' (tool X = -spacing/2) or 'right' (tool X = +spacing/2).
        """
        x = -self.spacing_m / 2.0 if side == "left" else self.spacing_m / 2.0
        zs = np.linspace(self.root_z_m, self.tip_z_m, n)
        return np.array([[x, self.center_y_m, z] for z in zs])

    def tip_point(self, side: str) -> np.ndarray:
        x = -self.spacing_m / 2.0 if side == "left" else self.spacing_m / 2.0
        return np.array([x, self.center_y_m, self.tip_z_m])

    def insertion_axis_point(self, side: str) -> np.ndarray:
        """A point one metre further along the insertion (tool +Z) from the tip."""
        p = self.tip_point(side)
        return p + np.array([0.0, 0.0, 1.0])


# --------------------------------------------------------------------------
# Shelf alignment: projected tines vs detected strips, in the image
# --------------------------------------------------------------------------
def project_tines(k: np.ndarray, t_optical_from_tool0: np.ndarray, tines: TineGeometry):
    """Project both tine tips into the wrist image.

    Returns dict side -> {"tip_uv": (2,), "axis_uv": (2,)} where axis_uv is the
    projection of the insertion-axis point (used for the yaw reference line).
    """
    out = {}
    for side in ("left", "right"):
        tip = tines.tip_point(side)[None, :]
        axis = tines.insertion_axis_point(side)[None, :]
        tip_uv, _ = project_points(k, t_optical_from_tool0, tip)
        axis_uv, _ = project_points(k, t_optical_from_tool0, axis)
        out[side] = {"tip_uv": tip_uv[0], "axis_uv": axis_uv[0]}
    return out


def pixels_per_metre(k: np.ndarray, depth_m: float) -> float:
    """Lateral image scale (px per metre) of a fronto-parallel plane at depth."""
    return float(k[0, 0]) / float(depth_m)


def backproject_pixel(k: np.ndarray, uv, depth_m: float) -> np.ndarray:
    """Pixel (u, v) at optical depth Z -> 3D point in the camera optical frame."""
    u, v = float(uv[0]), float(uv[1])
    x = (u - k[0, 2]) / k[0, 0] * depth_m
    y = (v - k[1, 2]) / k[1, 1] * depth_m
    return np.array([x, y, depth_m])


def line_angle_deg(p0, p1) -> float:
    """Angle of the directed segment p0->p1 in image coords, degrees (-180,180]."""
    d = np.asarray(p1) - np.asarray(p0)
    return float(np.degrees(np.arctan2(d[1], d[0])))


def signed_angle_diff_deg(a: float, b: float) -> float:
    """a - b wrapped to (-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def compare_place_to_tag(t_base_tag, taught_xy, taught_yaw_deg, offset_xy):
    """Planar (base XY + yaw-about-Z) offset of a taught place pose from the
    tag-derived dock target. Pure; no I/O.

    Returns (dx_mm, dy_mm, dyaw_deg, tag_xy, tag_yaw_deg, target_xy).
    dx/dy = taught - target (what the taught pose is OFF the tag target by).
    """
    t_base_tag = np.asarray(t_base_tag, dtype=float)
    tag_xy = t_base_tag[:2, 3]
    tag_yaw = yaw_about_z_deg(t_base_tag[:3, :3])
    target_xy = tag_xy + np.asarray(offset_xy, dtype=float)
    dx_mm = float((taught_xy[0] - target_xy[0]) * 1000.0)
    dy_mm = float((taught_xy[1] - target_xy[1]) * 1000.0)
    dyaw = 0.0 if taught_yaw_deg is None else float(signed_angle_diff_deg(taught_yaw_deg, tag_yaw))
    return dx_mm, dy_mm, dyaw, tag_xy, tag_yaw, target_xy
