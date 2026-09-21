# SPDX-License-Identifier: Apache-2.0
"""Pure camera-pose maths shared by the Isaac scene, the TF generator and the
offline analysis — no ROS, no Isaac, no torch, so it is unit-tested offline.

One source of truth ties three things together that MUST agree or the whole test
is meaningless:

  * where the Isaac ``UsdGeom.Camera`` prim is placed (a USD 4x4, row-major, the
    camera looking down its local -Z with +Y up / +X right);
  * the ``base_link -> <optical_frame>`` static TF the detector node looks up
    (ROS optical convention: +X right, +Y down, +Z forward into the scene);
  * the pin-hole projection the analysis uses to decide whether the ground-truth
    person is inside a camera's field of view.

Because all three are derived here from the same (eye, target) look-at, the TF we
hand the detector is exactly the pose Isaac renders from. The base frame is the
world origin in this scene (the robot base sits at the world origin), so
``base_link`` and Isaac ``world`` coincide and no extra base<-world hop is needed.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("cannot normalize a zero-length vector")
    return v / n


def look_at_basis(eye, target, world_up=(0.0, 0.0, 1.0)
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the USD camera basis (x_right, y_up, z_back) for a look-at.

    The USD camera looks down its local -Z, so local +Z points back from the
    target toward the eye. Matches the look-at used in the shelfbot reference
    ``scene_cameras._build_arm_overview_camera``.
    """
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    fwd = _normalize(target - eye)
    z = -fwd                                  # camera local +Z = back
    x = _normalize(np.cross(np.asarray(world_up, dtype=float), z))
    y = np.cross(z, x)                        # already unit (orthonormal)
    return x, y, z


def usd_camera_matrix(eye, target, world_up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """4x4 row-major USD transform for a look-at camera prim.

    Rows 0..2 are the camera x/y/z basis vectors in world coords, row 3 is the
    eye (translation) — the layout ``UsdGeom.Xformable(cam).AddTransformOp()``
    expects (GfMatrix4d is row-major with basis vectors as rows).
    """
    x, y, z = look_at_basis(eye, target, world_up)
    eye = np.asarray(eye, dtype=float)
    m = np.eye(4, dtype=float)
    m[0, :3] = x
    m[1, :3] = y
    m[2, :3] = z
    m[3, :3] = eye
    return m


def base_to_optical(eye, target, world_up=(0.0, 0.0, 1.0)
                    ) -> Tuple[np.ndarray, np.ndarray]:
    """``base_link -> optical_frame`` as (translation[3], quaternion_xyzw[4]).

    The ROS optical frame is derived from the USD camera basis: optical +X =
    camera right (x), optical +Y = camera down (-y), optical +Z = forward (-z).
    Returns the translation (the eye) and the rotation as a quaternion in
    (x, y, z, w) order — the order the human_safety camera_poses loader and
    tf2 expect.
    """
    x, y, z = look_at_basis(eye, target, world_up)
    r = np.column_stack([x, -y, -z])          # columns = optical axes in base
    return np.asarray(eye, dtype=float), rotation_matrix_to_quat_xyzw(r)


def rotation_matrix_to_quat_xyzw(r: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w)."""
    r = np.asarray(r, dtype=float)
    t = np.trace(r)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def intrinsics(focal_mm: float, h_aperture_mm: float, v_aperture_mm: float,
               width: int, height: int) -> Tuple[float, float, float, float]:
    """(fx, fy, cx, cy) in pixels from USD camera focal length + apertures.

    Isaac derives the published CameraInfo K the same way: fx = width * f /
    horizontal_aperture, cx = width / 2 (principal point centred).
    """
    fx = width * focal_mm / h_aperture_mm
    fy = height * focal_mm / v_aperture_mm
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def project_base_point(point, eye, target, K, width, height,
                       world_up=(0.0, 0.0, 1.0)
                       ) -> Optional[Tuple[float, float, float]]:
    """Project a base-frame point into a camera; (u, v, depth_m) or None.

    ``None`` when the point is behind the camera. The caller decides whether
    (u, v) falls inside [0,width) x [0,height) for an in-FOV test. ``depth_m`` is
    the planar Z depth along the optical axis (what Isaac's ``depth`` annotator
    and the detector's deprojection use).
    """
    x, y, z = look_at_basis(eye, target, world_up)
    r = np.column_stack([x, -y, -z])          # base<-optical rotation
    p = np.asarray(point, dtype=float) - np.asarray(eye, dtype=float)
    p_opt = r.T @ p                            # optical<-base
    depth = float(p_opt[2])
    if depth <= 1e-6:
        return None
    fx, fy, cx, cy = K
    u = fx * p_opt[0] / depth + cx
    v = fy * p_opt[1] / depth + cy
    return float(u), float(v), depth


def in_fov(point, eye, target, K, width, height, margin=0.0,
           world_up=(0.0, 0.0, 1.0)) -> bool:
    """True if a base-frame point projects inside the image (with pixel margin)."""
    pr = project_base_point(point, eye, target, K, width, height, world_up)
    if pr is None:
        return False
    u, v, _ = pr
    return (margin <= u < width - margin) and (margin <= v < height - margin)


@dataclass(frozen=True)
class SimCamera:
    """One simulated camera: look-at pose + intrinsics + names. The single source
    of truth for Isaac placement, the detector TF, and the analysis projection."""
    role: str
    optical_frame: str
    eye: Tuple[float, float, float]
    target: Tuple[float, float, float]
    width: int = 640
    height: int = 480
    focal_mm: float = 24.0
    h_aperture_mm: float = 33.0            # ~= 69 deg HFOV at f=24 (RealSense-like)
    v_aperture_mm: float = 24.75           # square pixels for 640x480

    def K(self) -> Tuple[float, float, float, float]:
        return intrinsics(self.focal_mm, self.h_aperture_mm,
                          self.v_aperture_mm, self.width, self.height)

    def usd_matrix(self) -> np.ndarray:
        return usd_camera_matrix(self.eye, self.target)

    def base_to_optical(self) -> Tuple[np.ndarray, np.ndarray]:
        return base_to_optical(self.eye, self.target)

    def in_fov(self, point, margin=0.0) -> bool:
        return in_fov(point, self.eye, self.target, self.K(),
                      self.width, self.height, margin)

    def project(self, point):
        return project_base_point(point, self.eye, self.target, self.K(),
                                  self.width, self.height)
