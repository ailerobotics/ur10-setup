# SPDX-License-Identifier: Apache-2.0
"""Pure synthetic-depth renderer for the nvblox dynamic-layer proofs.

No ROS, no GPU, standard library + numpy only, so the geometry is unit-tested
offline and the ROS publisher (``synth_depth_pub.py``) and the tests share ONE
code path. It renders a pinhole depth image (16UC1, millimetres, the cell's
``/camera/depth/image_mm`` convention) of a handful of axis-aligned boxes and an
optional ground/back wall, seen from a fixed camera whose pose is expressed as a
``base_link -> optical`` 4x4 transform.

Why synthetic depth at all: this task is SIMULATION ONLY and the two fixed D415
cameras are not calibrated into ``base_link`` yet, so there is no trustworthy way
to place a *recorded* obstacle at a known metric location. A synthetic scene lets
a proof assert "an obstacle at exactly (x,y,z) enters the ESDF and then decays
out of it" against ground truth. A short recorded bag is used only as an
additional, un-asserted realism replay (see ``proof/``); the pass/fail proofs run
on this renderer.

Optical-frame convention matches RealSense: x right, y down, z forward; the depth
value stored at a pixel is the point's z-coordinate in the optical frame (the
perpendicular distance to the image plane), which is what nvblox integrates.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics for the rendered depth image."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @staticmethod
    def d415_like(width: int = 640, height: int = 480) -> "Intrinsics":
        """A plausible D415 depth intrinsic at 640x480 (~69 deg HFOV). Only used
        for simulation; the real intrinsic comes from the camera's CameraInfo."""
        fx = fy = 0.5 * width / np.tan(np.radians(69.0) / 2.0)
        return Intrinsics(width, height, fx, fy, (width - 1) / 2.0, (height - 1) / 2.0)


@dataclass(frozen=True)
class Box:
    """Axis-aligned box in base_link, [min_corner, max_corner] (metres)."""

    min_corner: Tuple[float, float, float]
    max_corner: Tuple[float, float, float]
    label: str = "box"


@dataclass
class Scene:
    """A static backdrop (ground plane + optional back wall) plus dynamic boxes.

    The backdrop gives nvblox stable static geometry to fuse; the boxes are the
    obstacles a proof makes appear and disappear."""

    boxes: List[Box] = field(default_factory=list)
    ground_z_m: Optional[float] = -0.02      # thin floor just below base
    back_wall_y_m: Optional[float] = 2.4     # far wall at the shelf front
    max_range_m: float = 4.0                 # beyond this, depth is invalid (0)


def _homogeneous(t: np.ndarray) -> np.ndarray:
    return np.asarray(t, dtype=np.float64).reshape(4, 4)


def _ray_dirs_optical(intr: Intrinsics) -> np.ndarray:
    """Unit ray directions (H, W, 3) in the optical frame for every pixel."""
    us = np.arange(intr.width, dtype=np.float64)
    vs = np.arange(intr.height, dtype=np.float64)
    uu, vv = np.meshgrid(us, vs)
    x = (uu - intr.cx) / intr.fx
    y = (vv - intr.cy) / intr.fy
    z = np.ones_like(x)
    d = np.stack([x, y, z], axis=-1)
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    return d


def _slab_t_boxes(origin: np.ndarray, dirs: np.ndarray, boxes: List[Box]) -> np.ndarray:
    """Nearest positive ray parameter t (H, W) hitting any box, inf if none.

    origin: (3,) world ray origin; dirs: (H, W, 3) world ray directions."""
    h, w, _ = dirs.shape
    best = np.full((h, w), np.inf)
    inv = np.where(np.abs(dirs) < 1e-12, 1e12, 1.0 / dirs)
    for b in boxes:
        lo = np.asarray(b.min_corner, dtype=np.float64)
        hi = np.asarray(b.max_corner, dtype=np.float64)
        t1 = (lo - origin) * inv
        t2 = (hi - origin) * inv
        tmin = np.max(np.minimum(t1, t2), axis=-1)
        tmax = np.min(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= np.maximum(tmin, 0.0))
        t_enter = np.where(tmin > 0.0, tmin, tmax)  # camera may be outside/inside
        cand = np.where(hit & (t_enter > 0.0), t_enter, np.inf)
        best = np.minimum(best, cand)
    return best


def _plane_t(origin: np.ndarray, dirs: np.ndarray, axis: int, value: float) -> np.ndarray:
    """Positive t (H, W) where the ray crosses the plane coord[axis]==value."""
    denom = dirs[..., axis]
    t = np.where(np.abs(denom) < 1e-9, np.inf, (value - origin[axis]) / denom)
    return np.where(t > 0.0, t, np.inf)


def render_depth_m(intr: Intrinsics, base_T_opt: np.ndarray, scene: Scene) -> np.ndarray:
    """Render a float depth image (H, W) in METRES; 0.0 marks invalid/too-far."""
    T = _homogeneous(base_T_opt)
    R = T[:3, :3]
    origin = T[:3, 3]
    d_opt = _ray_dirs_optical(intr)                 # (H, W, 3)
    d_world = d_opt @ R.T                            # rotate into base_link
    dz_opt = d_opt[..., 2]                           # optical-z component (>0)

    t = _slab_t_boxes(origin, d_world, scene.boxes)
    if scene.ground_z_m is not None:
        t = np.minimum(t, _plane_t(origin, d_world, 2, scene.ground_z_m))
    if scene.back_wall_y_m is not None:
        t = np.minimum(t, _plane_t(origin, d_world, 1, scene.back_wall_y_m))

    depth_z = t * dz_opt                             # perpendicular distance
    invalid = ~np.isfinite(depth_z) | (depth_z <= 0.0) | (depth_z > scene.max_range_m)
    depth_z = np.where(invalid, 0.0, depth_z)
    return depth_z.astype(np.float32)


def render_depth_mm(intr: Intrinsics, base_T_opt: np.ndarray, scene: Scene) -> np.ndarray:
    """Render a 16UC1 depth image (H, W) in MILLIMETRES (cell convention)."""
    depth_m = render_depth_m(intr, base_T_opt, scene)
    mm = np.round(depth_m * 1000.0)
    mm = np.clip(mm, 0, 65535)
    return mm.astype(np.uint16)


def render_mask(intr: Intrinsics, base_T_opt: np.ndarray, scene: Scene,
                labels) -> np.ndarray:
    """Render a mono8 segmentation mask (H, W): 255 where the NEAREST surface a
    pixel sees is a box whose label is in ``labels``, else 0. This stands in for a
    people-segmentation network (e.g. PeopleSemSegNet) in simulation — clearly a
    synthetic mask, so nvblox's ``human_with_static_*`` mode can be exercised
    without the NGC model. The nearest-surface rule means an obstacle in front of
    the backdrop is masked but the backdrop behind it is not."""
    T = _homogeneous(base_T_opt)
    R = T[:3, :3]
    origin = T[:3, 3]
    d_opt = _ray_dirs_optical(intr)
    d_world = d_opt @ R.T
    h, w, _ = d_world.shape

    best_t = np.full((h, w), np.inf)
    if scene.ground_z_m is not None:
        best_t = np.minimum(best_t, _plane_t(origin, d_world, 2, scene.ground_z_m))
    if scene.back_wall_y_m is not None:
        best_t = np.minimum(best_t, _plane_t(origin, d_world, 1, scene.back_wall_y_m))
    mask = np.zeros((h, w), dtype=np.uint8)
    labels = set(labels)
    for b in scene.boxes:
        t_box = _slab_t_boxes(origin, d_world, [b])
        nearer = np.isfinite(t_box) & (t_box < best_t)
        best_t = np.where(nearer, t_box, best_t)
        if b.label in labels:
            mask[nearer] = 255
        else:
            mask[nearer] = 0        # a non-labelled box occludes a labelled one behind
    return mask


def look_at(eye: Tuple[float, float, float],
            target: Tuple[float, float, float],
            up: Tuple[float, float, float] = (0.0, 0.0, 1.0)) -> np.ndarray:
    """Build a base_link -> optical 4x4 for a camera at ``eye`` looking at
    ``target``. Optical z points from eye toward target (forward), x is right,
    y is down — the RealSense optical convention."""
    eye = np.asarray(eye, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-9:                     # looking along up: pick another
        x = np.cross(z, np.array([1.0, 0.0, 0.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)                               # y down (right-handed with z fwd)
    T = np.eye(4)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = eye
    return T


def matrix_to_quaternion_xyzw(T: np.ndarray) -> Tuple[float, float, float, float]:
    """Rotation part of a 4x4 -> (qx, qy, qz, qw), for the static TF publisher."""
    R = np.asarray(T, dtype=np.float64)[:3, :3]
    tr = np.trace(R)
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return (float(qx), float(qy), float(qz), float(qw))
