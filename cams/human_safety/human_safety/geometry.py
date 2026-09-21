# SPDX-License-Identifier: Apache-2.0
"""Pure-NumPy 3D geometry for lifting a person detection to the robot base frame.

Conventions
-----------
* Camera *optical* frame is the ROS/REP-103 optical convention: +x right,
  +y down, +z forward (into the scene). RealSense ``*_color_optical_frame`` and
  the wrist ``wrist_color_upright_optical_frame`` are such frames.
* A pose ``T_base_optical`` is a 4x4 homogeneous matrix that maps a point
  expressed in the optical frame to the base frame:  ``p_base = T @ [p_opt; 1]``.
* Intrinsics come straight from ``CameraInfo.k`` as ``fx, fy, cx, cy``.

None of this imports ROS; the nodes build ``T_base_optical`` from tf2 and pass
it in. All functions are deterministic and unit-tested offline.
"""
from __future__ import annotations

import numpy as np


# --------------------------------------------------------------------------- #
# Pose helpers
# --------------------------------------------------------------------------- #
def pose_matrix(translation, quaternion_xyzw) -> np.ndarray:
    """Build a 4x4 pose from a translation and an (x, y, z, w) quaternion."""
    from scipy.spatial.transform import Rotation

    t = np.asarray(translation, dtype=float).reshape(3)
    r = Rotation.from_quat(np.asarray(quaternion_xyzw, dtype=float).reshape(4))
    m = np.eye(4)
    m[:3, :3] = r.as_matrix()
    m[:3, 3] = t
    return m


def invert_pose(t: np.ndarray) -> np.ndarray:
    """Inverse of a rigid 4x4 transform."""
    r = t[:3, :3]
    p = t[:3, 3]
    out = np.eye(4)
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ p
    return out


def transform_points(t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) array (or a single (3,) point)."""
    pts = np.asarray(pts, dtype=float)
    single = pts.ndim == 1
    p = pts.reshape(-1, 3)
    out = (t[:3, :3] @ p.T).T + t[:3, 3]
    return out[0] if single else out


# --------------------------------------------------------------------------- #
# Pixel <-> ray / point in the optical frame
# --------------------------------------------------------------------------- #
def deproject(u, v, depth, fx, fy, cx, cy) -> np.ndarray:
    """Back-project a pixel + metric depth to a 3D point in the optical frame."""
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    z = np.asarray(depth, dtype=float)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=-1)


def pixel_ray(u, v, fx, fy, cx, cy) -> np.ndarray:
    """Unit ray direction in the optical frame through pixel (u, v)."""
    d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    return d / np.linalg.norm(d)


# --------------------------------------------------------------------------- #
# Depth lifting
# --------------------------------------------------------------------------- #
def _region_mask(shape, bbox, keypoints=None, kp_conf=0.3):
    """Boolean mask selecting pixels belonging to the person.

    Uses the bounding box shrunk 15% toward its centre (drops background that
    always frames a box), and — when keypoints are available — additionally
    keeps only the box, since a mask model is not assumed here. The shrink keeps
    the depth robust estimate off the far wall visible around a standing person.
    """
    h, w = shape
    x0, y0, x1, y1 = bbox
    cx = 0.5 * (x0 + x1)
    cy = 0.5 * (y0 + y1)
    sx = 0.85 * 0.5 * (x1 - x0)
    sy = 0.85 * 0.5 * (y1 - y0)
    ix0 = max(0, int(np.floor(cx - sx)))
    ix1 = min(w, int(np.ceil(cx + sx)))
    iy0 = max(0, int(np.floor(cy - sy)))
    iy1 = min(h, int(np.ceil(cy + sy)))
    mask = np.zeros(shape, dtype=bool)
    if ix1 > ix0 and iy1 > iy0:
        mask[iy0:iy1, ix0:ix1] = True
    return mask


def valid_depth_samples(depth, mask, fx, fy, cx, cy, min_range, max_range,
                        max_samples=4000):
    """Return (N, 3) optical-frame points for valid depth pixels under ``mask``.

    A depth pixel is valid when it is finite, strictly positive, and inside
    ``[min_range, max_range]`` — this is how the RealSense minimum/maximum range
    and depth holes (published as 0 or NaN) are handled. Sampling is capped so a
    full-frame person stays cheap.
    """
    depth = np.asarray(depth, dtype=float)
    good = mask & np.isfinite(depth) & (depth >= min_range) & (depth <= max_range)
    vs, us = np.nonzero(good)
    if us.size == 0:
        return np.empty((0, 3))
    if us.size > max_samples:
        idx = np.linspace(0, us.size - 1, max_samples).astype(int)
        us, vs = us[idx], vs[idx]
    z = depth[vs, us]
    return deproject(us.astype(float), vs.astype(float), z, fx, fy, cx, cy)


def lift_depth(depth, bbox, K, min_range, max_range, keypoints=None,
               nearest_percentile=5.0, min_valid=25, fg_band=1.0):
    """Robust body-centre + nearest credible point from depth, optical frame.

    Returns ``(center, nearest, n_valid)``. ``nearest`` is the point at the
    ``nearest_percentile`` of range — NOT the single closest pixel, so one flying
    near-range speckle can't pull the safety distance in.

    ``center`` is the component-wise median of the **foreground cluster only**:
    the valid points whose range is within ``fg_band`` metres of the near surface
    (``nearest_percentile`` of range). A plain median over the whole person box is
    *not* robust to the floor/wall visible inside the box — in clean depth the
    background behind the person can be more than half of the valid pixels, which
    drags a component-wise median metres into the background (the overhead camera
    seeing the most floor is the worst; a wide-armed pose amplifies it). Clustering
    in range around the near surface keeps only the body's own depth band, so the
    centre no longer over-states the person's distance. If too few foreground
    points survive the band, fall back to the all-points median (never worse than
    before). ``fg_band`` (≈ a person's front-to-back depth plus margin) is generous
    on purpose: clipping the far side of the body only biases the centre *toward*
    the robot (the safe direction) and the zone anyway reacts to ``nearest``.

    Both outputs are None when fewer than ``min_valid`` depth pixels survive
    (caller then falls back to monocular or presence).
    """
    fx, fy, cx, cy = K
    mask = _region_mask(depth.shape, bbox, keypoints)
    pts = valid_depth_samples(depth, mask, fx, fy, cx, cy, min_range, max_range)
    if pts.shape[0] < min_valid:
        return None, None, pts.shape[0]
    rng = np.linalg.norm(pts, axis=1)
    thresh = np.percentile(rng, nearest_percentile)
    near_pts = pts[rng <= thresh]
    nearest = np.median(near_pts, axis=0)
    # Foreground-only body centre: keep the body's own depth band off the near
    # surface, drop the background ring behind it.
    fg = pts[rng <= thresh + fg_band]
    if fg.shape[0] < min_valid:
        fg = pts
    center = np.median(fg, axis=0)
    return center, nearest, pts.shape[0]


# --------------------------------------------------------------------------- #
# Monocular lifting (no depth): ground-plane foot point + height-prior fallback
# --------------------------------------------------------------------------- #
def _foot_pixel(bbox, keypoints=None, kp_conf=0.3):
    """Best estimate of the person's foot pixel (u, v).

    Prefers the mean of confident ankle keypoints (COCO indices 15, 16); falls
    back to the bottom-centre of the bounding box.
    """
    if keypoints is not None and len(keypoints) >= 17:
        ankles = keypoints[[15, 16]]
        good = ankles[ankles[:, 2] >= kp_conf]
        if good.shape[0] > 0:
            return float(good[:, 0].mean()), float(good[:, 1].mean())
    x0, y0, x1, y1 = bbox
    return 0.5 * (x0 + x1), float(y1)


def _head_pixel(bbox, keypoints=None, kp_conf=0.3):
    if keypoints is not None and len(keypoints) >= 1:
        nose = keypoints[0]
        if nose[2] >= kp_conf:
            return float(nose[0]), float(nose[1])
    x0, y0, x1, y1 = bbox
    return 0.5 * (x0 + x1), float(y0)


def lift_monocular(bbox, K, T_base_optical, ground_z=0.0, height_prior=1.7,
                   keypoints=None, base_sigma=0.35, range_sigma_frac=0.15):
    """Monocular 3D estimate in the base frame from a known camera pose.

    Primary method: intersect the ray through the *foot* pixel with the ground
    plane ``z == ground_z`` in the base frame; the person centre is then lifted
    half a body height above that foot point. This needs the camera to be above
    the ground and looking down enough that the foot ray actually descends to
    the plane.

    Fallback (feet occluded / ray not descending): use the bounding-box pixel
    height and the ``height_prior`` to estimate range along the optical axis via
    the pinhole relation ``range = fy * H_real / h_pixels``, then deproject the
    box centre.

    Returns ``(center, nearest, sigma)`` in the base frame, or
    ``(None, None, None)`` if neither method is usable. ``sigma`` is an honestly
    inflated 1-sigma position uncertainty (monocular is far less certain than
    depth). ``nearest`` equals ``center`` here — a monocular estimate has no
    resolved near surface.
    """
    fx, fy, cx, cy = K
    cam_origin = T_base_optical[:3, 3]
    R = T_base_optical[:3, :3]

    fu, fv = _foot_pixel(bbox, keypoints)
    ray_opt = pixel_ray(fu, fv, fx, fy, cx, cy)
    ray_base = R @ ray_opt
    center = None
    sigma = None

    if ray_base[2] < -1e-3 and cam_origin[2] > ground_z:
        s = (ground_z - cam_origin[2]) / ray_base[2]
        if s > 0:
            foot = cam_origin + s * ray_base
            center = foot.copy()
            center[2] = ground_z + 0.5 * height_prior
            # Uncertainty grows with range (foot pixel error projects farther).
            sigma = float(base_sigma + range_sigma_frac * s)

    if center is None:
        # Height-prior fallback.
        x0, y0, x1, y1 = bbox
        h_px = max(1.0, (y1 - y0))
        rng = fy * height_prior / h_px
        p_opt = deproject(0.5 * (x0 + x1), 0.5 * (y0 + y1), rng, fx, fy, cx, cy)
        center = transform_points(T_base_optical, p_opt)
        sigma = float(base_sigma + range_sigma_frac * rng + 0.25)  # extra: prior

    return center, center.copy(), sigma


# --------------------------------------------------------------------------- #
# Distance to the robot
# --------------------------------------------------------------------------- #
def distance_to_base(point, base_xy=None) -> float:
    """Horizontal distance from a base-frame point to the base origin.

    Horizontal (xy) distance is used because a person's height above the floor
    is irrelevant to whether they are inside the robot's footprint.
    """
    p = np.asarray(point, dtype=float)
    bx, by = (0.0, 0.0) if base_xy is None else base_xy
    return float(np.hypot(p[0] - bx, p[1] - by))


def distance_to_envelope(point, reach_radius, base_xy=None) -> float:
    """Distance from a base-frame point to the robot's reach envelope.

    First-version geometry: the reach envelope is a cylinder of radius
    ``reach_radius`` about the base z-axis. Returns 0 when the point is inside
    the envelope. The code is structured so a later change can swap this for the
    live link geometry (per-link capsule distances) without touching callers.
    """
    d = distance_to_base(point, base_xy)
    return max(0.0, d - reach_radius)
