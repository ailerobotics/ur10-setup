# SPDX-License-Identifier: Apache-2.0
"""Lift one 2-D person detection to a base-frame :class:`Detection3D`.

Pure NumPy (no ROS, no torch): given the detector's 2-D box/keypoints, the
camera pose ``T`` (base <- optical, or None for presence-only), the colour
intrinsics, and — when available — the aligned depth, apply the same lifting
policy both detector nodes use:

  * pose known AND depth valid        -> DEPTH    (robust centre + nearest point)
  * pose known, depth missing/sparse  -> MONOCULAR (ground-plane / height prior)
  * pose unknown                      -> PRESENCE  (seen, no trustworthy position)

Factored out of the per-camera node so the shared (batched) detector node applies
byte-for-byte the same policy — there is exactly one lifting code path.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from . import geometry as geo
from .messages import Detection2D, Detection3D, Method


def lift_detection(d2d: Detection2D, T: Optional[np.ndarray], Kc,
                   *, camera: str, stamp: float,
                   has_depth: bool = False,
                   depth_metres: Optional[np.ndarray] = None,
                   Kd=None, min_range: float = 0.2, max_range: float = 6.0,
                   ground_z: float = 0.0, height_prior: float = 1.7) -> Detection3D:
    if T is None:
        return Detection3D(camera=camera, stamp=stamp, score=d2d.score,
                           method=Method.PRESENCE, present=True)

    # DEPTH first.
    if has_depth and depth_metres is not None:
        center, nearest, n = geo.lift_depth(
            depth_metres, d2d.bbox, Kd if Kd is not None else Kc,
            min_range, max_range, keypoints=d2d.keypoints)
        if center is not None:
            c_base = geo.transform_points(T, center)
            n_base = geo.transform_points(T, nearest)
            rng = float(np.linalg.norm(center))
            sigma = 0.01 * rng + 0.02          # ~1% of range + a voxel
            return Detection3D(
                camera=camera, stamp=stamp, score=d2d.score,
                method=Method.DEPTH, present=True, position=c_base,
                nearest_point=n_base, sigma=sigma, n_depth_valid=n)

    # MONOCULAR fallback (pose known, depth missing/insufficient).
    center, nearest, sigma = geo.lift_monocular(
        d2d.bbox, Kc, T, ground_z=ground_z, height_prior=height_prior,
        keypoints=d2d.keypoints)
    if center is not None:
        return Detection3D(
            camera=camera, stamp=stamp, score=d2d.score,
            method=Method.MONOCULAR, present=True, position=center,
            nearest_point=nearest, sigma=sigma)

    return Detection3D(camera=camera, stamp=stamp, score=d2d.score,
                       method=Method.PRESENCE, present=True)
