# SPDX-License-Identifier: Apache-2.0
"""Offline geometry tests: depth lifting, monocular lifting, frame transforms.
No ROS, no torch, no GPU."""
import numpy as np
import pytest

from human_safety import geometry as g


K = (600.0, 600.0, 320.0, 240.0)   # fx, fy, cx, cy for a 640x480 camera


def test_deproject_roundtrip():
    u, v, z = 400.0, 300.0, 2.5
    p = g.deproject(u, v, z, *K)
    fx, fy, cx, cy = K
    assert p[2] == pytest.approx(z)
    # reproject
    uu = p[0] * fx / p[2] + cx
    vv = p[1] * fy / p[2] + cy
    assert (uu, vv) == pytest.approx((u, v))


def test_pose_matrix_invert_transform():
    T = g.pose_matrix([1, 2, 3], [0, 0, 0, 1])   # identity rotation
    p = g.transform_points(T, np.array([0.0, 0.0, 1.0]))
    assert p == pytest.approx([1, 2, 4])
    Ti = g.invert_pose(T)
    back = g.transform_points(Ti, p)
    assert back == pytest.approx([0, 0, 1])


def test_transform_points_rotation():
    # 90 deg about base z: optical +z (forward) should map somewhere sensible.
    from scipy.spatial.transform import Rotation
    q = Rotation.from_euler("z", 90, degrees=True).as_quat()
    T = g.pose_matrix([0, 0, 0], q)
    p = g.transform_points(T, np.array([1.0, 0.0, 0.0]))
    assert p == pytest.approx([0, 1, 0], abs=1e-9)


def _synthetic_depth(shape, bbox, dist, holes=False, background=None):
    """Depth image: person slab at ``dist`` inside bbox, background farther."""
    h, w = shape
    depth = np.full(shape, np.nan if background is None else background,
                    dtype=np.float32)
    x0, y0, x1, y1 = [int(v) for v in bbox]
    depth[y0:y1, x0:x1] = dist
    if holes:
        depth[y0:y1, x0:x0 + (x1 - x0) // 3] = 0.0   # a column of invalid depth
    return depth


def test_lift_depth_center_and_nearest():
    bbox = (280, 180, 360, 400)
    depth = _synthetic_depth((480, 640), bbox, 2.0, background=5.0)
    center, nearest, n = g.lift_depth(depth, bbox, K, 0.2, 6.0)
    assert n > 100
    # slab is flat at z=2.0, centre near the optical axis
    assert center[2] == pytest.approx(2.0, abs=0.05)
    assert nearest[2] == pytest.approx(2.0, abs=0.05)


def test_lift_depth_holes_are_dropped():
    bbox = (280, 180, 360, 400)
    depth = _synthetic_depth((480, 640), bbox, 2.0, holes=True, background=5.0)
    center, nearest, n = g.lift_depth(depth, bbox, K, 0.2, 6.0)
    # holes (0.0) must not be read as a near person at 0 m
    assert center[2] == pytest.approx(2.0, abs=0.05)
    assert nearest[2] > 1.5


def test_lift_depth_respects_range_window():
    bbox = (280, 180, 360, 400)
    # person beyond max_range -> dropped -> not enough valid -> None
    depth = _synthetic_depth((480, 640), bbox, 8.0, background=np.nan)
    center, nearest, n = g.lift_depth(depth, bbox, K, 0.2, 6.0)
    assert center is None and nearest is None


def test_lift_depth_nearest_robust_to_single_speckle():
    bbox = (280, 180, 360, 400)
    depth = _synthetic_depth((480, 640), bbox, 3.0, background=5.0)
    depth[300, 320] = 0.25          # one flying near-range speckle
    center, nearest, n = g.lift_depth(depth, bbox, K, 0.2, 6.0)
    # percentile-based nearest must ignore the single 0.25 m speckle
    assert nearest[2] > 2.5


def test_monocular_ground_plane_foot():
    # Camera 2.5 m up at base origin, looking down and forward.
    from scipy.spatial.transform import Rotation
    # optical +z forward -> point it down 45 deg by rotating about base x.
    R = Rotation.from_euler("x", 135, degrees=True)   # optical z tilts toward -z_base-ish
    T = g.pose_matrix([0.0, 0.0, 2.5], R.as_quat())
    # Foot pixel = bottom centre of bbox.
    bbox = (300, 200, 360, 460)
    center, nearest, sigma = g.lift_monocular(
        bbox, K, T, ground_z=0.0, height_prior=1.7)
    assert center is not None
    assert center[2] == pytest.approx(0.85, abs=0.05)   # ~half body height
    assert sigma > 0.3                                   # honestly uncertain
    assert nearest is not None


def test_monocular_fallback_when_ray_not_descending():
    # Camera at ground level looking straight ahead (ray never hits ground).
    T = g.pose_matrix([0.0, 0.0, 0.0], [0, 0, 0, 1])
    bbox = (300, 100, 360, 400)
    center, nearest, sigma = g.lift_monocular(bbox, K, T, ground_z=-1.0)
    assert center is not None            # height-prior fallback used
    assert sigma > 0.3


def test_distance_helpers():
    assert g.distance_to_base([3.0, 4.0, 9.9]) == pytest.approx(5.0)
    assert g.distance_to_envelope([3.0, 4.0, 0.0], reach_radius=1.3) == \
        pytest.approx(3.7)
    assert g.distance_to_envelope([0.5, 0.0, 0.0], reach_radius=1.3) == 0.0
