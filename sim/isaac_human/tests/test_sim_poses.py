# SPDX-License-Identifier: Apache-2.0
"""Pose maths: the TF handed to the detector must be the exact pose Isaac renders
from, and projection/round-trip must be self-consistent with human_safety's own
depth-lift transform."""
import numpy as np

import sim_poses as sp
from cell_layout import CAMERAS
from human_safety import geometry as geo


def test_quaternion_is_unit():
    for cam in CAMERAS:
        _, q = cam.base_to_optical()
        assert abs(np.linalg.norm(q) - 1.0) < 1e-9


def test_target_projects_to_image_centre():
    for cam in CAMERAS:
        pr = cam.project(np.array(cam.target))
        assert pr is not None
        u, v, depth = pr
        cx, cy = cam.width / 2.0, cam.height / 2.0
        assert abs(u - cx) < 1.0
        assert abs(v - cy) < 1.0
        assert depth > 0


def test_point_right_of_target_moves_u_right():
    cam = CAMERAS[0]
    # optical +X is camera-right; nudge the point along camera-right and u grows.
    x, y, z = sp.look_at_basis(cam.eye, cam.target)
    right = np.array(cam.target) + 0.3 * x
    pr = cam.project(right)
    assert pr is not None
    assert pr[0] > cam.width / 2.0


def test_point_behind_camera_not_in_fov():
    cam = CAMERAS[0]
    # a point behind the eye (opposite the look direction) is not visible
    behind = np.array(cam.eye) + (np.array(cam.eye) - np.array(cam.target))
    assert cam.project(behind) is None
    assert not cam.in_fov(behind)


def test_tf_matches_human_safety_depth_lift():
    """A synthetic point in the OPTICAL frame, transformed to base by
    human_safety.geometry.pose_matrix(t, q) using our published TF, must land at
    the same base point our own projection inverts to. This is the exact path the
    detector uses (lift_depth -> transform_points), so agreement proves the TF we
    hand the detector reconstructs true base positions."""
    for cam in CAMERAS:
        t, q = cam.base_to_optical()
        T = geo.pose_matrix(list(t), list(q))     # base <- optical (4x4)
        # pick a base point in front of the camera (the target), project to get
        # its optical coords, then transform back and compare.
        target = np.array(cam.target, dtype=float)
        u, v, depth = cam.project(target)
        fx, fy, cx, cy = cam.K()
        p_opt = np.array([(u - cx) * depth / fx,
                          (v - cy) * depth / fy,
                          depth])
        p_base = geo.transform_points(T, p_opt)
        assert np.linalg.norm(np.asarray(p_base).reshape(3) - target) < 1e-6
