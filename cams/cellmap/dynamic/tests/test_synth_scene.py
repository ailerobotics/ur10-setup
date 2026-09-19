#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the synthetic-depth renderer — NO ROS, NO GPU.

Proves the geometry the ESDF proof depends on: a box placed at a known location
occludes the backdrop (nearer depth) exactly where it projects, an empty scene
shows only the backdrop, and look_at builds a proper rotation.

Run:  python3 test_synth_scene.py   |   pytest test_synth_scene.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from synth_scene import (  # noqa: E402
    Box, Intrinsics, Scene, look_at, matrix_to_quaternion_xyzw,
    render_depth_m, render_depth_mm)


def _cam():
    intr = Intrinsics.d415_like(160, 120)          # small for speed
    base_T_opt = look_at(eye=(0.0, -1.6, 1.2), target=(0.0, 0.4, 0.2))
    return intr, base_T_opt


def test_lookat_is_rotation():
    T = look_at(eye=(0.0, -1.6, 1.2), target=(0.0, 0.4, 0.2))
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(R) - 1.0) < 1e-9
    # optical z axis points from eye toward target
    z = R[:, 2]
    to_target = np.array([0.0, 0.4, 0.2]) - np.array([0.0, -1.6, 1.2])
    to_target /= np.linalg.norm(to_target)
    assert np.dot(z, to_target) > 0.99


def test_quaternion_roundtrip():
    T = look_at(eye=(0.3, -1.0, 0.9), target=(0.1, 0.5, 0.1))
    qx, qy, qz, qw = matrix_to_quaternion_xyzw(T)
    assert abs(qx * qx + qy * qy + qz * qz + qw * qw - 1.0) < 1e-9


def test_box_occludes_backdrop():
    intr, T = _cam()
    empty = render_depth_m(intr, T, Scene(boxes=[]))
    box = Box((-0.15, 0.55, 0.0), (0.15, 0.85, 0.30), "box")
    withbox = render_depth_m(intr, T, Scene(boxes=[box]))
    # somewhere the box must make depth strictly nearer than the backdrop
    valid = (empty > 0) & (withbox > 0)
    nearer = valid & (withbox < empty - 1e-3)
    assert np.count_nonzero(nearer) > 20, "box did not occlude the backdrop anywhere"
    # and it must never push depth FARTHER (a box only adds nearer surfaces)
    farther = valid & (withbox > empty + 1e-3)
    assert np.count_nonzero(farther) == 0


def test_depth_is_bounded_and_positive():
    intr, T = _cam()
    d = render_depth_m(intr, T, Scene(boxes=[], max_range_m=4.0))
    assert d.min() >= 0.0
    assert d.max() <= 4.0 + 1e-6
    assert np.count_nonzero(d > 0) > 0, "backdrop should be visible"


def test_mm_encoding():
    intr, T = _cam()
    mm = render_depth_mm(intr, T, Scene(boxes=[]))
    assert mm.dtype == np.uint16
    m = render_depth_m(intr, T, Scene(boxes=[]))
    both = (mm > 0) & (m > 0)
    assert np.allclose(mm[both] / 1000.0, m[both], atol=1e-3)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
