#!/usr/bin/env python3
"""Offline tests for the measured joint-window guard (no GPU, no ROS)."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(PKG)
sys.path.insert(0, PKG)

from robot.joint_windows import JOINT_ORDER, load_measured_windows  # noqa: E402

LIMITS_YAML = os.path.join(REPO, "cumotion", "measured_joint_limits.yaml")


@pytest.fixture(scope="module")
def windows():
    return load_measured_windows(LIMITS_YAML)


def test_all_six_joints_present(windows):
    assert windows.joint_names == list(JOINT_ORDER)


def test_windows_match_urdf_limit_tags(windows):
    # These are the exact <limit lower/upper> tags in cumotion/ur10_measured.urdf,
    # which cuRobo reads. The guard must reproduce them from the YAML so a drift
    # between the two is caught.
    expected = {
        "shoulder_pan_joint": (-1.052772, 1.499542),
        "shoulder_lift_joint": (-3.057365, -0.196061),
        "elbow_joint": (-1.093788, 1.906338),
        "wrist_1_joint": (2.840541, 6.077391),
        "wrist_2_joint": (-1.454848, 2.013530),
        "wrist_3_joint": (-1.462148, 2.874244),
    }
    for j, (lo, hi) in expected.items():
        got_lo, got_hi = windows.windows[j]
        assert math.isclose(got_lo, lo, abs_tol=1e-5), (j, got_lo, lo)
        assert math.isclose(got_hi, hi, abs_tol=1e-5), (j, got_hi, hi)


def test_reference_pose_is_inside_every_window(windows):
    # The reference (park/pneumatic) pose is the window centre, so it must be in.
    ref = {
        "shoulder_pan_joint": 0.223385, "shoulder_lift_joint": -1.626713,
        "elbow_joint": 0.406275, "wrist_1_joint": 4.458966,
        "wrist_2_joint": 0.279341, "wrist_3_joint": 0.706048,
    }
    assert windows.within(ref)


def test_violation_detected_below_and_above(windows):
    lo, hi = windows.windows["elbow_joint"]
    assert windows.violations({"elbow_joint": lo - 0.01})
    assert windows.violations({"elbow_joint": hi + 0.01})
    assert not windows.violations({"elbow_joint": (lo + hi) / 2.0})


def test_clamp_pulls_into_window(windows):
    lo, hi = windows.windows["shoulder_pan_joint"]
    clamped = windows.clamp({"shoulder_pan_joint": hi + 1.0})
    assert math.isclose(clamped["shoulder_pan_joint"], hi)
    clamped = windows.clamp({"shoulder_pan_joint": lo - 1.0})
    assert math.isclose(clamped["shoulder_pan_joint"], lo)


def test_tolerance_boundary(windows):
    lo, hi = windows.windows["wrist_3_joint"]
    # exactly on the boundary is allowed; a hair past (beyond tol) is not.
    assert windows.within({"wrist_3_joint": hi})
    assert not windows.within({"wrist_3_joint": hi + 1e-3})
