#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the analytic planner + Cartesian->joint surrogate. Proves the
5 % speed cap and the joint-window guard are enforced (execute_plan.py parity)."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from planner_port import (  # noqa: E402
    JOINTS, SPEED, VEL, AnalyticJointPlanner, CartesianToJointSurrogate,
    in_window_wrapped, joint_windows, wrap_to_nearest)

# the real measured window for wrist_1 (ref 4.459 rad, +-92.7 deg) — the sim arm
# reports this joint a whole turn away (-1.57 rad), which must still pass.
WRIST1_LO, WRIST1_HI = 2.84, 6.08

# a compact measured-limits-style dict (same shape as measured_joint_limits.yaml)
LIMITS = {
    'reference_pose_rad': {j: 0.0 for j in JOINTS},
    'window_deg': {j: 90.0 for j in JOINTS},   # +-90 deg windows
}


def test_windows_parse():
    lo, hi = joint_windows(LIMITS)
    assert np.allclose(lo, -math.radians(90))
    assert np.allclose(hi, math.radians(90))


def test_plan_respects_speed_cap():
    lo, hi = joint_windows(LIMITS)
    p = AnalyticJointPlanner(dt_s=0.2)
    q0 = np.zeros(6)
    qg = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # 1 rad on shoulder_pan
    res = p.plan(q0, qg, lo, hi)
    assert res.ok
    pos = np.array(res.positions)
    t = np.array(res.times_s)
    # peak per-joint velocity must be <= SPEED * VEL (a hair of tolerance)
    vmax = np.array([VEL[j] for j in JOINTS])
    dv = np.abs(np.diff(pos, axis=0)) / np.diff(t)[:, None]
    peak_frac = (dv / vmax).max()
    assert peak_frac <= SPEED * 1.02, f"peak {peak_frac*100:.1f}% exceeds 5% cap"


def test_plan_refuses_goal_outside_windows():
    lo, hi = joint_windows(LIMITS)
    p = AnalyticJointPlanner()
    res = p.plan(np.zeros(6), np.array([2.0, 0, 0, 0, 0, 0]), lo, hi)   # 2 rad > 90 deg
    assert not res.ok and "outside" in res.message


def test_plan_every_waypoint_in_windows():
    lo, hi = joint_windows(LIMITS)
    p = AnalyticJointPlanner(dt_s=0.1)
    res = p.plan(np.array([-1.0, 0, 0, 0, 0, 0]), np.array([1.0, 0, 0, 0, 0, 0]), lo, hi)
    assert res.ok
    for w in res.positions:
        w = np.array(w)
        assert ((lo - 1e-9 <= w) & (w <= hi + 1e-9)).all()


def test_plan_terminal_point_at_goal():
    lo, hi = joint_windows(LIMITS)
    p = AnalyticJointPlanner()
    qg = np.array([0.5, -0.3, 0.2, 0.0, 0.1, 0.0])
    res = p.plan(np.zeros(6), qg, lo, hi)
    assert res.ok and np.allclose(res.positions[-1], qg)


def test_surrogate_stays_in_windows_and_is_monotonic():
    lo, hi = joint_windows(LIMITS)
    surr = CartesianToJointSurrogate(lo=lo, hi=hi)
    prev = None
    for x in np.linspace(0.3, 0.9, 10):
        q = surr.to_joint([x, 0.0, 0.3])
        assert ((lo <= q) & (q <= hi)).all()
        if prev is not None:
            assert q[0] >= prev[0] - 1e-9, "pan joint not monotonic with target x"
        prev = q


def test_surrogate_clamps_out_of_box_targets():
    lo, hi = joint_windows(LIMITS)
    surr = CartesianToJointSurrogate(lo=lo, hi=hi)
    q_far = surr.to_joint([5.0, 5.0, 5.0])     # way outside the box
    assert ((lo <= q_far) & (q_far <= hi)).all()


def test_surrogate_roundtrip_target_to_joint_to_cartesian():
    lo, hi = joint_windows(LIMITS)
    surr = CartesianToJointSurrogate(lo=lo, hi=hi)
    for tgt in ([0.4, -0.2, 0.2], [0.7, 0.1, 0.5], [0.6, 0.0, 0.3]):
        q = surr.to_joint(tgt)
        back = surr.to_cartesian(q)
        assert np.allclose(back, tgt, atol=1e-6), f"{tgt} -> {back}"


def test_wrapping_aware_window_accepts_other_branch():
    # the sim arm's wrist_1 = -1.57 rad is the SAME physical angle as 4.71 rad,
    # which is inside the measured window -> must be accepted.
    assert in_window_wrapped(-1.57, WRIST1_LO, WRIST1_HI)
    assert in_window_wrapped(4.712, WRIST1_LO, WRIST1_HI)
    # a genuinely out-of-window angle (window centre + 180 deg) is rejected.
    assert not in_window_wrapped(4.459 + math.pi, WRIST1_LO, WRIST1_HI)


def test_wrap_to_nearest_is_shortest_travel():
    # commanding the window-branch centre (4.459) from the sim home (-1.57) should
    # move the SHORT way (~0.25 rad), not a full ~6 rad turn.
    cmd = wrap_to_nearest(4.459, -1.57)
    assert abs(cmd - (-1.57)) < math.pi
    assert abs((cmd - 4.459) % (2 * math.pi)) < 1e-9 or abs((cmd - 4.459) % (2 * math.pi) - 2 * math.pi) < 1e-9


def test_plan_from_sim_home_branch_ok():
    # a realistic case: shoulder/elbow in-window, wrist_1 a turn away. The plan must
    # be produced (start exactly at q0, waypoints physically in-window).
    lo = np.array([-1.05, -3.06, -1.09, WRIST1_LO, -1.454, -1.46])
    hi = np.array([1.50, -0.196, 1.90, WRIST1_HI, 2.013, 2.87])
    q0 = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])       # sim home
    qg = np.array([0.22, -1.63, 0.41, 4.459, 0.28, 0.71])   # window-branch centres
    res = AnalyticJointPlanner(dt_s=0.2).plan(q0, qg, lo, hi)
    assert res.ok, res.message
    assert np.allclose(res.positions[0], q0, atol=1e-9)     # starts at the live pose
    for w in res.positions:
        assert in_window_wrapped(np.array(w), lo, hi).all()


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn(); print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1; print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1; print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
