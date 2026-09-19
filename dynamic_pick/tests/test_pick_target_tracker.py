#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the pick-target tracker: confirmation, deadband/hysteresis,
occlusion and lost states. NO ROS, NO torch (report §3)."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pick_target_tracker import (  # noqa: E402
    ACQUIRING, LOST, OCCLUDED, TRACKING, PickTargetTracker)


def _feed(tr, pos, t):
    """One measurement at wall/mono time t (stamp == mono here for simplicity)."""
    return tr.update(pos, None, stamp=t, now=t)


def test_confirmation_needs_n_hits():
    tr = PickTargetTracker(n_confirm=5)
    est = None
    for i in range(4):
        est = _feed(tr, [0.5, 0.0, 0.3], 0.1 * i)
        assert est.state == ACQUIRING, f"hit {i}"
        assert est.committed_target is None    # no goal while acquiring
    est = _feed(tr, [0.5, 0.0, 0.3], 0.5)
    assert est.state == TRACKING
    assert est.committed_target is not None
    assert est.committed_moved is True         # first commit


def test_deadband_holds_committed_goal_on_jitter():
    tr = PickTargetTracker(n_confirm=3, deadband_m=0.03, n_sustain=3)
    for i in range(3):
        _feed(tr, [0.5, 0.0, 0.3], 0.1 * i)
    committed0 = None
    # feed sub-deadband jitter (< 3 cm) for many frames: committed must not move.
    moved_any = False
    for i in range(10):
        jit = np.array([0.5, 0.0, 0.3]) + np.array([0.005 * ((-1) ** i), 0.004, 0.0])
        est = _feed(tr, jit, 0.3 + 0.1 * i)
        committed0 = committed0 if committed0 is not None else est.committed_target
        moved_any = moved_any or est.committed_moved
    assert not moved_any, "committed goal moved on sub-deadband jitter"


def test_hysteresis_needs_sustain_before_commit_moves():
    tr = PickTargetTracker(n_confirm=3, deadband_m=0.03, n_sustain=3, sigma_meas=0.001)
    for i in range(3):
        _feed(tr, [0.5, 0.0, 0.3], 0.1 * i)
    c0 = tr._committed.copy()
    # a single big jump then back inside deadband should NOT move the committed goal
    _feed(tr, [0.7, 0.0, 0.3], 0.4)        # beyond deadband once
    _feed(tr, [0.5, 0.0, 0.3], 0.5)        # back inside -> not sustained
    assert np.allclose(tr._committed, c0), "committed moved without a sustained shift"
    # now a sustained shift for n_sustain fresh updates -> commit moves
    moved = False
    for i in range(4):
        est = _feed(tr, [0.75, 0.0, 0.3], 0.6 + 0.1 * i)
        moved = moved or est.committed_moved
    assert moved, "committed goal never moved despite a sustained shift"


def test_occluded_then_lost_and_drops_committed():
    tr = PickTargetTracker(n_confirm=3, occlusion_after_s=0.3, max_coast_time=0.8)
    for i in range(3):
        _feed(tr, [0.5, 0.0, 0.3], 0.1 * i)
    assert tr.state == TRACKING
    # a normal inter-frame gap (controller faster than detector) must NOT occlude.
    assert tr.step(0.3).state == TRACKING       # 0.1 s since last meas
    # a real gap (> occlusion_after_s) -> OCCLUDED; committed is only exposed for
    # TRACKING, so it must report None committed while OCCLUDED.
    est = tr.step(0.65)        # 0.45 s since last meas, < 0.8 coast
    assert est.state == OCCLUDED
    assert est.committed_target is None, "must not expose a committed goal while occluded"
    est = tr.step(1.2)         # > coast budget -> LOST
    assert est.state == LOST
    assert est.committed_target is None


def test_lost_never_extrapolates_a_phantom():
    tr = PickTargetTracker(n_confirm=3, max_coast_time=0.5)
    for i in range(3):
        _feed(tr, [0.5, 0.0, 0.3], 0.1 * i)
    est = tr.step(1.0)     # LOST
    assert est.state == LOST
    # a lost target yields no committed goal at all: the controller cannot chase it.
    assert est.committed_target is None


def test_acquiring_times_out_to_lost():
    tr = PickTargetTracker(n_confirm=5, acquire_timeout=0.6)
    _feed(tr, [0.5, 0.0, 0.3], 0.0)
    _feed(tr, [0.5, 0.0, 0.3], 0.1)
    assert tr.state == ACQUIRING
    est = tr.step(1.0)     # 0.9 s of silence while unconfirmed
    assert est.state == LOST


def test_moving_target_tracks_and_recommits():
    tr = PickTargetTracker(n_confirm=3, deadband_m=0.03, n_sustain=2, sigma_meas=0.001)
    for i in range(3):
        _feed(tr, [0.5, 0.0, 0.3], 0.1 * i)
    # target walks +x steadily; committed should follow in steps of >= deadband
    commits = []
    x = 0.5
    for i in range(20):
        x += 0.02
        est = _feed(tr, [x, 0.0, 0.3], 0.4 + 0.1 * i)
        if est.committed_moved:
            commits.append(est.committed_target[0])
    assert len(commits) >= 2, "committed goal did not follow a genuinely moving target"
    assert commits == sorted(commits), "committed goal did not move monotonically with the target"


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
