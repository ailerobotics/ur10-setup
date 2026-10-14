# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the efficient frame-processing scheduler bounds.

Covers the four safety-critical rules the brief calls out explicitly: the hard
T_max bound, the motion trigger, the near-robot priority, and staleness -> the
existing health machine degrading toward SLOW/STOP. Deterministic, no ROS/torch.
"""
import pytest

from human_safety.messages import CameraHealth, Zone
from human_safety.safety import SafetyEvaluator, SafetyParams
from human_safety.scheduler import (CameraTick, Decision, FrameScheduler,
                                     SchedulerParams)


def _sched(**kw):
    defaults = dict(enabled=True, t_max=0.4, busy_min_period=1 / 15,
                    motion_threshold=0.02)
    defaults.update(kw)
    return FrameScheduler(SchedulerParams(**defaults))


def _tick(role="shelf", have=True, score=0.0, last=0.0):
    return CameraTick(role=role, have_new_frame=have, change_score=score,
                      last_full_detect_time=last)


def _only(decisions):
    assert len(decisions) == 1
    return decisions[0]


# --- disabled == every-frame baseline -------------------------------------- #
def test_disabled_detects_every_new_frame():
    sched = FrameScheduler(SchedulerParams(enabled=False))
    d = _only(sched.decide(100.0, [_tick(score=0.0, last=99.99)]))
    assert d.detect is True


def test_no_new_frame_never_detects():
    # Even disabled, a camera with no new frame cannot be detected.
    for enabled in (True, False):
        sched = FrameScheduler(SchedulerParams(enabled=enabled))
        d = _only(sched.decide(100.0, [_tick(have=False, last=99.0)]))
        assert d.detect is False


# --- first detection -------------------------------------------------------- #
def test_first_frame_always_detects():
    d = _only(_sched().decide(5.0, [_tick(last=None)]))
    assert d.detect is True
    assert "first" in d.reason


# --- hard T_max bound ------------------------------------------------------- #
def test_t_max_forces_detection_even_when_quiet():
    sched = _sched()
    # Quiet (score below threshold), not near the robot, but t_max has elapsed.
    d = _only(sched.decide(10.40, [_tick(score=0.0, last=10.0)]))
    assert d.detect is True
    assert "t_max" in d.reason


def test_within_t_max_and_quiet_skips():
    sched = _sched()
    d = _only(sched.decide(10.30, [_tick(score=0.0, last=10.0)]))
    assert d.detect is False
    assert "quiet" in d.reason


def test_t_max_is_the_floor_rate():
    # A perfectly still, far-away scene detects on roughly the t_max cadence.
    dt = 0.1
    sched = _sched(t_max=0.4)
    last = 0.0
    detects = []
    for k in range(1, 41):                 # 4 s at 10 Hz
        now = k * dt
        d = _only(sched.decide(now, [_tick(score=0.0, last=last)]))
        if d.detect:
            detects.append(now)
            last = now
    # Floor rate ~2.5 Hz -> ~8-10 detections in 4 s.
    assert 7 <= len(detects) <= 11
    # On discrete ticks a detection lands on the first tick at or past t_max, so
    # the gap never exceeds t_max + one frame interval.
    gaps = [b - a for a, b in zip(detects, detects[1:])]
    assert max(gaps) <= 0.4 + dt + 1e-6


# --- motion trigger --------------------------------------------------------- #
def test_motion_triggers_immediate_detection():
    sched = _sched()
    # Within t_max and not near the robot, but the change score is above the
    # motion threshold -> detect now (a person entering view).
    d = _only(sched.decide(10.10, [_tick(score=0.05, last=10.0)]))
    assert d.detect is True
    assert "motion" in d.reason


def test_motion_below_threshold_does_not_trigger():
    sched = _sched(motion_threshold=0.02)
    d = _only(sched.decide(10.10, [_tick(score=0.01, last=10.0)]))
    assert d.detect is False


def test_unknown_change_score_fails_toward_looking():
    sched = _sched()
    d = _only(sched.decide(10.10, [_tick(score=None, last=10.0)]))
    assert d.detect is True
    assert "unavailable" in d.reason


def test_busy_rate_is_capped_by_busy_min_period():
    # Two motion frames closer together than busy_min_period: the second is held
    # off (the busy rate is bounded), but never past t_max.
    sched = _sched(busy_min_period=0.2)
    assert _only(sched.decide(10.05, [_tick(score=0.5, last=10.0)])).detect is False
    assert _only(sched.decide(10.25, [_tick(score=0.5, last=10.0)])).detect is True


# --- near-robot priority ---------------------------------------------------- #
def test_near_ring_forces_full_rate_even_when_still():
    sched = _sched()
    # Still person (score below threshold), within t_max, but a track is near the
    # robot -> every camera runs at the busy rate.
    d = _only(sched.decide(10.10, [_tick(score=0.0, last=10.0)], near_ring=True))
    assert d.detect is True
    assert "near-robot" in d.reason


def test_near_ring_applies_to_all_cameras():
    sched = _sched()
    ticks = [_tick(role="shelf", score=0.0, last=10.0),
             _tick(role="table", score=0.0, last=10.0),
             _tick(role="wrist", score=0.0, last=10.0)]
    decisions = sched.decide(10.10, ticks, near_ring=True)
    assert all(d.detect for d in decisions)


# --- staleness -> the existing health machine degrades --------------------- #
def test_staleness_feeds_unhealthy_then_slow_then_stop():
    # If a camera stops delivering frames, the scheduler never detects it, so its
    # last-full-detect age grows. Feed that growing age through the UNCHANGED
    # safety health machine and confirm it degrades fresh -> SLOW -> STOP.
    sched = _sched()
    last_full = 10.0
    sp = SafetyParams(frame_age_budget=0.5, detector_stall_timeout=1.5)
    ev = SafetyEvaluator(sp)

    def zone_at(now):
        # The camera has no new frame, so the scheduler does not detect it.
        d = _only(sched.decide(now, [_tick(have=False, last=last_full)]))
        assert d.detect is False
        age = now - last_full
        fresh = age <= sp.frame_age_budget
        h = CameraHealth(camera="shelf", present_in_graph=True,
                         detector_alive=True, frame_age=age, fresh=fresh)
        return ev.evaluate([], [h], now).zone

    assert zone_at(10.3) == Zone.CLEAR    # age 0.3 < budget -> fresh
    assert zone_at(10.9) == Zone.SLOW     # age 0.9 in (budget, stall] -> SLOW
    assert zone_at(12.0) == Zone.STOP     # age 2.0 > stall timeout -> STOP


def test_t_max_at_or_below_budget_keeps_camera_fresh():
    # The config invariant: with t_max <= frame_age_budget and frames arriving,
    # the worst-case age before the next detection never exceeds the budget.
    sched = _sched(t_max=0.4)
    budget = 0.5
    last = 0.0
    worst = 0.0
    for k in range(1, 101):
        now = k * 0.05                     # 20 Hz frames, 5 s
        worst = max(worst, now - last)
        d = _only(sched.decide(now, [_tick(score=0.0, last=last)]))
        if d.detect:
            last = now
    assert worst <= budget


# --- parameter validation --------------------------------------------------- #
def test_params_reject_bad_values():
    with pytest.raises(ValueError):
        SchedulerParams(t_max=0.0)
    with pytest.raises(ValueError):
        SchedulerParams(busy_min_period=0.0)
    with pytest.raises(ValueError):
        SchedulerParams(busy_min_period=1.0, t_max=0.4)   # busy slower than floor
    with pytest.raises(ValueError):
        SchedulerParams(motion_threshold=-0.1)
    with pytest.raises(ValueError):
        SchedulerParams(near_margin=-0.1)
    with pytest.raises(ValueError):
        SchedulerParams(downscale=0)


def test_decision_dataclass_shape():
    d = Decision(role="shelf", detect=True, reason="x")
    assert (d.role, d.detect) == ("shelf", True)
