# SPDX-License-Identifier: Apache-2.0
"""Offline safety state-machine tests: zones, hysteresis, dwell, and every
health rule. Deterministic, no ROS."""
import numpy as np
import pytest

from human_safety.messages import CameraHealth, Track, Zone
from human_safety.safety import SafetyEvaluator, SafetyParams


def track(tid, xy, confirmed=True):
    return Track(id=tid, position=np.array([xy[0], xy[1], 0.9]),
                 velocity=np.zeros(3), covariance=np.eye(6), stamp=0.0,
                 hits=5, age=5, time_since_update=0.0, confirmed=confirmed)


def healthy(cameras):
    return [CameraHealth(camera=c, present_in_graph=True, frame_age=0.05,
                         detector_alive=True, positioned=True, fresh=True)
            for c in cameras]


def params():
    # slow_on_any_person disabled here so the distance ladder / hysteresis / dwell
    # can be tested in isolation; the any-person rule has its own test below.
    return SafetyParams(reach_radius=1.0, stop_radius=0.5, slow_radius=1.5,
                        hysteresis=0.25, min_clear_dwell=2.0,
                        slow_on_any_person=False)


# --- basic zoning ---------------------------------------------------------- #
def test_clear_when_no_one_around():
    ev = SafetyEvaluator(params())
    s = ev.evaluate([], healthy(["shelf"]), now=0.0)
    assert s.zone == Zone.CLEAR
    assert s.nearest_distance == float("inf")


def test_stop_when_person_inside_stop_radius():
    ev = SafetyEvaluator(params())
    # envelope dist = base_dist - reach(1.0). Person at 1.2 m base -> 0.2 m env.
    s = ev.evaluate([track(1, (1.2, 0.0))], healthy(["shelf"]), now=0.0)
    assert s.zone == Zone.STOP
    assert s.nearest_distance == pytest.approx(0.2, abs=1e-6)


def test_slow_zone():
    ev = SafetyEvaluator(params())
    # base 2.2 -> env 1.2 -> within slow(1.5), outside stop(0.5)
    s = ev.evaluate([track(1, (2.2, 0.0))], healthy(["shelf"]), now=0.0)
    assert s.zone == Zone.SLOW


# --- hysteresis ------------------------------------------------------------ #
def test_hysteresis_prevents_chatter_at_slow_boundary():
    ev = SafetyEvaluator(params())
    # Enter SLOW at env=1.4 (base 2.4)
    assert ev.evaluate([track(1, (2.4, 0.0))], healthy(["s"]), 0.0).zone == Zone.SLOW
    # env just above slow_radius (1.55) but within hysteresis band -> stays SLOW
    assert ev.evaluate([track(1, (2.55, 0.0))], healthy(["s"]), 0.1).zone == Zone.SLOW
    # clearly beyond slow_radius+hysteresis (env 1.9) -> CLEAR
    assert ev.evaluate([track(1, (2.9, 0.0))], healthy(["s"]), 0.2).zone == Zone.CLEAR


# --- dwell ----------------------------------------------------------------- #
def test_min_clear_dwell_before_leaving_stop():
    ev = SafetyEvaluator(params())
    # In STOP with the person inside the stop radius.
    assert ev.evaluate([track(1, (1.2, 0.0))], healthy(["s"]), 0.0).zone == Zone.STOP
    # Same person (same id) walks OUT to the CLEAR band, still tracked -> no
    # lost-close hold, but the dwell must elapse before STOP releases.
    assert ev.evaluate([track(1, (2.9, 0.0))], healthy(["s"]), 0.5).zone == Zone.STOP
    assert ev.evaluate([track(1, (2.9, 0.0))], healthy(["s"]), 1.9).zone == Zone.STOP
    # After dwell satisfied -> releases.
    assert ev.evaluate([track(1, (2.9, 0.0))], healthy(["s"]), 2.6).zone == Zone.CLEAR


def test_dwell_timer_resets_if_person_returns():
    ev = SafetyEvaluator(params())
    ev.evaluate([track(1, (1.2, 0.0))], healthy(["s"]), 0.0)          # STOP
    ev.evaluate([track(1, (2.9, 0.0))], healthy(["s"]), 1.0)          # clearing 1.0s
    # person back inside stop -> timer resets
    ev.evaluate([track(1, (1.2, 0.0))], healthy(["s"]), 1.5)          # STOP again
    # only 0.4 s clear now -> still STOP
    assert ev.evaluate([track(1, (2.9, 0.0))], healthy(["s"]), 1.9).zone == Zone.STOP


# --- health rules (fail toward STOP) --------------------------------------- #
def test_missing_camera_forces_stop():
    # A detector whose status stopped is not answering -> STOP (fail toward stop).
    ev = SafetyEvaluator(params())
    h = [CameraHealth(camera="shelf", present_in_graph=False)]
    s = ev.evaluate([], h, now=0.0)
    assert s.zone == Zone.STOP
    assert any("not answering" in r for r in s.reasons)


def test_slow_but_answering_detector_forces_slow_not_stop():
    # Frame age over the freshness budget (0.5) but under the stall timeout
    # (1.5) and still answering -> SLOW, not STOP.
    ev = SafetyEvaluator(params())
    h = [CameraHealth(camera="shelf", present_in_graph=True, frame_age=1.0,
                      detector_alive=True, positioned=True, fresh=False)]
    s = ev.evaluate([], h, now=0.0)
    assert s.zone == Zone.SLOW
    assert any("still answering" in r for r in s.reasons)


def test_stalled_detector_forces_stop():
    # No fresh frame for longer than the stall timeout -> detector stalled -> STOP.
    ev = SafetyEvaluator(params())
    h = [CameraHealth(camera="shelf", present_in_graph=True, frame_age=3.0,
                      detector_alive=True, positioned=True, fresh=False)]
    s = ev.evaluate([], h, now=0.0)
    assert s.zone == Zone.STOP
    assert any("stalled" in r for r in s.reasons)


def test_dead_detector_forces_stop():
    ev = SafetyEvaluator(params())
    h = [CameraHealth(camera="shelf", present_in_graph=True, frame_age=0.05,
                      detector_alive=False, positioned=True, fresh=True)]
    s = ev.evaluate([], h, now=0.0)
    assert s.zone == Zone.STOP


def test_latency_over_reaction_budget_forces_stop():
    ev = SafetyEvaluator(params())   # reaction_time_budget default 0.7
    # A localised person far away (SLOW by distance) but the pipeline latency
    # blew the reaction budget -> STOP (the ring math is no longer valid).
    s = ev.evaluate([track(1, (5.0, 0.0))], healthy(["s"]), now=0.0, latency=1.2)
    assert s.zone == Zone.STOP
    assert any("latency" in r for r in s.reasons)


def test_latency_within_budget_does_not_force_stop():
    ev = SafetyEvaluator(params())
    s = ev.evaluate([], healthy(["s"]), now=0.0, latency=0.2)
    assert s.zone == Zone.CLEAR


def test_presence_only_person_forces_non_clear():
    ev = SafetyEvaluator(params())
    s = ev.evaluate([], healthy(["shelf"]), now=0.0, any_presence=True)
    assert s.zone == Zone.SLOW
    assert s.any_person_visible


def test_track_lost_while_close_holds_stop():
    ev = SafetyEvaluator(params())
    # confirmed track well inside stop radius
    ev.evaluate([track(1, (1.1, 0.0))], healthy(["s"]), 0.0)   # STOP, close
    # track vanishes: no detections at all
    s1 = ev.evaluate([], healthy(["s"]), 0.2)
    assert s1.zone == Zone.STOP
    assert any("lost while close" in r for r in s1.reasons)
    # still held within lost_hold_time (3.0 s from last seen at 0.0)
    s2 = ev.evaluate([], healthy(["s"]), 2.9)
    assert s2.zone == Zone.STOP
    # lost hold (3.0) then the clear dwell (2.0) must BOTH elapse -> conservative
    assert ev.evaluate([], healthy(["s"]), 3.1).zone == Zone.STOP   # dwell starts
    assert ev.evaluate([], healthy(["s"]), 5.3).zone == Zone.CLEAR  # dwell done


def test_any_person_present_forces_at_least_slow():
    # Default params: slow_on_any_person is True. A person far outside the SLOW
    # ring, but present and confirmed, still forces SLOW (report §3, small cell).
    ev = SafetyEvaluator(SafetyParams(reach_radius=1.0, stop_radius=0.5,
                                      slow_radius=1.5, slow_on_any_person=True))
    s = ev.evaluate([track(1, (8.0, 0.0))], healthy(["s"]), now=0.0)
    assert s.zone == Zone.SLOW
    assert s.distance_status.value == "known"


def test_distance_status_unknown_for_presence_only():
    from human_safety.messages import DistanceStatus
    ev = SafetyEvaluator(params())
    s = ev.evaluate([], healthy(["shelf"]), now=0.0, any_presence=True)
    assert s.distance_status == DistanceStatus.UNKNOWN_PRESENCE_ONLY
    assert s.zone == Zone.SLOW   # unknown is not 'far'


def test_distance_status_none_when_empty():
    from human_safety.messages import DistanceStatus
    ev = SafetyEvaluator(params())
    s = ev.evaluate([], healthy(["shelf"]), now=0.0)
    assert s.distance_status == DistanceStatus.NONE


def test_health_floor_cannot_lower_a_distance_stop():
    ev = SafetyEvaluator(params())
    # distance says STOP; healthy cameras; zone must be STOP not lowered
    s = ev.evaluate([track(1, (1.1, 0.0))], healthy(["s"]), now=0.0,
                    any_presence=True)
    assert s.zone == Zone.STOP
