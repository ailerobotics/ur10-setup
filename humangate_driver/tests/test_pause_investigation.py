#!/usr/bin/env python3
"""Offline unit tests for sim/assert_pause_investigation.py — the analyzer + the
regression guard for PAUSE_RESUME_FINDING.md. No ROS, no robot, no docker: it feeds
synthetic RESULT dicts (the measured headless case, a hypothetical "pause now
works" case, and invalid/degenerate cases) and checks the verdict.

Run: python3 tests/test_pause_investigation.py   (or: pytest -q tests)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
import assert_pause_investigation as api  # noqa: E402


# The REAL measured headless result (data/human-gate-sim-v1/ursim-pause).
HEADLESS_REJECTED = {
    "dashboard_before_motion": {"program_state": "STOPPED", "loaded_program": "",
                                "program_running": True},
    "goal_accepted": True,
    "dashboard_during_motion": {"program_state": "STOPPED", "loaded_program": "",
                                "program_running": True},
    "speed_before_pause": 0.05865,
    "clearly_moving_before_pause": True,
    "pan_at_pause": -1.28529,
    "scaling_before_pause": 100.0,
    "pause_call": {"ok": False,
                   "msg": "Expected: Pausing program, but received: Failed to execute: pause"},
    "froze_after_pause": False,
    "time_to_freeze_s": None,
    "goal_aborted_by_pause": False,
    "pan_drift_during_pause": 0.47059,
    "joint_states_flowing_during_pause": True,
    "dashboard_during_pause": {"program_state": "STOPPED", "loaded_program": "",
                               "program_running": True},
    "goal_done_during_pause": False,
    "play_call": {"ok": False,
                  "msg": "Expected: Starting program, but received: Failed to execute: play"},
    "speed_after_play": 0.05996,
    "pan_advanced_since_pause": 0.93741,
    "final_goal_result_code": 0,
    "final_goal_succeeded": True,
    "pause_executed": False,
    "play_executed": False,
    "true_freeze_and_continue": False,
}


def _passed(checks):
    return all(ok for _, ok, _ in checks)


def test_headless_finding_holds():
    """The measured headless result: all checks pass, no RE-EVALUATE (exit 0)."""
    checks, reevaluate = api.evaluate(HEADLESS_REJECTED)
    assert not reevaluate, "measured headless run must not trip RE-EVALUATE"
    assert _passed(checks), [c for c in checks if not c[1]]


def test_pause_now_works_triggers_reevaluate():
    """If a future driver makes headless pause EXECUTE and freeze-and-continue, the
    guard must flag RE-EVALUATE (nonzero) so the owner revisits the stop path."""
    works = dict(HEADLESS_REJECTED)
    works["pause_call"] = {"ok": True, "msg": "Pausing program"}
    works["play_call"] = {"ok": True, "msg": "Starting program"}
    works["froze_after_pause"] = True
    works["pan_drift_during_pause"] = 0.0
    works["true_freeze_and_continue"] = True
    _, reevaluate = api.evaluate(works)
    assert reevaluate, "pause executing must trip RE-EVALUATE"


def test_play_alone_executing_triggers_reevaluate():
    """Even play executing (loaded program appeared) is enough to re-evaluate."""
    r = dict(HEADLESS_REJECTED)
    r["play_call"] = {"ok": True, "msg": "Starting program"}
    _, reevaluate = api.evaluate(r)
    assert reevaluate


def test_invalid_experiment_arm_not_moving_fails():
    """A run where the arm was not actually moving cannot validate the finding."""
    r = dict(HEADLESS_REJECTED)
    r["clearly_moving_before_pause"] = False
    r["speed_before_pause"] = 0.001
    checks, reevaluate = api.evaluate(r)
    assert not reevaluate
    assert not _passed(checks), "must fail when the arm was not moving"


def test_probe_error_fails():
    """A probe that errored out (no joint_states etc.) fails, never silently passes."""
    checks, reevaluate = api.evaluate({"error": "no joint_states"})
    assert not reevaluate
    assert not _passed(checks)


def test_pause_dropped_connection_fails():
    """Fault case: had pause dropped the connection / aborted the goal, that is NOT
    the measured no-op behaviour and must not be reported as the finding."""
    r = dict(HEADLESS_REJECTED)
    r["goal_aborted_by_pause"] = True
    r["joint_states_flowing_during_pause"] = False
    r["final_goal_succeeded"] = False
    r["final_goal_result_code"] = -1
    checks, reevaluate = api.evaluate(r)
    assert not reevaluate
    assert not _passed(checks)


# ---------------------------------------------------------------------------- #
# PENDANT mode (the pendant-pause task): with an External Control program LOADED and
# PLAYING, dashboard pause DID deliver a true freeze-and-continue. These feed the REAL
# measured pendant result (data/human-gate-sim-v1/ursim-pendant-pause/pause_play_5s)
# and synthetic fault-case shapes through evaluate_pendant.
# ---------------------------------------------------------------------------- #

PENDANT_PAUSE_PLAY = {
    "mode": "pendant", "action": "pause_play", "pause_hold": 5.0,
    "jtc_state_before": "active",
    "dashboard_before_motion": {"program_state": "PLAYING",
                                "loaded_program": "/ursim/programs/external_control.urp",
                                "program_running": True},
    "goal_accepted": True, "speed_before_pause": 0.0574,
    "clearly_moving_before_pause": True, "jtc_state_before_pause": "active",
    "pause_call": {"ok": True, "msg": "Pausing program"},
    "froze_after_pause": True, "time_to_freeze_s": 1.807, "stopping_distance_rad": 0.00695,
    "goal_aborted_by_pause": False, "goal_code_at_pause": None,
    "pan_drift_during_pause": 0.0, "joint_states_flowing_during_pause": True,
    "scaling_during_pause_min": 0.0, "scaling_during_pause_last": 0.0,
    "dashboard_during_pause": {"program_state": "PAUSED",
                               "loaded_program": "/ursim/programs/external_control.urp",
                               "program_running": False},
    "goal_done_during_pause": False,
    "play_call": {"ok": True, "msg": "Starting program"},
    "speed_after_play": 0.05876, "pan_advanced_since_pause": 0.18463,
    "final_goal_result_code": 0, "final_goal_succeeded": True,
    "motion_resumed_after_play": True,
    "pause_executed": True, "play_executed": True, "true_freeze_and_continue": True,
}


def test_pendant_pause_play_holds():
    """The measured pendant pause_play run: all checks pass, no RE-EVALUATE."""
    checks, reevaluate = api.evaluate_pendant(PENDANT_PAUSE_PLAY)
    assert not reevaluate
    assert _passed(checks), [c for c in checks if not c[1]]


def test_pendant_regression_guard_trips_if_freeze_stops_working():
    """If a future driver/PolyScope change stops delivering the freeze-and-continue
    the strategy relies on, the pendant guard must RE-EVALUATE (nonzero)."""
    r = dict(PENDANT_PAUSE_PLAY)
    r["goal_aborted_by_pause"] = True          # goal no longer survives the pause
    r["final_goal_succeeded"] = False
    r["true_freeze_and_continue"] = False
    _, reevaluate = api.evaluate_pendant(r)
    assert reevaluate


def test_pendant_pause_play_invalid_if_no_loaded_program():
    """A pendant run where no program was loaded cannot validate the finding."""
    r = dict(PENDANT_PAUSE_PLAY)
    r["dashboard_before_motion"] = {"program_state": "STOPPED", "loaded_program": "",
                                    "program_running": False}
    checks, _ = api.evaluate_pendant(r)
    assert not _passed(checks)


def test_pendant_pause_only_stays_stopped():
    r = dict(PENDANT_PAUSE_PLAY)
    r["action"] = "pause_only"
    r["play_call"] = {"ok": False, "msg": "not attempted"}
    r["final_goal_succeeded"] = False
    checks, reevaluate = api.evaluate_pendant(r)
    assert not reevaluate
    assert _passed(checks), [c for c in checks if not c[1]]


def test_pendant_foreign_play_documents_resume_hazard():
    r = dict(PENDANT_PAUSE_PLAY)
    r["action"] = "foreign_play"
    r["pan_advanced_since_pause"] = 0.9        # a foreign play DID resume motion
    checks, reevaluate = api.evaluate_pendant(r)
    assert not reevaluate
    assert _passed(checks), [c for c in checks if not c[1]]


def test_pendant_stop_instead_is_terminal():
    r = dict(PENDANT_PAUSE_PLAY)
    r["action"] = "stop_instead"
    r["pause_call"] = {"ok": True, "msg": "Stopped"}
    r["goal_aborted_by_pause"] = True
    r["final_goal_succeeded"] = False
    r["final_goal_result_code"] = -4
    checks, reevaluate = api.evaluate_pendant(r)
    assert not reevaluate
    assert _passed(checks), [c for c in checks if not c[1]]


def test_pendant_pause_no_program_is_rejected():
    r = dict(PENDANT_PAUSE_PLAY)
    r["action"] = "pause_no_program"
    r["pause_call"] = {"ok": False, "msg": "Failed to execute: pause"}
    r["froze_after_pause"] = False
    r["pan_drift_during_pause"] = 0.4
    checks, reevaluate = api.evaluate_pendant(r)
    assert not reevaluate
    assert _passed(checks), [c for c in checks if not c[1]]


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    fail = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS {fn.__name__}")
        except AssertionError as e:
            fail += 1
            print(f"  FAIL {fn.__name__}: {e}")
    print(f"\n{len(fns) - fail}/{len(fns)} tests passed")
    sys.exit(1 if fail else 0)
