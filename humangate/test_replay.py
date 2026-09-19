#!/usr/bin/env python3
"""
Scenario assertions on the replay harness — NO ROS, NO robot (report §8).

Each scripted sequence from the report is replayed and both the decisions and
their timing are asserted: person approaches / hovers / leaves, heartbeat drops,
stamps go stale, distance unknown, person vanishes close, clock skew, and the
STOP->BACKSTOP fall-through.

Run standalone:  python3 test_replay.py
Or under pytest: pytest test_replay.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import replay as rp  # noqa: E402
import gate_core as gc  # noqa: E402


def _actions(trace):
    return [d.action for _, d in trace]


def _compressed(trace):
    a = _actions(trace)
    return [x for i, x in enumerate(a) if i == 0 or x != a[i - 1]]


def _final(trace):
    return trace[-1][1]


def _first_time(trace, action):
    for t, d in trace:
        if d.action == action:
            return t
    return None


def test_approach_ends_stopped_close():
    trace = rp.run_scenario(rp.approach())
    assert _final(trace).action == gc.STOP
    assert gc.RUN not in _actions(trace)


def test_hover_never_runs():
    trace = rp.run_scenario(rp.hover_at_boundary())
    assert gc.RUN not in _actions(trace)


def test_approach_hover_leave_runs_only_after_dwell():
    cfg = None  # default config: clear_dwell_s = 2.0
    trace = rp.run_scenario(rp.approach_hover_leave(), cfg)
    assert gc.RUN in _actions(trace), "a tracked departure must eventually RUN"
    t_run = _first_time(trace, gc.RUN)
    # CLEAR begins at t=1.1 in the scenario; RUN must not precede t=1.1 + dwell
    assert t_run >= 1.1 + 2.0 - 1e-6, f"RUN at {t_run} came before the 2 s dwell"
    assert _final(trace).action == gc.RUN


def test_heartbeat_drop_is_stop():
    trace = rp.run_scenario(rp.heartbeat_drop())
    assert gc.RUN not in _actions(trace)
    assert _final(trace).action == gc.STOP
    # once silent past the timeout, the reason must name the heartbeat
    assert "heartbeat" in _final(trace).reason


def test_heartbeat_stall_is_stop():
    trace = rp.run_scenario(rp.heartbeat_stall())
    assert _final(trace).action == gc.STOP
    assert "stall" in _final(trace).reason


def test_stale_stamps_is_stop():
    trace = rp.run_scenario(rp.stale_stamps())
    assert _actions(trace).count(gc.RUN) == 0
    assert "stale" in _final(trace).reason


def test_distance_unknown_slows_never_runs():
    trace = rp.run_scenario(rp.distance_unknown())
    assert set(_actions(trace)) <= {gc.SLOW}, \
        "unknown-distance present person must be SLOW throughout, never RUN/CLEAR"


def test_vanish_close_stays_stopped():
    trace = rp.run_scenario(rp.vanish_close())
    assert gc.RUN not in _actions(trace), "a person lost close must not enable resume"
    assert _final(trace).action == gc.STOP
    assert _final(trace).provenance == gc.PROV_LOST_CLOSE


def test_vanish_close_flagged_stays_stopped():
    trace = rp.run_scenario(rp.vanish_close_flagged())
    assert gc.RUN not in _actions(trace)
    assert _final(trace).provenance == gc.PROV_LOST_CLOSE


def test_clock_skew_is_stop():
    trace = rp.run_scenario(rp.clock_skew())
    assert gc.RUN not in _actions(trace)
    assert "skew" in _final(trace).reason


def test_stop_not_confirmed_escalates_to_backstop():
    trace = rp.run_scenario(rp.stop_not_confirmed())
    assert gc.BACKSTOP in _actions(trace)
    # BACKSTOP must not appear before the deadline (0.5 s of hazard STOP)
    t_bs = _first_time(trace, gc.BACKSTOP)
    assert t_bs is not None and t_bs > 0.5 - 1e-6


def test_every_scenario_replays_deterministically():
    # same scenario twice -> identical decision timeline (no hidden clock reads)
    for builder in rp.ALL_SCENARIOS.values():
        a = _actions(rp.run_scenario(builder()))
        b = _actions(rp.run_scenario(builder()))
        assert a == b


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
