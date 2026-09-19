#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the segmented re-plan/preempt controller core (report §1-3).

The load-bearing invariant is proved here at the controller level too: NO input
sequence from the tracker or phase machine produces a PLAN while the gate is not a
fresh RUN."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from gate_acceptance import BACKSTOP, GateView, RUN, SLOW, STOP  # noqa: E402
from pick_target_tracker import TargetEstimate, TRACKING, LOST  # noqa: E402
from replan_controller_core import HOLD, NOOP, PLAN, ReplanController  # noqa: E402


def _est(state=TRACKING, committed=(0.6, 0.0, 0.3), moved=False):
    c = None if committed is None else np.array(committed, float)
    return TargetEstimate(state=state, filtered_position=c, velocity=np.zeros(3),
                          committed_target=c, committed_moved=moved, hits=5,
                          time_since_measurement=0.0, stamp=1.0)


def _run(t=100.0):
    return GateView(RUN, t)


def test_plans_a_first_leg_when_run_and_target():
    c = ReplanController(min_replan_interval_s=0.0)
    tcp = np.array([0.2, 0.0, 0.3])
    i = c.decide(100.0, _est(), tcp, _run(100.0))
    assert i.action == PLAN and not i.preempt
    assert np.allclose(i.goal, [0.6, 0.0, 0.3])


def test_no_plan_while_not_run_any_tracker_state():
    """Exhaustive: for every gate action != RUN and every tracker state, the
    controller must NOT emit PLAN."""
    for gate_act in (SLOW, STOP, BACKSTOP, None):
        for state in (TRACKING, LOST):
            for moved in (False, True):
                c = ReplanController(min_replan_interval_s=0.0)
                tcp = np.array([0.2, 0.0, 0.3])
                gv = GateView(gate_act, 100.0) if gate_act else GateView(None, None)
                i = c.decide(100.0, _est(state=state, moved=moved), tcp, gv)
                assert i.action != PLAN, f"PLAN leaked for gate={gate_act} state={state}"
                assert not i.gate_forward or gate_act == RUN


def test_stale_run_does_not_plan():
    c = ReplanController(min_replan_interval_s=0.0, shim=None)
    # a RUN decision 1 s old is stale (default 0.3 s window) -> HOLD, no motion
    i = c.decide(100.0, _est(), np.array([0.2, 0, 0.3]), GateView(RUN, 99.0))
    assert i.action == HOLD


def test_holds_when_target_lost_even_if_run():
    c = ReplanController(min_replan_interval_s=0.0)
    i = c.decide(100.0, _est(state=LOST, committed=None), np.array([0.2, 0, 0.3]), _run())
    assert i.action == HOLD


def test_preempts_only_on_genuine_move():
    c = ReplanController(min_replan_interval_s=0.0)
    tcp = np.array([0.2, 0.0, 0.3])
    # start a leg
    i0 = c.decide(100.0, _est(), tcp, _run(100.0))
    assert i0.action == PLAN
    c.note_leg_started(i0.goal, 100.0, 10.0)
    # committed unchanged -> NOOP (let it run)
    i1 = c.decide(100.5, _est(moved=False), tcp, _run(100.5))
    assert i1.action == NOOP
    # committed moved to a new place -> PREEMPT
    i2 = c.decide(101.0, _est(committed=(0.7, 0.0, 0.3), moved=True), tcp, _run(101.0))
    assert i2.action == PLAN and i2.preempt


def test_rate_limit_blocks_rapid_preempt():
    c = ReplanController(min_replan_interval_s=1.0)
    tcp = np.array([0.2, 0.0, 0.3])
    i0 = c.decide(100.0, _est(), tcp, _run(100.0))
    c.note_leg_started(i0.goal, 100.0, 10.0)
    # moved, but only 0.2 s later -> rate limited -> NOOP
    i1 = c.decide(100.2, _est(committed=(0.7, 0, 0.3), moved=True), tcp, _run(100.2))
    assert i1.action == NOOP
    # after the interval -> allowed
    i2 = c.decide(101.2, _est(committed=(0.7, 0, 0.3), moved=True), tcp, _run(101.2))
    assert i2.action == PLAN and i2.preempt


def test_grasp_phase_freezes_preemption():
    from pick_phases import PhaseParams, PickPhaseMachine
    phases = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.05))
    c = ReplanController(min_replan_interval_s=0.0, phases=phases)
    tgt = (0.6, 0.0, 0.3)
    # drive TCP to grasp radius so phase becomes GRASP
    tcp = np.array([0.6, 0.0, 0.31])   # ~1 cm from target
    i0 = c.decide(100.0, _est(committed=tgt), tcp, _run(100.0))
    c.note_leg_started(i0.goal, 100.0, 10.0)
    # target "moves" but we are in GRASP -> frozen -> NOOP (never preempt)
    i1 = c.decide(101.0, _est(committed=(0.7, 0.0, 0.3), moved=True), tcp, _run(101.0))
    assert c.phases.phase == "GRASP"
    assert i1.action == NOOP


def test_retreat_uses_fixed_pose_and_freezes():
    c = ReplanController(min_replan_interval_s=0.0)
    c.begin_retreat()
    retreat = np.array([0.1, 0.0, 0.5])
    i = c.decide(100.0, _est(), np.array([0.6, 0, 0.3]), _run(100.0), retreat_target=retreat)
    assert i.action == PLAN and np.allclose(i.goal, retreat)
    c.note_leg_started(i.goal, 100.0, 10.0)
    # even a moved target does not preempt a retreat
    i2 = c.decide(101.0, _est(committed=(0.9, 0, 0.3), moved=True),
                  np.array([0.6, 0, 0.3]), _run(101.0), retreat_target=retreat)
    assert i2.action == NOOP


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
