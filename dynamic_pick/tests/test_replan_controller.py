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
    # start a leg while FAR (a leg must be running to test that GRASP freezes its
    # preemption; you never START in grasp — you arrive there via a running leg).
    i0 = c.decide(100.0, _est(committed=tgt), np.array([0.2, 0.0, 0.3]), _run(100.0))
    assert i0.action == PLAN
    c.note_leg_started(i0.goal, 100.0, 10.0)
    # TCP now at the grasp radius -> GRASP; target jitters within it -> frozen -> NOOP
    grasp_tcp = np.array([0.6, 0.0, 0.31])   # ~1 cm from target
    i1 = c.decide(101.0, _est(committed=(0.61, 0.0, 0.30), moved=True), grasp_tcp, _run(101.0))
    assert c.phases.phase == "GRASP"
    assert i1.action == NOOP


def test_no_fresh_plan_to_moved_target_in_near():
    """A leg that COMPLETES in NEAR with the committed target moved OUTSIDE the
    pre-cleared box must NOT re-plan to that moved target (LIMITED policy) — the
    fresh-leg path is gated by the phase policy, not only the preempt path."""
    from pick_phases import PhaseParams, PickPhaseMachine
    phases = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.03,
                                          near_box_half_m=0.10))
    c = ReplanController(min_replan_interval_s=0.0, phases=phases)
    tgt = (0.6, 0.0, 0.3)
    # a leg runs in FAR toward the target
    i0 = c.decide(100.0, _est(committed=tgt), np.array([0.35, 0.0, 0.3]), _run(100.0))
    assert i0.action == PLAN
    c.note_leg_started(i0.goal, 100.0, 10.0)
    # TCP arrives in the near band -> NEAR (box centred at tgt)
    near_tcp = np.array([0.55, 0.0, 0.3])
    c.decide(101.0, _est(committed=tgt), near_tcp, _run(101.0))
    assert c.phases.phase == "NEAR"
    # the leg completes; the target then jumps 0.15 m -> OUTSIDE the 0.10 m box
    c.note_leg_finished()
    moved = (0.75, 0.0, 0.3)
    i = c.decide(102.0, _est(committed=moved, moved=True), near_tcp, _run(102.0))
    assert c.phases.phase == "NEAR"
    assert i.action != PLAN and i.goal is None


def test_no_fresh_plan_to_moved_target_in_grasp():
    """A leg that COMPLETES in GRASP with the target moved must NOT re-plan to the
    moved target (FROZEN): the fresh-leg path is frozen too, not just preemption."""
    from pick_phases import PhaseParams, PickPhaseMachine
    phases = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.05))
    c = ReplanController(min_replan_interval_s=0.0, phases=phases)
    tgt = (0.6, 0.0, 0.3)
    i0 = c.decide(100.0, _est(committed=tgt), np.array([0.2, 0.0, 0.3]), _run(100.0))
    assert i0.action == PLAN
    c.note_leg_started(i0.goal, 100.0, 10.0)
    grasp_tcp = np.array([0.6, 0.0, 0.31])   # within grasp radius -> GRASP
    c.decide(101.0, _est(committed=tgt), grasp_tcp, _run(101.0))
    assert c.phases.phase == "GRASP"
    # leg completes; target moves -> a fresh leg must NOT chase it (frozen)
    c.note_leg_finished()
    i = c.decide(102.0, _est(committed=(0.62, 0.0, 0.30), moved=True), grasp_tcp, _run(102.0))
    assert c.phases.phase == "GRASP"
    assert i.action != PLAN and i.goal is None


def test_no_fresh_plan_to_moved_target_in_retreat():
    """In RETREAT the tracker target is ignored entirely: the ONLY goal ever planned
    is the fixed pre-planned retreat pose, never the (moved) pick target."""
    retreat = np.array([0.1, 0.0, 0.5])
    moved = (0.9, 0.0, 0.3)
    c = ReplanController(min_replan_interval_s=0.0)
    c.begin_retreat()
    # nothing in flight, target has moved far: a fresh RETREAT leg goes ONLY to the
    # fixed retreat pose.
    i = c.decide(100.0, _est(committed=moved, moved=True), np.array([0.6, 0, 0.3]),
                 _run(100.0), retreat_target=retreat)
    assert i.action == PLAN
    assert np.allclose(i.goal, retreat) and not np.allclose(i.goal, moved)
    # and with NO retreat pose supplied it HOLDS rather than chasing the moved target
    c2 = ReplanController(min_replan_interval_s=0.0)
    c2.begin_retreat()
    i2 = c2.decide(100.0, _est(committed=moved, moved=True), np.array([0.6, 0, 0.3]),
                   _run(100.0))
    assert i2.action == HOLD and i2.goal is None


def test_scripted_near_grasp_retreat_sequence():
    """Scripted (pure) end-to-end sequence: an approach leg drives FAR->NEAR->GRASP
    toward a target that keeps jittering, the grasp completes -> begin_retreat ->
    the retreat leg goes to the FIXED pose. No PLAN in NEAR/GRASP ever targets a
    moved target outside the box."""
    from pick_phases import PhaseParams, PickPhaseMachine
    phases = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.05,
                                          near_box_half_m=0.10))
    c = ReplanController(min_replan_interval_s=0.0, phases=phases)
    retreat = np.array([0.2, -0.2, 0.5])

    # FAR: fresh leg to the target
    i = c.decide(100.0, _est(committed=(0.6, 0.0, 0.3)), np.array([0.2, 0.0, 0.3]),
                 _run(100.0), retreat_target=retreat)
    assert i.action == PLAN and c.phases.phase == "FAR" and not i.preempt
    c.note_leg_started(i.goal, 100.0, 5.0)

    # leg completes; TCP in the near band, a small jitter INSIDE the box is planned
    c.note_leg_finished()
    near_tcp = np.array([0.50, 0.0, 0.3])
    i = c.decide(106.0, _est(committed=(0.605, 0.0, 0.3), moved=True), near_tcp,
                 _run(106.0), retreat_target=retreat)
    assert c.phases.phase == "NEAR" and i.action == PLAN
    c.note_leg_started(i.goal, 106.0, 2.0)

    # leg completes; a jump OUTSIDE the box must NOT be chased
    c.note_leg_finished()
    i = c.decide(109.0, _est(committed=(0.78, 0.0, 0.3), moved=True), near_tcp,
                 _run(109.0), retreat_target=retreat)
    assert c.phases.phase == "NEAR" and i.action != PLAN

    # TCP reaches the target -> GRASP; frozen even for a completed leg
    grasp_tcp = np.array([0.605, 0.0, 0.31])
    c.decide(110.0, _est(committed=(0.605, 0.0, 0.3)), grasp_tcp, _run(110.0),
             retreat_target=retreat)
    assert c.phases.phase == "GRASP"
    c.note_leg_finished()
    i = c.decide(111.0, _est(committed=(0.63, 0.0, 0.3), moved=True), grasp_tcp,
                 _run(111.0), retreat_target=retreat)
    assert i.action != PLAN

    # grasp complete -> begin_retreat -> the retreat leg goes to the FIXED pose
    c.begin_retreat()
    i = c.decide(112.0, _est(committed=(0.9, 0.0, 0.3), moved=True), grasp_tcp,
                 _run(112.0), retreat_target=retreat)
    assert c.phases.phase == "RETREAT"
    assert i.action == PLAN and np.allclose(i.goal, retreat)


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
