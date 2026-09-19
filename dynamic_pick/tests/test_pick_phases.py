#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the pick-phase machine + per-phase re-plan policy (report §3)."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pick_phases import (  # noqa: E402
    ALLOWED, FAR, FROZEN, GRASP, LIMITED, NEAR, RETREAT, PhaseParams,
    PickPhaseMachine)

TGT = np.array([0.6, 0.0, 0.3])


def test_starts_far_and_replan_allowed():
    m = PickPhaseMachine()
    assert m.phase == FAR
    r = m.rule_replan(TGT)
    assert r.policy == ALLOWED and r.allow_replan


def test_far_to_near_by_proximity():
    m = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.03))
    # TCP far from target -> stays FAR
    assert m.update_phase(TGT + np.array([0.5, 0, 0]), TGT) == FAR
    # TCP within near band -> NEAR
    assert m.update_phase(TGT + np.array([0.15, 0, 0]), TGT) == NEAR


def test_near_is_limited_to_the_box():
    m = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.03,
                                     near_box_half_m=0.10))
    m.update_phase(TGT + np.array([0.15, 0, 0]), TGT)   # enter NEAR, box centred at TGT
    assert m.phase == NEAR
    # a re-plan to a nearby goal (inside the box) is allowed
    r_in = m.rule_replan(TGT + np.array([0.05, 0, 0]))
    assert r_in.policy == LIMITED and r_in.allow_replan
    # a re-plan to a goal outside the box is refused (do not chase)
    r_out = m.rule_replan(TGT + np.array([0.30, 0, 0]))
    assert r_out.policy == LIMITED and not r_out.allow_replan


def test_near_to_grasp_and_frozen():
    m = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.03))
    m.update_phase(TGT + np.array([0.15, 0, 0]), TGT)   # NEAR
    m.update_phase(TGT + np.array([0.02, 0, 0]), TGT)   # within grasp radius -> GRASP
    assert m.phase == GRASP
    r = m.rule_replan(TGT)
    assert r.policy == FROZEN and not r.allow_replan


def test_retreat_is_frozen_and_explicit():
    m = PickPhaseMachine()
    m.begin_retreat()
    assert m.phase == RETREAT
    r = m.rule_replan(TGT)
    assert r.policy == FROZEN and not r.allow_replan
    # retreat does not fall back to FAR on geometry
    assert m.update_phase(TGT + np.array([1.0, 0, 0]), TGT) == RETREAT


def test_lost_target_holds_phase_never_advances():
    m = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.03))
    # committed target None (lost) must not advance FAR->NEAR even if tcp is close
    assert m.update_phase(TGT + np.array([0.01, 0, 0]), None) == FAR


def test_near_drops_to_far_when_target_moves_well_out():
    m = PickPhaseMachine(PhaseParams(near_enter_m=0.2, grasp_enter_m=0.03))
    m.update_phase(TGT + np.array([0.15, 0, 0]), TGT)   # NEAR
    assert m.phase == NEAR
    # target now 0.4 m from tcp (> 1.5 * near_enter) -> back to FAR
    assert m.update_phase(TGT + np.array([0.40, 0, 0]), TGT) == FAR


def test_params_validation():
    try:
        PhaseParams(near_enter_m=0.02, grasp_enter_m=0.05).validate()
        assert False, "should reject grasp >= near"
    except ValueError:
        pass


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
