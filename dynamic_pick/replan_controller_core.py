# SPDX-License-Identifier: Apache-2.0
"""
Segmented re-plan + preempt controller — PURE decision core
(human-dynamic-plan-s1 report §1 candidate C1, §2, §3).

This is the brain of the dynamic-pick controller: given the pick-target tracker's
estimate, the arm's TCP position, and the latest human-safety gate decision, it
decides each tick whether to

  PLAN   plan a short leg to the freshest committed target and put it on the wire
         (preempting any leg already running) — ONLY through the gate acceptance
         shim, so this can never authorise motion while the gate is not RUN;
  NOOP   let the leg already running continue (nothing genuinely changed);
  HOLD   command no new motion (gate not RUN, no committed target, frozen phase,
         or rate-limited) — the gate driver owns freeze/slow/cancel of any in-flight
         leg independently.

It composes three pure pieces built alongside it: `GateAcceptanceShim` (the sole
motion authority — report §2), `PickPhaseMachine` (per-phase re-plan policy —
report §3), and the tracker's deadband/hysteresis `TargetEstimate`. The design
invariant, verified by unit tests and an end-to-end assertion, is:

  **A PLAN intent is emitted ONLY when the gate acceptance shim returned FORWARD
  (gate RUN + fresh). No tracker/phase input can produce a PLAN while the gate is
  not RUN.**

Re-planning is rate-limited by `min_replan_interval_s`: there is no benefit to
re-planning faster than the target estimate settles or faster than the 5 % speed
cap can act on (report §3). Preemption only happens on a GENUINE move (the
tracker's committed goal shifted beyond its deadband AND differs from the goal in
flight), never on estimate jitter.

Pure: standard library + NumPy only; no rclpy, no torch, all time passed in.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from gate_acceptance import FORWARD, GateAcceptanceShim, GateView
from pick_phases import PickPhaseMachine, RETREAT
from pick_target_tracker import TargetEstimate, TRACKING

# intents
PLAN = "PLAN"
NOOP = "NOOP"
HOLD = "HOLD"


@dataclass(frozen=True)
class Intent:
    """One controller decision for a tick."""

    action: str                    # PLAN / NOOP / HOLD
    goal: Optional[np.ndarray]     # target to plan to (base frame, m) when action == PLAN
    reason: str
    phase: str
    gate_action: Optional[str]
    gate_forward: bool             # did the acceptance shim FORWARD this tick?
    preempt: bool                  # PLAN that preempts a running leg (vs. a fresh start)


class ReplanController:
    """Stateful pure controller. Owns the acceptance shim and the phase machine;
    the tracker lives outside and its estimate is passed in each tick."""

    def __init__(
        self,
        *,
        shim: Optional[GateAcceptanceShim] = None,
        phases: Optional[PickPhaseMachine] = None,
        min_replan_interval_s: float = 0.5,
    ) -> None:
        if min_replan_interval_s < 0:
            raise ValueError("min_replan_interval_s must be >= 0")
        self.shim = shim or GateAcceptanceShim()
        self.phases = phases or PickPhaseMachine()
        self.min_replan_interval_s = float(min_replan_interval_s)
        # in-flight bookkeeping. A leg is considered RUNNING until its planned
        # DURATION elapses (or note_leg_finished fires early) — NOT until the async
        # FollowJointTrajectory result callback returns. The result callback is
        # subject to accept/preempt ordering races that make a still-executing leg
        # look finished, which would turn every preemption into a fresh plan; timing
        # the leg by its own duration makes "is a leg running when the target moved?"
        # deterministic (report §1: preemption is the behaviour under measurement).
        self._goal_in_flight: Optional[np.ndarray] = None
        self._leg_end_mono: Optional[float] = None
        self._last_plan_mono: Optional[float] = None

    # ---- leg lifecycle (the ROS node calls these on goal accept / result) --- #
    def note_leg_started(self, goal, now: float, duration_s: float) -> None:
        self._goal_in_flight = np.asarray(goal, float).reshape(3).copy()
        self._leg_end_mono = float(now) + max(float(duration_s), 0.1)
        self._last_plan_mono = float(now)

    def note_leg_finished(self) -> None:
        self._leg_end_mono = None
        self._goal_in_flight = None

    def _leg_running(self, now: float) -> bool:
        return self._leg_end_mono is not None and now < self._leg_end_mono

    def begin_retreat(self) -> None:
        self.phases.begin_retreat()

    # ---- the per-tick decision -------------------------------------------- #
    def decide(
        self,
        now: float,
        est: TargetEstimate,
        tcp_xyz,
        gate: GateView,
        *,
        retreat_target=None,
    ) -> Intent:
        """Decide this tick. `retreat_target` is the fixed pre-planned retreat pose
        used only in the RETREAT phase (the tracker's target is ignored there)."""
        # 1) the gate acceptance shim is consulted FIRST and is decisive: if it does
        #    not FORWARD, no PLAN can be emitted, whatever the tracker/phase say.
        ruling = self.shim.admit(gate, now)

        # advance the phase machine on current geometry (safe regardless of gate).
        phase = self.phases.update_phase(tcp_xyz, est.committed_target)

        if ruling.verdict != FORWARD:
            return Intent(HOLD, None,
                          f"acceptance shim HOLD: {ruling.reason}",
                          phase, ruling.gate_action, False, False)

        # 2) gate is RUN + fresh. Choose the goal for this phase.
        if phase == RETREAT:
            goal = None if retreat_target is None else np.asarray(retreat_target, float).reshape(3)
            if goal is None:
                return Intent(HOLD, None, "retreat: no retreat pose supplied -> hold",
                              phase, ruling.gate_action, True, False)
        else:
            goal = est.committed_target
            if goal is None or est.state != TRACKING:
                return Intent(HOLD, None,
                              f"no committed target (tracker {est.state}) -> hold, do not chase",
                              phase, ruling.gate_action, True, False)

        # 3) is a new leg needed?
        if not self._leg_running(now):
            # nothing in flight: start this phase's leg (rate-limited so we do not
            # spam plans while one is being set up).
            if self._rate_limited(now):
                return Intent(NOOP, None, "gate RUN but within re-plan rate limit -> wait",
                              phase, ruling.gate_action, True, False)
            return Intent(PLAN, goal, "gate RUN, no leg in flight -> plan a leg to the target",
                          phase, ruling.gate_action, True, False)

        # a leg IS running: only PREEMPT on a genuine move that the phase policy
        # permits and the rate limit allows.
        moved = self._goal_differs(goal)
        if phase == RETREAT:
            # retreat is frozen and to a fixed pose: never preempt.
            return Intent(NOOP, None, "retreat leg running (frozen) -> let it complete",
                          phase, ruling.gate_action, True, False)
        if not (est.committed_moved and moved):
            return Intent(NOOP, None, "committed target unchanged -> let the leg run",
                          phase, ruling.gate_action, True, False)
        replan = self.phases.rule_replan(goal)
        if not replan.allow_replan:
            return Intent(NOOP, None, f"target moved but {replan.reason} -> let the leg run",
                          phase, ruling.gate_action, True, False)
        if self._rate_limited(now):
            return Intent(NOOP, None, "target moved but within re-plan rate limit -> wait",
                          phase, ruling.gate_action, True, False)
        return Intent(PLAN, goal, f"target moved ({replan.reason}) -> preempt with a fresh leg",
                      phase, ruling.gate_action, True, True)

    # ---- helpers ---------------------------------------------------------- #
    def _rate_limited(self, now: float) -> bool:
        if self._last_plan_mono is None:
            return False
        return (now - self._last_plan_mono) < self.min_replan_interval_s

    def _goal_differs(self, goal) -> bool:
        if self._goal_in_flight is None:
            return True
        return bool(np.linalg.norm(np.asarray(goal, float).reshape(3)
                                   - self._goal_in_flight) > 1e-9)
