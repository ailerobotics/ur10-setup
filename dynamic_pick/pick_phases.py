# SPDX-License-Identifier: Apache-2.0
"""
Pick phases and where re-planning is allowed (human-dynamic-plan-s1 report §3).

A pick is not one motion; it is a sequence of phases, and how much the arm may
re-plan toward a moving target differs by phase. The report's table:

  | Phase        | Re-planning | Rationale                                            |
  |--------------|-------------|------------------------------------------------------|
  | FAR approach | ALLOWED     | large clearances; capped speed; cuMotion knows the cell |
  | NEAR approach| LIMITED     | precision matters but only inside a small pre-cleared box |
  | GRASP        | FROZEN      | extrinsics +-5 cm, re-estimating mid-grasp is dangerous |
  | RETREAT      | FROZEN      | a known collision-checked path back; no reason to re-estimate |

This module is the PURE phase state machine + the per-phase policy. It owns NO
motion; it only answers, for the current phase, whether a re-plan toward a freshly
moved target is ALLOWED, LIMITED (only inside a small workspace box around the
committed goal), or FROZEN (never re-plan; run the committed leg to completion).

The machine advances FAR -> NEAR by TCP proximity to the committed target and
NEAR -> GRASP when within the grasp radius; GRASP -> RETREAT and RETREAT ->
(done) are commanded explicitly by the controller (a grasp completes, a retreat
target is a fixed pre-planned pose). Losing the target in FAR/NEAR sends the
machine back toward hold; it never advances INTO grasp without a fresh committed
target (fail toward hold, report §3).

Pure: standard library + NumPy only; every transition is unit-tested offline.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

# --- phases --- #
FAR = "FAR"
NEAR = "NEAR"
GRASP = "GRASP"
RETREAT = "RETREAT"
PHASES = (FAR, NEAR, GRASP, RETREAT)

# --- per-phase re-plan policy --- #
ALLOWED = "ALLOWED"   # re-plan toward the freshest committed target, rate-limited
LIMITED = "LIMITED"   # re-plan only if the new goal stays inside the near-approach box
FROZEN = "FROZEN"     # never re-plan; run the committed leg to completion
POLICY = {FAR: ALLOWED, NEAR: LIMITED, GRASP: FROZEN, RETREAT: FROZEN}


@dataclass(frozen=True)
class PhaseParams:
    """Geometry that drives the phase transitions and the LIMITED box. All metres.

    Provisional/simulation values; the report marks the near-approach box and the
    grasp radius as owner-tuned per cell. Tune the box DOWN (tighter) only."""

    near_enter_m: float = 0.20      # TCP within this of the committed target -> NEAR (report "last ~10-20 cm")
    grasp_enter_m: float = 0.03     # within this -> GRASP (freeze)
    near_box_half_m: float = 0.10   # LIMITED: a re-planned near goal must stay within this of the box centre

    def validate(self) -> "PhaseParams":
        if not (self.grasp_enter_m < self.near_enter_m):
            raise ValueError("grasp_enter_m must be < near_enter_m")
        if self.near_box_half_m <= 0:
            raise ValueError("near_box_half_m must be > 0")
        return self


@dataclass(frozen=True)
class ReplanRuling:
    """The phase machine's answer for a proposed re-plan this tick."""

    phase: str
    policy: str
    allow_replan: bool           # may the controller plan a new leg to the freshest target now?
    reason: str


class PickPhaseMachine:
    """Tracks the current pick phase and rules on re-plans. The controller feeds it
    the TCP position and the committed target each tick and calls transition
    helpers when a grasp completes or a retreat is commanded."""

    def __init__(self, params: Optional[PhaseParams] = None) -> None:
        self.params = (params or PhaseParams()).validate()
        self._phase = FAR
        self._near_box_center: Optional[np.ndarray] = None   # frozen when NEAR is entered

    @property
    def phase(self) -> str:
        return self._phase

    def reset(self) -> None:
        self._phase = FAR
        self._near_box_center = None

    def begin_retreat(self) -> None:
        """Called by the controller once a grasp is complete (or aborted): the
        retreat is a fixed, pre-planned pose and is FROZEN."""
        self._phase = RETREAT
        self._near_box_center = None

    def update_phase(self, tcp_xyz, committed_target) -> str:
        """Advance FAR/NEAR/GRASP by proximity. GRASP and RETREAT are only entered
        here (GRASP) or by begin_retreat (RETREAT); a lost target (committed None)
        holds the phase, never advances it."""
        if self._phase == RETREAT:
            return self._phase
        if committed_target is None or tcp_xyz is None:
            # no fresh committed target: hold the current phase, do not advance into
            # NEAR/GRASP on stale geometry (fail toward hold).
            return self._phase
        d = float(np.linalg.norm(np.asarray(tcp_xyz, float).reshape(3)
                                 - np.asarray(committed_target, float).reshape(3)))
        p = self.params
        if self._phase == FAR:
            if d <= p.grasp_enter_m:
                self._enter_near(committed_target)
                self._phase = GRASP
            elif d <= p.near_enter_m:
                self._enter_near(committed_target)
                self._phase = NEAR
        elif self._phase == NEAR:
            if d <= p.grasp_enter_m:
                self._phase = GRASP
            elif d > p.near_enter_m * 1.5:
                # target moved well back out of the near band: drop to FAR (hysteresis
                # via the 1.5x band so we do not flap on the boundary).
                self._phase = FAR
                self._near_box_center = None
        # GRASP does not auto-leave; the controller calls begin_retreat().
        return self._phase

    def _enter_near(self, committed_target) -> None:
        if self._near_box_center is None:
            self._near_box_center = np.asarray(committed_target, float).reshape(3).copy()

    def rule_replan(self, proposed_target) -> ReplanRuling:
        """Decide whether a re-plan to `proposed_target` is allowed in the current
        phase. FAR: allowed. NEAR: allowed only if the proposed goal stays inside the
        pre-cleared box around the near-entry centre. GRASP/RETREAT: frozen."""
        phase = self._phase
        policy = POLICY[phase]
        if policy == ALLOWED:
            return ReplanRuling(phase, policy, True,
                                "far approach: re-plan to freshest target allowed (rate-limited)")
        if policy == FROZEN:
            return ReplanRuling(phase, policy, False,
                                f"{phase.lower()} is frozen: run the committed leg, do not re-estimate")
        # LIMITED (NEAR): only inside the pre-cleared box.
        if proposed_target is None or self._near_box_center is None:
            return ReplanRuling(phase, policy, False,
                                "near approach: no committed target / box -> hold")
        off = float(np.linalg.norm(np.asarray(proposed_target, float).reshape(3)
                                   - self._near_box_center))
        if off <= self.params.near_box_half_m:
            return ReplanRuling(phase, policy, True,
                                f"near approach: re-plan allowed inside the pre-cleared box ({off*100:.1f} cm from centre)")
        return ReplanRuling(phase, policy, False,
                            f"near approach: proposed goal {off*100:.1f} cm outside the "
                            f"{self.params.near_box_half_m*100:.0f} cm box -> hold, do not chase")
