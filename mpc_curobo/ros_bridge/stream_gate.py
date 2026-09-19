#!/usr/bin/env python3
"""Gate acceptance shim for a streamed MPC command (report human-dynamic-plan-s1
§2 "motion-authority mediator", §10 Phase A).

The MPC solver proposes a fresh joint command every control tick. This pure core
sits between that stream and the driver's forward-position controller and decides,
per tick, whether to FORWARD a command or HOLD — so that:

  * nothing moves unless the merged human-safety gate says RUN (task hard rule);
  * a command arriving while the gate is STOP/BACKSTOP is DROPPED, never
    forwarded — "a goal arriving during STOP must not move the arm" is true by
    construction, not by timing;
  * when RUN resumes, the arm heads for the *freshest* MPC command, never a
    stored stale one (report §2 "the resume target is re-derived, never replayed");
  * every forwarded command is inside the cell's measured joint windows and no
    single tick steps more than a capped per-joint delta (AGENTS.md delta guard);
  * on HOLD the shim republishes the last safe position, so the arm holds whether
    the controller starves or is actively commanded.

This core is PURE (stdlib + JointWindows) — no rclpy, no torch, no wall clock of
its own — so every rule is unit-tested offline and replayed. The ROS node
(mpc_gate_bridge.py) is a thin shell that feeds it the gate Decision + the MPC
command and publishes whatever it returns.

SLOW is treated as HOLD here (conservative): the task hard rule is "nothing moves
unless the gate says RUN". The report's graded-SLOW (keep moving slower) is a
later, owner-gated option and is deliberately NOT enabled in this prototype.
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.joint_windows import JOINT_ORDER, JointWindows, wrap_near  # noqa: E402

# gate actions (mirror humangate/gate_core.py so the shim needs no import of it)
RUN = "RUN"
SLOW = "SLOW"
STOP = "STOP"
BACKSTOP = "BACKSTOP"

# shim outcomes
FORWARD = "FORWARD"  # publish target_position (a bounded step toward the MPC cmd)
HOLD = "HOLD"        # do not advance; republish hold_position


@dataclass(frozen=True)
class StreamAction:
    """What the ROS node should do this tick."""

    kind: str                       # FORWARD | HOLD
    target_position: List[float]    # joint positions to publish (always populated)
    joint_names: List[str]
    speed_fraction: float           # effective fraction applied to the step cap
    backstop_engaged: bool          # gate demanded the independent backstop
    reason: str
    window_ok: bool                 # was the raw MPC command inside the windows


@dataclass
class GatedStreamController:
    """Stateful acceptance shim. One instance per bridge; call step() per tick."""

    windows: JointWindows
    joint_names: List[str] = field(default_factory=lambda: list(JOINT_ORDER))
    # Max per-joint change per control tick at fraction 1.0 (rad). With a 0.02 s
    # tick this bounds commanded joint speed to max_step_per_tick / dt. Kept small
    # so the streamed motion stays at a crawl comparable to the cell's 5% cap.
    max_step_per_tick: float = 0.01
    operating_fraction: float = 1.0
    # Internal hold target (last safe commanded position). None until first seed.
    _hold: Optional[List[float]] = None

    def seed(self, position: Sequence[float]) -> None:
        """Initialise the hold position from the measured start configuration."""
        self._hold = [float(v) for v in position]

    def _current_hold(self, measured: Optional[Sequence[float]]) -> List[float]:
        if self._hold is not None:
            return list(self._hold)
        if measured is not None:
            return [float(v) for v in measured]
        # fail-safe: hold at zeros only if we truly have nothing (never in practice)
        return [0.0] * len(self.joint_names)

    def step(
        self,
        gate_action: str,
        gate_speed_fraction: Optional[float],
        mpc_command: Optional[Sequence[float]],
        measured_position: Optional[Sequence[float]] = None,
    ) -> StreamAction:
        """Decide FORWARD vs HOLD for this tick.

        gate_action / gate_speed_fraction come straight from the gate Decision.
        mpc_command is the freshest MPC-proposed joint position (or None).
        measured_position seeds the hold if we have not committed one yet.
        """
        hold = self._current_hold(measured_position)

        # --- not RUN => hold, drop any MPC command ------------------------- #
        if gate_action != RUN:
            self._hold = hold  # keep holding wherever we last safely were
            return StreamAction(
                kind=HOLD,
                target_position=hold,
                joint_names=list(self.joint_names),
                speed_fraction=0.0,
                backstop_engaged=(gate_action == BACKSTOP),
                reason=f"gate {gate_action}: holding, MPC command dropped",
                window_ok=True,
            )

        # --- RUN but no fresh target => hold ------------------------------- #
        if mpc_command is None:
            self._hold = hold
            return StreamAction(
                kind=HOLD, target_position=hold, joint_names=list(self.joint_names),
                speed_fraction=0.0, backstop_engaged=False,
                reason="gate RUN but no fresh MPC command — holding", window_ok=True,
            )

        cmd = {j: float(v) for j, v in zip(self.joint_names, mpc_command)}

        # --- window guard: an out-of-window MPC command is refused (hold) -- #
        viol = self.windows.violations(cmd)
        if viol:
            self._hold = hold
            j, v, lo, hi = viol[0]
            return StreamAction(
                kind=HOLD, target_position=hold, joint_names=list(self.joint_names),
                speed_fraction=0.0, backstop_engaged=False,
                reason=(f"MPC command outside measured window: {j}={v:.4f} "
                        f"not in [{lo:.4f},{hi:.4f}] — holding"),
                window_ok=False,
            )

        # --- bounded step toward the freshest command --------------------- #
        frac = self.operating_fraction if gate_speed_fraction is None \
            else max(0.0, min(1.0, gate_speed_fraction))
        max_step = self.max_step_per_tick * frac
        target = []
        for j, cur in zip(self.joint_names, hold):
            # step toward the 2*pi-equivalent of the command nearest the current
            # position, so a wrapped angle (e.g. wrist_1) moves the short way.
            want = wrap_near(cmd[j], cur)
            delta = want - cur
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            target.append(cur + delta)
        # defensive: keep the stepped target inside the windows
        target = [self.windows.clamp({j: v})[j]
                  for j, v in zip(self.joint_names, target)]
        self._hold = list(target)
        return StreamAction(
            kind=FORWARD, target_position=target, joint_names=list(self.joint_names),
            speed_fraction=frac, backstop_engaged=False,
            reason=f"gate RUN: stepping toward freshest MPC command (frac={frac:.2f})",
            window_ok=True,
        )
