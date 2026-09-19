# SPDX-License-Identifier: Apache-2.0
"""
Gate acceptance shim — the single motion-authority mediator between the pick
controller and the driver (human-dynamic-plan-s1 report §2).

In a plan-then-execute world the gate was a supervisor watching one goal. In a
re-planning world the gate must become an explicit motion-authority mediator: the
pick controller may only ever put a new trajectory goal on the wire THROUGH this
shim, and the shim forwards a new goal ONLY while the gate says RUN. This is the
line that makes "a new goal arriving while the gate is not RUN must not move the
arm" true BY CONSTRUCTION, not by timing luck.

The mapping (this task's rule set item 3, report §2 column C1):

  RUN         -> FORWARD the goal (motion toward the freshest target is authorised)
  SLOW        -> HOLD: do NOT forward a NEW leg; the in-flight leg keeps running,
                 slowed through the existing speed-slider path owned by the gate
                 driver. A person is near enough to slow; we do not START new motion
                 toward a (possibly moving) target while they are close.
  STOP        -> HOLD: stop issuing goals and hold. The gate driver freezes/cancels
                 the in-flight leg; the shim forwards nothing.
  BACKSTOP    -> HOLD (same as STOP; the gate has cancelled and the arm holds)
  KEEP-MOVING -> this is simply the gate returning to RUN after its healthy CLEAR
                 dwell; the controller then plans AFRESH toward the freshest target
                 (never replays a stale goal). The shim treats it as RUN.

Freshness: a stale or missing gate decision is NEVER RUN. Absence of a RUN signal
is not clearance — the shim fails toward HOLD, matching the gate core's own
fail-toward-stop discipline. The gate publishes a decision every ~50 ms
(decide_rate_hz 20); a decision older than `max_decision_age_s` is treated as
"gate silent" and the shim holds.

This module is PURE: standard library only, no rclpy, no NumPy. The verification
that "no input sequence from the tracker or planner produces motion while the gate
is not RUN" is an exhaustive offline unit test over this function, plus an
end-to-end assertion that every forwarded goal was forwarded under a RUN decision.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# gate actions (mirror humangate.gate_core; duplicated as bare strings so this
# module stays import-pure and has no dependency on the gate package).
RUN = "RUN"
SLOW = "SLOW"
STOP = "STOP"
BACKSTOP = "BACKSTOP"

# shim verdicts
FORWARD = "FORWARD"   # put the goal on the wire now
HOLD = "HOLD"         # keep the goal pending; do not command new motion


@dataclass(frozen=True)
class GateView:
    """The controller's read of the latest gate decision. `action` is the gate's
    action string; `t_mono` is when it was received (monotonic seconds); None means
    no decision has ever been seen."""

    action: Optional[str]
    t_mono: Optional[float]


@dataclass(frozen=True)
class AcceptanceRuling:
    verdict: str          # FORWARD or HOLD
    reason: str
    gate_action: Optional[str]
    gate_fresh: bool


class GateAcceptanceShim:
    """Stateless-per-call acceptance decision. Hold a reference to it and call
    `admit()` every time the controller wants to put a NEW goal on the wire."""

    def __init__(self, *, max_decision_age_s: float = 0.30) -> None:
        # Defaults to the gate's own heartbeat_timeout_s (0.3 s): if the gate has
        # gone silent for longer than one heartbeat window, we cannot trust that it
        # is still RUN, so we hold. Fail toward hold.
        if max_decision_age_s <= 0:
            raise ValueError("max_decision_age_s must be > 0")
        self.max_decision_age_s = float(max_decision_age_s)

    def _fresh(self, gate: GateView, now: float) -> bool:
        if gate.action is None or gate.t_mono is None:
            return False
        return (now - gate.t_mono) <= self.max_decision_age_s

    def admit(self, gate: GateView, now: float) -> AcceptanceRuling:
        """May the controller forward a NEW trajectory goal right now?

        FORWARD iff the latest gate decision is RUN and fresh. Everything else —
        SLOW, STOP, BACKSTOP, an unknown action, a stale decision, or no decision
        at all — HOLDs. This is the whole safety contract of the shim.
        """
        fresh = self._fresh(gate, now)
        if not fresh:
            age = ("never" if gate.t_mono is None
                   else f"{(now - gate.t_mono) * 1e3:.0f} ms old")
            return AcceptanceRuling(
                HOLD, f"gate decision not fresh ({age}) -> HOLD (absence of RUN is not clearance)",
                gate.action, False)
        if gate.action == RUN:
            return AcceptanceRuling(
                FORWARD, "gate RUN -> forward the goal", gate.action, True)
        if gate.action == SLOW:
            return AcceptanceRuling(
                HOLD, "gate SLOW -> hold new goals; the in-flight leg is slowed via the slider",
                gate.action, True)
        # STOP / BACKSTOP / anything else
        return AcceptanceRuling(
            HOLD, f"gate {gate.action} -> hold; issue no goals, the gate holds/cancels the arm",
            gate.action, True)
