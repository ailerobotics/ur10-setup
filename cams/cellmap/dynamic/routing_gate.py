# SPDX-License-Identifier: Apache-2.0
"""The safety principle of the dynamic layer, as pure, testable code.

PRINCIPLE (report human-dynamic-plan-s1 §5): a person — or any obstacle — in the
map is a ROUTING aid, never the stop mechanism. The finished human-safety design
rejected the map as a stop because a missed detection means no obstacle voxel and
the arm would proceed (it fails OPEN). So:

  * The merged human-safety gate (``humangate/gate_core.py``, fail-toward-stop) is
    the SINGLE authority over motion. Whether the arm may move is a function of
    the camera-side human-safety state ONLY — it never depends on what nvblox did
    or did not put in the map.
  * The nvblox dynamic layer only changes the SHAPE of the plan cuMotion returns
    (the route). It can make a route go around a box or a person-sized volume, and
    let the route come back when they leave. It can never, by adding or removing
    an obstacle, change a STOP into motion.

This module makes that separation executable. ``authorize_plan`` is the acceptance
point: a planned trajectory (whatever cuMotion produced against whatever map) is
forwarded to the driver ONLY when the gate action is RUN. It is PLAN-ONLY here —
it returns the trajectory to forward or ``None`` to hold; it imports no driver,
controller, or dashboard interface, and it does not touch the arming interlock in
``humangate_driver/arming_interlock.py`` (that remains the sole path to a real
arm). The GPU proofs show routes change with the map; the unit test shows that
map changes never flip a STOP to motion.
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Optional, Sequence

# Reuse the merged gate's action vocabulary as the single source of truth.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "humangate"))
try:
    from gate_core import BACKSTOP, RUN, SLOW, STOP  # noqa: F401
except Exception:  # pragma: no cover - fall back to literals if path differs
    RUN, SLOW, STOP, BACKSTOP = "RUN", "SLOW", "STOP", "BACKSTOP"


@dataclass(frozen=True)
class PlannedRoute:
    """A cuMotion result, reduced to what a proof needs to compare routes.

    ``waypoints`` are joint configurations (or TCP points); ``routed_around`` is
    the set of obstacle labels the planner steered clear of. Two routes differ
    when their waypoints differ — that is how "the route changed" is asserted."""

    waypoints: Sequence[Sequence[float]] = field(default_factory=list)
    routed_around: frozenset = frozenset()
    planning_time_s: Optional[float] = None
    map_had_dynamic: bool = False

    def differs_from(self, other: "PlannedRoute", tol: float = 1e-6) -> bool:
        a, b = list(self.waypoints), list(other.waypoints)
        if len(a) != len(b):
            return True
        for wa, wb in zip(a, b):
            if len(wa) != len(wb):
                return True
            if any(abs(x - y) > tol for x, y in zip(wa, wb)):
                return True
        return False


@dataclass(frozen=True)
class Authorization:
    """Result of the acceptance point."""

    forward: bool                       # True => hand the plan to the driver
    plan: Optional[PlannedRoute]        # the plan to forward, or None on hold
    reason: str


def authorize_plan(gate_action: str, plan: Optional[PlannedRoute]) -> Authorization:
    """THE acceptance point. Forward the planned trajectory to the driver ONLY
    when the gate says RUN. This depends on ``gate_action`` alone — never on the
    plan's contents or on whether the map had a dynamic obstacle — so no map state
    can turn a non-RUN gate into motion.

    A missing plan is never motion. SLOW keeps commanding but is the gate node's
    speed-slider concern, not a new-goal decision: to stay fail-safe here, only an
    explicit RUN forwards a *new* goal; SLOW/STOP/BACKSTOP hold new goals."""
    if plan is None:
        return Authorization(False, None, "no plan to forward")
    if gate_action == RUN:
        return Authorization(True, plan, "gate RUN — plan forwarded to driver")
    return Authorization(
        False, None,
        f"gate {gate_action} — new goal held (map contents are irrelevant to this)")


def route_changed_by_dynamic(without_dynamic: PlannedRoute,
                             with_dynamic: PlannedRoute) -> bool:
    """True when adding the dynamic layer changed the planned route. Used by the
    GPU route proof and re-used in the unit test with synthetic routes."""
    return with_dynamic.differs_from(without_dynamic)


def map_can_never_start_motion(gate_actions: Sequence[str],
                               plans_by_map: Sequence[Optional[PlannedRoute]]) -> bool:
    """Exhaustive check backing the principle: across every gate action that is
    not RUN and every possible plan (any map, with or without a dynamic
    obstacle), the acceptance point forwards nothing. Returns True iff the
    invariant holds for all combinations."""
    for action in gate_actions:
        for plan in plans_by_map:
            auth = authorize_plan(action, plan)
            if action != RUN and auth.forward:
                return False
    return True
