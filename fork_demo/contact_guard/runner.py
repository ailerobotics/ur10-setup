"""ContactGuardRunner: the pure facade the ROS node, the mock E2E, and the
step_replay / real_fork_pick integration all drive.

It owns the per-leg :class:`detector.ContactGuard`, the stop-and-probe baseline,
and the reaction decision. It never imports rclpy and never commands the arm --
callers feed it samples and act on the :class:`reaction.ReactionPlan` it returns
(cancel is the caller's existing cancel-all path; any back-off goes through the
gated execute path on URSim/mock only).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .detector import ContactGuard, Detection
from .envelope import Envelope, LegBand
from .params import ContactGuardParams
from .reaction import AABB, ReactionPlan, plan_reaction


@dataclass
class LegContext:
    name: str
    contact: bool
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)   # base_link travel dir
    box: AABB = (0.10, 1.35, -0.45, 1.25, -0.35, 1.00)   # real_fork_pick default


class ContactGuardRunner:
    def __init__(self, params: ContactGuardParams, env: Optional[Envelope] = None):
        self.p = params
        self.env = env or Envelope()
        self._guard: Optional[ContactGuard] = None
        self._ctx: Optional[LegContext] = None
        self._creeping = False
        self._baseline_samples: List[Dict[str, float]] = []

    # -- leg / segment lifecycle --------------------------------------------
    def start_leg(self, ctx: LegContext) -> None:
        lb: Optional[LegBand] = self.env.leg(ctx.name)
        self._guard = ContactGuard(self.p, lb, ctx.name, ctx.contact)
        self._ctx = ctx
        self._creeping = False
        self._baseline_samples = []

    def begin_probe_dwell(self) -> None:
        """Enter the halt-and-baseline dwell for the next stop-and-probe segment."""
        self._baseline_samples = []
        self._creeping = False

    def add_baseline_sample(self, feats: Dict[str, float]) -> None:
        self._baseline_samples.append(dict(feats))

    def begin_creep(self) -> bool:
        """Finish the dwell, install the mean baseline, start creeping.

        Returns False (fail toward stop) if too few baseline samples were
        gathered to trust the fresh baseline.
        """
        if self._guard is None:
            return False
        if len(self._baseline_samples) < self.p.probe_min_samples:
            return False
        keys = set().union(*(s.keys() for s in self._baseline_samples))
        mean = {k: float(np.mean([s.get(k, 0.0) for s in self._baseline_samples]))
                for k in keys}
        self._guard.set_probe_baseline(mean)
        self._creeping = True
        return True

    # -- per-sample ----------------------------------------------------------
    def feed(self, t: float, s_cmd: float, s_act: float, feats: Dict[str, float],
             executing: bool = True) -> Optional[Detection]:
        if self._guard is None:
            return None
        return self._guard.update(t, s_cmd, s_act, feats, executing=executing,
                                  creeping=self._creeping)

    def react(self, det: Detection, tcp_pos: Sequence[float]) -> ReactionPlan:
        ctx = self._ctx
        assert ctx is not None
        return plan_reaction(det, tcp_pos, ctx.axis, ctx.box, self.p, ctx.contact)
