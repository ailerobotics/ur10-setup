"""Reaction policy: cancel, optional in-axis back-off, HOLD, one-screen verdict.

Pure geometry + text. On a detection the live node always cancels the running
goal through the existing cancel-all path (``force_abort.py`` / execute_plan's
FollowJointTrajectory cancel). This module decides the *follow-up*: on a contact
leg, a short back-off along that leg's own axis (default 15 mm at 2 %), but ONLY
if the back-off target is inside the workspace box; otherwise HOLD. It never
retries automatically and never itself commands the arm -- it returns a plan the
(gated) execute path may carry out on URSim/mock.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from .detector import Detection
from .params import ContactGuardParams

AABB = Tuple[float, float, float, float, float, float]  # xmin,xmax,ymin,ymax,zmin,zmax


def in_box(p: Sequence[float], box: AABB) -> bool:
    x, y, z = p[0], p[1], p[2]
    return (box[0] <= x <= box[1] and box[2] <= y <= box[3]
            and box[4] <= z <= box[5])


@dataclass
class ReactionPlan:
    action: str                 # "BACKOFF" | "HOLD"
    backoff_target: Optional[Tuple[float, float, float]]
    speed_frac: float
    reason: str
    verdict: str                # the one-screen operator text

    @property
    def will_move(self) -> bool:
        return self.action == "BACKOFF" and self.backoff_target is not None


def plan_reaction(det: Detection, tcp_pos: Sequence[float],
                  leg_axis: Sequence[float], box: AABB,
                  p: ContactGuardParams, contact: bool) -> ReactionPlan:
    """Decide the reaction for a detection.

    ``leg_axis`` is the leg's unit travel direction in base_link (insertion
    direction for a contact leg); back-off is along ``-leg_axis``. Transit legs
    and the STALE trigger HOLD (no in-axis retreat defined).
    """
    verdict = _verdict(det)
    if not contact or det.trigger == "STALE":
        return ReactionPlan("HOLD", None, 0.0,
                            "no in-axis back-off for this leg/trigger -> HOLD",
                            verdict)
    axis = np.asarray(leg_axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        return ReactionPlan("HOLD", None, 0.0, "leg axis unknown -> HOLD", verdict)
    axis = axis / norm
    target = np.asarray(tcp_pos, dtype=float) - p.backoff_m * axis
    if not in_box(target, box):
        return ReactionPlan("HOLD", None, 0.0,
                            "back-off target leaves the workspace box -> HOLD",
                            verdict)
    return ReactionPlan(
        "BACKOFF", (float(target[0]), float(target[1]), float(target[2])),
        p.backoff_speed_frac,
        f"back off {p.backoff_m * 1000:.0f} mm along -leg-axis at "
        f"{p.backoff_speed_frac * 100:.0f} %, then HOLD",
        verdict)


def _verdict(det: Detection) -> str:
    lines = [det.one_screen(),
             "ACTION: goal cancelled (cancel-all). Will NOT retry automatically.",
             "Operator: inspect the forks/dock before any re-arm; relay a fresh "
             "CAPTAIN GO to resume."]
    return "\n".join(lines)
