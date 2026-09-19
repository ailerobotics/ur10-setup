#!/usr/bin/env python3
"""Measured joint-window guard for the cuRobo MPC prototype.

The cell never plans or executes outside the captain-measured joint windows in
``cumotion/measured_joint_limits.yaml`` (AGENTS.md: "never loosen the measured
joint windows"). cuRobo already sees the narrowed limits because
``cumotion/ur10_measured.urdf`` carries them in its ``<limit lower/upper>`` tags
(verified: they equal ``reference_pose_rad +- radians(window_deg)`` per joint).

This module is the *independent* check: it recomputes each effective window from
the YAML and asserts every commanded/planned joint position stays inside it, so a
future robot-config change that widened a limit would be caught. It is PURE
(stdlib + pyyaml) so it unit-tests offline and runs in the ROS bridge without a
GPU.

Effective window per joint (from the YAML header):
    lo = reference_pose_rad - radians(window_deg)
    hi = reference_pose_rad + radians(window_deg)
which is exactly ``[measured_min + margin, measured_max - margin]`` and, for the
three captain-declared joints, the declared window.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Sequence, Tuple

import yaml

_TWO_PI = 2.0 * math.pi


def wrap_near(q: float, ref: float) -> float:
    """Shift q by multiples of 2*pi so it lands within +-pi of ref. Revolute
    joints are periodic, so a commanded angle and its 2*pi-equivalent are the
    SAME physical configuration. The cell keeps some joints unwrapped (e.g.
    wrist_1 in [2.84, 6.08]) while cuRobo/ROS emit the [-pi, pi] equivalent; this
    reconciles the two so the guard neither rejects nor mis-clamps a legal angle."""
    while q - ref > math.pi:
        q -= _TWO_PI
    while q - ref < -math.pi:
        q += _TWO_PI
    return q

# canonical joint order used everywhere in the cell
JOINT_ORDER: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


@dataclass(frozen=True)
class JointWindows:
    """Effective per-joint [lo, hi] windows in radians, keyed by joint name."""

    windows: Dict[str, Tuple[float, float]]

    @property
    def joint_names(self) -> List[str]:
        return [j for j in JOINT_ORDER if j in self.windows]

    def lower(self, joints: Sequence[str] = JOINT_ORDER) -> List[float]:
        return [self.windows[j][0] for j in joints]

    def upper(self, joints: Sequence[str] = JOINT_ORDER) -> List[float]:
        return [self.windows[j][1] for j in joints]

    def clamp(self, q: Mapping[str, float]) -> Dict[str, float]:
        """Clamp a joint dict into the windows (used to keep a warm-start seed
        legal; the guard below is what actually *rejects* an illegal command).
        Wrap-aware: the value is first mapped to its 2*pi-equivalent nearest the
        window centre, then clamped."""
        out = {}
        for j, v in q.items():
            if j in self.windows:
                lo, hi = self.windows[j]
                vw = wrap_near(v, (lo + hi) / 2.0)
                out[j] = min(max(vw, lo), hi)
            else:
                out[j] = v
        return out

    def violations(
        self, q: Mapping[str, float], tol: float = 1e-6
    ) -> List[Tuple[str, float, float, float]]:
        """Return [(joint, value, lo, hi), ...] for every joint whose 2*pi-
        equivalent nearest the window centre is outside [lo, hi] by more than
        ``tol`` radians. Empty list == inside all windows."""
        bad = []
        for j, v in q.items():
            if j not in self.windows:
                continue
            lo, hi = self.windows[j]
            vw = wrap_near(v, (lo + hi) / 2.0)
            if vw < lo - tol or vw > hi + tol:
                bad.append((j, vw, lo, hi))
        return bad

    def within(self, q: Mapping[str, float], tol: float = 1e-6) -> bool:
        return not self.violations(q, tol)


def load_measured_windows(yaml_path: str) -> JointWindows:
    """Build :class:`JointWindows` from ``measured_joint_limits.yaml``."""
    # explicit utf-8: the cell YAML has non-ASCII header comments, and a process
    # launched under an ASCII/POSIX locale would otherwise fail to read it.
    with open(yaml_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    ref = data["reference_pose_rad"]
    win_deg = data["window_deg"]
    windows: Dict[str, Tuple[float, float]] = {}
    for j in JOINT_ORDER:
        if j not in ref or j not in win_deg:
            continue
        half = math.radians(float(win_deg[j]))
        centre = float(ref[j])
        windows[j] = (centre - half, centre + half)
    if not windows:
        raise ValueError(f"no joint windows parsed from {yaml_path}")
    return JointWindows(windows=windows)
