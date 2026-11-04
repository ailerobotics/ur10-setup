"""Pure signal math for the contact guard.

No ROS, no file IO. Everything here is numpy + stdlib so it is unit-testable
offline and replayable deterministically.

Signal vocabulary (all per-sample, all expressed so a contact fault shows as a
band excursion):

* ``force_mag``   -- |F| at the fork, N
* ``f_ins``       -- force along the fork insertion axis (fork +z), N (signed)
* ``f_lat``       -- lateral force sqrt(fx^2+fy^2) at the fork, N
* ``torque_mag``  -- |M| about the fork tip, Nm
* ``t_pry``       -- prying torque: max |Mx|,|My| about the fork tip, Nm
* ``eff_max``     -- max |joint effort| (CB3 current-estimate), Nm
* ``tcp_vel``     -- TCP speed, m/s
* ``track_err``   -- commanded-vs-actual joint error, rad (runtime-only)

``force_mag``/``f_ins``/``f_lat``/``torque_mag``/``t_pry``/``eff_max``/``tcp_vel``
are observable in a recorded teach bag; ``track_err`` needs a commanded
trajectory, so it is banded only when a plan is replayed (runtime / URSim), and
is reported N/A for a freedrive teach bag.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .params import FORK_OFFSET

# Canonical joint order (matches build_replay.py / execute_plan.py JOINTS and the
# plan_*.yaml joint_path columns). The bag's /joint_states.name is alphabetical,
# so always reorder by name before use.
JOINT_ORDER: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Signals present in a teach bag (banded offline) and the runtime-only one.
BAG_SIGNALS: Tuple[str, ...] = (
    "force_mag", "f_ins", "f_lat", "torque_mag", "t_pry", "eff_max", "tcp_vel",
)
RUNTIME_SIGNALS: Tuple[str, ...] = ("track_err",)
ALL_SIGNALS: Tuple[str, ...] = BAG_SIGNALS + RUNTIME_SIGNALS


def reorder_joints(names: Sequence[str], values: Sequence[float],
                   order: Sequence[str] = JOINT_ORDER) -> np.ndarray:
    """Return ``values`` reordered to ``order`` by matching ``names``.

    Raises KeyError if a joint in ``order`` is missing from ``names``.
    """
    idx = {n: i for i, n in enumerate(names)}
    return np.array([values[idx[j]] for j in order], dtype=float)


def wrench_to_fork(force: Sequence[float], torque: Sequence[float],
                   offset: Sequence[float] = FORK_OFFSET
                   ) -> Tuple[np.ndarray, np.ndarray]:
    """Shift a wrench from the tool0 reference point to the fork tip.

    The wrench is published in ``tool0_controller`` whose axes equal the fork
    frame axes (the fork TCP offset is a pure translation). Force is unchanged;
    the torque about the fork tip is ``M_fork = M_tool0 - offset x F`` where
    ``offset`` is the fork tip position in tool0.
    """
    f = np.asarray(force, dtype=float)
    m = np.asarray(torque, dtype=float)
    r = np.asarray(offset, dtype=float)
    m_fork = m - np.cross(r, f)
    return f, m_fork


def sample_features(force_fork: Sequence[float], torque_fork: Sequence[float],
                    effort: Sequence[float], tcp_vel: float,
                    track_err: Optional[float] = None) -> Dict[str, float]:
    """Reduce a raw sample (already in fork frame) to the banded scalar signals."""
    f = np.asarray(force_fork, dtype=float)
    m = np.asarray(torque_fork, dtype=float)
    e = np.asarray(effort, dtype=float)
    feats = {
        "force_mag": float(np.linalg.norm(f)),
        "f_ins": float(f[2]),                       # fork +z is the insertion axis
        "f_lat": float(np.hypot(f[0], f[1])),
        "torque_mag": float(np.linalg.norm(m)),
        "t_pry": float(max(abs(m[0]), abs(m[1]))),  # bending about a horizontal axis
        "eff_max": float(np.max(np.abs(e))) if e.size else 0.0,
        "tcp_vel": float(tcp_vel),
        # raw fork-frame components (NOT banded) for baseline-relative probe deltas
        "fx": float(f[0]), "fy": float(f[1]), "fz": float(f[2]),
        "mx": float(m[0]), "my": float(m[1]), "mz": float(m[2]),
    }
    if track_err is not None:
        feats["track_err"] = float(track_err)
    return feats


def _ang_delta(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Shortest-angle elementwise difference b-a wrapped to (-pi, pi]."""
    d = b - a
    return (d + np.pi) % (2 * np.pi) - np.pi


def path_arclengths(path: np.ndarray) -> np.ndarray:
    """Cumulative joint-space (L2, shortest-angle) arc length along ``path``.

    ``path`` is (M, 6). Returns (M,) cumulative lengths starting at 0.
    """
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[0] < 2:
        return np.zeros(max(1, path.shape[0]))
    steps = np.linalg.norm(_ang_delta(path[:-1], path[1:]), axis=1)
    return np.concatenate([[0.0], np.cumsum(steps)])


def progress_of(q: Sequence[float], path: np.ndarray,
                cum: Optional[np.ndarray] = None) -> float:
    """Normalised progress (0..1) of joint config ``q`` along ``path``.

    Projects ``q`` onto the nearest segment of the reference joint path and
    returns the fraction of total arc length. Robust to a path that revisits
    poses: picks the projection with the smallest perpendicular distance.
    """
    path = np.asarray(path, dtype=float)
    q = np.asarray(q, dtype=float)
    if path.shape[0] < 2:
        return 0.0
    if cum is None:
        cum = path_arclengths(path)
    total = cum[-1]
    if total <= 0:
        return 0.0
    best_d = np.inf
    best_s = 0.0
    for i in range(path.shape[0] - 1):
        a = path[i]
        seg = _ang_delta(a, path[i + 1])
        L2 = float(seg @ seg)
        if L2 <= 1e-12:
            continue
        t = float(_ang_delta(a, q) @ seg) / L2
        t = min(1.0, max(0.0, t))
        foot = a + t * seg
        d = float(np.linalg.norm(_ang_delta(foot, q)))
        if d < best_d:
            best_d = d
            best_s = (cum[i] + t * np.linalg.norm(seg)) / total
    return float(min(1.0, max(0.0, best_s)))


@dataclass
class LegTimeSeries:
    """Raw per-leg time series resampled onto a common clock.

    Arrays are time-aligned (same length T). ``progress`` is monotonic-ish
    normalised arc length; ``feats`` maps signal name -> (T,) array.
    """
    leg: str
    t: np.ndarray                                   # seconds, relative to leg start
    progress: np.ndarray                            # 0..1
    feats: Dict[str, np.ndarray] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.t = np.asarray(self.t, dtype=float)
        self.progress = np.asarray(self.progress, dtype=float)

    @property
    def n(self) -> int:
        return int(self.t.shape[0])

    @property
    def duration_s(self) -> float:
        return float(self.t[-1] - self.t[0]) if self.n else 0.0
