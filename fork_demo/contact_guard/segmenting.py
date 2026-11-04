"""Incremental contact legs: cut a contact leg into short stop-and-probe segments.

Each contact leg runs as a sequence of <= ``segment_m`` (15 mm TCP) segments, so
the arm stops within one segment. Each segment is executed as DWELL -> fresh
at-pose wrench BASELINE -> CREEP the segment with the between-sample check
(probe / tracking / stall) -> CHECK before the next. Segment interiors are
DENSE (the 132-point lesson: >= ``segment_min_points``), which kept the real
coarse-waypoint tracking excursions from 12.6 deg down to 3.6 deg.

Pure: takes the leg's joint path and the matching base_link TCP path (the caller
computes TCP via FK/TF) and returns the segment plan. Transit legs keep their
existing path and are returned as a single non-probe segment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence

import numpy as np

from .params import ContactGuardParams


@dataclass
class Segment:
    index: int
    joints: np.ndarray          # (K, 6) dense joint waypoints
    tcp_start: np.ndarray       # (3,)
    tcp_end: np.ndarray         # (3,)
    length_m: float
    probe: bool                 # True -> stop-and-probe (contact)

    @property
    def n_points(self) -> int:
        return int(self.joints.shape[0])


@dataclass
class SegmentPlan:
    leg: str
    contact: bool
    segments: List[Segment] = field(default_factory=list)
    probe_dwell_s: float = 0.0

    @property
    def n_segments(self) -> int:
        return len(self.segments)

    def added_cycle_time_s(self) -> float:
        """Extra wall-clock vs an un-segmented leg: one probe dwell per segment."""
        return self.probe_dwell_s * sum(1 for s in self.segments if s.probe)


def _tcp_cumlen(tcp: np.ndarray) -> np.ndarray:
    if tcp.shape[0] < 2:
        return np.zeros(tcp.shape[0])
    steps = np.linalg.norm(np.diff(tcp, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(steps)])


def _densify(joints: np.ndarray, min_points: int) -> np.ndarray:
    """Linearly resample a joint polyline to >= ``min_points`` points."""
    m = joints.shape[0]
    if m >= min_points:
        return joints
    # cumulative joint arclength parameterisation
    seg = np.linalg.norm(np.diff(joints, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1] if cum[-1] > 0 else 1.0
    u = cum / total
    uu = np.linspace(0.0, 1.0, min_points)
    out = np.stack([np.interp(uu, u, joints[:, c]) for c in range(joints.shape[1])],
                   axis=1)
    return out


def segment_leg(leg: str, contact: bool, joint_path: Sequence[Sequence[float]],
                tcp_path: Sequence[Sequence[float]], p: ContactGuardParams
                ) -> SegmentPlan:
    """Cut a leg into stop-and-probe segments (contact) or one segment (transit)."""
    jp = np.asarray(joint_path, dtype=float)
    tcp = np.asarray(tcp_path, dtype=float)
    if jp.shape[0] != tcp.shape[0]:
        raise ValueError("joint_path and tcp_path must have the same length")
    plan = SegmentPlan(leg=leg, contact=contact, probe_dwell_s=p.probe_dwell_s)
    if not contact or jp.shape[0] < 2:
        plan.segments.append(Segment(
            0, _densify(jp, p.segment_min_points) if jp.shape[0] >= 2 else jp,
            tcp[0] if tcp.shape[0] else np.zeros(3),
            tcp[-1] if tcp.shape[0] else np.zeros(3),
            float(_tcp_cumlen(tcp)[-1]) if tcp.shape[0] >= 2 else 0.0,
            probe=False))
        return plan
    cum = _tcp_cumlen(tcp)
    total = float(cum[-1])
    n_seg = max(1, int(np.ceil(total / p.segment_m)))
    edges = np.linspace(0.0, total, n_seg + 1)

    def at_len(L: float):
        """Interpolate (joints, tcp) at cumulative TCP length ``L``."""
        j = np.array([np.interp(L, cum, jp[:, c]) for c in range(jp.shape[1])])
        t = np.array([np.interp(L, cum, tcp[:, c]) for c in range(3)])
        return j, t

    for k in range(n_seg):
        lo, hi = edges[k], edges[k + 1]
        # dense joint samples between the exact segment edges (interpolated), so
        # each segment is exactly <= segment_m regardless of input coarseness
        Ls = np.linspace(lo, hi, p.segment_min_points)
        seg_j = np.stack([at_len(L)[0] for L in Ls])
        t_lo, t_hi = at_len(lo)[1], at_len(hi)[1]
        plan.segments.append(Segment(
            index=k, joints=seg_j, tcp_start=t_lo, tcp_end=t_hi,
            length_m=float(hi - lo), probe=True))
    return plan
