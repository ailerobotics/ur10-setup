# SPDX-License-Identifier: Apache-2.0
"""Offline, deterministic comparison of the every-frame BASELINE against the
EFFICIENT frame-selection path, on one shared multi-camera sequence.

This is the safety evidence for the efficient path. It is pure (no ROS, no torch,
no GPU): it drives the exact same decision logic the nodes use — ``FrameScheduler``
for the selection, ``Fusion``/``MultiTargetTracker`` for the fusion, and
``SafetyEvaluator`` for the zone/health state machine — over a scripted sequence
whose per-camera, per-frame ground-truth detections and change scores are known,
so both paths see identical input and the comparison is apples-to-apples.

Sequence record (one object per scheduling tick), e.g. from
``tests/data/multicam_approach.jsonl``::

    {"t": 0.0, "persons": 0,
     "cams": {"shelf": {"have_new_frame": true, "change_score": 0.0, "dets": []},
              "table": {"have_new_frame": true, "change_score": 0.0, "dets": []}}}

Each detection dict carries the base-frame fields a full detector would yield for
that camera that frame: ``position`` (3), optional ``nearest_point`` (3),
``method`` (depth|monocular|presence), ``score``, ``sigma``. ``persons`` is the
ground-truth count of people in the monitored volume (for recall).

The BASELINE runs a full detection on every camera with a new frame, every tick.
The EFFICIENT path runs ``FrameScheduler`` and only detects the chosen cameras;
a skipped camera's last full detection persists (exactly as the real fusion node
re-consumes a camera's last published message until it ages out of the gather
window), and the tracker coasts. Both paths then run the identical fusion + safety
machine, mirroring ``nodes/fusion_node.py`` (including ``gather_evidence``'s
budget drop and the oldest-positioned-stamp latency).

The headline safety assertion: there must be **zero** ticks where the efficient
path is LESS safe than the baseline (efficient zone strictly below baseline zone).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .config import HumanSafetyConfig
from .fusion import Fusion, gather_evidence
from .messages import Detection3D, Method, Zone
from .safety import SafetyEvaluator
from .scheduler import CameraTick, FrameScheduler
from .tracker import MultiTargetTracker


# --------------------------------------------------------------------------- #
def _det_from_dict(camera: str, stamp: float, d: dict) -> Detection3D:
    pos = d.get("position")
    npt = d.get("nearest_point", pos)
    return Detection3D(
        camera=camera,
        stamp=stamp,
        score=float(d.get("score", 0.9)),
        method=Method(d.get("method", "depth")),
        present=bool(d.get("present", True)),
        position=None if pos is None else np.asarray(pos, dtype=float),
        nearest_point=None if npt is None else np.asarray(npt, dtype=float),
        sigma=float(d.get("sigma", 0.1)),
        n_depth_valid=int(d.get("n_depth_valid", 400)),
    )


def _health(roles, latest, now, budget):
    """Build CameraHealth exactly as fusion_node._assess_health would from the
    per-camera last full detection: a detector process that is alive but whose
    last fresh frame is old reads as stale (-> SLOW, then STOP past the stall
    timeout). A camera that never detected has frame_age inf -> stale."""
    from .messages import CameraHealth
    out = []
    for role in roles:
        rec = latest.get(role)
        if rec is None:
            out.append(CameraHealth(
                camera=role, present_in_graph=True, detector_alive=True,
                frame_age=float("inf"), fresh=False, note="no frame yet"))
            continue
        recv_wall, _dets, _stamp, positioned = rec
        age = now - recv_wall
        out.append(CameraHealth(
            camera=role, present_in_graph=True, detector_alive=True,
            frame_age=age, positioned=positioned, fresh=age <= budget,
            note="" if age <= budget else "stale"))
    return out


@dataclass
class ModeResult:
    """Per-tick outputs of one path (baseline or efficient)."""

    zones: List[Zone] = field(default_factory=list)
    n_confirmed: List[int] = field(default_factory=list)
    any_visible: List[bool] = field(default_factory=list)
    detector_invocations: int = 0           # total full detections run
    frames_offered: int = 0                 # total (camera, new-frame) pairs seen
    worst_staleness: Dict[str, float] = field(default_factory=dict)  # per camera


@dataclass
class BenchResult:
    times: List[float]
    persons: List[int]
    baseline: ModeResult
    efficient: ModeResult

    # --- safety agreement ---------------------------------------------------- #
    @property
    def less_safe_ticks(self) -> List[int]:
        """Tick indices where the efficient zone is STRICTLY below the baseline
        zone — the forbidden direction (efficient says safer than the baseline)."""
        return [i for i, (b, e) in enumerate(zip(self.baseline.zones,
                                                 self.efficient.zones))
                if e.severity < b.severity]

    @property
    def disagreement_ticks(self) -> List[int]:
        return [i for i, (b, e) in enumerate(zip(self.baseline.zones,
                                                 self.efficient.zones))
                if e != b]

    # --- recall / continuity ------------------------------------------------- #
    def recall(self, mode: ModeResult) -> Optional[float]:
        """Fraction of ground-truth person-present ticks where the mode reports a
        person (confirmed track or any visible presence)."""
        person_ticks = [i for i, n in enumerate(self.persons) if n > 0]
        if not person_ticks:
            return None
        hit = sum(1 for i in person_ticks
                  if mode.n_confirmed[i] > 0 or mode.any_visible[i])
        return hit / len(person_ticks)

    @property
    def track_continuity(self) -> Optional[float]:
        """Of the ticks where the BASELINE holds a confirmed track, the fraction
        where the EFFICIENT path also holds one."""
        base_ticks = [i for i, n in enumerate(self.baseline.n_confirmed) if n > 0]
        if not base_ticks:
            return None
        both = sum(1 for i in base_ticks if self.efficient.n_confirmed[i] > 0)
        return both / len(base_ticks)


# --------------------------------------------------------------------------- #
def _run(records, cfg: HumanSafetyConfig, roles, efficient: bool) -> ModeResult:
    budget = cfg.safety.frame_age_budget
    sched = FrameScheduler(cfg.frame_processing.scheduler)
    fusion = Fusion(MultiTargetTracker(
        sigma_meas=0.15, n_confirm=3, max_confirmed_coast_time=2.5))
    ev = SafetyEvaluator(cfg.safety)
    res = ModeResult(worst_staleness={r: 0.0 for r in roles})

    latest: Dict[str, tuple] = {}          # role -> (recv_wall, dets, stamp, positioned)
    last_full: Dict[str, Optional[float]] = {r: None for r in roles}
    near_ring = False                       # derived from the previous tick's state

    for rec in records:
        now = float(rec["t"])
        cams = rec.get("cams", {})

        # Build the per-camera scheduling ticks for this frame.
        ticks = []
        for role in roles:
            c = cams.get(role, {})
            have = bool(c.get("have_new_frame", role in cams))
            ticks.append(CameraTick(
                role=role, have_new_frame=have,
                change_score=c.get("change_score"),
                last_full_detect_time=last_full[role]))

        if efficient:
            decisions = {d.role: d.detect
                         for d in sched.decide(now, ticks, near_ring=near_ring)}
        else:
            # Baseline: detect every camera that has a new frame, every tick.
            decisions = {t.role: t.have_new_frame for t in ticks}

        for t in ticks:
            if t.have_new_frame:
                res.frames_offered += 1
            if decisions.get(t.role) and t.have_new_frame:
                c = cams.get(t.role, {})
                dets = [_det_from_dict(t.role, now, d) for d in c.get("dets", [])]
                positioned = any(d.positioned for d in dets)
                latest[t.role] = (now, dets, now, positioned)
                last_full[t.role] = now
                res.detector_invocations += 1

        # Per-camera worst-case staleness = longest gap since a full detection
        # (only meaningful once the camera has detected at least once).
        for role in roles:
            if last_full[role] is not None:
                res.worst_staleness[role] = max(
                    res.worst_staleness[role], now - last_full[role])

        # Fusion input = each camera's last full detection within the gather
        # window, exactly as the live node re-consumes the last published message.
        dets_by_role = {role: (rec_[0], rec_[1], rec_[2])
                        for role, rec_ in latest.items()}
        all_dets, evidence_stamp, latency = gather_evidence(
            dets_by_role, now, budget)
        tracks, presence_cams = fusion.step(all_dets, now)
        health = _health(roles, latest, now, budget)
        any_presence = bool(presence_cams)

        state = ev.evaluate(tracks, health, now, any_presence=any_presence,
                            source_stamp=evidence_stamp, latency=latency)

        res.zones.append(state.zone)
        res.n_confirmed.append(sum(1 for t in tracks if t.confirmed))
        res.any_visible.append(bool(state.any_person_visible))

        # Advisory near-ring signal for the NEXT tick (mirrors the shared
        # detector reading the fusion node's published state): any track inside
        # or approaching the SLOW ring.
        near_ring = False
        if efficient:
            slow_env = cfg.safety.slow_radius
            margin = cfg.frame_processing.scheduler.near_margin
            if state.zone.severity >= Zone.SLOW.severity:
                near_ring = True
            elif (state.nearest_distance != float("inf")
                  and state.nearest_distance <= slow_env + margin):
                near_ring = True

    return res


def run_baseline_vs_efficient(records: List[dict],
                              cfg: HumanSafetyConfig) -> BenchResult:
    """Run both paths over the same sequence and return the comparison.

    The efficient path uses ``cfg.frame_processing.scheduler`` (which must have
    ``enabled=True`` to exercise selection); the baseline ignores it and detects
    every new frame. Both share ``cfg.safety``.
    """
    roles = sorted({r for rec in records for r in rec.get("cams", {})})
    times = [float(rec["t"]) for rec in records]
    persons = [int(rec.get("persons", 0)) for rec in records]
    baseline = _run(records, cfg, roles, efficient=False)
    efficient = _run(records, cfg, roles, efficient=True)
    return BenchResult(times=times, persons=persons,
                       baseline=baseline, efficient=efficient)
