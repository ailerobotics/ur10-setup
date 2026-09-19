# SPDX-License-Identifier: Apache-2.0
"""Zone / hysteresis / dwell / health state machine. Fails toward STOP.

This is the judgement the package publishes. It is a deterministic function of
(tracks, per-camera health, presence, clock) plus its own retained state
(previous zone and dwell timers), so a recorded sequence replays bit-for-bit and
every rule is unit-tested offline.

**This is a supplementary camera layer, not a certified safety function.** The
default radii and time budgets are provisional (see the README and
task human-safety-s1); they are conservative placeholders, not derived
separation distances.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .geometry import distance_to_base, distance_to_envelope
from .messages import (CameraHealth, DistanceStatus, SafetyState, Track, Zone)


@dataclass
class SafetyParams:
    """All radii in metres, all times in seconds. PROVISIONAL defaults from
    human-safety-s1 §3: STOP when a person is within 2.0 m of the robot base
    (= reach_radius 1.30 + stop_radius 0.70), leave STOP only after clearing that
    ring by +0.30 m (hysteresis) and >=2.0 s of continuous healthy CLEAR."""

    reach_radius: float = 1.30          # UR10 reach; envelope = cylinder r about base z
    stop_radius: float = 0.70           # envelope distance below which -> STOP (base 2.0 m)
    slow_radius: float = 2.00           # envelope distance below which -> SLOW (outer ring)
    hysteresis: float = 0.30            # extra clearance required to de-escalate
    min_clear_dwell: float = 2.0        # must be clear this long before leaving STOP
    frame_age_budget: float = 0.5       # image-stamp-vs-clock age before "stale"
    lost_close_margin: float = 0.30     # a track within stop_radius+this is "close"
    lost_hold_time: float = 3.0         # hold STOP this long after losing a close track
    presence_floor_slow: bool = True    # an unlocalised visible person forces >= SLOW
    # human-safety-s1 §3: in this small cell SSM says any person present -> at
    # least SLOW. Any confirmed track (even beyond slow_radius) forces >= SLOW.
    slow_on_any_person: bool = True


class SafetyEvaluator:
    """Stateful evaluator. Call :meth:`evaluate` once per fusion cycle."""

    def __init__(self, params: Optional[SafetyParams] = None):
        self.p = params or SafetyParams()
        self._zone = Zone.CLEAR
        self._not_stop_since: Optional[float] = None   # dwell timer for leaving STOP
        self._recent_close = {}   # track id -> (last_seen_time, last_env_dist)
        self._lost_close_until: float = -np.inf

    # ------------------------------------------------------------------ #
    def evaluate(self, tracks: List[Track], health: List[CameraHealth],
                 now: float, any_presence: bool = False,
                 source_stamp: Optional[float] = None) -> SafetyState:
        p = self.p
        reasons: List[str] = []

        # --- nearest person distance over confirmed, positioned tracks ----- #
        nearest_env = float("inf")
        nearest_base = float("inf")
        nearest_id: Optional[int] = None
        close_ids = {}
        current_ids = set()
        for t in tracks:
            if not t.confirmed:
                continue
            current_ids.add(t.id)
            d_env = distance_to_envelope(t.position, p.reach_radius)
            d_base = distance_to_base(t.position)
            if d_env < nearest_env:
                nearest_env, nearest_base, nearest_id = d_env, d_base, t.id
            if d_env <= p.stop_radius + p.lost_close_margin:
                close_ids[t.id] = (now, d_env)

        # --- distance-based zone with hysteresis (uses previous zone) ------ #
        dist_zone = self._distance_zone(nearest_env)
        # Any confirmed person present forces at least SLOW (report §3: small
        # cell, SSM says any person -> SLOW), even beyond the SLOW ring.
        if self.p.slow_on_any_person and current_ids:
            dist_zone = _max_zone(dist_zone, Zone.SLOW)

        # --- health floor (fail toward STOP) ------------------------------- #
        floor, health_reasons = self._health_floor(
            health, now, any_presence, close_ids, current_ids)
        reasons.extend(health_reasons)

        candidate = _max_zone(dist_zone, floor)
        if nearest_id is not None:
            reasons.append(
                f"nearest person id={nearest_id} at {nearest_base:.2f} m "
                f"from base ({nearest_env:.2f} m from reach envelope)")

        # --- distance honesty: UNKNOWN is not 'far' (report §7 #5) --------- #
        if nearest_id is not None:
            dist_status = DistanceStatus.KNOWN
        elif any_presence:
            dist_status = DistanceStatus.UNKNOWN_PRESENCE_ONLY
        else:
            dist_status = DistanceStatus.NONE

        # --- dwell: cannot leave STOP until clear for min_clear_dwell ------- #
        zone = self._apply_dwell(candidate, now, reasons)

        self._zone = zone
        self._remember_close(close_ids, current_ids, now)

        return SafetyState(
            stamp=now,
            zone=zone,
            nearest_distance=nearest_env,
            nearest_distance_base=nearest_base,
            distance_status=dist_status,
            nearest_track_id=nearest_id,
            any_person_visible=bool(any_presence or nearest_id is not None),
            reasons=reasons,
            health=list(health),
            tracks=list(tracks),
            evidence_stamp=source_stamp,
        )

    # ------------------------------------------------------------------ #
    def _distance_zone(self, nearest_env: float) -> Zone:
        p = self.p
        prev = self._zone
        h = p.hysteresis
        # De-escalation requires crossing the threshold PLUS hysteresis; this is
        # what stops the zone chattering right at a boundary.
        if prev == Zone.STOP:
            if nearest_env <= p.stop_radius + h:
                return Zone.STOP
            if nearest_env <= p.slow_radius + h:
                return Zone.SLOW
            return Zone.CLEAR
        if prev == Zone.SLOW:
            if nearest_env <= p.stop_radius:
                return Zone.STOP
            if nearest_env <= p.slow_radius + h:
                return Zone.SLOW
            return Zone.CLEAR
        # prev CLEAR
        if nearest_env <= p.stop_radius:
            return Zone.STOP
        if nearest_env <= p.slow_radius:
            return Zone.SLOW
        return Zone.CLEAR

    def _health_floor(self, health, now, any_presence, close_ids, current_ids):
        p = self.p
        floor = Zone.CLEAR
        reasons = []

        for hc in health:
            if not hc.present_in_graph:
                floor = _max_zone(floor, Zone.SLOW)
                reasons.append(f"camera '{hc.camera}' missing from graph -> SLOW")
                continue
            if not hc.detector_alive:
                floor = _max_zone(floor, Zone.SLOW)
                reasons.append(f"camera '{hc.camera}' detector not answering -> SLOW")
            if not hc.fresh:
                floor = _max_zone(floor, Zone.SLOW)
                reasons.append(
                    f"camera '{hc.camera}' frame stale ({hc.frame_age:.2f}s "
                    f"> {p.frame_age_budget:.2f}s) -> SLOW")

        if any_presence and p.presence_floor_slow:
            floor = _max_zone(floor, Zone.SLOW)
            reasons.append("person visible without a trustworthy position -> SLOW")

        # Track lost while close: a confirmed track that was inside
        # stop_radius+margin has DISAPPEARED from the tracker entirely (deleted,
        # not merely moved away — a person seen walking out keeps its id and is
        # handled by the distance zone). We can't tell a deletion from an
        # occlusion/detector-miss, so we fail toward STOP and hold it.
        for tid, (last_seen, last_dist) in list(self._recent_close.items()):
            if tid in current_ids:
                continue   # still tracked (possibly moved out) -> not lost
            if now - last_seen <= p.lost_hold_time:
                self._lost_close_until = max(
                    self._lost_close_until, last_seen + p.lost_hold_time)
                reasons.append(
                    f"track id={tid} lost while close ({last_dist:.2f} m) -> "
                    f"hold STOP")
        if now <= self._lost_close_until:
            floor = _max_zone(floor, Zone.STOP)

        return floor, reasons

    def _apply_dwell(self, candidate: Zone, now: float, reasons: List[str]) -> Zone:
        p = self.p
        if self._zone == Zone.STOP:
            if candidate == Zone.STOP:
                self._not_stop_since = None
                return Zone.STOP
            # candidate is below STOP; require sustained clearance.
            if self._not_stop_since is None:
                self._not_stop_since = now
            waited = now - self._not_stop_since
            if waited < p.min_clear_dwell:
                reasons.append(
                    f"holding STOP: clear for {waited:.2f}s < dwell "
                    f"{p.min_clear_dwell:.2f}s")
                return Zone.STOP
            reasons.append(f"leaving STOP: clear for {waited:.2f}s")
            return candidate
        # Not currently in STOP.
        if candidate == Zone.STOP:
            self._not_stop_since = None
        return candidate

    def _remember_close(self, close_ids, current_ids, now):
        p = self.p
        for tid, (t, d) in close_ids.items():
            self._recent_close[tid] = (now, d)
        # A track still present but no longer close walked out safely -> forget
        # it so it never triggers the lost-while-close hold.
        for tid in (current_ids - set(close_ids)):
            self._recent_close.pop(tid, None)
        # Forget memory of vanished tracks once the hold window has elapsed.
        self._recent_close = {
            tid: v for tid, v in self._recent_close.items()
            if now - v[0] <= p.lost_hold_time}


def _max_zone(a: Zone, b: Zone) -> Zone:
    return a if a.severity >= b.severity else b
