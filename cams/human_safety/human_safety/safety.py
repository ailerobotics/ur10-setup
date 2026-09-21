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

from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from .geometry import distance_to_base
from .messages import (CameraHealth, DistanceStatus, SafetyState, Track, Zone)


class NearestRangeFilter:
    """Robust, safety-biased filter on a track's nearest-point RANGE (metres to
    the base origin).

    The tracker copies ``nearest_point`` from the latest detection **unfiltered**,
    so a single-frame depth speckle would flap the zone if it drove STOP directly.
    This filter keeps the last ``window`` raw ranges, drops the ``reject`` closest
    (a possible speckle), and returns the MIN of what remains — a small-window
    low-order statistic that leans toward the robot (the safe direction):

    * one spuriously-close frame is the single closest sample, so it is dropped
      and cannot pull the zone in (no STOP chatter);
    * a genuinely close hand survives once it persists ``reject + 1`` frames, so
      it is **not** smoothed away (fast toward the robot);
    * a single far outlier while the person is close is likewise dropped, so the
      zone does not release early (slow away). The evaluator's minimum-clear
      dwell provides the remaining hysteretic exit.

    Pure and deterministic; unit-tested in both directions.
    """

    def __init__(self, window: int = 5, reject: int = 1):
        self.window = max(1, int(window))
        self.reject = max(0, int(reject))
        self._buf: deque = deque(maxlen=self.window)

    def update(self, raw_range: float) -> float:
        self._buf.append(float(raw_range))
        vals = sorted(self._buf)
        k = min(self.reject, len(vals) - 1)   # drop k closest, take the next
        return vals[k]


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
    frame_age_budget: float = 0.5       # frame age beyond which a camera is "slow/lagging" -> SLOW
    # A detector still ANSWERING but lagging (frame_age in (frame_age_budget,
    # detector_stall_timeout]) -> SLOW; once it has produced no fresh frame for
    # detector_stall_timeout, or its status stopped entirely, it has STOPPED
    # answering -> STOP (human-safety-s1 §5 fault 3: a stalled/crashed detector
    # cannot see a person who is there; fail toward STOP).
    detector_stall_timeout: float = 1.5
    # End-to-end image->state latency beyond which the STOP-ring sizing (Tr) is no
    # longer valid -> STOP (human-safety-s1 §5 fault 5).
    reaction_time_budget: float = 0.7
    lost_close_margin: float = 0.30     # a track within stop_radius+this is "close"
    lost_hold_time: float = 3.0         # hold STOP this long after losing a close track
    presence_floor_slow: bool = True    # an unlocalised visible person forces >= SLOW
    # human-safety-s1 §3: in this small cell SSM says any person present -> at
    # least SLOW. Any confirmed track (even beyond slow_radius) forces >= SLOW.
    slow_on_any_person: bool = True
    # Zone sizing reacts to the CLOSEST credible body point, not the body centre:
    # the per-track nearest-point range is passed through NearestRangeFilter with
    # this window / reject before it is compared (smaller-of-the-two) against the
    # centre distance. Window 5 / reject 1 rejects a single-frame speckle while a
    # genuinely close reading survives after 2 frames (~0.13 s at 15 Hz). See the
    # README "Zones react to the nearest credible body point".
    nearest_filter_window: int = 5
    nearest_filter_reject: int = 1


class SafetyEvaluator:
    """Stateful evaluator. Call :meth:`evaluate` once per fusion cycle."""

    def __init__(self, params: Optional[SafetyParams] = None):
        self.p = params or SafetyParams()
        self._zone = Zone.CLEAR
        self._not_stop_since: Optional[float] = None   # dwell timer for leaving STOP
        self._recent_close = {}   # track id -> (last_seen_time, last_env_dist)
        self._lost_close_until: float = -np.inf
        # Per-track nearest-point range filters (id -> NearestRangeFilter).
        self._nearest_filters: Dict[int, NearestRangeFilter] = {}

    # ------------------------------------------------------------------ #
    def evaluate(self, tracks: List[Track], health: List[CameraHealth],
                 now: float, any_presence: bool = False,
                 source_stamp: Optional[float] = None,
                 latency: Optional[float] = None) -> SafetyState:
        p = self.p
        reasons: List[str] = []

        # --- nearest person distance over confirmed, positioned tracks ----- #
        # The zone reacts to the CLOSEST credible body point, never to a point
        # farther than the body centre: a person's outstretched hand can be inside
        # the STOP ring while their centre is outside it. For each confirmed track
        # we take min(centre distance, filtered nearest-point distance); the
        # nearest-point range is speckle-filtered first (NearestRangeFilter). The
        # centre distance is kept alongside for diagnostics.
        nearest_env = float("inf")
        nearest_base = float("inf")
        nearest_env_centre = float("inf")
        nearest_base_centre = float("inf")
        nearest_id: Optional[int] = None
        close_ids = {}
        current_ids = set()
        for t in tracks:
            if not t.confirmed:
                continue
            current_ids.add(t.id)
            d_base_centre = distance_to_base(t.position)
            d_base = d_base_centre
            if t.nearest_point is not None:
                flt = self._nearest_filters.get(t.id)
                if flt is None:
                    flt = NearestRangeFilter(
                        p.nearest_filter_window, p.nearest_filter_reject)
                    self._nearest_filters[t.id] = flt
                filt_near_base = flt.update(distance_to_base(t.nearest_point))
                # Never LARGER than the centre distance (use the smaller of the
                # two). A monocular/presence-only track whose nearest equals its
                # centre therefore behaves exactly as before.
                d_base = min(d_base_centre, filt_near_base)
            d_env = max(0.0, d_base - p.reach_radius)
            d_env_centre = max(0.0, d_base_centre - p.reach_radius)
            if d_env < nearest_env:
                nearest_env, nearest_base, nearest_id = d_env, d_base, t.id
                nearest_env_centre = d_env_centre
                nearest_base_centre = d_base_centre
            # lost-close bookkeeping uses the SAME (combined) distance as the zone.
            if d_env <= p.stop_radius + p.lost_close_margin:
                close_ids[t.id] = (now, d_env)
        # Drop nearest-range filters for tracks that no longer exist.
        self._nearest_filters = {
            tid: f for tid, f in self._nearest_filters.items()
            if tid in current_ids}

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

        # Latency over the reaction-time budget invalidates the STOP-ring math
        # (report §5 fault 5) -> STOP.
        if latency is not None and latency > p.reaction_time_budget:
            floor = _max_zone(floor, Zone.STOP)
            reasons.append(
                f"img->state latency {latency:.2f}s > reaction budget "
                f"{p.reaction_time_budget:.2f}s -> STOP")

        candidate = _max_zone(dist_zone, floor)
        if nearest_id is not None:
            msg = (f"nearest person id={nearest_id} at {nearest_base:.2f} m "
                   f"from base ({nearest_env:.2f} m from reach envelope)")
            if nearest_base_centre - nearest_base > 0.05:
                msg += (f"; nearest credible body point, centre is "
                        f"{nearest_base_centre:.2f} m")
            reasons.append(msg)

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
            nearest_distance_centre=nearest_env_centre,
            nearest_distance_base_centre=nearest_base_centre,
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
            # Detector STOPPED answering (status gone, or no fresh frame for
            # detector_stall_timeout) -> STOP. A stalled/crashed detector cannot
            # see a person who is there (report §5 fault 3; fail toward STOP).
            if not hc.present_in_graph or not hc.detector_alive:
                floor = _max_zone(floor, Zone.STOP)
                reasons.append(
                    f"camera '{hc.camera}' detector not answering (no status) "
                    f"-> STOP")
                continue
            if hc.frame_age > p.detector_stall_timeout:
                floor = _max_zone(floor, Zone.STOP)
                reasons.append(
                    f"camera '{hc.camera}' detector stalled (no frame for "
                    f"{hc.frame_age:.2f}s > {p.detector_stall_timeout:.2f}s) "
                    f"-> STOP")
                continue
            # Still answering but lagging beyond the freshness budget -> SLOW.
            if hc.frame_age > p.frame_age_budget:
                floor = _max_zone(floor, Zone.SLOW)
                reasons.append(
                    f"camera '{hc.camera}' slow ({hc.frame_age:.2f}s > "
                    f"{p.frame_age_budget:.2f}s, still answering) -> SLOW")

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
