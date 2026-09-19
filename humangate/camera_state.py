#!/usr/bin/env python3
"""
The camera-side safety state, as the robot-side gate consumes it.

This is the gate's OWN stable representation of the message described in
human-safety-s1/report.md §7 ("Interface contract — camera side <-> robot-side
gate"). The perception package (branch `fm/human-detect-core-v1`, task
human-detect-core-v1) publishes the wire message; `state_adapter.py` maps that
wire message onto this dataclass so a field rename on the camera side is a
one-line change here, never a change scattered through the decision core.

This module is deliberately PURE: it imports only the standard library, never
rclpy or any ROS package, so every offline test (and any host without a ROS
stack) can build and reason about a camera state exactly as the gate does.

`nearest_distance_m is None` means "distance unknown" (§7.5): only presence-only
(uncalibrated / FK-less) cameras saw the person, so there is no trustworthy
distance. The gate must NEVER read an unknown distance as "far" — see gate_core.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import FrozenSet, Optional, Tuple

# Camera-side zone enum (report §7.1). These are the three values the perception
# layer publishes; the gate's own richer decision (RUN/SLOW/STOP/BACKSTOP) lives
# in gate_core.Action and is a separate vocabulary.
ZONE_CLEAR = "CLEAR"
ZONE_SLOW = "SLOW"
ZONE_STOP = "STOP"
ZONES = (ZONE_CLEAR, ZONE_SLOW, ZONE_STOP)

# Health flags the camera side may raise (report §5). The gate honours ANY flag
# in `health_flags` as forcing non-CLEAR; it does not need to understand each one
# to fail safe. TRACK_LOST_CLOSE is called out by name because gate_core uses it,
# together with its own observation of the STOP->CLEAR transition, to refuse a
# resume when a person was lost close to the robot rather than tracked leaving
# (report §5 #9, this task's rule set item 3).
HEALTH_TRACK_LOST_CLOSE = "TRACK_LOST_CLOSE"


@dataclass(frozen=True)
class CameraState:
    """One camera-side safety-state sample, in the robot base frame.

    Fields mirror report §7.1. Everything the gate needs to decide is here; the
    per-person tracks travel on a separate topic the gate does not consume.
    """

    # Evidence time (seconds, wall/ROS clock) — the time the SENSING happened,
    # NOT the time of publication (report §7.1, §7.2). Staleness is judged from
    # this, so a node that republishes a latched frame cannot look fresh.
    stamp: float
    # Camera-side zone: one of ZONES.
    zone: str
    # Distance to the nearest credible person point, metres, base frame. None =
    # unknown (only presence-only cameras saw the person; report §7.5).
    nearest_distance_m: Optional[float] = None
    # Nearest credible person point in base frame (x, y, z), optional (§7.1).
    nearest_point: Optional[Tuple[float, float, float]] = None
    # True iff the camera side asserts the judgement is fully healthy. Any health
    # fault (stale frame, dropped camera, stalled detector, ...) makes this False
    # and MUST force the gate off CLEAR (report §5, §7.4).
    health_ok: bool = True
    # Which specific rules are currently forcing non-CLEAR (informational; the
    # gate honours their presence, see HEALTH_TRACK_LOST_CLOSE).
    health_flags: FrozenSet[str] = field(default_factory=frozenset)
    # Monotonic heartbeat counter (§7.3). If this stops advancing while messages
    # keep arriving, the publisher is stuck -> the gate treats it as dead (STOP).
    heartbeat_seq: int = 0
    # Per-camera fresh/stale/absent map (§7.1), carried for the decision log; the
    # core does not need to parse it because `health_ok`/`health_flags` already
    # summarise it.
    cameras: Tuple[Tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        if self.zone not in ZONES:
            raise ValueError(f"zone {self.zone!r} not one of {ZONES}")
        if self.nearest_distance_m is not None and math.isnan(self.nearest_distance_m):
            # NaN is the wire encoding of "unknown"; normalise it to None so the
            # rest of the gate has exactly one "unknown" sentinel to test.
            object.__setattr__(self, "nearest_distance_m", None)

    @property
    def distance_known(self) -> bool:
        return self.nearest_distance_m is not None

    @property
    def is_clear(self) -> bool:
        """A CLEAR that is also healthy. A CLEAR zone with any health fault is
        NOT a clearance (report §5: absence of evidence is never clearance)."""
        return self.zone == ZONE_CLEAR and self.health_ok and not self.health_flags
