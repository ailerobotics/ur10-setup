#!/usr/bin/env python3
"""
Thin adapter: the camera side's REAL wire message  ->  the gate's CameraState.

The whole point of this file is that the decision core (gate_core.py) NEVER sees
the camera-side wire format. It only ever sees `camera_state.CameraState`. When
the perception layer changes a field, ONLY this file changes.

REAL MESSAGE (confirmed against the committed perception package, 2026-09-19):
`cams/human_safety` publishes its safety state on `std_msgs/String` topic
`/human_safety/state` as the JSON produced by
`cams/human_safety/human_safety/json_util.py::state_to_json`. It uses a
`std_msgs/String` JSON payload deliberately — that package is NOT colcon-built
(no package.xml anywhere in the repo), so a custom .msg would force a build step
into the otherwise script-based workflow (see its `messages.py` docstring). So
this JSON transport is the PERMANENT contract, not a stand-in.

The real payload fields (see `state_to_json`) and how the gate maps them:

  schema_version   int         -> checked against EXPECTED_SCHEMA_VERSION
  frame_id         "base_link" -> informational (the gate assumes base frame)
  stamp            float       -> PUBLICATION wall time
  evidence_stamp   float|null  -> oldest positioned-detection stamp sizing the
                                  STOP ring; null when nobody is being localised
  heartbeat_seq    int         -> CameraState.heartbeat_seq (watchdog)
  zone             enum        -> CameraState.zone
  distance_status  enum        -> "known"|"unknown"|"none"; gates distance honesty
  nearest_distance      float|null -> distance to the reach ENVELOPE
  nearest_distance_base float|null -> distance to the base ORIGIN
  nearest_track_id int|null    -> informational
  any_person_visible bool      -> informational (presence-only forces SLOW already)
  reasons          list[str]   -> which rules force non-CLEAR (health + narration)
  cameras          list[dict]  -> per-camera fresh/stale/absent + health booleans
  tracks           list        -> not consumed by the gate

Two mappings are safety-critical and are spelled out where they happen below:

1. STALENESS STAMP. `CameraState.stamp` is the time the gate ages against
   `state_max_age_s`. We use `evidence_stamp` when a person is being localised
   (its true evidence age is what matters, and a republished latched frame keeps
   an old evidence stamp so it cannot look fresh — report §7.2). When
   `evidence_stamp` is null (a CLEAR, or presence-only, cycle: nobody localised)
   we fall back to the publication `stamp`; message recency is then guaranteed by
   the gate's arrival-based heartbeat watchdog, and per-camera staleness has
   already been folded into `zone`/health by the camera side.

2. DISTANCE HONESTY (report §7.5). `nearest_distance_m` is populated ONLY when
   `distance_status == "known"`; for "unknown" (presence-only) and "none" it is
   left None. The gate never treats an unknown distance as "far" — and in fact
   never routes distance to RUN at all (see gate_core).

This module is PURE: it imports only the standard library, never rclpy or any ROS
message package, so every offline test (and any host without a ROS stack) can
build and reason about a camera state exactly as the gate does. `parse_string_msg`
duck-types `.data` off whatever std_msgs/String-like object it is handed.
"""
from __future__ import annotations

import json
import math
from typing import Any, Optional

from camera_state import (  # noqa: E402 (loose-module import, see tests)
    HEALTH_TRACK_LOST_CLOSE,
    CameraState,
    ZONES,
)

# Topic the camera side publishes on (json_util / fusion_node). Rename here ==
# one-line change for the whole gate.
TOPIC_DEFAULT = "/human_safety/state"

# Mirrors cams/human_safety/human_safety/json_util.py::STATE_SCHEMA_VERSION. A
# message that declares a DIFFERENT schema version is a contract break: we refuse
# it (ValueError) so the node logs and the watchdog holds the gate at STOP,
# rather than silently mis-parsing a changed wire format. A message with NO
# schema_version is tolerated (older/hand-built), still validated field-by-field.
EXPECTED_SCHEMA_VERSION = 1

# --- real wire field names (json_util.state_to_json). ----------------------- #
FIELD_SCHEMA = "schema_version"
FIELD_STAMP = "stamp"                          # publication wall time
FIELD_EVIDENCE_STAMP = "evidence_stamp"        # oldest positioned-detection stamp
FIELD_HEARTBEAT_SEQ = "heartbeat_seq"          # int, monotonic
FIELD_ZONE = "zone"                            # one of camera_state.ZONES
FIELD_DISTANCE_STATUS = "distance_status"      # known | unknown | none
FIELD_NEAREST_DISTANCE = "nearest_distance"    # to the reach envelope, m
FIELD_NEAREST_DISTANCE_BASE = "nearest_distance_base"  # to the base origin, m
FIELD_NEAREST_TRACK_ID = "nearest_track_id"    # int | null
FIELD_ANY_PERSON_VISIBLE = "any_person_visible"  # bool
FIELD_REASONS = "reasons"                      # list[str]
FIELD_CAMERAS = "cameras"                      # list[dict] per-camera health

DIST_KNOWN = "known"

# The exact substring the camera side emits when a confirmed track vanished while
# inside stop_radius+margin (safety.py::_health_floor):
#   f"track id={tid} lost while close ({last_dist:.2f} m) -> hold STOP"
# The gate keys on the STOP->CLEAR transition for its resume refusal (gate_core),
# so this flag is a belt-and-suspenders confirmation, not the sole signal.
LOST_CLOSE_REASON_SUBSTR = "lost while close"


def _to_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    f = float(v)
    return None if math.isnan(f) else f


def _cam_healthy(entry: dict) -> bool:
    """A camera entry counts as healthy iff it is in the graph, its detector is
    answering, and its latest frame is fresh. Missing booleans fall back to the
    coarse `status` string; anything not clearly fresh is treated as unhealthy
    (fail toward non-CLEAR)."""
    if not isinstance(entry, dict):
        return False
    present = entry.get("present_in_graph")
    alive = entry.get("detector_alive")
    fresh = entry.get("fresh")
    if present is None and alive is None and fresh is None:
        return entry.get("status") == "fresh"
    return bool(present) and bool(alive) and bool(fresh)


def from_dict(d: dict) -> CameraState:
    """Build a CameraState from the camera side's decoded JSON payload.

    Raises ValueError on any contract violation (bad/absent zone, no usable
    stamp, wrong schema version). The caller (parse_string_msg / the node)
    surfaces the error and the watchdog keeps the gate at STOP — a malformed
    state is never a clearance.
    """
    schema = d.get(FIELD_SCHEMA)
    if schema is not None and int(schema) != EXPECTED_SCHEMA_VERSION:
        raise ValueError(
            f"{FIELD_SCHEMA}={schema} != expected {EXPECTED_SCHEMA_VERSION} "
            "(camera-side wire format changed; refusing to guess)")

    zone = d.get(FIELD_ZONE)
    if zone not in ZONES:
        raise ValueError(f"{FIELD_ZONE}={zone!r} not one of {ZONES}")

    # Staleness stamp: evidence age when a person is localised, else publication.
    evidence = _to_float(d.get(FIELD_EVIDENCE_STAMP))
    pub = _to_float(d.get(FIELD_STAMP))
    stamp = evidence if evidence is not None else pub
    if stamp is None:
        raise ValueError(
            f"neither {FIELD_EVIDENCE_STAMP} nor {FIELD_STAMP} present "
            "(no usable evidence time)")

    # Distance honesty: only a KNOWN distance is a real distance. We carry the
    # base-origin distance (matches the gate's stop_radius_m base semantics).
    status = d.get(FIELD_DISTANCE_STATUS)
    if status == DIST_KNOWN:
        dist = _to_float(d.get(FIELD_NEAREST_DISTANCE_BASE))
        if dist is None:
            dist = _to_float(d.get(FIELD_NEAREST_DISTANCE))
    else:
        dist = None

    # Health: derive a single health_ok from the structured per-camera booleans
    # (defence in depth — a CLEAR zone alongside a stale/absent camera is an
    # internal contradiction and must force STOP). An empty camera list is itself
    # untrustworthy -> not healthy.
    cams_raw = d.get(FIELD_CAMERAS) or []
    if not isinstance(cams_raw, list):
        cams_raw = []
    health_ok = bool(cams_raw) and all(_cam_healthy(c) for c in cams_raw)

    # Health flags the gate reasons about explicitly. Only the lost-close signal
    # is mapped (gate_core special-cases it); the many narration reasons
    # ("nearest person id=...") are deliberately NOT turned into flags so a normal
    # SLOW/STOP does not masquerade as a health fault, and a genuine healthy CLEAR
    # carries no flags.
    reasons = d.get(FIELD_REASONS) or []
    flags = set()
    if any(LOST_CLOSE_REASON_SUBSTR in str(r) for r in reasons):
        flags.add(HEALTH_TRACK_LOST_CLOSE)

    cameras = tuple(
        (str(c.get("camera", "?")), str(c.get("status", "?")))
        for c in cams_raw if isinstance(c, dict))

    return CameraState(
        stamp=float(stamp),
        zone=zone,
        nearest_distance_m=dist,
        nearest_point=None,   # the real state carries points only inside `tracks`
        health_ok=health_ok,
        health_flags=frozenset(flags),
        heartbeat_seq=int(d.get(FIELD_HEARTBEAT_SEQ, 0)),
        cameras=cameras,
    )


def parse_string_msg(msg: Any) -> CameraState:
    """Adapt a std_msgs/String-like message (has `.data`, a JSON object) to a
    CameraState. This IS the real, permanent transport (the camera side uses
    std_msgs/String JSON by design; see the module docstring)."""
    return from_dict(json.loads(msg.data))


def build_state_json(
    zone: str,
    *,
    stamp: float,
    evidence_stamp: Optional[float] = None,
    heartbeat_seq: int = 0,
    distance_status: str = "none",
    nearest_distance_base: Optional[float] = None,
    nearest_distance: Optional[float] = None,
    nearest_track_id: Optional[int] = None,
    any_person_visible: bool = False,
    reasons: Optional[list] = None,
    cameras: Optional[list] = None,
    schema_version: int = EXPECTED_SCHEMA_VERSION,
) -> str:
    """Build a message in the EXACT camera-side wire format (json_util.state_to_json
    shape) from high-level fields. Used by the offline round-trip tests, the ROS
    integration test, and the scripted-state publisher in humangate_driver/sim so
    every producer speaks the real contract. `cameras` defaults to one fresh,
    healthy camera so a CLEAR built here is a genuine healthy CLEAR."""
    if cameras is None:
        cameras = [{
            "camera": "sim", "status": "fresh", "present_in_graph": True,
            "detector_alive": True, "positioned": True, "fresh": True,
            "frame_age": 0.0, "note": "",
        }]
    return json.dumps({
        FIELD_SCHEMA: schema_version,
        "frame_id": "base_link",
        FIELD_STAMP: stamp,
        FIELD_EVIDENCE_STAMP: evidence_stamp,
        FIELD_HEARTBEAT_SEQ: heartbeat_seq,
        FIELD_ZONE: zone,
        FIELD_DISTANCE_STATUS: distance_status,
        FIELD_NEAREST_DISTANCE: nearest_distance,
        FIELD_NEAREST_DISTANCE_BASE: nearest_distance_base,
        FIELD_NEAREST_TRACK_ID: nearest_track_id,
        FIELD_ANY_PERSON_VISIBLE: any_person_visible,
        FIELD_REASONS: list(reasons or []),
        FIELD_CAMERAS: cameras,
        "tracks": [],
    })
