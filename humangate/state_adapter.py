#!/usr/bin/env python3
"""
Thin adapter: camera-side wire message  ->  gate's CameraState.

The whole point of this file is that the decision core (gate_core.py) NEVER sees
the camera-side wire format. It only ever sees `camera_state.CameraState`. When
the perception package pins its real message, ONLY this file changes, and a
field rename is a one-line change to the field-name constants below.

STATUS (2026-09-19): the perception package on branch `fm/human-detect-core-v1`
is NOT yet committed to this worktree (its branch is at the same commit as main),
so there is no custom message to build against. Per this task's brief we:
  * define the adapter against report §7, and
  * record precisely what must be confirmed when the real message lands
    (see CONTRACT_TO_CONFIRM below and data/human-gate-core-v1/decision.md).

PROVISIONAL TRANSPORT: until the custom message exists, the gate consumes the
state as a std_msgs/String carrying a JSON object with the §7 fields. This keeps
the whole gate — including the one ROS integration test — runnable now without
depending on an uncommitted package, and makes the swap to the real message a
localised change (see `parse_custom_msg`). The JSON key names and the eventual
custom-message field names are the SAME strings (the FIELD_* constants), so
aligning them later is trivial.

This module is import-safe without ROS: it never imports rclpy or any message
package. `parse_string_msg` duck-types `.data` off whatever std_msgs/String-like
object it is handed, so the offline tests and replay harness use it directly.
"""
from __future__ import annotations

import json
import math
from typing import Any, Optional

from camera_state import (  # noqa: E402 (loose-module import, see tests)
    CameraState,
    ZONES,
)

# --- Topic + field-name contract (report §7). Rename here == one-line change. --
TOPIC_DEFAULT = "/human_safety/state"

FIELD_STAMP = "stamp"                     # evidence time, seconds (float)
FIELD_ZONE = "zone"                       # one of camera_state.ZONES
FIELD_NEAREST_DISTANCE = "nearest_distance_m"   # float; null/NaN == unknown
FIELD_NEAREST_POINT = "nearest_point"     # [x, y, z] in base frame, optional
FIELD_HEALTH_OK = "health_ok"             # bool
FIELD_HEALTH_FLAGS = "health_flags"       # list[str]
FIELD_HEARTBEAT_SEQ = "heartbeat_seq"     # int, monotonic
FIELD_CAMERAS = "cameras"                 # {name: "fresh"|"stale"|"absent"}

# What MUST be confirmed / diffed against report §7 once the perception message
# is committed (task brief item 2). Kept machine-visible so a later change can
# assert against it; mirrored in data/human-gate-core-v1/decision.md.
CONTRACT_TO_CONFIRM = (
    "exact custom message name and package (adapter currently uses a std_msgs/"
    "String JSON transport as a stand-in)",
    "exact topic name (default assumed: %s)" % TOPIC_DEFAULT,
    "whether the evidence stamp is a builtin_interfaces/Time header stamp or a "
    "float field, and that it is EVIDENCE time not publication time (§7.1/§7.2)",
    "whether health is an enum, a bool, or a bitfield, and the flag naming "
    "(gate needs a distinguishable TRACK_LOST_CLOSE signal, §5 #9)",
    "whether a separate machine-readable heartbeat is published or heartbeat_seq "
    "is a message field (§7.3)",
    "the unknown-distance encoding (NaN vs field-absent vs null) (§7.5)",
    "the guaranteed minimum publish rate (§7.2, recommend >=10 Hz)",
)


def _to_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    f = float(v)
    return None if math.isnan(f) else f


def from_dict(d: dict) -> CameraState:
    """Build a CameraState from a plain dict using the §7 field names.

    Missing optional fields fall back to safe defaults. A missing or unknown
    `zone` is itself a contract violation; we do NOT guess a safe zone here —
    the caller (parse_string_msg) surfaces the error, and the node's watchdog
    keeps the gate at STOP whenever no valid state has been received.
    """
    zone = d.get(FIELD_ZONE)
    if zone not in ZONES:
        raise ValueError(f"{FIELD_ZONE}={zone!r} not one of {ZONES}")
    stamp = d.get(FIELD_STAMP)
    if stamp is None:
        raise ValueError(f"{FIELD_STAMP} is required (evidence time, seconds)")
    np_ = d.get(FIELD_NEAREST_POINT)
    nearest_point = tuple(float(x) for x in np_) if np_ else None
    if nearest_point is not None and len(nearest_point) != 3:
        raise ValueError(f"{FIELD_NEAREST_POINT} must be [x, y, z]")
    flags = d.get(FIELD_HEALTH_FLAGS) or []
    cams = d.get(FIELD_CAMERAS) or {}
    cameras = tuple(sorted((str(k), str(v)) for k, v in dict(cams).items()))
    return CameraState(
        stamp=float(stamp),
        zone=zone,
        nearest_distance_m=_to_float(d.get(FIELD_NEAREST_DISTANCE)),
        nearest_point=nearest_point,  # type: ignore[arg-type]
        health_ok=bool(d.get(FIELD_HEALTH_OK, True)),
        health_flags=frozenset(str(x) for x in flags),
        heartbeat_seq=int(d.get(FIELD_HEARTBEAT_SEQ, 0)),
        cameras=cameras,
    )


def parse_string_msg(msg: Any) -> CameraState:
    """Adapt a std_msgs/String-like message (has `.data`, a JSON object) to a
    CameraState. This is the PROVISIONAL transport; see module docstring."""
    return from_dict(json.loads(msg.data))


def to_json(state: CameraState) -> str:
    """Encode a CameraState as the provisional JSON transport payload. Used by
    the replay harness and the ROS integration test's fake publisher so both
    sides speak exactly the same contract."""
    payload = {
        FIELD_STAMP: state.stamp,
        FIELD_ZONE: state.zone,
        FIELD_NEAREST_DISTANCE: state.nearest_distance_m,
        FIELD_NEAREST_POINT: list(state.nearest_point) if state.nearest_point else None,
        FIELD_HEALTH_OK: state.health_ok,
        FIELD_HEALTH_FLAGS: sorted(state.health_flags),
        FIELD_HEARTBEAT_SEQ: state.heartbeat_seq,
        FIELD_CAMERAS: {k: v for k, v in state.cameras},
    }
    return json.dumps(payload)


def parse_custom_msg(msg: Any) -> CameraState:  # pragma: no cover - not wired yet
    """SWAP POINT for when human_detect_core_v1 commits its custom message.

    Replace the body with a direct field read off the real message, e.g.:

        return CameraState(
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            zone=_ZONE_FROM_ENUM[msg.state],
            nearest_distance_m=_to_float(msg.nearest_distance_m),
            ...
        )

    and point gate_node at the real message type + this function. Until then it
    is intentionally unimplemented so nothing silently depends on a phantom msg.
    """
    raise NotImplementedError(
        "custom camera-side message not committed yet; using parse_string_msg. "
        "See CONTRACT_TO_CONFIRM and data/human-gate-core-v1/decision.md."
    )
