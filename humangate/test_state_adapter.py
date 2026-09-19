#!/usr/bin/env python3
"""
Offline tests for the camera-side -> gate state adapter — NO ROS, NO robot.

Asserts the adapter maps the REAL camera wire message
(cams/human_safety json_util.state_to_json) onto CameraState: it uses the
evidence stamp when a person is localised and the publication stamp otherwise,
honours distance_status (unknown/none never become a distance), derives a single
health_ok from the per-camera booleans, maps only the lost-close reason to a
flag, refuses a wrong schema version and a malformed message, and round-trips
through build_state_json (the exact wire shape the sim/integration publishers
use).

Run standalone:  python3 test_state_adapter.py
Or under pytest: pytest test_state_adapter.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import state_adapter as sa  # noqa: E402
from camera_state import (  # noqa: E402
    HEALTH_TRACK_LOST_CLOSE, ZONE_CLEAR, ZONE_SLOW, ZONE_STOP)


class _StringMsg:
    """A std_msgs/String stand-in: the adapter only touches `.data`."""

    def __init__(self, data):
        self.data = data


_FRESH_CAM = {"camera": "wrist", "status": "fresh", "present_in_graph": True,
              "detector_alive": True, "fresh": True, "positioned": True}
_STALE_CAM = {"camera": "table", "status": "stale", "present_in_graph": True,
              "detector_alive": True, "fresh": False, "positioned": True}


def test_known_distance_uses_base_distance():
    raw = sa.build_state_json(
        ZONE_STOP, stamp=100.0, evidence_stamp=99.9, heartbeat_seq=7,
        distance_status="known", nearest_distance_base=1.2, nearest_distance=0.4,
        cameras=[_FRESH_CAM])
    s = sa.parse_string_msg(_StringMsg(raw))
    assert s.zone == ZONE_STOP
    # localised -> ages against the evidence stamp, not the publication stamp
    assert s.stamp == 99.9
    # base-origin distance (matches the gate's stop_radius_m semantics)
    assert s.nearest_distance_m == 1.2 and s.distance_known
    assert s.heartbeat_seq == 7 and s.health_ok


def test_unknown_and_none_distance_are_never_a_distance():
    for status in ("unknown", "none"):
        raw = sa.build_state_json(
            ZONE_SLOW, stamp=5.0, distance_status=status,
            nearest_distance_base=None, cameras=[_FRESH_CAM])
        s = sa.parse_string_msg(_StringMsg(raw))
        assert s.nearest_distance_m is None and not s.distance_known


def test_clear_cycle_falls_back_to_publication_stamp():
    # No person localised -> evidence_stamp is null -> age against publication.
    raw = sa.build_state_json(
        ZONE_CLEAR, stamp=42.0, evidence_stamp=None, distance_status="none",
        cameras=[_FRESH_CAM])
    s = sa.parse_string_msg(_StringMsg(raw))
    assert s.stamp == 42.0 and s.is_clear


def test_stale_camera_forces_not_healthy_even_on_clear():
    raw = sa.build_state_json(
        ZONE_CLEAR, stamp=1.0, distance_status="none",
        cameras=[_FRESH_CAM, _STALE_CAM])
    s = sa.parse_string_msg(_StringMsg(raw))
    assert not s.health_ok and not s.is_clear   # contradiction -> not a clearance


def test_empty_camera_list_is_not_healthy():
    raw = sa.build_state_json(ZONE_CLEAR, stamp=1.0, distance_status="none",
                              cameras=[])
    s = sa.parse_string_msg(_StringMsg(raw))
    assert not s.health_ok


def test_lost_close_reason_maps_to_flag_but_narration_does_not():
    lost = sa.build_state_json(
        ZONE_STOP, stamp=1.0, evidence_stamp=1.0, distance_status="known",
        nearest_distance_base=0.8,
        reasons=["track id=3 lost while close (0.80 m) -> hold STOP"],
        cameras=[_FRESH_CAM])
    s = sa.parse_string_msg(_StringMsg(lost))
    assert HEALTH_TRACK_LOST_CLOSE in s.health_flags
    # a normal narration reason must NOT become a health flag
    narr = sa.build_state_json(
        ZONE_SLOW, stamp=1.0, evidence_stamp=1.0, distance_status="known",
        nearest_distance_base=2.1,
        reasons=["nearest person id=3 at 2.10 m from base (0.80 m from reach envelope)"],
        cameras=[_FRESH_CAM])
    s2 = sa.parse_string_msg(_StringMsg(narr))
    assert not s2.health_flags


def test_wrong_schema_version_rejected():
    raw = sa.build_state_json(ZONE_CLEAR, stamp=1.0, distance_status="none",
                              cameras=[_FRESH_CAM], schema_version=999)
    try:
        sa.parse_string_msg(_StringMsg(raw))
    except ValueError:
        return
    raise AssertionError("a mismatched schema_version must be rejected")


def test_bad_zone_rejected():
    for bad in ("NOPE", None, "clear"):
        try:
            sa.from_dict({sa.FIELD_STAMP: 1.0, sa.FIELD_ZONE: bad})
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for zone={bad!r}")


def test_missing_stamp_rejected():
    try:
        sa.from_dict({sa.FIELD_ZONE: ZONE_CLEAR})
    except ValueError:
        return
    raise AssertionError("expected ValueError for a message with no usable time")


def test_nan_distance_becomes_none():
    raw = json.dumps({sa.FIELD_SCHEMA: 1, sa.FIELD_ZONE: ZONE_SLOW,
                      sa.FIELD_STAMP: 1.0, sa.FIELD_DISTANCE_STATUS: "known",
                      sa.FIELD_NEAREST_DISTANCE_BASE: float("nan"),
                      sa.FIELD_CAMERAS: [_FRESH_CAM]})
    s = sa.parse_string_msg(_StringMsg(raw))
    assert s.nearest_distance_m is None


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
