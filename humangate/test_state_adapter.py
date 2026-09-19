#!/usr/bin/env python3
"""
Offline tests for the camera-side -> gate state adapter — NO ROS, NO robot.

Asserts the adapter maps the report §7 fields onto CameraState, round-trips
through the provisional JSON transport, normalises the "unknown distance"
encodings, and rejects a message that violates the contract (so the node keeps
the gate at STOP rather than acting on a malformed state).

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
    HEALTH_TRACK_LOST_CLOSE, ZONE_CLEAR, ZONE_SLOW, ZONE_STOP, CameraState)


class _StringMsg:
    """A std_msgs/String stand-in: the adapter only touches `.data`."""

    def __init__(self, data):
        self.data = data


def test_from_dict_maps_all_fields():
    d = {
        sa.FIELD_STAMP: 12.5, sa.FIELD_ZONE: ZONE_STOP,
        sa.FIELD_NEAREST_DISTANCE: 0.8, sa.FIELD_NEAREST_POINT: [0.1, 0.2, 0.3],
        sa.FIELD_HEALTH_OK: True, sa.FIELD_HEALTH_FLAGS: [HEALTH_TRACK_LOST_CLOSE],
        sa.FIELD_HEARTBEAT_SEQ: 7,
        sa.FIELD_CAMERAS: {"wrist": "stale", "table": "fresh"},
    }
    s = sa.from_dict(d)
    assert s.stamp == 12.5 and s.zone == ZONE_STOP
    assert s.nearest_distance_m == 0.8 and s.nearest_point == (0.1, 0.2, 0.3)
    assert HEALTH_TRACK_LOST_CLOSE in s.health_flags and s.heartbeat_seq == 7
    assert dict(s.cameras) == {"wrist": "stale", "table": "fresh"}


def test_unknown_distance_null_and_nan_both_become_none():
    for enc in (None,):
        s = sa.from_dict({sa.FIELD_STAMP: 1.0, sa.FIELD_ZONE: ZONE_SLOW,
                          sa.FIELD_NEAREST_DISTANCE: enc})
        assert s.nearest_distance_m is None and not s.distance_known
    # NaN via the JSON transport
    raw = json.dumps({sa.FIELD_STAMP: 1.0, sa.FIELD_ZONE: ZONE_SLOW,
                      sa.FIELD_NEAREST_DISTANCE: float("nan")})
    s = sa.parse_string_msg(_StringMsg(raw))
    assert s.nearest_distance_m is None


def test_string_msg_round_trip():
    original = CameraState(stamp=3.0, zone=ZONE_SLOW, nearest_distance_m=2.1,
                           nearest_point=(1.0, 2.0, 3.0), health_ok=True,
                           health_flags=frozenset({"GPU_SLOW"}), heartbeat_seq=42,
                           cameras=(("shelf", "absent"),))
    s = sa.parse_string_msg(_StringMsg(sa.to_json(original)))
    assert s.stamp == original.stamp and s.zone == original.zone
    assert s.nearest_distance_m == original.nearest_distance_m
    assert s.nearest_point == original.nearest_point
    assert s.health_flags == original.health_flags
    assert s.heartbeat_seq == original.heartbeat_seq
    assert dict(s.cameras) == dict(original.cameras)


def test_unknown_distance_json_encodes_as_null():
    s = CameraState(stamp=1.0, zone=ZONE_SLOW, nearest_distance_m=None)
    payload = json.loads(sa.to_json(s))
    assert payload[sa.FIELD_NEAREST_DISTANCE] is None


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
    raise AssertionError("expected ValueError for a message with no stamp")


def test_custom_msg_parser_is_unimplemented_until_message_lands():
    try:
        sa.parse_custom_msg(object())
    except NotImplementedError:
        return
    raise AssertionError("parse_custom_msg must stay unimplemented until the "
                         "perception message is committed")


def test_contract_to_confirm_is_recorded():
    # the swap list must be non-empty and mention the key ambiguities
    text = " ".join(sa.CONTRACT_TO_CONFIRM).lower()
    assert sa.CONTRACT_TO_CONFIRM
    for needle in ("message name", "topic", "stamp", "health", "heartbeat", "distance"):
        assert needle in text, f"CONTRACT_TO_CONFIRM should mention {needle!r}"


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
