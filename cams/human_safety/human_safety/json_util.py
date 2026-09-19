# SPDX-License-Identifier: Apache-2.0
"""JSON (de)serialisation of the data model for ROS ``std_msgs/String`` topics
and for the replay log. Kept separate from ``messages`` so the dataclasses stay
free of transport concerns.
"""
from __future__ import annotations

import json
from typing import List

import numpy as np

from .messages import (CameraHealth, Detection3D, Method, SafetyState, Track,
                       Zone)


def _vec(a):
    return None if a is None else [float(x) for x in np.asarray(a).reshape(-1)]


def _arr(a):
    return None if a is None else np.asarray(a, dtype=float)


# --------------------------------------------------------------------------- #
def detection_to_dict(d: Detection3D) -> dict:
    return {
        "camera": d.camera,
        "stamp": d.stamp,
        "score": d.score,
        "method": d.method.value,
        "present": d.present,
        "position": _vec(d.position),
        "nearest_point": _vec(d.nearest_point),
        "sigma": d.sigma,
        "n_depth_valid": d.n_depth_valid,
    }


def detection_from_dict(x: dict) -> Detection3D:
    return Detection3D(
        camera=x["camera"],
        stamp=x["stamp"],
        score=x["score"],
        method=Method(x["method"]),
        present=x.get("present", True),
        position=_arr(x.get("position")),
        nearest_point=_arr(x.get("nearest_point")),
        sigma=x.get("sigma", 0.0),
        n_depth_valid=x.get("n_depth_valid", 0),
    )


def detections_to_json(camera: str, stamp: float, dets: List[Detection3D]) -> str:
    return json.dumps({
        "camera": camera,
        "stamp": stamp,
        "detections": [detection_to_dict(d) for d in dets],
    })


def frame_to_json(camera: str, stamp: float, dets3d: List[Detection3D],
                  dets2d: list) -> str:
    """Serialise a processed frame carrying BOTH the 3D detection fields (which
    ``detections_from_json`` reads) and the 2D overlay fields (bbox, keypoints)
    the viewer draws. Extra keys are ignored by the 3D decoder."""
    out = []
    for d3, d2 in zip(dets3d, dets2d):
        rec = detection_to_dict(d3)
        rec["bbox"] = [float(v) for v in d2.bbox]
        rec["kp"] = (None if d2.keypoints is None
                     else [[float(x) for x in row] for row in d2.keypoints])
        out.append(rec)
    return json.dumps({"camera": camera, "stamp": stamp, "detections": out})


def detections_from_json(s: str) -> List[Detection3D]:
    x = json.loads(s)
    return [detection_from_dict(d) for d in x["detections"]]


# --------------------------------------------------------------------------- #
def track_to_dict(t: Track) -> dict:
    return {
        "id": t.id,
        "position": _vec(t.position),
        "velocity": _vec(t.velocity),
        "stamp": t.stamp,
        "hits": t.hits,
        "age": t.age,
        "time_since_update": t.time_since_update,
        "confirmed": t.confirmed,
        "nearest_point": _vec(t.nearest_point),
        "cameras": list(t.cameras),
    }


def camera_status(h: CameraHealth) -> str:
    """Per-camera state for the interface contract (report §7 #1)."""
    if not h.present_in_graph:
        return "absent"
    return "fresh" if h.fresh else "stale"


def health_to_dict(h: CameraHealth) -> dict:
    return {
        "camera": h.camera,
        "status": camera_status(h),               # fresh | stale | absent
        "present_in_graph": h.present_in_graph,
        "frame_age": None if h.frame_age == float("inf") else h.frame_age,
        "detector_alive": h.detector_alive,
        "positioned": h.positioned,
        "fresh": h.fresh,
        "note": h.note,
    }


# Schema version of the /human_safety/state JSON. Bump on any breaking change so
# the robot-side gate's adapter can pin it (report §7 "thin adapter").
STATE_SCHEMA_VERSION = 1


def state_to_json(s: SafetyState) -> str:
    return json.dumps({
        "schema_version": STATE_SCHEMA_VERSION,
        "frame_id": "base_link",
        "stamp": s.stamp,                          # publication wall time
        "evidence_stamp": s.evidence_stamp,        # oldest positioned-detection stamp (STOP-ring evidence)
        "heartbeat_seq": s.heartbeat_seq,          # monotonic; gate watchdog
        "zone": s.zone.value,                      # CLEAR | SLOW | STOP
        "distance_status": s.distance_status.value,  # known | unknown | none
        # nearest_distance is null when distance_status != known: UNKNOWN is NOT
        # 'far' (report §7 #5) -- the gate must read distance_status, not treat a
        # null distance as clear.
        "nearest_distance": (_num(s.nearest_distance)
                             if s.distance_status.value == "known" else None),
        "nearest_distance_base": (_num(s.nearest_distance_base)
                                  if s.distance_status.value == "known" else None),
        "nearest_track_id": s.nearest_track_id,
        "any_person_visible": s.any_person_visible,
        "reasons": list(s.reasons),                # which rules force non-CLEAR
        "cameras": [health_to_dict(h) for h in s.health],
        "tracks": [track_to_dict(t) for t in s.tracks],
    })


def _num(x):
    return None if x == float("inf") else float(x)


def state_from_json(s: str) -> dict:
    """Decode a state JSON to a plain dict (viz/monitor consumer)."""
    return json.loads(s)
