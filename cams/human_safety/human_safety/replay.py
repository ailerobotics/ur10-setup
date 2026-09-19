# SPDX-License-Identifier: Apache-2.0
"""Deterministic replay of a recorded detection log through Fusion + Safety.

A recording is a JSON-lines file, one object per fusion cycle:

    {"now": <float>, "detections": [<Detection3D dict>, ...],
     "health": [<CameraHealth dict>, ...], "any_presence": <bool>}

``record_replay.py`` writes this from live topics; this module replays it with no
ROS, no cameras and no GPU, so a captured walk-through becomes a deterministic
regression test (the state sequence must reproduce exactly).
"""
from __future__ import annotations

import json
from typing import List

from .fusion import Fusion
from .json_util import detection_from_dict
from .messages import CameraHealth
from .safety import SafetyEvaluator, SafetyParams


def _health_from_dict(x: dict) -> CameraHealth:
    return CameraHealth(
        camera=x["camera"],
        present_in_graph=x.get("present_in_graph", True),
        frame_age=(float("inf") if x.get("frame_age") is None
                   else float(x["frame_age"])),
        detector_alive=x.get("detector_alive", True),
        positioned=x.get("positioned", False),
        fresh=x.get("fresh", True),
        note=x.get("note", ""),
    )


def replay_records(records: List[dict], params: SafetyParams | None = None,
                   tracker_kw: dict | None = None):
    """Replay a list of cycle records. Returns a list of SafetyState."""
    fusion = Fusion()
    if tracker_kw:
        from .tracker import MultiTargetTracker
        fusion = Fusion(MultiTargetTracker(**tracker_kw))
    ev = SafetyEvaluator(params)
    out = []
    for rec in records:
        now = float(rec["now"])
        dets = [detection_from_dict(d) for d in rec.get("detections", [])]
        health = [_health_from_dict(h) for h in rec.get("health", [])]
        tracks, presence_cams = fusion.step(dets, now)
        any_presence = bool(rec.get("any_presence", False) or presence_cams)
        # Match the live fusion node: freshest fused evidence (max) and the
        # end-to-end latency, so a real recording reproduces fault-5 STOPs.
        evidence_stamp = max((d.stamp for d in dets), default=None)
        latency = (now - evidence_stamp) if evidence_stamp is not None else None
        state = ev.evaluate(tracks, health, now, any_presence=any_presence,
                            source_stamp=evidence_stamp, latency=latency)
        out.append(state)
    return out


def load_jsonl(path: str) -> List[dict]:
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def replay_file(path: str, params: SafetyParams | None = None):
    return replay_records(load_jsonl(path), params)
