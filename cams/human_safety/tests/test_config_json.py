# SPDX-License-Identifier: Apache-2.0
"""Config loading + JSON round-trip tests."""
import pathlib

import numpy as np

from human_safety.config import load_config
from human_safety.json_util import (detection_from_dict, detection_to_dict,
                                    detections_from_json, detections_to_json,
                                    state_to_json)
from human_safety.messages import Detection3D, Method
from human_safety.replay import replay_file
from human_safety.safety import SafetyEvaluator

PARAMS = pathlib.Path(__file__).parents[1] / "params" / "human_safety.yaml"


def test_shipped_config_loads():
    cfg = load_config(str(PARAMS))
    roles = {c.role for c in cfg.cameras}
    assert roles == {"wrist", "shelf", "table"}
    w = cfg.camera("wrist")
    assert w.color_topic == "/wrist/color_upright/image_raw"
    assert w.max_rate == 6.0                     # wrist only delivers ~6 fps
    assert cfg.safety.stop_radius < cfg.safety.slow_radius
    assert cfg.base_frame == "base_link"


def test_detection_json_roundtrip():
    d = Detection3D(camera="shelf", stamp=1.5, score=0.8, method=Method.DEPTH,
                    position=np.array([1.0, 2.0, 0.9]),
                    nearest_point=np.array([1.0, 1.9, 0.9]), sigma=0.12,
                    n_depth_valid=321)
    back = detection_from_dict(detection_to_dict(d))
    assert back.camera == d.camera
    assert back.method == Method.DEPTH
    assert np.allclose(back.position, d.position)
    assert back.n_depth_valid == 321


def test_presence_detection_roundtrip():
    d = Detection3D(camera="wrist", stamp=2.0, score=0.7, method=Method.PRESENCE,
                    present=True, position=None)
    s = detections_to_json("wrist", 2.0, [d])
    back = detections_from_json(s)
    assert len(back) == 1 and not back[0].positioned and back[0].present


def test_state_json_serialises():
    states = replay_file(
        str(pathlib.Path(__file__).parent / "data" / "synthetic_approach.jsonl"))
    js = state_to_json(states[35])
    assert '"zone"' in js and '"nearest_distance"' in js
