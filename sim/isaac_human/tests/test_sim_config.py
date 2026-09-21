# SPDX-License-Identifier: Apache-2.0
"""The sim config must load through the REAL human_safety loader, must not drift
any safety threshold from the real config, and its generated TF file must be
in sync with cell_layout."""
import os

import gen_sim_config
from cell_layout import CAMERAS
from human_safety.camera_poses import load_with_reasons
from human_safety.config import load_config

_HERE = os.path.dirname(os.path.abspath(__file__))
_HARNESS = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(_HARNESS))
_SIM_CFG = os.path.join(_HARNESS, "params", "human_safety.sim.yaml")
_SIM_POSES = os.path.join(_HARNESS, "params", "camera_poses.sim.yaml")
_REAL_CFG = os.path.join(_REPO, "cams", "human_safety", "params", "human_safety.yaml")


def test_sim_config_loads():
    cfg = load_config(_SIM_CFG)
    roles = [c.role for c in cfg.cameras]
    assert roles == ["wrist", "shelf", "table"]
    for c in cfg.cameras:
        assert c.color_topic.startswith("/sim/")
        assert c.has_depth
        assert c.pose_source == "tf"


def test_sim_config_changes_no_safety_threshold():
    sim = load_config(_SIM_CFG)
    real = load_config(_REAL_CFG)
    # every safety parameter identical
    assert sim.safety.__dict__ == real.safety.__dict__
    # perception thresholds identical too (nothing loosened to flatter a result)
    assert sim.score_thresh == real.score_thresh
    assert sim.detector == real.detector
    assert sim.ground_z == real.ground_z
    assert sim.height_prior == real.height_prior


def test_sim_poses_all_valid_and_calibrated():
    good, rejected = load_with_reasons(_SIM_POSES)
    assert rejected == []
    roles = sorted(t.role for t in good)
    assert roles == ["shelf", "table", "wrist"]
    for t in good:
        assert t.parent == "base_link"
        assert t.child.endswith("_color_optical_frame")
        # unit quaternion
        n = sum(v * v for v in t.rotation) ** 0.5
        assert abs(n - 1.0) < 1e-6


def test_generated_poses_in_sync_with_layout(tmp_path):
    # Regenerating must reproduce the committed file byte-for-byte.
    rendered = gen_sim_config.render()
    with open(_SIM_POSES) as f:
        committed = f.read()
    assert rendered == committed, (
        "params/camera_poses.sim.yaml is stale — run "
        "`python3 sim/isaac_human/gen_sim_config.py`")


def test_sim_config_frames_match_layout():
    cfg = load_config(_SIM_CFG)
    layout_frames = {c.role: c.optical_frame for c in CAMERAS}
    for c in cfg.cameras:
        assert c.optical_frame == layout_frames[c.role]
