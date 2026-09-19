# SPDX-License-Identifier: Apache-2.0
"""Load and validate the human_safety YAML config (cameras + safety params)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml

from .safety import SafetyParams


@dataclass
class CameraConfig:
    role: str
    color_topic: str
    info_topic: str
    depth_topic: Optional[str] = None
    depth_info_topic: Optional[str] = None
    has_depth: bool = True
    optical_frame: str = ""
    max_rate: float = 15.0            # cap FPS; the wrist only delivers ~6 fps
    min_range: float = 0.2            # RealSense min sensing range, metres
    max_range: float = 6.0           # trust depth no farther than this
    # Pose source. "tf": look up base<-optical_frame via tf2 (wrist uses live
    # robot TF; a static D415 uses a static_transform_publisher). "none":
    # presence-only until calibrated.
    pose_source: str = "tf"
    presence_only: bool = False       # force presence-only regardless of TF


@dataclass
class HumanSafetyConfig:
    base_frame: str = "base_link"
    ground_z: float = 0.0
    height_prior: float = 1.7
    detector: str = "keypoint_rcnn"
    device: str = "cuda"
    score_thresh: float = 0.6
    fusion_rate: float = 15.0         # Hz; >=10 Hz per human-safety-s1 §7
    viz_rate: float = 6.0
    cameras: List[CameraConfig] = field(default_factory=list)
    safety: SafetyParams = field(default_factory=SafetyParams)

    def camera(self, role: str) -> CameraConfig:
        for c in self.cameras:
            if c.role == role:
                return c
        raise KeyError(role)


def load_config(path: str) -> HumanSafetyConfig:
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    return config_from_dict(raw)


def config_from_dict(raw: Dict) -> HumanSafetyConfig:
    cams = [CameraConfig(**c) for c in raw.get("cameras", [])]
    safety_kw = raw.get("safety", {}) or {}
    known = SafetyParams().__dict__.keys()
    unknown = set(safety_kw) - set(known)
    if unknown:
        raise ValueError(f"unknown safety params: {sorted(unknown)}")
    safety = SafetyParams(**safety_kw)
    top = {k: v for k, v in raw.items() if k not in ("cameras", "safety")}
    return HumanSafetyConfig(cameras=cams, safety=safety, **top)
