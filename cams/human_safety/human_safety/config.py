# SPDX-License-Identifier: Apache-2.0
"""Load and validate the human_safety YAML config (cameras + safety params)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml

from .safety import SafetyParams
from .scheduler import SchedulerParams


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
class FrameProcessingConfig:
    """How frames are turned into detections.

    ``mode``:
      * ``per_camera`` (DEFAULT) — one detector process per camera, each
        detecting every frame up to its rate cap, in parallel. This is the
        shipped, every-frame baseline.
      * ``shared`` — ONE detector instance serves all cameras and batches the
        frames that need detection in a single forward pass per scheduling tick
        (much less GPU memory; one pass instead of N contending processes).

    ``schedule_rate`` is the shared detector's tick rate (Hz). ``scheduler`` holds
    the frame-selection parameters; with ``scheduler.enabled`` False the shared
    detector still batches but detects every camera every tick (behaviourally the
    baseline, just batched). ``fp16`` enables half-precision inference where the
    device supports it (verify it does not change detections materially — see the
    benchmark); it defaults off so the baseline is bit-for-bit unchanged.
    """

    mode: str = "per_camera"          # per_camera | shared
    schedule_rate: float = 15.0       # Hz, shared-detector tick rate
    fp16: bool = False                # half-precision inference (opt-in)
    scheduler: SchedulerParams = field(default_factory=SchedulerParams)


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
    frame_processing: FrameProcessingConfig = field(
        default_factory=FrameProcessingConfig)

    def __post_init__(self):
        fp = self.frame_processing
        if fp.mode not in ("per_camera", "shared"):
            raise ValueError(
                f"frame_processing.mode must be 'per_camera' or 'shared' "
                f"(got {fp.mode!r})")
        # Safety invariant: when frame selection is on, a camera gets a full
        # detection at least every t_max. Freshness is driven by the last full
        # detection, so t_max must not exceed the safety freshness budget or a
        # quiet camera could be reported fresh while overdue for a look.
        if fp.scheduler.enabled and fp.scheduler.t_max > self.safety.frame_age_budget:
            raise ValueError(
                f"frame_processing.scheduler.t_max ({fp.scheduler.t_max}s) must be "
                f"<= safety.frame_age_budget ({self.safety.frame_age_budget}s) so "
                f"the efficient path can never report a camera fresh on skipped "
                f"frames; lower t_max or raise the budget")

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
    fp = _frame_processing_from_dict(raw.get("frame_processing", {}) or {})
    top = {k: v for k, v in raw.items()
           if k not in ("cameras", "safety", "frame_processing")}
    return HumanSafetyConfig(
        cameras=cams, safety=safety, frame_processing=fp, **top)


def _frame_processing_from_dict(raw: Dict) -> FrameProcessingConfig:
    sched_kw = raw.get("scheduler", {}) or {}
    known = SchedulerParams().__dict__.keys()
    unknown = set(sched_kw) - set(known)
    if unknown:
        raise ValueError(f"unknown scheduler params: {sorted(unknown)}")
    scheduler = SchedulerParams(**sched_kw)
    top = {k: v for k, v in raw.items() if k != "scheduler"}
    return FrameProcessingConfig(scheduler=scheduler, **top)
