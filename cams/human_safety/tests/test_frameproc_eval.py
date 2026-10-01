# SPDX-License-Identifier: Apache-2.0
"""The safety evidence for the efficient frame-processing path, as a committed,
deterministic regression: on the shared multi-camera sequence the efficient path
must NEVER be less safe than the every-frame baseline, while using strictly fewer
detector invocations and keeping every camera fresh. No ROS/torch/GPU."""
import pathlib

import pytest

from human_safety.config import (FrameProcessingConfig, HumanSafetyConfig,
                                  config_from_dict)
from human_safety.frameproc_eval import run_baseline_vs_efficient
from human_safety.replay import load_jsonl
from human_safety.safety import SafetyParams
from human_safety.scheduler import SchedulerParams

DATA = pathlib.Path(__file__).with_name("data") / "multicam_approach.jsonl"


def _cfg(t_max=0.4):
    return HumanSafetyConfig(
        safety=SafetyParams(),
        frame_processing=FrameProcessingConfig(
            mode="shared",
            scheduler=SchedulerParams(enabled=True, t_max=t_max)))


@pytest.fixture(scope="module")
def result():
    records = load_jsonl(str(DATA))
    return run_baseline_vs_efficient(records, _cfg())


# --- the headline safety assertion ----------------------------------------- #
def test_efficient_is_never_less_safe_than_baseline(result):
    # Zero ticks where the efficient zone is strictly below the baseline zone.
    assert result.less_safe_ticks == []


def test_zones_match_tick_for_tick_on_this_fixture(result):
    # On this committed sequence the efficient path reproduces the baseline zone
    # sequence exactly (no disagreement in EITHER direction). A future param
    # change that loosens this should update the fixture with justification.
    assert result.disagreement_ticks == []


# --- efficiency ------------------------------------------------------------- #
def test_efficient_runs_fewer_detections(result):
    assert result.efficient.detector_invocations < result.baseline.detector_invocations
    # The baseline runs a full detection on every offered frame.
    assert result.baseline.detector_invocations == result.baseline.frames_offered


# --- recall / continuity ---------------------------------------------------- #
def test_recall_and_continuity_not_degraded(result):
    assert result.recall(result.baseline) == 1.0
    assert result.recall(result.efficient) == 1.0
    assert result.track_continuity == 1.0


# --- staleness stays within the freshness budget --------------------------- #
def test_worst_case_staleness_within_budget(result):
    budget = SafetyParams().frame_age_budget
    for role, worst in result.efficient.worst_staleness.items():
        assert worst <= budget + 1e-9, f"{role} went stale ({worst}s > {budget}s)"


# --- config invariant: t_max must not exceed the freshness budget ---------- #
def test_config_rejects_t_max_above_budget():
    with pytest.raises(ValueError):
        HumanSafetyConfig(
            safety=SafetyParams(frame_age_budget=0.5),
            frame_processing=FrameProcessingConfig(
                scheduler=SchedulerParams(enabled=True, t_max=0.6)))


def test_config_allows_large_t_max_when_selection_disabled():
    # When frame selection is off the bound does not apply (every-frame path).
    HumanSafetyConfig(
        safety=SafetyParams(frame_age_budget=0.5),
        frame_processing=FrameProcessingConfig(
            scheduler=SchedulerParams(enabled=False, t_max=0.6)))


def test_config_rejects_bad_mode():
    with pytest.raises(ValueError):
        HumanSafetyConfig(
            frame_processing=FrameProcessingConfig(mode="nonsense"))


# --- yaml round-trip through config_from_dict ------------------------------ #
def test_frame_processing_parses_from_dict():
    cfg = config_from_dict({
        "frame_processing": {
            "mode": "shared", "schedule_rate": 12.0, "fp16": True,
            "scheduler": {"enabled": True, "t_max": 0.3, "motion_threshold": 0.03},
        }})
    assert cfg.frame_processing.mode == "shared"
    assert cfg.frame_processing.fp16 is True
    assert cfg.frame_processing.scheduler.enabled is True
    assert cfg.frame_processing.scheduler.t_max == 0.3
    assert cfg.frame_processing.scheduler.motion_threshold == 0.03


def test_frame_processing_rejects_unknown_scheduler_param():
    with pytest.raises(ValueError):
        config_from_dict({"frame_processing": {"scheduler": {"bogus": 1}}})


def test_default_config_is_per_camera_baseline():
    # The shipped default must be the every-frame baseline (selection off).
    cfg = config_from_dict({})
    assert cfg.frame_processing.mode == "per_camera"
    assert cfg.frame_processing.scheduler.enabled is False
    assert cfg.frame_processing.fp16 is False
