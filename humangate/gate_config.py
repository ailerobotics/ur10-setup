#!/usr/bin/env python3
"""
Reader + validator for the robot-side human-safety gate's thresholds.

All thresholds live in ONE documented place, `humangate/gate_params.yaml`, and
flow through this loader — never as magic numbers scattered across the node and
the tests (the same discipline as cams/cellmap/tools/collision_force.py).

Every default here is the CONSERVATIVE, PROVISIONAL value from
human-safety-s1/report.md §3 and §7. "Provisional" means: it is deliberately
larger/safer than the real number will be, and must be RE-DERIVED from measured
end-to-end latency (Tr) and stopping time (Ts) in a later, owner-present phase
(report §8, §10 Phase 4). Tune these UP, never down, until measured.

This module is PURE: standard library + yaml only, no rclpy, so the offline
tests read and check the config exactly as the node does.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "gate_params.yaml")


@dataclass(frozen=True)
class GateConfig:
    """Gate thresholds. See gate_params.yaml for the rationale on each value."""

    # --- interface freshness / liveness (report §7.2, §7.3) ---
    # Oldest a state message may be, by its OWN evidence stamp, before the gate
    # treats it as stale and STOPs (§7.2). The interface analogue of the
    # latched-frame trap: publication cadence is irrelevant, evidence age is not.
    state_max_age_s: float = 0.200
    # No state message within this wall of silence -> perception dead -> STOP
    # (§7.3). Also the window within which heartbeat_seq must advance.
    heartbeat_timeout_s: float = 0.300
    # A stamp this far in the FUTURE relative to the gate clock is clock skew
    # (report §5 #6); the sample is treated as stale (STOP), never trusted.
    clock_skew_tolerance_s: float = 0.050

    # --- resume gating (report §3, §7.6) ---
    # Continuous healthy-CLEAR time the gate must OBSERVE ITSELF before leaving
    # STOP, even though the camera side also applies a dwell (defence in depth,
    # §7.6). Provisional; tune up.
    clear_dwell_s: float = 2.0

    # --- zone geometry, used only for resume-provenance + logging (report §3) ---
    # Hysteresis/zoning proper live on the camera side (§7.6); the gate keys on
    # the published zone enum. These radii are used ONLY to classify whether a
    # person that vanished was "close" (for the lost-close resume refusal) and
    # for the decision log.
    stop_radius_m: float = 2.0
    slow_clear_hysteresis_m: float = 0.3

    # --- primary lever fractions (report §2.1) ---
    # Speed-slider fractions the RECORDER logs (this task issues NO real slider
    # write). RUN = operating fraction; SLOW = a low fraction; STOP = zero.
    operating_speed_fraction: float = 1.0
    slow_speed_fraction: float = 0.1
    stop_speed_fraction: float = 0.0

    # --- primary -> backstop fall-through (report §2, §5 #15) ---
    # After commanding STOP, if driver feedback does not confirm the arm stopped
    # within this deadline, escalate to BACKSTOP (cancel the goal, hold).
    backstop_deadline_s: float = 0.5

    # --- gate cycle (report §3 latency table: gate decide 10-50 ms) ---
    decide_rate_hz: float = 20.0

    def validate(self) -> "GateConfig":
        """Raise ValueError on any incoherent / unsafe combination."""
        if not (0.0 < self.state_max_age_s <= 1.0):
            raise ValueError("state_max_age_s must be in (0, 1] s")
        if not (0.0 < self.heartbeat_timeout_s <= 2.0):
            raise ValueError("heartbeat_timeout_s must be in (0, 2] s")
        if self.heartbeat_timeout_s < self.state_max_age_s:
            raise ValueError(
                "heartbeat_timeout_s must be >= state_max_age_s "
                "(a message can be older than the stamp budget yet still arriving)"
            )
        if self.clock_skew_tolerance_s < 0.0:
            raise ValueError("clock_skew_tolerance_s must be >= 0")
        if self.clear_dwell_s <= 0.0:
            raise ValueError("clear_dwell_s must be > 0 (a resume needs a real dwell)")
        if self.stop_radius_m <= 0.0 or self.slow_clear_hysteresis_m < 0.0:
            raise ValueError("stop_radius_m must be > 0 and hysteresis >= 0")
        if not (0.0 <= self.stop_speed_fraction < self.slow_speed_fraction
                <= self.operating_speed_fraction <= 1.0):
            raise ValueError(
                "speed fractions must satisfy 0 <= stop < slow <= operating <= 1"
            )
        if self.backstop_deadline_s <= 0.0:
            raise ValueError("backstop_deadline_s must be > 0")
        if self.decide_rate_hz <= 0.0:
            raise ValueError("decide_rate_hz must be > 0")
        return self


def load(path: str = CONFIG_PATH) -> GateConfig:
    """Parse gate_params.yaml into a validated GateConfig.

    Unknown keys in the file are ignored (forward-compatible); missing keys fall
    back to the conservative dataclass defaults above.
    """
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: expected a mapping, got {type(raw).__name__}")
    fields = {f_ for f_ in GateConfig.__dataclass_fields__}
    kwargs = {k: v for k, v in raw.items() if k in fields}
    return GateConfig(**kwargs).validate()


def load_or_default(path: str = CONFIG_PATH) -> GateConfig:
    """Like load(), but fall back to the built-in conservative defaults if the
    file is missing or malformed — the gate must still come up STOP-safe, not
    crash, when its param file is absent."""
    try:
        return load(path)
    except (OSError, ValueError, yaml.YAMLError):
        return GateConfig().validate()


if __name__ == "__main__":
    cfg = load_or_default()
    print(f"config: {CONFIG_PATH}")
    for name in cfg.__dataclass_fields__:
        print(f"  {name}: {getattr(cfg, name)}")
