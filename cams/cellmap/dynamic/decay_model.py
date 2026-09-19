# SPDX-License-Identifier: Apache-2.0
"""A pure, testable model of nvblox occupancy decay for the dynamic mapper.

nvblox's dynamic mapper keeps a probabilistic occupancy grid and, at
``decay_dynamic_occupancy_rate_hz``, decays every voxel's occupancy probability
toward its region's "decay" value (``occupied_region_decay_probability`` /
``free_region_decay_probability`` in the specialization YAML). A voxel is an
obstacle in the ESDF while its occupancy probability stays above the integrator
threshold. This module reproduces that behaviour in log-odds so a proof can
assert the two properties the captain asked for — "stale obstacles do not linger,
fresh ones are not missed" — and so the YAML can be tuned against numbers instead
of guesses.

This is a MODEL used for tuning and offline assertions; the GPU proof
(``proof/run_dynamic_esdf_proof.sh``) measures the REAL ESDF and is the ground
truth. Where the two disagree the real measurement wins; the model exists to give
the tests a deterministic, ROS-free target and to document the intent of each
YAML value.

Convention: probabilities are P(occupied); log-odds l = ln(p/(1-p)); a voxel is
"present" (an obstacle nvblox will route the planner around) when P >= p_present.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def _logodds(p: float) -> float:
    p = min(max(p, 1e-6), 1.0 - 1e-6)
    return math.log(p / (1.0 - p))


def _prob(l: float) -> float:
    return 1.0 / (1.0 + math.exp(-l))


@dataclass(frozen=True)
class DecayParams:
    """Mirror of the dynamic_mapper YAML knobs that govern appear/clear timing."""

    integrate_rate_hz: float = 10.0        # depth integration rate (observations/s)
    decay_rate_hz: float = 10.0            # decay_dynamic_occupancy_rate_hz
    occupied_obs_prob: float = 0.9         # occupied_region_occupancy_probability
    free_obs_prob: float = 0.2             # free_region_occupancy_probability (miss)
    occupied_decay_prob: float = 0.30      # occupied_region_decay_probability
    p_present: float = 0.5                 # ESDF occupancy threshold for "obstacle"
    p_prior: float = 0.35                  # unobserved_region_occupancy_probability

    def validate(self) -> "DecayParams":
        for name in ("integrate_rate_hz", "decay_rate_hz"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be > 0")
        for name in ("occupied_obs_prob", "free_obs_prob",
                     "occupied_decay_prob", "p_present", "p_prior"):
            v = getattr(self, name)
            if not 0.0 < v < 1.0:
                raise ValueError(f"{name} must be in (0,1), got {v}")
        return self


def frames_to_appear(p: DecayParams) -> int:
    """How many consecutive occupied observations lift a voxel from the prior to
    'present'. Small => a fresh obstacle is not missed."""
    p.validate()
    l = _logodds(p.p_prior)
    step = _logodds(p.occupied_obs_prob)          # additive log-odds per hit
    target = _logodds(p.p_present)
    n = 0
    while _prob(l) < p.p_present and n < 10000:
        l += step
        n += 1
        if l >= target and _prob(l) >= p.p_present:
            break
    return n


def seconds_to_appear(p: DecayParams) -> float:
    """Wall time for a fresh obstacle to enter the ESDF, at the integrate rate."""
    return frames_to_appear(p) / p.integrate_rate_hz


def time_to_clear_s(p: DecayParams, observed_prob: float = None) -> float:
    """Seconds after the LAST observation until a departed obstacle drops below
    'present' and stops being routed around. Large-but-bounded => stale obstacles
    do not linger forever but also do not flicker out prematurely.

    Decay is modelled as nvblox does it: each decay tick moves the occupancy
    probability a fixed fraction toward ``occupied_decay_prob``."""
    p.validate()
    start = _logodds(observed_prob if observed_prob is not None else p.occupied_obs_prob)
    floor = _logodds(p.occupied_decay_prob)
    target = _logodds(p.p_present)
    if start <= target:
        return 0.0
    # geometric approach toward the floor: l_{k+1} = floor + alpha*(l_k - floor),
    # alpha derived so one tick moves prob from occupied_obs_prob toward the decay
    # value by the configured fraction (occupied_decay_prob is the per-tick pull).
    alpha = 1.0 - p.occupied_decay_prob
    l = start
    ticks = 0
    while _prob(l) >= p.p_present and ticks < 100000:
        l = floor + alpha * (l - floor)
        ticks += 1
        if l <= target:
            break
    return ticks / p.decay_rate_hz


def half_life_s(p: DecayParams) -> float:
    """Occupancy-odds half-life during decay (seconds), a tuning summary."""
    p.validate()
    alpha = 1.0 - p.occupied_decay_prob
    if alpha <= 0.0:
        return 0.0
    ticks = math.log(0.5) / math.log(alpha)
    return ticks / p.decay_rate_hz
