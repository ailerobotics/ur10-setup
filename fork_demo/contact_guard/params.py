"""Contact-guard parameters — one documented home, a pure loader.

Mirrors the discipline of ``humangate/gate_config.py`` and
``cams/cellmap/tools/collision_force.py``: a frozen dataclass with conservative
defaults, a ``validate()`` that refuses incoherent combinations, and loaders
that keep only known keys (forward-compatible) and fall back to defaults so a
node comes up fail-safe rather than crashing.

Project convention: thresholds are **tune-up-never-down**. ``validate()``
enforces that the band widths, persistence and agreement requirements loaded
from yaml are never *looser* (more permissive) than the baked-in conservative
defaults — a yaml may only tighten detection, never relax it below the floor.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields, replace
from typing import Optional

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "params.yaml")

# The fork TCP offset from tool0 (metres), used to express the wrench at the
# fork tip. Matches fork_demo/real_fork_pick.py and teach_dwells.json.
FORK_OFFSET = (0.0, -0.038, 0.427)


@dataclass(frozen=True)
class ContactGuardParams:
    # --- envelope band construction (used offline by the builder) ---
    n_progress_bins: int = 20          # resample each leg to this many bins
    band_k_sigma: float = 6.0          # band half-width in robust sigmas (MAD)
    # absolute tolerance floors per signal (band never narrower than this)
    floor_force_n: float = 20.0        # |F| and force components (N)
    floor_torque_nm: float = 8.0       # torque components / magnitude (Nm)
    floor_eff_nm: float = 12.0         # per-joint effort (Nm, CB3 current-estimate)
    floor_tcp_vel: float = 0.05        # TCP speed (m/s)
    floor_track_rad: float = 0.03      # tracking error (rad)

    # --- runtime detection ---
    persist_samples: int = 4           # consecutive out-of-band samples to confirm
    persist_s: float = 0.12            # OR this long out-of-band (whichever first)
    agree_signals: int = 2             # multi-signal agreement: >= this many out
    big_margin_mult: float = 2.5       # one signal this far past the band == agreement
    progress_stall_s: float = 0.6      # window over which to judge a stall
    stall_min_cmd_progress: float = 0.003  # commanded must advance >= this in the
    #                                    window for a stall to be meaningful (noise floor)
    stall_ratio: float = 0.25          # STALL: actual advanced < this * commanded
    progress_stall_frac: float = 0.02  # (load-step) progress must advance >= this
    stale_s: float = 0.35              # no fresh sample this long -> fail-toward-stop

    # execute_plan.py keeps its own absolute --max-track-rad backstop (0.15 rad);
    # the contact guard does NOT re-use it as a standalone trip, because the real
    # session's 8.64 deg / 12.6 deg loaded/coarse-waypoint lags (both NON-contact)
    # would false-trip an absolute ceiling. The guard trips on band-relative
    # tracking + an ACTUAL-PROGRESS STALL instead. This value is kept only to
    # record / sanity-check that execute_plan's separate backstop exists.
    track_hard_rad: float = 0.15       # == execute_plan.py --max-track-rad (separate layer)
    track_rise_rad: float = 0.035      # tracking "elevated" floor (rad, ~2 deg)
    track_rate_rad_s: float = 0.05     # tracking "diverging" rate (rad/s): rising,
    #                                    not a benign plateaued lag (the 4.6 deg one)
    load_step_persist_s: float = 1.2   # a carry load STEP must persist this long
    #                                    across advancing progress (vs a pose-locked
    #                                    force-estimate bump that clears with the pose)

    # --- stop-and-probe (contact segments) ---
    # The fork is a 380 mm lever: a tip force F shows at the wrist as a torque
    # F x fork_lever_m. A fresh baseline captured at the halted contact pose
    # cancels the pose bias, so a baseline-relative torque/force delta resolves
    # ~3-5 N of tip force (vs ~400 N in motion). Source: the sibling scout
    # data/ur10-pick-failure-modes-s1/report.md §0,§2.3.
    fork_lever_m: float = 0.38         # fork tip lever arm (torque -> tip force)
    probe_dwell_s: float = 0.40        # halt-and-baseline dwell per segment (short)
    probe_min_samples: int = 20        # min samples to accept a fresh baseline
    probe_force_n: float = 5.0         # baseline-relative |F| delta trip (N)
    probe_torque_nm: float = 1.5       # baseline-relative torque delta trip (Nm)
    #                                    ~1.5 Nm / 0.38 m ~= 3.9 N of tip force

    # --- reaction policy ---
    backoff_m: float = 0.015           # in-axis back-off distance (15 mm)
    backoff_speed_frac: float = 0.02   # back-off speed scaling (2 %)

    # --- incremental contact legs ---
    segment_m: float = 0.015           # contact-leg segment length (15 mm TCP)
    segment_min_points: int = 12       # dense points per segment (132-pt lesson)

    # --- pre-contact gate ---
    speed_cap_frac: float = 0.10       # speed scaling must be <= this before contact
    min_free_gb: float = 20.0          # free disk floor
    fork_tcp_z_min: float = 0.30       # a fork TCP has z above this (read-back check)
    fork_tcp_absy_min: float = 0.01    # ... and |y| above this
    payload_unloaded_lo: float = 0.10  # acceptable payload band, unloaded (kg)
    payload_unloaded_hi: float = 1.20
    payload_loaded_lo: float = 0.10    # loaded band (captain chose 0.3 kg; sim 4.42)
    payload_loaded_hi: float = 5.50
    pendant_force_recommended_n: float = 100.0  # scout recommends the pendant
    #   Safety Force limit be set to the CB3 floor (100 N) with the correct
    #   payload; the operator must TYPE the value configured on the pendant
    #   (it cannot be read from the PC). data/ur10-pick-failure-modes-s1 §0,§4.5.

    # --- post-action verification ---
    lift_min_load_n: float = 15.0      # min |F| step that confirms tray on forks
    lift_min_dz_m: float = 0.015       # min TCP rise over a lift to confirm
    release_max_resid_n: float = 20.0  # load must drop below this after LOWER

    def validate(self) -> None:
        """Refuse incoherent / tune-*down* configs. Raises ValueError."""
        d = ContactGuardParams()  # the conservative floor
        if self.n_progress_bins < 4:
            raise ValueError("n_progress_bins must be >= 4")
        if self.segment_min_points < 2:
            raise ValueError("segment_min_points must be >= 2")
        # tune-up-never-down: bands may only get TIGHTER than the floor, never
        # wider; persistence/agreement may only get STRICTER (fewer misses).
        if self.band_k_sigma > d.band_k_sigma:
            raise ValueError(
                f"band_k_sigma {self.band_k_sigma} > floor {d.band_k_sigma}: "
                "widening the band loosens detection (tune-up-never-down)")
        for name in ("floor_force_n", "floor_torque_nm", "floor_eff_nm",
                     "floor_tcp_vel", "floor_track_rad"):
            if getattr(self, name) > getattr(d, name):
                raise ValueError(
                    f"{name} {getattr(self, name)} > floor {getattr(d, name)}: "
                    "a larger tolerance floor loosens detection")
        if self.persist_samples > d.persist_samples or self.persist_s > d.persist_s:
            raise ValueError("persistence may only be tightened below the floor")
        if self.agree_signals > d.agree_signals:
            raise ValueError("agree_signals above floor loosens detection")
        if self.stall_min_cmd_progress > d.stall_min_cmd_progress:
            raise ValueError("stall_min_cmd_progress above floor loosens detection")
        if self.stall_ratio > d.stall_ratio:
            raise ValueError("stall_ratio above floor loosens stall detection")
        if self.stale_s > d.stale_s:
            raise ValueError("stale_s above floor delays fail-toward-stop")
        if self.track_hard_rad > d.track_hard_rad:
            raise ValueError("track_hard_rad must not exceed the 0.15 rad ceiling")
        if self.track_rise_rad > d.track_rise_rad:
            raise ValueError("track_rise_rad above floor loosens detection")
        for name in ("probe_force_n", "probe_torque_nm"):
            if getattr(self, name) > getattr(d, name):
                raise ValueError(
                    f"{name} above floor loosens stop-and-probe detection")
        if self.fork_lever_m <= 0 or self.probe_dwell_s <= 0:
            raise ValueError("fork_lever_m and probe_dwell_s must be positive")
        if self.backoff_m <= 0 or self.backoff_speed_frac <= 0:
            raise ValueError("back-off distance/speed must be positive")
        if self.backoff_speed_frac > self.speed_cap_frac:
            raise ValueError("back-off speed must not exceed the contact speed cap")
        if self.segment_m <= 0:
            raise ValueError("segment_m must be positive")


def _from_dict(raw: dict) -> ContactGuardParams:
    known = {f.name for f in fields(ContactGuardParams)}
    kwargs = {k: v for k, v in (raw or {}).items() if k in known}
    return ContactGuardParams(**kwargs)


def load(path: str = CONFIG_PATH) -> ContactGuardParams:
    """Load params from yaml and validate. Raises on an invalid/tuned-down file."""
    with open(path) as fh:
        raw = yaml.safe_load(fh) or {}
    p = _from_dict(raw)
    p.validate()
    return p


def load_or_default(path: str = CONFIG_PATH) -> ContactGuardParams:
    """Load params, falling back to conservative defaults on any error.

    A node must come up fail-safe (strict defaults) rather than crash on a bad
    file. Returns the default params if the file is missing or invalid.
    """
    try:
        return load(path)
    except Exception:
        p = ContactGuardParams()
        return p


def with_overrides(p: ContactGuardParams, **overrides) -> ContactGuardParams:
    """Return a validated copy with the given field overrides (for CLI flags)."""
    out = replace(p, **overrides)
    out.validate()
    return out
