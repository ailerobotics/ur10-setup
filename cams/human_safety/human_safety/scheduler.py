# SPDX-License-Identifier: Apache-2.0
"""Efficient frame-processing scheduler (pure, ROS-free, torch-free).

Today every camera runs its own detector process and detects **every** frame up
to its rate cap, in parallel. This module is the decision core of an *efficient*
path that, instead, chooses per scheduling tick which cameras actually need a
full (expensive) neural detection — so a shared detector can batch just those
frames in one forward pass, and quiet cameras drop to a conservative floor rate.

Design invariants — the efficient path must be **at least as safe** as the
every-frame baseline, so every rule here only ever moves a camera toward *more*
detection, never less:

* **Hard T_max bound.** Every camera gets a full detection at least every
  ``t_max`` seconds, regardless of how quiet it looks. Freshness in the safety
  state machine is driven by the last *full* detection, so if ``t_max`` is kept
  at or below the safety layer's ``frame_age_budget`` (enforced at config load),
  a camera can never be reported *fresh* on skipped frames — and if detection
  ever fails to keep up, the camera ages out to stale and the existing health
  rules degrade the state toward SLOW then STOP (fail toward STOP).
* **Motion trigger.** A per-camera change score above ``motion_threshold`` (a
  person entering view moves a large block of pixels) forces an immediate full
  detection at up to the busy rate — so a new entrant is caught the same tick the
  every-frame baseline would see them. An *unknown* change score (could not be
  computed) counts as motion: fail toward looking.
* **Near-robot priority.** When any confirmed track is inside or approaching the
  SLOW/STOP rings, every camera runs at the busy rate (full continuity where it
  matters most); a quiet cell with nobody near the robot drops to the floor rate.
* **No CLEAR from skipped frames.** Skipping only ever *carries forward* the last
  full detection (the fused tracker coasts between runs); it never fabricates an
  empty/clear observation. CLEAR still rests on a real full detection within
  ``t_max``.

When ``enabled`` is False the scheduler returns "detect every new frame" — i.e.
the exact every-frame baseline — so the baseline is this same code path with the
selection rules switched off. All rules are deterministic and unit-tested
offline (``tests/test_scheduler.py``).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SchedulerParams:
    """Frame-selection parameters. All times in seconds.

    Conservative defaults: ``t_max`` 0.4 s sits below the safety layer's default
    ``frame_age_budget`` (0.5 s) so the floor rate (1/t_max = 2.5 Hz per camera)
    keeps every camera fresh with margin for jitter; ``busy_min_period`` 1/15 s
    lets a busy camera run at the full fusion rate.
    """

    enabled: bool = False          # frame selection on/off; off == every-frame baseline
    t_max: float = 0.4             # hard max interval between FULL detections per camera
    busy_min_period: float = 1.0 / 15.0   # min interval between detections for a busy camera
    motion_threshold: float = 0.02  # normalised change score (0..1) above which a camera is "busy"
    near_margin: float = 0.5       # a confirmed track within (slow envelope + this) m makes ALL cameras busy
    downscale: int = 8             # block-mean factor for the change-score thumbnail
    max_batch: int = 8             # cap frames per batched forward pass

    def __post_init__(self):
        if self.t_max <= 0.0:
            raise ValueError(f"t_max must be > 0 (got {self.t_max})")
        if self.busy_min_period <= 0.0:
            raise ValueError(
                f"busy_min_period must be > 0 (got {self.busy_min_period})")
        if self.busy_min_period > self.t_max:
            raise ValueError(
                f"busy_min_period ({self.busy_min_period}) must be <= t_max "
                f"({self.t_max}); the busy rate can never be slower than the floor")
        if self.motion_threshold < 0.0:
            raise ValueError(
                f"motion_threshold must be >= 0 (got {self.motion_threshold})")
        if self.near_margin < 0.0:
            raise ValueError(f"near_margin must be >= 0 (got {self.near_margin})")
        if self.downscale < 1:
            raise ValueError(f"downscale must be >= 1 (got {self.downscale})")
        if self.max_batch < 1:
            raise ValueError(f"max_batch must be >= 1 (got {self.max_batch})")


@dataclass
class CameraTick:
    """One camera's state at a scheduling tick (what the scheduler needs to
    decide). Pure scalars so it is trivial to construct in tests and in the
    node alike."""

    role: str
    have_new_frame: bool                      # a frame arrived since the last processed one
    change_score: Optional[float] = None      # normalised change vs last full detection, or None
    last_full_detect_time: Optional[float] = None  # wall time of this camera's last FULL detection


@dataclass
class Decision:
    """The scheduler's per-camera verdict for one tick."""

    role: str
    detect: bool
    reason: str


class FrameScheduler:
    """Decide, per tick, which cameras get a full detection. Stateless across
    ticks except for its parameters: the caller owns ``last_full_detect_time``
    (so the node and the offline evaluator share exactly this logic)."""

    def __init__(self, params: Optional[SchedulerParams] = None):
        self.p = params or SchedulerParams()

    def decide(self, now: float, ticks: List[CameraTick],
               near_ring: bool = False) -> List[Decision]:
        p = self.p
        out: List[Decision] = []
        for tk in ticks:
            if not tk.have_new_frame:
                # No frame to run on; the camera's freshness ages naturally and
                # the safety health rules handle a camera that goes stale.
                out.append(Decision(tk.role, False, "no new frame"))
                continue
            if not p.enabled:
                out.append(Decision(tk.role, True, "every-frame (selection off)"))
                continue
            last = tk.last_full_detect_time
            if last is None:
                out.append(Decision(tk.role, True, "first detection"))
                continue
            elapsed = now - last
            if elapsed >= p.t_max:
                out.append(Decision(
                    tk.role, True, f"t_max bound ({elapsed:.3f}s >= {p.t_max:.3f}s)"))
                continue
            # "Busy" raises a camera to the full rate. An unknown (None) change
            # score counts as busy: fail toward looking.
            motion = tk.change_score is None or tk.change_score >= p.motion_threshold
            busy = near_ring or motion
            if busy and elapsed >= p.busy_min_period:
                if near_ring:
                    why = "near-robot track"
                elif tk.change_score is None:
                    why = "change score unavailable"
                else:
                    why = f"motion {tk.change_score:.3f} >= {p.motion_threshold:.3f}"
                out.append(Decision(tk.role, True, f"busy: {why}"))
                continue
            out.append(Decision(
                tk.role, False, f"quiet ({elapsed:.3f}s < t_max {p.t_max:.3f}s)"))
        return out
