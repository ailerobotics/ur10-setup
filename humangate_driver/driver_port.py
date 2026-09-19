#!/usr/bin/env python3
"""
Driver-backed implementation of the gate's command port (task brief item 1).

The gate core hands its intent to a `CommandPort` (defined in the plan-only
`humangate` package). Phase 1 shipped only a RecorderPort. This is the Phase 3
port that actually drives a SIMULATED arm:

  * primary lever  — `set_speed_fraction` writes the speed slider via
    `set_speed_slider` and records the intent with the sole-writer monitor;
  * backstop       — `cancel_and_hold` cancels the active FollowJointTrajectory
    goal so the arm decelerates to a hold;
  * confirmation   — the port tracks whether the commanded fraction is reflected
    on the speed-scaling topic within a deadline (advisory log); the ENFORCED
    fall-through to the backstop is driven by the gate core via the driver-stopped
    feedback below, and independently by the backstop watcher process;
  * fail-to-zero   — on any self-fault, on losing the camera-state heartbeat, and
    on shutdown, the node leaves the slider at ZERO rather than restoring it; this
    port exposes `zero_slider()` for that and never itself raises the slider.

Import path: this package sits beside `humangate`; the port reuses the abstract
bases from there so the gate core is unchanged. It is deliberately OUTSIDE the
`humangate` package so `humangate/test_no_driver_interface.py` still proves that
package imports no driver interface — the driver coupling lives only here.
"""
from __future__ import annotations

import os
import sys
from typing import Optional

# reuse the abstract bases from the plan-only package (siblings on disk).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "humangate"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from command_port import CommandPort  # noqa: E402
from driver_feedback import DriverFeedback  # noqa: E402
from stop_strategy import (  # noqa: E402
    STRAT_CANCEL, PendantPauseParams, make_stop_strategy)


class _SliderBookkeeping:
    """Adapter handed to the stop strategy as its ``slider``: every slider write the
    strategy makes still records the command with the sole-writer monitor and arms
    the confirmation bookkeeping, exactly as the old direct slider path did. Uses the
    port's current tick time (set at the top of ``apply``)."""

    def __init__(self, port):
        self._port = port

    def set(self, fraction: float) -> None:
        self._port._commanded_slider(float(fraction))


class DriverCommandPort(CommandPort):
    """Speed-slider primary + trajectory-cancel backstop, against the driver, with a
    selectable STOP strategy (``cancel`` default, or ``pendant_pause``). The strategy
    only changes what a full STOP does and whether a RESUME continues the same
    trajectory; the slider stays the SLOW lever and the goal-cancel backstop stays the
    independent net in both (see stop_strategy.py)."""

    def __init__(self, slider, scaling_monitor, canceller, sole_writer,
                 logger=None, confirm_deadline_s: float = 0.5,
                 stop_scaling_threshold: float = 0.05,
                 stop_strategy: str = STRAT_CANCEL, dashboard=None,
                 pendant_params: Optional[PendantPauseParams] = None):
        self._slider = slider
        self._scaling = scaling_monitor
        self._canceller = canceller
        self._sole_writer = sole_writer
        self._log = logger
        self._confirm_deadline = confirm_deadline_s
        self._stop_threshold = stop_scaling_threshold
        # pending confirmation bookkeeping
        self._commanded_fraction: Optional[float] = None
        self._commanded_at: Optional[float] = None
        self._confirmed: bool = True
        self._unconfirmed_warned: bool = False
        # the selectable STOP strategy, driven per-tick from apply()
        self._now: float = 0.0
        self.strategy = make_stop_strategy(
            stop_strategy, _SliderBookkeeping(self), canceller, dashboard, logger,
            pendant_params)

    def _commanded_slider(self, fraction: float) -> None:
        """One slider write WITH sole-writer + confirmation bookkeeping (the strategy
        routes all its slider writes through here via _SliderBookkeeping)."""
        self._slider.set(fraction)
        self._sole_writer.note_command(fraction, self._now)
        self._commanded_fraction = float(fraction)
        self._commanded_at = self._now
        self._confirmed = False
        self._unconfirmed_warned = False

    # ---- per-tick strategy dispatch (the node's single entry point) ------ #
    def apply(self, decision, now: float, program_running=None) -> None:
        """Effect the gate's decision this tick through the selected strategy. Reads
        ``decision.action`` / ``decision.speed_fraction``; ``program_running`` (from
        the robot_program_running topic) lets the pendant-pause strategy detect a
        foreign play and re-pause."""
        self._now = now
        self.strategy.apply(decision.action, decision.speed_fraction, decision.reason,
                            now, program_running=program_running)

    # ---- primary lever (compat API; still used directly by tests/tools) -- #
    def set_speed_fraction(self, fraction: float, reason: str, t: float) -> None:
        self._now = t
        self._commanded_slider(fraction)
        if self._log:
            self._log.info(f"slider -> {fraction:.2f} ({reason})")

    # ---- backstop -------------------------------------------------------- #
    def cancel_and_hold(self, reason: str, t: float) -> None:
        # Belt and suspenders: zero the slider too, then cancel the goal so the
        # arm holds. A cancel is terminal — a real resume needs a fresh plan.
        self._now = t
        self._slider.set(0.0)
        self._sole_writer.note_command(0.0, t)
        self._canceller.cancel_all()
        if self._log:
            self._log.warn(f"BACKSTOP: cancel trajectory + slider->0 ({reason})")

    # ---- fail-to-zero (never raises the slider) -------------------------- #
    def zero_slider(self, reason: str, t: float) -> None:
        self._slider.set(0.0)
        self._sole_writer.note_command(0.0, t)
        self._commanded_fraction = 0.0
        self._commanded_at = t
        if self._log:
            self._log.warn(f"slider -> 0 ({reason})")

    # ---- confirmation (advisory) ---------------------------------------- #
    def check_confirmation(self, now: float) -> Optional[bool]:
        """Has the last commanded fraction shown up on the scaling topic within
        the deadline? True/False/None(no deadline elapsed yet). Advisory only —
        the ENFORCED response is the gate/watcher backstop."""
        if self._commanded_fraction is None or self._commanded_at is None:
            return None
        if self._confirmed:
            return True
        observed = self._scaling.value
        if observed is not None and abs(observed - self._commanded_fraction) <= 0.05:
            self._confirmed = True
            return True
        if (now - self._commanded_at) < self._confirm_deadline:
            return None
        if self._log and not self._unconfirmed_warned:
            self._unconfirmed_warned = True   # warn once per command, not per tick
            obs = "unknown" if observed is None else f"{observed:.3f}"
            self._log.warn(
                f"slider effect NOT confirmed: commanded {self._commanded_fraction:.2f}, "
                f"scaling {obs} after {self._confirm_deadline:.2f}s")
        return False


class DriverFeedbackFromTopics(DriverFeedback):
    """Real 'is the arm stopped?' feedback for the gate core's primary->backstop
    fall-through. Confirmed at rest = joints not moving (position deltas) AND, if
    the scaling topic is live, scaling at/near zero. Fails toward 'not stopped'
    (None/False) so an unconfirmed stop still arms the fall-through."""

    def __init__(self, joint_monitor, scaling_monitor,
                 stop_scaling_threshold: float = 0.05):
        self._joints = joint_monitor
        self._scaling = scaling_monitor
        self._thresh = stop_scaling_threshold

    def arm_stopped(self) -> Optional[bool]:
        # A STOP is only CONFIRMED effected when the applied scaling has fallen to
        # ~0 (the slider took effect). Joint motion alone is not enough: a very
        # slow trajectory reads as "at rest" to a delta estimator, and in mock the
        # scaling is inert so the STOP is never actually applied. So we require the
        # scaling-topic confirmation the spec calls for; without it we do NOT
        # assert a stop, which correctly ARMS the primary->backstop fall-through.
        sc = self._scaling.value
        moving = self._joints.moving()
        if sc is not None and sc <= self._thresh:
            # scaling confirms the freeze; block only if joints clearly still move
            return False if moving is True else True
        if sc is None and moving is False:
            return None          # no scaling info + joints look still -> unknown
        return False             # scaling not near zero -> STOP not confirmed
