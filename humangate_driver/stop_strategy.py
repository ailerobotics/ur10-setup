#!/usr/bin/env python3
"""How the driver-backed gate EFFECTS a full STOP and a RESUME — a selectable,
PURE strategy behind the command port (pendant-pause task).

Two things never change between strategies and are NOT decided here:
  * the **speed slider is always the SLOW lever** (graded slow, same-trajectory
    resume above the driver's floor); and
  * the **independent goal-cancel backstop is always present** — it fires from the
    separate watcher process on a lost heartbeat, a gate self-fault, or a STOP the
    applied scaling did not effect within the deadline (`watcher_core.py`), and from
    the gate core's own primary->backstop fall-through. Neither strategy removes it.

What the strategy decides is only **what a full STOP does** and **whether a RESUME
continues the exact same trajectory**:

  * ``CancelHoldStrategy`` (default — how this cell runs today, HEADLESS): STOP drives
    the slider toward zero; because slider-zero floors ~10% on CB3 (SLIDER_ZERO_FINDING.md)
    the real full stop is delivered by the goal-cancel backstop, which is terminal, so
    a resume is a fresh plan (stop-then-replan). This reproduces the shipped behaviour.

  * ``PendantPauseStrategy`` (opt-in — only for a cell that runs a pendant-LOADED
    External Control program, non-headless): STOP issues a dashboard **pause**, which
    on CB3 freezes the arm on its planned path (applied scaling -> 0) while the running
    FollowJointTrajectory goal SURVIVES; a healthy CLEAR+dwell RESUME issues **play**,
    continuing that SAME trajectory with no replan (PENDANT_PAUSE_FINDING.md). It:
      - RESUMES (play) only on the gate's own RUN (i.e. after the full healthy
        CLEAR+dwell) — never on a mere SLOW retreat and never on invisibility;
      - REFUSES to resume once the pause has lasted longer than ``resume_max_pause_s``
        (a stale frozen plan): it cancels instead, so the resume becomes a fresh plan;
      - RE-PAUSES and surfaces a conflict if anything plays the program while the gate
        still wants STOP (a foreign play on the pendant / the re-arm watchdog).

This module is PURE: it is handed small port objects (a slider with ``set``, a
dashboard with ``pause``/``play``/``stop``, a canceller with ``cancel_all``) and all
time is passed in, so every rule is unit-tested offline with fakes. It NEVER reads a
clock of its own and imports no ROS.
"""
from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Optional

# Gate action vocabulary, mirrored here as plain strings so this module does not
# import the gate core (same pattern as watcher_core.py).
RUN = "RUN"
SLOW = "SLOW"
STOP = "STOP"
BACKSTOP = "BACKSTOP"

STRAT_CANCEL = "cancel"
STRAT_PENDANT_PAUSE = "pendant_pause"


class StopStrategy(abc.ABC):
    """Applied once per decide tick with the gate's current decision. Implementations
    throttle their own side effects to edges. Public latched signals the node reads:
      * ``conflict`` / ``conflict_reason`` — a foreign play was detected + re-paused;
      * ``resume_needs_replan`` — a resume was refused (goal cancelled); the caller
        must plan afresh rather than expect the frozen trajectory to continue."""

    name = "abstract"

    # while a backstop is latched the action stays BACKSTOP every tick; re-assert the
    # cancel at most this often instead of flooding the action's cancel service.
    cancel_reassert_s = 1.0

    def __init__(self, slider, canceller, dashboard=None, logger=None):
        self._slider = slider
        self._canceller = canceller
        self._dashboard = dashboard
        self._log = logger
        self._last_slider: Optional[float] = None
        self._last_cancel_t: Optional[float] = None
        self.conflict = False
        self.conflict_reason = ""
        self.resume_needs_replan = False

    def _set_slider(self, fraction: float) -> None:
        """Write the slider only on a change (fire-and-forget)."""
        if self._last_slider is None or abs(fraction - self._last_slider) > 1e-6:
            self._slider.set(fraction)
            self._last_slider = fraction

    def _cancel(self, now: float) -> None:
        """Cancel the goal, throttled so a latched BACKSTOP does not flood the
        cancel service every tick."""
        if (self._last_cancel_t is None
                or (now - self._last_cancel_t) >= self.cancel_reassert_s):
            self._canceller.cancel_all()
            self._last_cancel_t = now

    @abc.abstractmethod
    def apply(self, action: str, fraction: Optional[float], reason: str, now: float,
              *, program_running: Optional[bool] = None) -> None:
        ...

    def reset(self) -> None:
        self.conflict = False
        self.conflict_reason = ""
        self.resume_needs_replan = False


class CancelHoldStrategy(StopStrategy):
    """Default. STOP = slider toward zero (the backstop delivers the real full stop);
    BACKSTOP = cancel the goal + slider zero. Exactly the shipped behaviour."""

    name = STRAT_CANCEL

    def apply(self, action, fraction, reason, now, *, program_running=None):
        if action == BACKSTOP:
            self._set_slider(0.0)
            self._cancel(now)
            if self._log and self._last_cancel_t == now:
                self._log.warn(f"BACKSTOP: cancel trajectory + slider->0 ({reason})")
            return
        # RUN / SLOW / STOP: the slider carries all of them (STOP fraction == 0).
        if fraction is not None:
            self._set_slider(fraction)


@dataclass
class PendantPauseParams:
    # A resume is refused once the pause has lasted longer than this (the frozen
    # plan is treated as stale) — then the action becomes cancel + fresh plan. Set
    # at/below the pause duration measured safe in sim (PENDANT_PAUSE_FINDING.md);
    # provisional, tune DOWN only if a measurement demands it, never up past measured.
    resume_max_pause_s: float = 120.0

    def validate(self) -> "PendantPauseParams":
        if self.resume_max_pause_s <= 0.0:
            raise ValueError("resume_max_pause_s must be > 0")
        return self


class PendantPauseStrategy(StopStrategy):
    """Opt-in. STOP = dashboard pause (freeze on path); RUN = dashboard play (resume
    the SAME trajectory) within the resume window, else cancel + replan. Re-pauses on
    a foreign play. Requires a pendant-loaded External Control program (non-headless)."""

    name = STRAT_PENDANT_PAUSE

    def __init__(self, slider, canceller, dashboard, logger=None,
                 params: Optional[PendantPauseParams] = None):
        super().__init__(slider, canceller, dashboard, logger)
        if dashboard is None:
            raise ValueError("PendantPauseStrategy requires a dashboard client")
        self.p = (params or PendantPauseParams()).validate()
        self._paused = False
        self._pause_since: Optional[float] = None
        self._prev_prog_running: Optional[bool] = None

    @property
    def paused(self) -> bool:
        return self._paused

    def _reassert_pause_if_foreign_play(self, program_running, now, where):
        """While the gate wants STOP, a False->True program_running edge means
        someone played the program (a foreign play / the re-arm watchdog). Re-pause
        once per edge and surface the conflict."""
        foreign = (program_running is True and self._prev_prog_running is not True)
        if foreign:
            self._dashboard.pause()
            self.conflict = True
            self.conflict_reason = (f"foreign PLAY while gate wants STOP ({where}) — "
                                    f"re-paused the program")
            if self._log:
                self._log.error(f"STOP CONFLICT: {self.conflict_reason}")

    def apply(self, action, fraction, reason, now, *, program_running=None):
        if action == BACKSTOP:
            # Terminal cancel supersedes any pause, in every strategy.
            self._set_slider(0.0)
            self._cancel(now)
            self._paused = False
            self._pause_since = None
            if self._log and self._last_cancel_t == now:
                self._log.warn(f"BACKSTOP: cancel trajectory + slider->0 ({reason})")
            self._prev_prog_running = program_running
            return

        if action == RUN:
            if self._paused:
                held = now - (self._pause_since if self._pause_since is not None else now)
                if held <= self.p.resume_max_pause_s:
                    self._dashboard.play()             # resume the SAME trajectory
                    if self._log:
                        self._log.warn(f"RESUME: play (paused {held:.1f}s) ({reason})")
                else:
                    self._cancel(now)                   # frozen plan too old — replan
                    self.resume_needs_replan = True
                    if self._log:
                        self._log.warn(f"RESUME REFUSED: paused {held:.1f}s > "
                                       f"{self.p.resume_max_pause_s:.0f}s — cancel + replan")
                self._paused = False
                self._pause_since = None
            self.conflict = False
            self.conflict_reason = ""
            if fraction is not None:
                self._set_slider(fraction)             # restore operating speed
            self._prev_prog_running = program_running
            return

        if action == SLOW:
            if self._paused:
                # Person retreated to the SLOW ring but has NOT cleared+dwelled: stay
                # frozen (resume only on RUN). Guard against a foreign play.
                self._reassert_pause_if_foreign_play(program_running, now, "SLOW-hold")
            else:
                if fraction is not None:
                    self._set_slider(fraction)         # graded slow while running
            self._prev_prog_running = program_running
            return

        # action == STOP
        if not self._paused:
            self._dashboard.pause()
            self._paused = True
            self._pause_since = now
            if self._log:
                self._log.warn(f"STOP: pause program (freeze on path) ({reason})")
        else:
            self._reassert_pause_if_foreign_play(program_running, now, "STOP")
        self._set_slider(0.0)                          # belt & suspenders
        self._prev_prog_running = program_running


def make_stop_strategy(name: str, slider, canceller, dashboard=None, logger=None,
                       pendant_params: Optional[PendantPauseParams] = None) -> StopStrategy:
    """Factory used by the node from config. Fails toward the safe default: an
    unknown name, or pendant_pause without a dashboard, raises rather than silently
    degrading."""
    if name == STRAT_CANCEL:
        return CancelHoldStrategy(slider, canceller, dashboard, logger)
    if name == STRAT_PENDANT_PAUSE:
        return PendantPauseStrategy(slider, canceller, dashboard, logger, pendant_params)
    raise ValueError(f"unknown stop_strategy {name!r} (want {STRAT_CANCEL!r} or "
                     f"{STRAT_PENDANT_PAUSE!r})")
