#!/usr/bin/env python3
"""
Driver-feedback port: "has the arm actually stopped?"

The primary lever is speed scaling; the gate falls through to the BACKSTOP
(cancel the goal, hold) if, after commanding STOP, the arm is NOT observed to
have stopped within a short deadline (report §2, §5 #15). "Observed to have
stopped" comes from the robot driver — which this task must not touch. So the
feedback is an INJECTABLE interface with a simulated implementation here; the
real implementation (reading /speed_scaling_state_broadcaster/speed_scaling and
joint motion) is wired in a later phase (report §10 Phase 3).

`arm_stopped()` returns:
  * True  — the arm is confirmed at rest,
  * False — the arm is confirmed still moving,
  * None  — unknown (no feedback yet). The gate treats None conservatively: it
            cannot CONFIRM a stop, so the fall-through deadline still applies.

PURE: standard library only, no rclpy.
"""
from __future__ import annotations

import abc
from typing import Callable, Optional


class DriverFeedback(abc.ABC):
    """Abstract source of 'is the arm stopped?' feedback."""

    @abc.abstractmethod
    def arm_stopped(self) -> Optional[bool]:
        ...


class SimulatedDriverFeedback(DriverFeedback):
    """Scriptable stand-in for the real driver feedback.

    Construct with a callable `stopped_fn` that maps the current time (seconds)
    to Optional[bool], so tests and the node can model any timeline:

      * always stopped:      SimulatedDriverFeedback(lambda t: True)
      * never stops (backstop trigger): SimulatedDriverFeedback(lambda t: False)
      * stops after a delay: use `stops_after(command_t, delay_s)`
      * unknown feedback:    SimulatedDriverFeedback(lambda t: None)
    """

    def __init__(self, stopped_fn: Callable[[float], Optional[bool]]) -> None:
        self._fn = stopped_fn
        self._now = 0.0

    def set_now(self, t: float) -> None:
        """Advance the simulated clock the feedback answers against."""
        self._now = t

    def arm_stopped(self) -> Optional[bool]:
        return self._fn(self._now)


def always_stopped() -> SimulatedDriverFeedback:
    return SimulatedDriverFeedback(lambda t: True)


def never_stops() -> SimulatedDriverFeedback:
    return SimulatedDriverFeedback(lambda t: False)


def unknown() -> SimulatedDriverFeedback:
    return SimulatedDriverFeedback(lambda t: None)
