#!/usr/bin/env python3
"""
The command interface, behind an abstract port with exactly one implementation
in this task: a RECORDER.

This is the plumbing that makes the whole gate provably plan-only (task brief
item 5). The decision core hands its intent to a `CommandPort`; in this phase the
only port is `RecorderPort`, which just remembers what it WOULD have done. There
is deliberately NO port here that talks to the robot: no `set_speed_slider`
client, no FollowJointTrajectory action client, no dashboard client, no publisher
on any controller topic. Wiring a real driver port is a later, separate phase
(report §10 Phase 3), and test_no_driver_interface.py asserts this package
imports none of those driver interfaces.

PURE: standard library only, no rclpy, no ROS message packages.
"""
from __future__ import annotations

import abc
import json
from dataclasses import asdict, dataclass
from typing import List, Optional


@dataclass(frozen=True)
class Command:
    """One thing the gate would command the robot to do (but does not, here)."""

    # "set_speed_fraction" (the primary lever) or "cancel_and_hold" (the backstop).
    kind: str
    # Slider fraction for set_speed_fraction (0.0 = soft STOP); None for cancel.
    fraction: Optional[float]
    # Human-readable reason, mirrored from the decision.
    reason: str
    # Monotonic-ish timestamp the command was issued (seconds); set by the caller.
    t: float


class CommandPort(abc.ABC):
    """Abstract sink for gate commands. The gate depends only on this."""

    @abc.abstractmethod
    def set_speed_fraction(self, fraction: float, reason: str, t: float) -> None:
        """Primary lever: what the speed slider WOULD be set to (report §2.1)."""

    @abc.abstractmethod
    def cancel_and_hold(self, reason: str, t: float) -> None:
        """Backstop: what would cancel the trajectory goal so the arm holds
        (report §2). Terminal — a real resume afterwards needs a fresh plan."""


class RecorderPort(CommandPort):
    """The ONLY port in this task. Records commands; touches nothing.

    Optionally appends each command as one JSON line to a log path, so a run
    produces a structured, replayable record of what the gate would have done.
    """

    def __init__(self, log_path: Optional[str] = None) -> None:
        self.commands: List[Command] = []
        self._log_path = log_path

    def _record(self, cmd: Command) -> None:
        self.commands.append(cmd)
        if self._log_path is not None:
            with open(self._log_path, "a") as f:
                f.write(json.dumps(asdict(cmd)) + "\n")

    def set_speed_fraction(self, fraction: float, reason: str, t: float) -> None:
        self._record(Command("set_speed_fraction", float(fraction), reason, t))

    def cancel_and_hold(self, reason: str, t: float) -> None:
        self._record(Command("cancel_and_hold", None, reason, t))

    @property
    def last(self) -> Optional[Command]:
        return self.commands[-1] if self.commands else None
