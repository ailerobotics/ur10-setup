#!/usr/bin/env python3
"""
ROS interface layer for the driver-backed gate — the ONLY place that touches the
Universal Robots driver interfaces (task brief item 1). Kept apart from the pure
decision cores so those stay ROS-free and unit-tested.

Interfaces verified against the installed driver (ros-jazzy-ur 3.8.0) and the
repo's own usage (RUNBOOK §5/§6b, README, execute_plan.py):

  * primary lever  — service `/io_and_status_controller/set_speed_slider`,
    type `ur_msgs/srv/SetSpeedSliderFraction` (field `speed_slider_fraction`,
    range [0,1]);
  * effect readout — topic `/speed_scaling_state_broadcaster/speed_scaling`,
    type `std_msgs/Float64` (the ACTUAL applied scaling; must be >0 for motion);
  * backstop       — cancel the active FollowJointTrajectory goal via the action's
    generic cancel service `<ns>/_action/cancel_goal`
    (`action_msgs/srv/CancelGoal`) with a ZERO goal id + ZERO stamp = "cancel all
    goals". This lets the gate/watcher cancel a trajectory they did NOT start
    (the mover owns it), which is exactly the supervisor relationship here;
  * rest detector  — `/joint_states` position deltas (robust: a populated
    velocity field is not guaranteed on every controller/sim).

This module is import-safe only where rclpy exists; it is never imported by the
offline unit tests.
"""
from __future__ import annotations

import subprocess
import time
from typing import List, Optional, Tuple

from action_msgs.srv import CancelGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from ur_msgs.srv import SetSpeedSliderFraction

SLIDER_SERVICE = "/io_and_status_controller/set_speed_slider"
SCALING_TOPIC = "/speed_scaling_state_broadcaster/speed_scaling"
PROGRAM_TOPIC = "/io_and_status_controller/robot_program_running"
FJT_ACTION_DEFAULT = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


class SpeedSliderClient:
    """Writes the speed slider. This is the SOLE-writer instrument."""

    def __init__(self, node, service=SLIDER_SERVICE):
        self._node = node
        self._cli = node.create_client(SetSpeedSliderFraction, service)
        self._service = service

    def available(self, timeout=2.0) -> bool:
        return self._cli.wait_for_service(timeout_sec=timeout)

    def set(self, fraction: float) -> None:
        """Fire-and-forget slider write (clamped to [0,1]). Non-blocking: the
        effect is confirmed separately on the scaling topic within a deadline."""
        f = max(0.0, min(1.0, float(fraction)))
        req = SetSpeedSliderFraction.Request()
        req.speed_slider_fraction = f
        # async; a blocking wait here would stall the decide loop.
        self._cli.call_async(req)


class ScalingMonitor:
    """Latest applied speed scaling + when it arrived (monotonic)."""

    def __init__(self, node, topic=SCALING_TOPIC):
        self.value: Optional[float] = None
        self.last_arrival: Optional[float] = None
        self._node = node
        node.create_subscription(Float64, topic, self._cb, 10)

    def _cb(self, msg: Float64):
        self.value = self._normalize(float(msg.data))
        self.last_arrival = time.monotonic()

    @staticmethod
    def _normalize(v: float) -> float:
        """Normalise to a 0..1 fraction. The applied speed scaling is a fraction
        on the real driver / URSim, but the mock-hardware interface reports it on
        a 0..100 PERCENT scale (observed: 100.0 at full speed). Values > 1.5 are
        therefore treated as percent and divided by 100 so the gate's thresholds
        (all in fractions) work on both."""
        return v / 100.0 if v > 1.5 else v


class ProgramMonitor:
    """robot_program_running (Bool, ON-CHANGE-ONLY — never latched; a fresh
    subscriber may never see it while healthy, so `running` starts unknown)."""

    def __init__(self, node, topic=PROGRAM_TOPIC):
        self.running: Optional[bool] = None
        node.create_subscription(Bool, topic, self._cb, 5)

    def _cb(self, msg: Bool):
        self.running = bool(msg.data)


class JointMotionMonitor:
    """Estimates whether the arm is at rest from /joint_states position deltas."""

    def __init__(self, node, rest_speed_rad_s=0.02):
        self._node = node
        self._rest = rest_speed_rad_s
        self._last: Optional[Tuple[float, dict]] = None   # (t, name->pos)
        self._speed: Optional[float] = None
        node.create_subscription(JointState, "/joint_states", self._cb, 20)

    def _cb(self, msg: JointState):
        t = time.monotonic()
        pos = dict(zip(msg.name, msg.position))
        if self._last is not None:
            t0, p0 = self._last
            dt = t - t0
            if dt > 1e-3:
                common = set(pos) & set(p0)
                if common:
                    self._speed = max(abs(pos[j] - p0[j]) / dt for j in common)
        self._last = (t, pos)

    def moving(self) -> Optional[bool]:
        """True = confirmed moving, False = confirmed at rest, None = unknown."""
        if self._speed is None:
            return None
        return self._speed > self._rest


class TrajectoryCanceller:
    """Cancels ALL active FollowJointTrajectory goals via the action's generic
    cancel service — the backstop. Works on goals this process did not start."""

    def __init__(self, node, action_ns=FJT_ACTION_DEFAULT):
        self._node = node
        self._cli = node.create_client(CancelGoal, action_ns + "/_action/cancel_goal")
        self.action_ns = action_ns

    def available(self, timeout=2.0) -> bool:
        return self._cli.wait_for_service(timeout_sec=timeout)

    def cancel_all(self):
        """Zero goal id + zero stamp = cancel every active goal (hold at position)."""
        req = CancelGoal.Request()   # default GoalInfo: zero uuid, zero stamp
        return self._cli.call_async(req)


def enumerate_host_subnets() -> Tuple[List[str], List[str]]:
    """Return (physical_cidrs, docker_cidrs) for the arming interlock.

    Parses `ip -o -4 addr`. Interfaces named like Docker/virtual bridges
    (docker*, br-*, veth*, virbr*, cni*) contribute DOCKER subnets; every other
    non-loopback interface contributes a PHYSICAL subnet. Loopback is excluded
    (the interlock treats loopback specially). Fails safe: on any error it returns
    ([], []), and the interlock then refuses every non-loopback address because it
    cannot be proven Docker-internal.
    """
    docker_like = ("docker", "br-", "veth", "virbr", "cni", "flannel", "cali")
    physical: List[str] = []
    docker: List[str] = []
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                             capture_output=True, text=True, timeout=5)
        for line in out.stdout.splitlines():
            parts = line.split()
            # format: "<idx>: <ifname> inet <cidr> ..."
            if len(parts) < 4 or parts[2] != "inet":
                continue
            ifname, cidr = parts[1], parts[3]
            if ifname == "lo" or cidr.startswith("127."):
                continue
            if any(ifname.startswith(p) for p in docker_like):
                docker.append(cidr)
            else:
                physical.append(cidr)
    except Exception:                                    # noqa: BLE001
        return [], []
    return physical, docker
