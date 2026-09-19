#!/usr/bin/env python3
"""Offline tests for the gate acceptance shim (no GPU, no ROS).

These encode the task's motion-authority hard rules directly:
  * nothing moves unless the gate says RUN;
  * a command arriving during STOP does not move the arm;
  * on RUN the arm heads for the freshest command (not a stored stale one);
  * out-of-window commands are refused;
  * per-tick step is capped.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(PKG)
sys.path.insert(0, PKG)

from robot.joint_windows import JOINT_ORDER, load_measured_windows  # noqa: E402
from ros_bridge.stream_gate import (  # noqa: E402
    BACKSTOP,
    FORWARD,
    HOLD,
    RUN,
    SLOW,
    STOP,
    GatedStreamController,
)

LIMITS_YAML = os.path.join(REPO, "cumotion", "measured_joint_limits.yaml")
REF = [0.223385, -1.626713, 0.406275, 4.458966, 0.279341, 0.706048]


@pytest.fixture
def ctrl():
    w = load_measured_windows(LIMITS_YAML)
    c = GatedStreamController(windows=w, max_step_per_tick=0.02)
    c.seed(REF)
    return c


def _cmd_towards(delta=0.5):
    # a legal command 0.5 rad away on shoulder_pan (well inside its window)
    c = list(REF)
    c[0] += delta
    return c


def test_run_forwards_bounded_step(ctrl):
    act = ctrl.step(RUN, 1.0, _cmd_towards(0.5), REF)
    assert act.kind == FORWARD
    # step is capped at max_step_per_tick * frac = 0.02
    assert abs(act.target_position[0] - REF[0]) <= 0.02 + 1e-9
    assert act.target_position[0] > REF[0]  # moved toward the command


def test_stop_holds_and_drops_command(ctrl):
    # even with a valid command, STOP must not move the arm
    act = ctrl.step(STOP, 0.0, _cmd_towards(0.5), REF)
    assert act.kind == HOLD
    assert act.target_position == REF
    assert "dropped" in act.reason.lower()


def test_slow_is_treated_as_hold(ctrl):
    act = ctrl.step(SLOW, 0.3, _cmd_towards(0.5), REF)
    assert act.kind == HOLD


def test_backstop_engaged_flag(ctrl):
    act = ctrl.step(BACKSTOP, None, _cmd_towards(0.5), REF)
    assert act.kind == HOLD
    assert act.backstop_engaged is True


def test_command_during_stop_then_resume_uses_freshest(ctrl):
    # advance one RUN step toward cmd A
    ctrl.step(RUN, 1.0, _cmd_towards(0.5), REF)
    # STOP arrives with a (stale) command far away; arm holds
    held = ctrl.step(STOP, 0.0, _cmd_towards(2.0), REF)
    assert held.kind == HOLD
    hold_pos = held.target_position
    # RUN resumes toward a NEW freshest command in the OTHER direction
    fresh = list(REF)
    fresh[0] -= 0.5
    act = ctrl.step(RUN, 1.0, fresh, hold_pos)
    assert act.kind == FORWARD
    # it must step toward the fresh (negative) command, not the stale (positive)
    assert act.target_position[0] < hold_pos[0]


def test_out_of_window_command_refused(ctrl):
    bad = list(REF)
    bad[2] = 5.0  # elbow way outside [-1.09, 1.91]
    act = ctrl.step(RUN, 1.0, bad, REF)
    assert act.kind == HOLD
    assert act.window_ok is False
    assert "window" in act.reason.lower()


def test_run_without_command_holds(ctrl):
    act = ctrl.step(RUN, 1.0, None, REF)
    assert act.kind == HOLD


def test_forwarded_target_always_in_window(ctrl):
    w = load_measured_windows(LIMITS_YAML)
    # push repeatedly toward the upper edge of shoulder_pan; never exceed it
    hi = w.windows["shoulder_pan_joint"][1]
    cmd = list(REF)
    cmd[0] = hi  # exactly the edge (legal)
    pos = REF
    for _ in range(500):
        act = ctrl.step(RUN, 1.0, cmd, pos)
        pos = act.target_position
    assert pos[0] <= hi + 1e-6
