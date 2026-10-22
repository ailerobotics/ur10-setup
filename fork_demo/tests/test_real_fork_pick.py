#!/usr/bin/env python3
"""Offline tests for the real fork pick-and-place geometry + safety logic.

No ROS, no robot. Run:  python3 -I -m pytest fork_demo/tests -q
                   or:  python3 -I fork_demo/tests/test_real_fork_pick.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # fork_demo/

from force_abort import ForceMonitor  # noqa: E402
from real_fork_pick import (  # noqa: E402
    AnchorPose, ForkPickPlan, MotionOffsets, SafetyLimits, _rotate, provisional_anchors,
)


def _plan(front_offset=0.0, safety=None):
    pick, drop = provisional_anchors()
    return ForkPickPlan(pick=pick, drop=drop, off=MotionOffsets(),
                        safety=safety or SafetyLimits(), front_offset=front_offset).build()


def test_rotate_identity_and_axis():
    # identity quaternion leaves vectors alone
    assert _rotate((0, 0, 0, 1), (1, 2, 3)) == (1, 2, 3)
    # (0.5,0.5,0.5,0.5) maps tool Z (0,0,1) -> +X
    rx, ry, rz = _rotate((0.5, 0.5, 0.5, 0.5), (0, 0, 1))
    assert abs(rx - 1) < 1e-9 and abs(ry) < 1e-9 and abs(rz) < 1e-9


def test_waypoint_shape_and_order():
    p = _plan()
    names = [w.name for w in p.waypoints]
    assert names == ["HOME", "PRE_APPROACH", "INSERT_START", "INSERTED", "LIFTED",
                     "EXTRACTED", "CARRY_OVER", "SET_DOWN", "LOWERED", "WITHDRAWN", "RETREAT"]
    # contact legs are exactly the insert/lift/extract + set-down/withdraw
    contact = {w.name for w in p.waypoints if w.leg_type == "contact"}
    assert contact == {"INSERT_START", "INSERTED", "LIFTED", "EXTRACTED",
                       "LOWERED", "WITHDRAWN"}


def test_insert_is_straight_along_fwd():
    p = _plan()
    wp = {w.name: w.position for w in p.waypoints}
    fwd = p.pick.fwd()
    # INSERTED should be insert_travel forward of INSERT_START along fwd, same height
    d = tuple(wp["INSERTED"][i] - wp["INSERT_START"][i] for i in range(3))
    assert abs(d[2]) < 1e-9                       # no vertical change on insert
    along = sum(d[i] * fwd[i] for i in range(3))
    assert abs(along - p.off.insert_travel) < 1e-9


def test_lift_is_pure_vertical():
    p = _plan()
    wp = {w.name: w.position for w in p.waypoints}
    d = tuple(wp["LIFTED"][i] - wp["INSERTED"][i] for i in range(3))
    assert abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9
    assert abs(d[2] - p.off.lift) < 1e-9


def test_speed_caps_respected():
    p = _plan()
    s = p.safety
    for w in p.waypoints:
        assert w.speed_frac <= s.speed_frac_leg + 1e-12
        if w.leg_type == "contact":
            assert w.speed_frac <= s.speed_frac_contact + 1e-12


def test_front_offset_shifts_back_and_never_increases_reach():
    base = _plan(front_offset=0.0)
    front = _plan(front_offset=0.15)
    fwd = base.pick.fwd()
    for wb, wf in zip(base.waypoints, front.waypoints):
        d = tuple(wf.position[i] - wb.position[i] for i in range(3))
        along = sum(d[i] * fwd[i] for i in range(3))
        assert abs(along + 0.15) < 1e-9            # exactly 150 mm back along fwd
    # the front-dry-run tip must be strictly LESS far along fwd than the contact one
    pb = {w.name: w.position for w in base.waypoints}["INSERTED"]
    pf = {w.name: w.position for w in front.waypoints}["INSERTED"]
    assert sum(pf[i] * fwd[i] for i in range(3)) < sum(pb[i] * fwd[i] for i in range(3))


def test_workspace_box():
    s = SafetyLimits()
    assert s.in_workspace((0.5, 0.0, 0.3))
    assert not s.in_workspace((2.0, 0.0, 0.3))     # beyond xmax
    assert not s.in_workspace((0.5, 0.0, 2.0))     # above zmax


def test_execute_refuses_provisional_anchors():
    import types
    p = _plan()
    args = types.SimpleNamespace(i_am_at_the_estop=True, tcp=None, robot_ip="x",
                                 limits="x")
    from real_fork_pick import execute
    assert execute(p, args) == 2                   # refuses placeholder anchors


def test_force_monitor_baseline_then_trip():
    m = ForceMonitor(backstop_n=200.0, baseline_n=10)
    # learn a quiet baseline around 15 N (CB3 standing bias)
    for i in range(10):
        assert not m.update(i * 0.01, 15.0, 0.0, 0.0)
    # small wobble within the clean envelope does not trip
    assert not m.update(0.2, 120.0, 0.0, 0.0)      # dev ~105 N < 200
    # a hard contact trips
    assert m.update(0.3, 400.0, 0.0, 0.0)          # dev ~385 N > 200
    assert "force deviation" in m.last_reason


def test_force_monitor_stale_fails_toward_stop():
    m = ForceMonitor(backstop_n=200.0, stale_s=0.5)
    m.update(100.0, 15.0, 0.0, 0.0)
    assert not m.is_stale(100.2)
    assert m.is_stale(101.0)
    assert "stale" in m.last_reason


def _run():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} tests passed")


if __name__ == "__main__":
    _run()
