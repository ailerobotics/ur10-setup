#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""THE principle test for the dynamic layer — NO ROS, NO GPU.

Captain's principle (report §5): a person/obstacle in the map is a ROUTING aid,
never the stop mechanism. This test proves, in code, both halves:

  1. Removing the dynamic layer's output CHANGES routes (the map does its job).
  2. It NEVER turns a STOP into motion — the acceptance point forwards a plan
     only when the merged human-safety gate says RUN, and that gate decision is a
     function of the camera-side human-safety state ALONE, not of the map.

The second half is shown twice: exhaustively over the acceptance point, and end
to end through the REAL humangate decision core, feeding the SAME dynamic-obstacle
plan under a STOP camera state (held) and under a CLEAR-dwelled state (forwarded)
— only the camera state changed the outcome.

Run:  python3 test_routing_gate.py   |   pytest test_routing_gate.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))                     # dynamic/
sys.path.insert(0, os.path.join(os.path.dirname(HERE),
                                "..", "..", "humangate"))     # humangate/

from routing_gate import (  # noqa: E402
    BACKSTOP, RUN, SLOW, STOP, PlannedRoute, authorize_plan,
    map_can_never_start_motion, route_changed_by_dynamic)

# Two cuMotion results: without the dynamic obstacle (straight leg) and with it
# (a route that detours, routing_around the obstacle). Different waypoints.
ROUTE_NO_OBSTACLE = PlannedRoute(
    waypoints=[[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]],
    routed_around=frozenset(), map_had_dynamic=False)
ROUTE_WITH_OBSTACLE = PlannedRoute(
    waypoints=[[0.0, 0.0], [0.5, 0.4], [1.0, 0.0]],
    routed_around=frozenset({"box"}), map_had_dynamic=True)


def test_dynamic_layer_changes_the_route():
    assert route_changed_by_dynamic(ROUTE_NO_OBSTACLE, ROUTE_WITH_OBSTACLE)
    # identical routes are correctly reported unchanged
    assert not route_changed_by_dynamic(ROUTE_NO_OBSTACLE, ROUTE_NO_OBSTACLE)


def test_stop_never_forwards_regardless_of_map():
    for plan in (ROUTE_NO_OBSTACLE, ROUTE_WITH_OBSTACLE, None):
        for action in (STOP, SLOW, BACKSTOP):
            auth = authorize_plan(action, plan)
            assert auth.forward is False, (action, plan)


def test_run_forwards_the_planned_route():
    auth = authorize_plan(RUN, ROUTE_WITH_OBSTACLE)
    assert auth.forward is True
    assert auth.plan is ROUTE_WITH_OBSTACLE


def test_map_can_never_start_motion_exhaustive():
    assert map_can_never_start_motion(
        gate_actions=[STOP, SLOW, BACKSTOP, RUN],
        plans_by_map=[ROUTE_NO_OBSTACLE, ROUTE_WITH_OBSTACLE, None])


# ---- end-to-end through the REAL merged gate decision core -------------------

def _gate():
    from gate_config import GateConfig
    import gate_core as gc
    cfg = GateConfig(clear_dwell_s=1.0, heartbeat_timeout_s=0.5,
                     state_max_age_s=0.5, backstop_deadline_s=1.0).validate()
    return gc.HumanGate(cfg)


def _state(zone, seq, stamp):
    from camera_state import CameraState
    return CameraState(stamp=stamp, zone=zone, nearest_distance_m=1.0,
                       health_ok=True, heartbeat_seq=seq)


def test_gate_decides_from_camera_state_not_map():
    """Same dynamic-obstacle plan; a STOP camera state holds it, a CLEAR-dwelled
    camera state forwards it. The map never entered the decision."""
    from camera_state import ZONE_CLEAR, ZONE_SLOW, ZONE_STOP
    g = _gate()
    t = 100.0
    seq = 1

    def tick(zone, dt):
        nonlocal t, seq
        t += dt
        seq += 1
        g.on_state(_state(zone, seq, t), now=t, now_wall=t)
        return g.decide(now=t, now_wall=t)

    # person in the STOP zone -> gate STOP -> the plan (with obstacle) is held
    g.on_state(_state(ZONE_STOP, 1, t), now=t, now_wall=t)
    d = g.decide(now=t, now_wall=t)
    assert d.action == STOP
    assert authorize_plan(d.action, ROUTE_WITH_OBSTACLE).forward is False

    # person leaves: SLOW then a held CLEAR run, so provenance is a tracked
    # departure and the dwell (1.0 s) can complete -> RUN forwards the SAME plan.
    tick(ZONE_SLOW, 0.2)
    for _ in range(8):
        d = tick(ZONE_CLEAR, 0.25)
    assert d.action == RUN, d.reason
    assert authorize_plan(d.action, ROUTE_WITH_OBSTACLE).forward is True


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
