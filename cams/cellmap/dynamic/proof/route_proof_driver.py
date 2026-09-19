#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""In-container driver for the ROUTE proof.

Proves the captain's ask: an obstacle that APPEARS changes the planned route, and
DISAPPEARS from routes after it leaves. cuMotion (in this container, this domain)
plans the SAME start->goal three times against nvblox's LIVE dynamic ESDF:

  A) no obstacle            -> route T_A
  B) a box on T_A's path    -> route T_B must DETOUR (or become infeasible)
  C) box removed + decayed  -> route T_C must RETURN to ~T_A

The obstacle is placed at the actual TCP midpoint of T_A (exact FK on the
calibrated URDF), injected as synthetic depth via /fm_dyn/set_obstacle, and its
presence in the ESDF is confirmed with the same service cuMotion consults. Every
planned waypoint is checked against measured_joint_limits.yaml exactly as the cell
does (execute_plan.py --limits); a plan with any out-of-window waypoint fails the
proof. PLAN-ONLY: plan_only=True, nothing is ever executed.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             RobotState, WorkspaceParameters)
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from esdf_probe import query_occupied  # noqa: E402
from fk_ur10 import JOINTS, UR10FK  # noqa: E402

SPEED = 0.05
# A LOW, forward sweep designed offline with fk_ur10 (both endpoints in-window;
# TCP endpoints straddle an obstacle centre that sits in the fixed camera's
# proven-clear view, so the box on the path is reliably observed). See README.
Q_START = [-0.627, -2.333, -0.15, 4.458966, 0.279341, 0.706048]
Q_GOAL = [0.649, -1.2, 1.6, 4.458966, 0.279341, 0.706048]
# Fallback obstacle centre (straight-line TCP midpoint) if no planned waypoint
# lands in the camera-clear volume. In base_link.
C_FALLBACK = np.array([-0.058, 0.756, 0.713])
# The volume the DEFAULT downward provisional camera observes well — measured in
# the canonical ESDF proof (box at y0.55-0.85 z0-0.3 and person at y0.5-0.9 z0-1.7
# both integrated). The obstacle is placed inside this so it is reliably seen.
CLEAR_MIN = np.array([-0.20, 0.48, 0.05])
CLEAR_MAX = np.array([0.85, 0.92, 1.00])


def _in_clear(p):
    return bool(np.all(p >= CLEAR_MIN) and np.all(p <= CLEAR_MAX))


class RouteProof(Node):
    def __init__(self, urdf, limits_path):
        super().__init__("fm_dyn_route_proof")
        self.fk = UR10FK(urdf)
        lim = yaml.safe_load(open(limits_path))
        ref, win = lim["reference_pose_rad"], lim["window_deg"]
        self.lo = np.array([ref[j] - math.radians(win[j]) for j in JOINTS])
        self.hi = np.array([ref[j] + math.radians(win[j]) for j in JOINTS])
        self.q_start = np.array(Q_START)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 20)
        self.obst_pub = self.create_publisher(String, "/fm_dyn/set_obstacle", 10)
        self.create_timer(1.0 / 30.0, self._pub_js)
        self.mg = ActionClient(self, MoveGroup, "/cumotion/move_group")
        self.esdf = self.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")

    def _pub_js(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(JOINTS)
        m.position = [float(v) for v in self.q_start]
        self.js_pub.publish(m)

    def plan(self, goal, plan_time=8.0):
        if not self.mg.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("cuMotion move_group unavailable")
        req = MotionPlanRequest()
        req.pipeline_id = "isaac_ros_cumotion"
        req.group_name = "ur_manipulator"
        req.num_planning_attempts = 1
        req.allowed_planning_time = float(plan_time)
        req.max_velocity_scaling_factor = SPEED
        req.max_acceleration_scaling_factor = SPEED
        req.workspace_parameters = WorkspaceParameters()
        rs = RobotState()
        rs.joint_state.name = list(JOINTS)
        rs.joint_state.position = [float(v) for v in self.q_start]
        req.start_state = rs
        c = Constraints()
        for j, v in zip(JOINTS, goal):
            c.joint_constraints.append(JointConstraint(
                joint_name=j, position=float(v),
                tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0))
        req.goal_constraints.append(c)
        g = MoveGroup.Goal()
        g.request = req
        g.planning_options.plan_only = True
        t0 = time.monotonic()
        fut = self.mg.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=40.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, None, time.monotonic() - t0
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=90.0)
        res = rf.result().result
        dt = time.monotonic() - t0
        if res.error_code.val != 1:
            return None, res.error_code.val, dt
        traj = res.planned_trajectory.joint_trajectory
        q = np.array([list(p.positions) for p in traj.points])
        if list(traj.joint_names) != JOINTS:
            idx = [list(traj.joint_names).index(j) for j in JOINTS]
            q = q[:, idx]
        return q, 1, dt

    def limits_violations(self, qpath):
        bad = []
        for k, w in enumerate(qpath):
            for i, v in enumerate(w):
                if not (self.lo[i] <= v <= self.hi[i]):
                    bad.append((k, JOINTS[i], round(float(v), 4)))
        return bad

    def tcp_path(self, qpath):
        return self.fk.tcp_path(qpath)

    def set_obstacle(self, label, present, mn=None, mx=None):
        d = {"label": label, "present": bool(present)}
        if present:
            d["min"], d["max"] = list(mn), list(mx)
        for _ in range(5):
            self.obst_pub.publish(String(data=json.dumps(d)))
            time.sleep(0.05)
            rclpy.spin_once(self, timeout_sec=0.02)

    def probe(self, center, half=0.20):
        mn = [center[i] - half for i in range(3)]
        size = [2 * half] * 3
        return query_occupied(self, self.esdf, mn, size, 0.0, "base_link", 5.0)


def _resample(path, n=60):
    """Arc-length resample a 3D polyline to n points."""
    path = np.asarray(path)
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < 1e-9:
        return np.repeat(path[:1], n, axis=0)
    u = np.linspace(0, s[-1], n)
    return np.stack([np.interp(u, s, path[:, k]) for k in range(3)], axis=1)


def _directed_hausdorff(a, b):
    a, b = _resample(a), _resample(b)
    return max(float(np.min(np.linalg.norm(b - p, axis=1))) for p in a)


def path_distance(a, b):
    """Symmetric Hausdorff distance between two TCP paths (metres)."""
    return max(_directed_hausdorff(a, b), _directed_hausdorff(b, a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="/cfg/ur10_calibrated.urdf")
    ap.add_argument("--limits", default="/cfg/measured_joint_limits.yaml")
    ap.add_argument("--out", required=True)
    ap.add_argument("--box-half", type=float, default=0.16)
    ap.add_argument("--integrate-wait", type=float, default=4.0)
    ap.add_argument("--decay-wait", type=float, default=6.0)
    ap.add_argument("--change-thresh", type=float, default=0.08,
                    help="min TCP path shift (m) to count as a route change")
    ap.add_argument("--return-thresh", type=float, default=0.05,
                    help="max TCP path shift (m) to count as a returned route")
    a = ap.parse_args()

    rclpy.init()
    n = RouteProof(a.urdf, a.limits)
    if not n.esdf.wait_for_service(timeout_sec=60.0):
        print("FAIL: esdf service down"); sys.exit(2)
    # let /joint_states flow so cuMotion has a start state
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.05)

    out = {"ok": True, "steps": {}}

    # A: no obstacle
    n.set_obstacle("routebox", False)
    qA, codeA, tA = n.plan(Q_GOAL)
    if qA is None:
        print(f"FAIL: baseline plan failed (code {codeA})");
        json.dump({"ok": False, "reason": f"baseline plan failed {codeA}"}, open(a.out, "w"))
        sys.exit(1)
    tcpA = n.tcp_path(qA)
    badA = n.limits_violations(qA)
    # Place the obstacle on the ACTUAL planned path, at the waypoint whose TCP is
    # inside the camera-clear volume and nearest the path's midpoint — so the box
    # is both on the route and reliably observed. Fall back to the straight-line
    # midpoint if the plan never enters the clear volume.
    mididx = len(tcpA) // 2
    clear_pts = [(i, p) for i, p in enumerate(tcpA) if _in_clear(p)]
    if clear_pts:
        mid = min(clear_pts, key=lambda ip: abs(ip[0] - mididx))[1]
    else:
        mid = C_FALLBACK
    out["steps"]["A_baseline"] = {"n_pts": len(qA), "plan_time_s": round(tA, 3),
                                  "tcp_mid": [round(float(x), 3) for x in mid],
                                  "obstacle_on_path": bool(clear_pts),
                                  "limit_violations": badA}

    # B: box on the path midpoint
    mn = [float(mid[i] - a.box_half) for i in range(3)]
    mx = [float(mid[i] + a.box_half) for i in range(3)]
    n.set_obstacle("routebox", True, mn, mx)
    t_place = time.monotonic()
    time.sleep(a.integrate_wait)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    qB_present = n.probe(mid, half=a.box_half + 0.02)
    occ_present = None if qB_present is None else qB_present.occupied_voxels
    qB, codeB, tB = n.plan(Q_GOAL)
    if qB is None:
        changed, distB, badB = True, None, []
        out["steps"]["B_obstacle"] = {"planned": False, "code": codeB,
                                      "esdf_occupied": occ_present,
                                      "note": "infeasible with obstacle = route changed"}
    else:
        tcpB = n.tcp_path(qB)
        distB = path_distance(tcpA, tcpB)
        badB = n.limits_violations(qB)
        changed = distB >= a.change_thresh
        out["steps"]["B_obstacle"] = {"planned": True, "n_pts": len(qB),
                                      "plan_time_s": round(tB, 3),
                                      "esdf_occupied": occ_present,
                                      "tcp_path_shift_m": round(distB, 4),
                                      "changed": changed, "limit_violations": badB}

    # C: remove + decay
    n.set_obstacle("routebox", False)
    time.sleep(a.decay_wait)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    qC_absent = n.probe(mid, half=a.box_half + 0.02)
    occ_absent = None if qC_absent is None else qC_absent.occupied_voxels
    qC, codeC, tC = n.plan(Q_GOAL)
    if qC is None:
        returned, distC, badC = False, None, []
        out["steps"]["C_cleared"] = {"planned": False, "code": codeC,
                                     "esdf_occupied": occ_absent}
    else:
        tcpC = n.tcp_path(qC)
        distC = path_distance(tcpA, tcpC)
        badC = n.limits_violations(qC)
        returned = distC <= a.return_thresh
        out["steps"]["C_cleared"] = {"planned": True, "n_pts": len(qC),
                                     "plan_time_s": round(tC, 3),
                                     "esdf_occupied": occ_absent,
                                     "tcp_path_shift_m": round(distC, 4),
                                     "returned": returned, "limit_violations": badC}

    # verdict
    esdf_ok = (occ_present or 0) > 0 and (occ_absent or 0) < max(1, (occ_present or 0) // 2)
    limits_ok = not (badA or badB or badC)
    out["ok"] = bool(changed and returned and esdf_ok and limits_ok)
    out["verdict"] = {"route_changed_on_appear": bool(changed),
                      "route_returned_on_leave": bool(returned),
                      "esdf_gained_then_lost_obstacle": bool(esdf_ok),
                      "all_waypoints_within_measured_limits": bool(limits_ok),
                      "occ_present": occ_present, "occ_absent": occ_absent}
    json.dump(out, open(a.out, "w"), indent=2)
    print("SUMMARY " + json.dumps(out["verdict"]))
    print("ROUTE PROOF " + ("PASS" if out["ok"] else "FAIL"))
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if out["ok"] else 1)


if __name__ == "__main__":
    main()
