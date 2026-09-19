#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
cuMotion-backed planner — the collision-aware `Planner` (report §1 candidate C1).

Implements the same `Planner` interface as the pure `AnalyticJointPlanner`, but
plans through NVIDIA cuMotion's `move_group` action, so a leg is routed around the
cell obstacles carried in the nvblox ESDF (the map variant,
`planner.launch.py:93 read_esdf_world:=true`). The controller does not know or care
which planner it holds; only the sim harness picks one.

This is the ONLY dynamic_pick module that talks to ROS/cuMotion. It reuses the
exact request the cell's executor uses (`execute_plan.py:80-110`): a JOINT-space
goal (seeded, predictable — AGENTS.md), `plan_only=True`, 5 % velocity/accel
scaling, `pipeline_id='isaac_ros_cumotion'`. Every returned waypoint is validated
inside the MEASURED joint windows (the guarantee the executor relies on); a plan
that would leave them is refused, exactly as `execute_plan.py:213-217` does.

It is used ONLY against the task's OWN cuMotion container on the isolated task
domain (83) — NEVER the cell's `ur10-cumotion` (domain 0). The action namespace is
a parameter so the container's namespace can be pointed at without touching the
live planner.
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             WorkspaceParameters)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from planner_port import (JOINTS, SPEED, Planner, PlannerResult,
                          in_window_wrapped, wrap_to_nearest)


class CuMotionPlanner(Planner):
    """Collision-aware planner via cuMotion's move_group action (plan-only)."""

    def __init__(self, node, action_ns: str = "/cumotion/move_group",
                 group_name: str = "ur_manipulator",
                 pipeline_id: str = "isaac_ros_cumotion",
                 speed: float = SPEED):
        # OWN node for the move_group action, spun synchronously in plan(). The
        # controller node is driven by rclpy.spin (a timer callback calls plan());
        # calling spin_until_future_complete on THAT node would re-enter its
        # executor ("Executor is already spinning"). A dedicated node has no
        # executor of its own, so spin_until_future_complete on it is safe. plan()
        # blocks the controller's tick for the solve (~0.1-0.5 s), like execute_plan.
        self._log = node.get_logger() if node is not None else None
        self._node = rclpy.create_node("dynamic_pick_cumotion_client")
        # a DEDICATED executor for this node's spins. rclpy.spin_until_future_complete
        # defaults to the GLOBAL executor, which the controller's rclpy.spin is already
        # driving — spinning it again raises "Executor is already spinning". Spinning a
        # SEPARATE executor (only this planner node on it) is safe from inside the
        # controller's tick callback.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._mg = ActionClient(self._node, MoveGroup, action_ns)
        self.action_ns = action_ns
        self.group_name = group_name
        self.pipeline_id = pipeline_id
        self.speed = float(speed)

    def available(self, timeout: float = 20.0) -> bool:
        return self._mg.wait_for_server(timeout_sec=timeout)

    def close(self) -> None:
        try:
            self._mg.destroy()
            self._node.destroy_node()
        except Exception:                                # noqa: BLE001
            pass

    def plan(self, q0, q_goal, lo, hi, seconds: float = 25.0) -> PlannerResult:
        q0 = np.asarray(q0, float).reshape(6)
        qg = np.asarray(q_goal, float).reshape(6)
        lo = np.asarray(lo, float).reshape(6)
        hi = np.asarray(hi, float).reshape(6)
        if not in_window_wrapped(qg, lo, hi).all():
            bad = [JOINTS[i] for i in range(6) if not in_window_wrapped(qg[i], lo[i], hi[i])]
            return PlannerResult(False, JOINTS, [], [], 0.0,
                                 f"goal outside measured joint windows: {bad}")
        # ask cuMotion for the goal on the branch nearest the live start (shortest
        # travel for the continuous joints), matching the sim arm's home branch.
        qg = wrap_to_nearest(qg, q0)

        if not self._mg.wait_for_server(timeout_sec=5.0):
            return PlannerResult(False, JOINTS, [], [], 0.0,
                                 "cuMotion move_group action unavailable")

        req = MotionPlanRequest()
        req.pipeline_id = self.pipeline_id
        req.group_name = self.group_name
        req.num_planning_attempts = 1
        req.allowed_planning_time = float(seconds)
        req.max_velocity_scaling_factor = self.speed
        req.max_acceleration_scaling_factor = self.speed
        req.workspace_parameters = WorkspaceParameters()
        c = Constraints()
        for j, v in zip(JOINTS, qg):
            c.joint_constraints.append(JointConstraint(
                joint_name=j, position=float(v),
                tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0))
        req.goal_constraints.append(c)
        g = MoveGroup.Goal()
        g.request = req
        g.planning_options.plan_only = True

        t0 = time.time()
        fut = self._mg.send_goal_async(g)
        rclpy.spin_until_future_complete(self._node, fut, executor=self._executor, timeout_sec=40.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return PlannerResult(False, JOINTS, [], [], time.time() - t0,
                                 "cuMotion planning goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self._node, rf, executor=self._executor, timeout_sec=90.0)
        result = rf.result().result if rf.result() else None
        solve_s = time.time() - t0
        if result is None or result.error_code.val != 1:
            code = None if result is None else result.error_code.val
            return PlannerResult(False, JOINTS, [], [], solve_s,
                                 f"cuMotion plan failed (code {code})")

        traj = result.planned_trajectory.joint_trajectory
        got = np.array([list(p.positions) for p in traj.points])
        names = list(traj.joint_names)
        if names != JOINTS:
            idx = [names.index(j) for j in JOINTS]
            got = got[:, idx]
        # joint-window guard on EVERY waypoint (execute_plan.py:213-217 parity),
        # wrapping-aware so a continuous joint reported a turn away still passes.
        out = [(k, JOINTS[i]) for k, w in enumerate(got)
               for i in range(6) if not in_window_wrapped(w[i], lo[i], hi[i])]
        if out:
            return PlannerResult(False, JOINTS, [], [], solve_s,
                                 f"{len(out)} cuMotion waypoints outside joint windows "
                                 f"(first {out[0]}) — refusing")
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                 for p in traj.points]
        # report the planner's OWN reported solve time when present (the fast number
        # the report cites, ~0.1 s), else the wall time.
        planning_time = getattr(result, "planning_time", 0.0) or solve_s
        return PlannerResult(
            True, JOINTS, [[float(v) for v in w] for w in got], times,
            float(planning_time),
            f"cuMotion plan: {len(got)} pts, {times[-1]:.1f} s, "
            f"solve {planning_time*1000:.0f} ms (wall {solve_s*1000:.0f} ms)")
