#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Segmented re-plan + preempt pick controller — SIMULATED arm only
(human-dynamic-plan-s1 report §10 Phase A; candidate C1 §1).

This is the dynamic-pick analog of `cams/cellmap/tools/loop_tags.py`: instead of
looping fixed waypoints, it plans a SHORT leg to the freshest tracked pick target,
executes it on `scaled_joint_trajectory_controller`, and PREEMPTS with a new goal
when the target has genuinely moved. It ties together the pure cores built beside
it:

  pick_target_tracker  filtered target estimate + deadband/hysteresis + lost/occluded
  pick_phases          FAR/NEAR/GRASP/RETREAT + per-phase re-plan policy
  gate_acceptance      the SOLE motion authority: a goal is forwarded ONLY on a
                       fresh gate RUN (this node is the only thing that puts a goal
                       on the wire, and it puts one there only through the shim)
  planner_port         AnalyticJointPlanner (mock/URSim) OR cumotion_planner (GPU)

MOTION AUTHORITY. The merged human-safety gate (`humangate`/`humangate_driver`)
remains the single authority over motion, unchanged. This node consults the gate's
published decision through the acceptance shim before EVERY goal it sends; while
the gate is not a fresh RUN it forwards nothing, and the gate driver + independent
watcher (separate processes) own SLOW (slider) and STOP (cancel/hold) of any leg
already running. This node NEVER writes the slider, never cancels for safety, and
never arms real hardware — it only produces goals to be gated.

SIMULATION ONLY. Targets are consumed in the base frame (the sim publishes there;
on the real cell a TF/seeded-IK step precedes this). The 5 % speed cap and the
MEASURED joint windows are enforced on every leg exactly as the cell's executor
does; a plan outside the windows is refused, never clamped.

Run (isolated task domain 83):
    ROS_DOMAIN_ID=83 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 segmented_pick_node.py \
        --ros-args -p planner:=analytic -p limits:=cumotion/measured_joint_limits.yaml \
        -p log:=/tmp/dp/pick.jsonl
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gate_acceptance import GateAcceptanceShim, GateView  # noqa: E402
from pick_phases import PhaseParams, PickPhaseMachine  # noqa: E402
from pick_target_tracker import PickTargetTracker  # noqa: E402
from planner_port import (  # noqa: E402
    JOINTS, SPEED, AnalyticJointPlanner, CartesianToJointSurrogate, joint_windows)
from replan_controller_core import PLAN, ReplanController  # noqa: E402

FJT_DEFAULT = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


class SegmentedPickNode(Node):
    def __init__(self):
        super().__init__("dynamic_pick_controller")

        # ---- params ---- #
        self.limits_path = str(self._p("limits", "cumotion/measured_joint_limits.yaml"))
        self.planner_kind = str(self._p("planner", "analytic"))
        self.target_topic = str(self._p("target_topic", "/dynamic_pick/target"))
        self.decision_topic = str(self._p("decision_topic", "/human_gate/decision"))
        self.fjt_action = str(self._p("fjt_action", FJT_DEFAULT))
        self.cumotion_action = str(self._p("cumotion_action", "/cumotion/move_group"))
        self.control_rate_hz = float(self._p("control_rate_hz", 15.0))
        self.min_replan_interval_s = float(self._p("min_replan_interval_s", 0.5))
        self.plan_time_s = float(self._p("plan_time_s", 5.0))
        self.deadband_m = float(self._p("deadband_m", 0.03))
        self.n_sustain = int(self._p("n_sustain", 3))
        self.log_path = str(self._p("log", ""))

        lim = yaml.safe_load(open(self.limits_path))
        self.lo, self.hi = joint_windows(lim)
        self.surrogate = CartesianToJointSurrogate(lo=self.lo, hi=self.hi)

        # ---- pure cores ---- #
        self.tracker = PickTargetTracker(deadband_m=self.deadband_m, n_sustain=self.n_sustain)
        self.controller = ReplanController(
            shim=GateAcceptanceShim(),
            phases=PickPhaseMachine(PhaseParams()),
            min_replan_interval_s=self.min_replan_interval_s)

        # ---- planner ---- #
        if self.planner_kind == "cumotion":
            from cumotion_planner import CuMotionPlanner
            self.planner = CuMotionPlanner(self, self.cumotion_action, speed=SPEED)
            self.get_logger().info(f"planner = cuMotion (collision-aware) @ {self.cumotion_action}")
        else:
            self.planner = AnalyticJointPlanner(speed=SPEED)
            self.get_logger().info("planner = analytic straight-line (GPU-free sim stand-in)")

        # ---- ROS I/O ---- #
        self.js = {}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.js.update(zip(m.name, m.position)), 20)
        self._pending_target = None      # (xyz_base, stamp) awaiting consumption
        self.create_subscription(PointStamped, self.target_topic, self._on_target, 10)
        self._gate = GateView(None, None)
        self.create_subscription(String, self.decision_topic, self._on_decision, 10)
        self.fjt = ActionClient(self, FollowJointTrajectory, self.fjt_action)

        # ---- goal / latency bookkeeping ---- #
        self._active_gh = None
        self._committed_goal = None      # xyz of the leg most recently committed
        self._plan_samples = []          # plan latency (s)
        self._preempt_samples = []       # decide -> new-goal-accepted latency (s)
        self._retarget_mono = []         # mono time of each accepted preempt goal (for rate)
        self._goals_sent = 0
        if self.log_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.log_path)), exist_ok=True)
            self._logf = open(self.log_path, "w")
        else:
            self._logf = None

        self.create_timer(1.0 / self.control_rate_hz, self._tick)
        self.get_logger().info(
            f"dynamic_pick controller up: target {self.target_topic}, gate {self.decision_topic}, "
            f"FJT {self.fjt_action}. Gate is the SOLE motion authority; SIMULATED arm only.")

    def _p(self, name, default):
        return self.declare_parameter(name, default).value

    # ---- subscriptions ---- #
    def _on_target(self, msg: PointStamped):
        xyz = np.array([msg.point.x, msg.point.y, msg.point.z])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0:
            stamp = time.time()
        self._pending_target = (xyz, stamp)

    def _on_decision(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._gate = GateView(d.get("action"), time.monotonic())
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f"bad gate decision: {e}")

    def q(self):
        return np.array([self.js.get(j, 0.0) for j in JOINTS])

    # ---- control tick ---- #
    def _tick(self):
        now = time.monotonic()
        if len(self.js) < 6:
            return
        q0 = self.q()
        tcp = self.surrogate.to_cartesian(q0)

        # tracker: consume a pending measurement, else advance the coast lifecycle.
        if self._pending_target is not None:
            xyz, stamp = self._pending_target
            self._pending_target = None
            est = self.tracker.update(xyz, None, stamp, now)
        else:
            est = self.tracker.step(now)

        intent = self.controller.decide(now, est, tcp, self._gate)
        self._log_tick(now, intent, est, tcp)

        if intent.action == PLAN:
            self._send_leg(intent.goal, now, preempt=intent.preempt)

    # ---- planning + goal dispatch ---- #
    def _send_leg(self, goal_xyz, now, preempt):
        # decision time = the instant we begin acting on this (pre)empt intent; the
        # preemption latency measured below spans plan + send + controller-accept.
        decide_t = time.time()
        q0 = self.q()
        self._committed_goal = np.asarray(goal_xyz, float).reshape(3)
        q_goal = self.surrogate.to_joint(goal_xyz)
        res = self.planner.plan(q0, q_goal, self.lo, self.hi, seconds=self.plan_time_s)
        plan_dt = time.time() - decide_t
        self._leg_duration = res.duration_s if res.ok else 0.0
        self._plan_samples.append(plan_dt)
        if not res.ok:
            self.get_logger().warn(f"plan refused: {res.message}")
            self._log_event(now, "plan_refused", {"reason": res.message,
                                                   "plan_s": round(plan_dt, 4)})
            return
        # start-state guard: the plan must begin at the live pose (loop_tags.py:106).
        got0 = np.array(res.positions[0])
        if np.abs(got0 - q0).max() > 0.05:
            self._log_event(now, "plan_bad_start",
                            {"off_deg": math.degrees(np.abs(got0 - q0).max())})
            return

        traj = self._to_traj(res)
        if not self.fjt.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("FJT action unavailable")
            return
        g = FollowJointTrajectory.Goal()
        g.trajectory = traj
        send_future = self.fjt.send_goal_async(g)
        send_future.add_done_callback(
            lambda fut, dt=decide_t, pmt=preempt, pdt=plan_dt: self._on_goal_response(fut, dt, pmt, pdt))
        self._log_event(now, "goal_sent", {
            "gate_action": self._gate.action, "preempt": preempt,
            "plan_s": round(plan_dt, 4), "solve_s": round(res.planning_time_s, 4),
            "duration_s": round(res.duration_s, 2), "n_pts": len(res.positions),
            "goal_xyz": [round(float(v), 3) for v in goal_xyz]})

    def _on_goal_response(self, fut, decide_t, preempt, plan_dt):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("goal rejected by controller")
            return
        t_acc = time.time()
        self._active_gh = gh
        self.controller.note_leg_started(self._committed_goal, time.monotonic(),
                                         getattr(self, "_leg_duration", 0.0))
        self._goals_sent += 1
        mono = time.monotonic()
        if preempt and decide_t is not None:
            lat = t_acc - decide_t
            self._preempt_samples.append(lat)
            self._retarget_mono.append(mono)
            self.get_logger().info(f"preempt accepted, latency {lat*1000:.0f} ms "
                                   f"(plan {plan_dt*1000:.0f} ms)")
            self._log_event(mono, "preempt_accepted", {"latency_s": round(lat, 4),
                                                        "plan_s": round(plan_dt, 4)})
        rf = gh.get_result_async()
        rf.add_done_callback(lambda fut, h=gh: self._on_goal_result(fut, h))

    def _on_goal_result(self, fut, gh):
        # Only the CURRENTLY active goal finishing means "no leg running". A
        # preempted older goal's result also fires here; ignore it, or it would
        # clear leg_running for the newer leg that just replaced it.
        if gh is not self._active_gh:
            return
        self.controller.note_leg_finished()
        self._active_gh = None

    def _to_traj(self, res):
        traj = JointTrajectory()
        traj.joint_names = list(res.joint_names)
        n = len(res.positions)
        for k in range(n):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in res.positions[k]]
            if k in (0, n - 1):
                pt.velocities = [0.0] * 6
            if k == n - 1:
                pt.accelerations = [0.0] * 6
            t = res.times_s[k]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        return traj

    # ---- logging ---- #
    def _log_tick(self, now, intent, est, tcp):
        if not self._logf:
            return
        rec = {"kind": "tick", "t_mono": round(now, 4), "intent": intent.action,
               "phase": intent.phase, "gate_action": intent.gate_action,
               "gate_forward": intent.gate_forward, "preempt": intent.preempt,
               "tracker_state": est.state, "committed_moved": est.committed_moved,
               "committed": None if est.committed_target is None
               else [round(float(v), 3) for v in est.committed_target],
               "tcp": [round(float(v), 3) for v in tcp], "reason": intent.reason}
        if intent.action == PLAN:
            self._committed_goal = intent.goal
        self._logf.write(json.dumps(rec) + "\n")
        self._logf.flush()

    def _log_event(self, now, kind, extra):
        if not self._logf:
            return
        rec = {"kind": kind, "t_mono": round(now, 4)}
        rec.update(extra)
        self._logf.write(json.dumps(rec) + "\n")
        self._logf.flush()

    def summary(self):
        def stats(xs):
            if not xs:
                return None
            a = np.array(xs)
            return {"n": len(a), "median_ms": round(float(np.median(a)) * 1e3, 1),
                    "min_ms": round(float(a.min()) * 1e3, 1),
                    "max_ms": round(float(a.max()) * 1e3, 1)}
        rate = None
        if len(self._retarget_mono) >= 2:
            span = self._retarget_mono[-1] - self._retarget_mono[0]
            if span > 0:
                rate = round((len(self._retarget_mono) - 1) / span, 3)
        return {"kind": "summary", "goals_sent": self._goals_sent,
                "preempts": len(self._preempt_samples),
                "plan_latency": stats(self._plan_samples),
                "preempt_latency": stats(self._preempt_samples),
                "retarget_rate_hz": rate}

    def on_shutdown(self):
        if hasattr(self.planner, "close"):
            self.planner.close()
        s = self.summary()
        self.get_logger().info(f"SUMMARY {json.dumps(s)}")
        if self._logf:
            self._logf.write(json.dumps(s) + "\n")
            self._logf.flush()
            self._logf.close()
            self._logf = None


def _raise_keyboard_interrupt(*_):
    raise KeyboardInterrupt()


def main(args=None):
    import signal
    # Treat SIGTERM (how the sim scripts stop us) like SIGINT so on_shutdown runs
    # and flushes the latency/rate summary before we exit.
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentedPickNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
