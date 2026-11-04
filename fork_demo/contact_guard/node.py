#!/usr/bin/env python3
"""ROS live contact-guard watcher (lazy rclpy).

Runs ALONGSIDE execute_plan.py for a contact leg (like force_abort.py), on the
same controller. It subscribes the telemetry, projects the measured joints onto
the active leg's plan to get actual progress + tracking error, computes the
fork-frame signals, and drives :class:`runner.ContactGuardRunner`. On a trip it
CANCELS the running FollowJointTrajectory goal (cancel-all, the existing path),
writes a one-screen verdict + a reaction JSON, and HOLDS. It never sends a
trajectory itself; any in-axis back-off is left to the gated execute path
(URSim/mock only).

This module is import-safe without rclpy (rclpy is imported inside ``run``); the
pure logic lives in :mod:`runner` / :mod:`detector` and is proven by the mock
and URSim E2E in ``sim/``. On the real cell this task never arms it (the cell is
powered off); it is wired for the captain's later, reviewed live test.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Optional

import numpy as np
import yaml

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))

from fork_demo.contact_guard import envelope, params, signals  # noqa: E402
from fork_demo.contact_guard.runner import ContactGuardRunner, LegContext  # noqa: E402

WRENCH_TOPIC = "/force_torque_sensor_broadcaster/wrench"
JS_TOPIC = "/joint_states"
TCP_TOPIC = "/tcp_pose_broadcaster/pose"
SPEED_TOPIC = "/speed_scaling_state_broadcaster/speed_scaling"
PROG_TOPIC = "/io_and_status_controller/robot_program_running"


def _load_plan(path: str):
    with open(path) as fh:
        d = yaml.safe_load(fh)
    jp = np.asarray(d["joint_path"], dtype=float)
    return jp, float(d.get("duration_s", 1.0)), signals.path_arclengths(jp)


def run(args) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import WrenchStamped, PoseStamped
    from std_msgs.msg import Bool, Float64

    p = params.load(args.params)
    env = (envelope.Envelope.from_dict(yaml.safe_load(open(args.env)))
           if os.path.exists(args.env) else envelope.Envelope())
    jp, dur, cum = _load_plan(args.plan)
    runner = ContactGuardRunner(p, env)
    runner.start_leg(LegContext(name=args.leg, contact=args.contact))

    class GuardNode(Node):
        def __init__(self):
            super().__init__("fork_contact_guard")
            self.force = np.zeros(3); self.torque = np.zeros(3)
            self.eff = np.zeros(6); self.q = None
            self.tcp = np.zeros(3); self.tcp_prev = None; self.tcp_t = None
            self.prog = True; self.speed = 1.0
            self.t0 = None; self.last = None; self.tripped = False
            self.create_subscription(WrenchStamped, WRENCH_TOPIC, self._w, 10)
            self.create_subscription(JointState, JS_TOPIC, self._j, 10)
            self.create_subscription(PoseStamped, TCP_TOPIC, self._t, 10)
            self.create_subscription(Float64, SPEED_TOPIC, self._s, 10)
            self.create_subscription(Bool, PROG_TOPIC, self._p, 10)
            self.create_timer(0.02, self._tick)
            self.get_logger().info(
                f"contact-guard armed: leg={args.leg} contact={args.contact}")

        def _w(self, m):
            self.force = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])
            self.torque = np.array([m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z])

        def _j(self, m):
            try:
                self.q = signals.reorder_joints(m.name, m.position)
                if len(m.effort) == len(m.name):
                    self.eff = signals.reorder_joints(m.name, m.effort)
            except Exception:
                pass

        def _t(self, m):
            self.tcp = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

        def _s(self, m):
            self.speed = float(m.data) / 100.0 if m.data > 1.5 else float(m.data)

        def _p(self, m):
            self.prog = bool(m.data)

        def _tick(self):
            if self.tripped or self.q is None:
                return
            now = time.time()
            if self.t0 is None:
                self.t0 = now
            s_cmd = min(1.0, (now - self.t0) / max(dur, 1e-6))
            s_act = signals.progress_of(self.q, jp, cum)
            # commanded joints at s_cmd (interp along the plan by arclength)
            target = cum[-1] * s_cmd
            cj = np.array([np.interp(target, cum, jp[:, c]) for c in range(6)])
            track = float(np.linalg.norm(cj - self.q))
            vel = 0.0
            if self.tcp_prev is not None and self.tcp_t is not None and now > self.tcp_t:
                vel = float(np.linalg.norm(self.tcp - self.tcp_prev) / (now - self.tcp_t))
            self.tcp_prev = self.tcp.copy(); self.tcp_t = now
            ff, mf = signals.wrench_to_fork(self.force, self.torque)
            feats = signals.sample_features(ff, mf, self.eff, vel, track_err=track)
            det = runner.feed(now, s_cmd, s_act, feats,
                              executing=self.prog and self.speed > 0.001)
            if det is not None:
                self.tripped = True
                plan = runner.react(det, self.tcp)
                print(plan.verdict, flush=True)
                self._cancel()
                self._write(det, plan)

        def _cancel(self):
            from action_msgs.srv import CancelGoal
            cli = self.create_client(
                CancelGoal,
                f"/{args.controller}/follow_joint_trajectory/_action/cancel_goal")
            if cli.wait_for_service(timeout_sec=1.0):
                cli.call_async(CancelGoal.Request())  # cancel-all
                self.get_logger().error("CONTACT-GUARD: cancelled running goal")

        def _write(self, det, plan):
            if not args.out:
                return
            try:
                with open(args.out, "w") as fh:
                    json.dump({"schema": "contact_guard.trip.v1",
                               "leg": det.leg, "trigger": det.trigger,
                               "families": list(det.families),
                               "s_at_trip": det.s_at_trip,
                               "mm_at_trip": det.mm_at_trip,
                               "action": plan.action,
                               "backoff_target": plan.backoff_target,
                               "verdict": plan.verdict}, fh, indent=2)
            except Exception:
                pass

    rclpy.init()
    nd = GuardNode()
    try:
        rclpy.spin(nd)
    except KeyboardInterrupt:
        pass
    finally:
        nd.destroy_node()
        rclpy.shutdown()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plan", required=True, help="active leg plan yaml (joint_path)")
    ap.add_argument("--leg", required=True)
    ap.add_argument("--contact", action="store_true")
    ap.add_argument("--controller", default="scaled_joint_trajectory_controller")
    ap.add_argument("--params", default=params.CONFIG_PATH)
    ap.add_argument("--env", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "envelopes", "teach-002.yaml"))
    ap.add_argument("--out", default=None, help="write a trip JSON here")
    a = ap.parse_args(argv)
    return run(a)


if __name__ == "__main__":
    raise SystemExit(main())
