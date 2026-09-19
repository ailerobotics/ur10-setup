#!/usr/bin/env python3
"""
Harmless trajectory mover for the simulation proofs. Sends ONE long, slow joint
trajectory to the FollowJointTrajectory action and logs where the arm is over
time, so the proof can show the motion FREEZE mid-trajectory (speed slider -> 0),
RESUME the SAME trajectory (slider raised), and be CANCELLED by the backstop.

This is the analog of `cams/cellmap/tools/execute_plan.py` for a SIMULATED arm.
It uses tiny joint excursions and the repo's 5% velocity/accel scaling, with a
zero start stamp and zero terminal velocity (the controller rejects a non-zero
terminal velocity — execute_plan.py). It never runs on real hardware: the gate
and watcher own the safety behaviour; this only produces motion to be gated.

Progress is logged as JSONL: {t, elapsed, tps (trajectory progress by joint
position), done, status}. A frozen slider shows up as `tps` flat-lining while
`elapsed` keeps rising; a resume shows `tps` advancing again from where it froze.

Run (task domain 79):
    ROS_DOMAIN_ID=79 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 trajectory_mover.py \
        --duration 40 --log /tmp/hg/mover.jsonl
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FJT = '/scaled_joint_trajectory_controller/follow_joint_trajectory'


class Mover(Node):
    def __init__(self):
        super().__init__("trajectory_mover")
        self.js = {}
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.js.update(zip(m.name, m.position)), 20)
        self.fjt = ActionClient(self, FollowJointTrajectory, FJT)

    def wait_states(self, t=15.0):
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.js) >= 6:
                return True
        return False

    def q(self):
        return [self.js.get(j, 0.0) for j in JOINTS]


def build_traj(q0, duration, amp=0.5, profile="sweep"):
    """A slow motion of shoulder_pan (and a touch of elbow), well within limits,
    sampled every 0.5 s, peak velocity far below 5%.

    profile="sweep" (default): a there-and-back 0 -> amp -> 0 cosine (the original;
      good for the cancel/backstop proofs where the arm holds where cancelled).
    profile="ramp": a MONOTONIC 0 -> amp ramp. Used by the pendant-pause gate e2e so a
      real freeze is unambiguous: progress advances steadily and stops DEAD if paused,
      never confused with the cosine's near-zero-velocity turnaround."""
    traj = JointTrajectory()
    traj.joint_names = JOINTS
    n = max(4, int(duration / 0.5))
    for k in range(n + 1):
        frac = k / n
        if profile == "ramp":
            s = frac                          # monotonic 0 -> 1
        else:
            s = 0.5 * (1 - math.cos(2 * math.pi * frac))   # there-and-back
        pt = JointTrajectoryPoint()
        pos = list(q0)
        pos[0] = q0[0] + amp * s              # shoulder_pan
        pos[2] = q0[2] + 0.3 * amp * s        # elbow (small)
        pt.positions = [float(v) for v in pos]
        t = frac * duration
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        traj.points.append(pt)
    traj.points[0].velocities = [0.0] * 6
    traj.points[-1].velocities = [0.0] * 6
    traj.points[-1].accelerations = [0.0] * 6
    traj.header.stamp.sec = 0
    traj.header.stamp.nanosec = 0
    return traj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--profile", choices=["sweep", "ramp"], default="sweep")
    ap.add_argument("--log", default="/tmp/hg/mover.jsonl")
    ap.add_argument("--watch", type=float, default=60.0,
                    help="max wall seconds to watch before giving up")
    a = ap.parse_args()

    rclpy.init()
    n = Mover()
    if not n.wait_states():
        print("mover: no /joint_states — driver/sim not up", file=sys.stderr)
        n.destroy_node(); rclpy.shutdown(); return 1
    q0 = n.q()
    traj = build_traj(q0, a.duration, a.amp, a.profile)
    span = abs(a.amp) + 1e-9

    if not n.fjt.wait_for_server(timeout_sec=15.0):
        print("mover: FJT action unavailable", file=sys.stderr)
        n.destroy_node(); rclpy.shutdown(); return 1
    g = FollowJointTrajectory.Goal()
    g.trajectory = traj
    fut = n.fjt.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("mover: trajectory goal REJECTED", file=sys.stderr)
        n.destroy_node(); rclpy.shutdown(); return 1
    print("mover: trajectory ACCEPTED, moving", flush=True)
    rf = gh.get_result_async()

    import os
    os.makedirs(os.path.dirname(os.path.abspath(a.log)), exist_ok=True)
    t0 = time.time()
    status = "running"
    with open(a.log, "w") as f:
        while not rf.done():
            rclpy.spin_once(n, timeout_sec=0.05)
            el = time.time() - t0
            # trajectory progress by shoulder_pan position (0..1), the signal
            # that flat-lines on a freeze and advances again on a resume.
            tps = abs(n.q()[0] - q0[0]) / span
            f.write(json.dumps({"t": time.time(), "elapsed": round(el, 3),
                                "tps": round(tps, 4),
                                "pan": round(n.q()[0], 4)}) + "\n")
            f.flush()
            if el > a.watch:
                status = "watch_timeout"
                break
        if rf.done():
            res = rf.result()
            code = res.result.error_code if res else None
            status = f"finished(code={code})"
        f.write(json.dumps({"t": time.time(), "elapsed": round(time.time() - t0, 3),
                            "done": True, "status": status}) + "\n")
    print(f"mover: {status}", flush=True)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
