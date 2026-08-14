#!/usr/bin/env python3
"""Clearly visible motion test: wrist_1 +0.15 rad (~8.6 deg) and back.

Unlike motion_test.py (wrist_3 = flange rotation, invisible without a tool
mounted), wrist_1 tilts the whole wrist assembly — unmistakable from across
the room. An independent /joint_states recorder tracks every joint's min/max
during execution, so the verdict comes from the robot's encoders, not from
the action result alone."""
import math
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
MOVE_JOINT = "wrist_1_joint"
DELTA = 0.15          # rad (~8.6 deg) — clearly visible on the wrist block
SEG_T = 5             # seconds per segment (slow)


class Recorder(Node):
    """Independent watcher: min/max of every joint straight from /joint_states."""

    def __init__(self):
        super().__init__("motion_recorder")
        self.lock = threading.Lock()
        self.minmax = {}
        self.latest = None
        self.samples = 0
        self.create_subscription(JointState, "/joint_states", self.cb, 10)

    def cb(self, msg):
        with self.lock:
            self.latest = dict(zip(msg.name, msg.position))
            self.samples += 1
            for n, p in self.latest.items():
                lo, hi = self.minmax.get(n, (p, p))
                self.minmax[n] = (min(lo, p), max(hi, p))


def main():
    rclpy.init()
    rec = Recorder()

    # settle: need a first full reading
    while rec.latest is None or not all(j in rec.latest for j in JOINTS):
        rclpy.spin_once(rec, timeout_sec=0.2)
    start = {j: rec.latest[j] for j in JOINTS}
    with rec.lock:            # reset stats so min/max cover only the test
        rec.minmax = {}
        rec.samples = 0
    print(f"START {MOVE_JOINT} = {start[MOVE_JOINT]:+.5f} rad", flush=True)
    print(f"commanding {MOVE_JOINT} {DELTA:+.2f} rad (~{math.degrees(DELTA):.1f} deg), "
          f"{SEG_T}s out, {SEG_T}s back — WATCH THE WRIST", flush=True)

    traj = JointTrajectory()
    traj.joint_names = JOINTS
    target = [start[j] + (DELTA if j == MOVE_JOINT else 0.0) for j in JOINTS]
    home = [start[j] for j in JOINTS]
    traj.points = [
        JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=SEG_T)),
        JointTrajectoryPoint(positions=home, time_from_start=Duration(sec=2 * SEG_T)),
    ]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    ac = ActionClient(rec, FollowJointTrajectory,
                      "/scaled_joint_trajectory_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10):
        print("FAIL: action server unavailable", flush=True); sys.exit(2)

    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(rec, send, timeout_sec=10)
    gh = send.result()
    if gh is None or not gh.accepted:
        print("FAIL: goal rejected", flush=True); sys.exit(2)

    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(rec, res_fut, timeout_sec=3 * SEG_T + 15)
    res = res_fut.result()
    if res is None:
        print("FAIL: no result", flush=True); sys.exit(2)
    err = res.result.error_code
    print(f"action result: error_code={err} ({'SUCCESSFUL' if err == 0 else 'ERROR'})", flush=True)

    # let the last samples land
    for _ in range(30):
        rclpy.spin_once(rec, timeout_sec=0.05)

    print(f"\nencoder evidence ({rec.samples} samples @125 Hz during test):", flush=True)
    verdict_move = False
    with rec.lock:
        for j in JOINTS:
            lo, hi = rec.minmax.get(j, (float("nan"), float("nan")))
            sweep = hi - lo
            mark = ""
            if j == MOVE_JOINT:
                verdict_move = sweep > 0.10
                mark = "  <-- commanded"
            print(f"  {j:22s} sweep {math.degrees(sweep):6.2f} deg{mark}", flush=True)
        final = rec.latest[MOVE_JOINT]
    ret = abs(final - start[MOVE_JOINT])
    print(f"return |final-start| = {ret:.5f} rad", flush=True)

    ok = err == 0 and verdict_move and ret < 0.01
    print("\nVISIBLE MOTION TEST " + ("PASSED — the arm physically moved"
                                      if ok else "FAILED"), flush=True)
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
