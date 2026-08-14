#!/usr/bin/env python3
"""Minimal UR10 motion test: wrist_3 +0.05 rad and back, via the
scaled_joint_trajectory_controller action interface. Verifies with live
/joint_states feedback that the arm moved and returned."""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
DELTA = 0.05          # rad on wrist_3 (~2.9 deg)
SEG_T = 4             # seconds per segment (slow)


class MotionTest(Node):
    def __init__(self):
        super().__init__("ur10_motion_test")
        self.positions = None
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 1)

    def cb(self, msg):
        # /joint_states name order is alphabetical; remap to JOINTS order
        m = dict(zip(msg.name, msg.position))
        if all(j in m for j in JOINTS):
            self.positions = [m[j] for j in JOINTS]


def wait_positions(node, timeout=10.0):
    node.positions = None
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while node.positions is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds > end:
            raise TimeoutError("no /joint_states")
    return list(node.positions)


def main():
    rclpy.init()
    node = MotionTest()
    start = wait_positions(node)
    print(f"START wrist_3 = {start[5]:+.5f} rad")

    target = list(start)
    target[5] += DELTA

    traj = JointTrajectory()
    traj.joint_names = JOINTS
    p1 = JointTrajectoryPoint(positions=target,
                              time_from_start=Duration(sec=SEG_T))
    p2 = JointTrajectoryPoint(positions=start,
                              time_from_start=Duration(sec=2 * SEG_T))
    traj.points = [p1, p2]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    ac = ActionClient(node, FollowJointTrajectory,
                      "/scaled_joint_trajectory_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10):
        print("FAIL: action server unavailable"); sys.exit(2)

    peak = {"v": start[5]}

    def fb_cb(fb):
        try:
            i = fb.feedback.joint_names.index("wrist_3_joint")
            v = fb.feedback.actual.positions[i]
            if abs(v - start[5]) > abs(peak["v"] - start[5]):
                peak["v"] = v
        except (ValueError, IndexError):
            pass

    send = ac.send_goal_async(goal, feedback_callback=fb_cb)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10)
    gh = send.result()
    if gh is None or not gh.accepted:
        print("FAIL: goal rejected"); sys.exit(2)
    print("goal accepted, executing...")

    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=3 * SEG_T + 10)
    res = res_fut.result()
    if res is None:
        print("FAIL: no result"); sys.exit(2)
    err = res.result.error_code
    print(f"result error_code = {err} ({'SUCCESSFUL' if err == 0 else 'ERROR'})")

    final = wait_positions(node)
    moved = abs(peak["v"] - start[5])
    returned = abs(final[5] - start[5])
    print(f"peak excursion   = {moved:.5f} rad ({math.degrees(moved):.2f} deg)")
    print(f"final vs start   = {returned:.6f} rad")

    ok = err == 0 and moved > 0.03 and returned < 0.005
    print("MOTION TEST " + ("PASSED" if ok else "FAILED"))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
