#!/usr/bin/env python3
"""First-light motion test: wrist_3 +0.1 rad and back, via scaled_joint_trajectory_controller.

Reads one live JointState (names paired with positions, so ordering is explicit),
sends a two-point FollowJointTrajectory goal: +0.1 rad on wrist_3 at t=4s, back to
the start pose at t=8s. All other joints commanded to hold their current positions.
"""
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
NUDGE = 0.10  # rad on wrist_3


class Nudge(Node):
    def __init__(self):
        super().__init__("wrist3_nudge")
        self.start = None
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)

    def cb(self, msg):
        if self.start is None and all(j in msg.name for j in UR_JOINTS):
            self.start = {n: p for n, p in zip(msg.name, msg.position)}


def main():
    rclpy.init()
    node = Nudge()
    while node.start is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    start = node.start
    print("start pose:", {j: round(start[j], 4) for j in UR_JOINTS}, flush=True)

    client = ActionClient(node, FollowJointTrajectory,
                          "/scaled_joint_trajectory_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=10.0):
        print("FAIL: action server not available", flush=True)
        sys.exit(1)

    traj = JointTrajectory()
    traj.joint_names = UR_JOINTS
    out = JointTrajectoryPoint()
    out.positions = [start[j] + (NUDGE if j == "wrist_3_joint" else 0.0) for j in UR_JOINTS]
    out.time_from_start = Duration(sec=4)
    back = JointTrajectoryPoint()
    back.positions = [start[j] for j in UR_JOINTS]
    back.time_from_start = Duration(sec=8)
    traj.points = [out, back]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    print("sending goal: wrist_3 +%.2f rad @4s, back @8s ..." % NUDGE, flush=True)
    gh_f = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh_f, timeout_sec=10.0)
    gh = gh_f.result()
    if gh is None or not gh.accepted:
        print("FAIL: goal not accepted", flush=True)
        sys.exit(1)
    print("goal accepted, executing...", flush=True)

    res_f = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_f, timeout_sec=75.0)
    res = res_f.result()
    if res is None:
        print("FAIL: no result within 75s", flush=True)
        sys.exit(1)
    code = res.result.error_code
    print("result: error_code=%d (%s)" % (code, "SUCCESSFUL" if code == 0 else res.result.error_string), flush=True)

    node.start = None
    while node.start is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    end = node.start
    drift = max(abs(end[j] - start[j]) for j in UR_JOINTS)
    print("end pose:  ", {j: round(end[j], 4) for j in UR_JOINTS}, flush=True)
    print("max |end-start| across joints: %.5f rad" % drift, flush=True)
    sys.exit(0 if code == 0 else 2)


if __name__ == "__main__":
    main()
