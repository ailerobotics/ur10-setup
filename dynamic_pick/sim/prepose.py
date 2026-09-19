#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Sim bootstrap: park the SIMULATED arm at the measured-limits reference pose (the
window centre) so the dynamic-pick controller starts from an IN-WINDOW pose, the
way the real cell always starts (parked inside its measured joint windows).

The real UR10 in this cell begins every run parked well inside
`cumotion/measured_joint_limits.yaml`; the mock/URSim arm boots at a factory home
pose that is OUTSIDE those cell windows (e.g. wrist_1), so the window-guarded
planner correctly refuses to plan a leg from it. This one-shot move places the sim
arm at the reference pose (guaranteed in-window: it is the window centre). It is a
SIM SETUP step only — the analog of URSim's power-on/brake-release — and it is run
AFTER the gate has reached RUN so even this bootstrap motion happens under a RUN
gate. It is NOT part of the dynamic-pick control loop and never runs on real
hardware.

Run (task domain 83):
    python3 prepose.py --ros-args -p limits:=cumotion/measured_joint_limits.yaml
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
VEL = {'shoulder_pan_joint': 2.09, 'shoulder_lift_joint': 2.09, 'elbow_joint': 3.14,
       'wrist_1_joint': 3.14, 'wrist_2_joint': 3.14, 'wrist_3_joint': 3.14}
SPEED = 0.05
FJT = '/scaled_joint_trajectory_controller/follow_joint_trajectory'


class Prepose(Node):
    def __init__(self):
        super().__init__("dp_prepose")
        self.limits = str(self.declare_parameter("limits", "cumotion/measured_joint_limits.yaml").value)
        # bootstrap speed (setup only, not a pick-controller motion). Higher than the
        # 5 % pick cap so the one-off ~6 rad wrist move to the measured branch is a
        # few seconds, not ~40 s.
        self.speed = float(self.declare_parameter("speed", 0.3).value)
        # park in the MEASURED branch (do NOT wrap to the arm's home branch): cuMotion
        # validates the start config against its URDF/XRDF in the measured joint
        # branch, so the arm must sit there (wrist_1 ~4.459 rad, not the mock home's
        # -1.57). The analytic planner is branch-agnostic (it wraps goals to nearest),
        # so this is safe for every rig.
        self.wrap = bool(self.declare_parameter("wrap_to_home_branch", False).value)
        self.js = {}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.js.update(zip(m.name, m.position)), 20)
        self.fjt = ActionClient(self, FollowJointTrajectory, FJT)

    def q(self):
        return np.array([self.js.get(j, 0.0) for j in JOINTS])


def main():
    rclpy.init()
    n = Prepose()
    t0 = time.time()
    while time.time() - t0 < 10 and len(n.js) < 6:
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(n.js) < 6:
        print("prepose: no /joint_states", file=sys.stderr); return 1
    ref = yaml.safe_load(open(n.limits))["reference_pose_rad"]
    goal = np.array([ref[j] for j in JOINTS])
    q0 = n.q()
    if n.wrap:
        # optional: command the reference on the branch nearest the arm's current
        # pose (shortest travel). NOT used for cuMotion (see __init__).
        goal = q0 + ((goal - q0 + math.pi) % (2 * math.pi) - math.pi)
    delta = goal - q0
    vmax = np.array([VEL[j] for j in JOINTS])
    dur = max(float(np.max(np.abs(delta) / (n.speed * vmax))), 1.0)
    traj = JointTrajectory(); traj.joint_names = JOINTS
    n_pts = max(4, int(dur / 0.5))
    for k in range(n_pts + 1):
        frac = k / n_pts
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in (q0 + delta * frac)]
        if k in (0, n_pts):
            pt.velocities = [0.0] * 6
        if k == n_pts:
            pt.accelerations = [0.0] * 6
        t = frac * dur
        pt.time_from_start.sec = int(t); pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        traj.points.append(pt)
    traj.header.stamp.sec = 0; traj.header.stamp.nanosec = 0
    if not n.fjt.wait_for_server(timeout_sec=15.0):
        print("prepose: FJT unavailable", file=sys.stderr); return 1
    g = FollowJointTrajectory.Goal(); g.trajectory = traj
    fut = n.fjt.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("prepose: goal rejected", file=sys.stderr); return 1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=max(30.0, dur + 10))
    err = float(np.degrees(np.abs(n.q() - goal).max()))
    print(f"prepose: parked at reference pose, {dur:.1f} s, final error {err:.2f} deg")
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
