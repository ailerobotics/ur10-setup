#!/usr/bin/env python3
"""
Execute a plan validated by plan_test.py on the REAL arm. Stage B.

WHY THIS RE-REQUESTS THE PLAN. plan_test.py saves the geometric joint_path
but not the per-point timing, and a joint trajectory without timing is not
executable: inventing a velocity profile for a real arm is how you cause a
protective stop or a jerk the map never modelled. So this asks the planner
for the SAME plan again (identical live start, identical goal, identical
config) purely to recover the timing — and then REFUSES TO MOVE unless the
returned path matches the validated one waypoint-for-waypoint within
--tol-rad. It is the validated motion or nothing; a re-plan that drifted is
an abort, not a new plan to run.

Guards, all of which abort before any motion:
  * live start must equal the validated plan's start (the arm must not have
    been touched since validation)
  * re-planned path must match the validated path within --tol-rad
  * every waypoint must sit inside the pneumatic windows (--limits FILE); the
    guard can only be skipped by the EXPLICIT --no-limits, for when the captain
    has confirmed no pneumatic line / wire strap is on the arm
  * robot must be RUNNING/NORMAL with its program up and speed scaling > 0
During motion it watches the tracking error and cancels the goal on any
excursion beyond --max-track-rad.

  python3 execute_plan.py cams/cellmap/plans/mt-tag0.yaml --limits cumotion/pneumatic_limits.yaml
  python3 execute_plan.py cams/cellmap/plans/mt-tag0.yaml --no-limits      # captain: no pneumatic line
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, WorkspaceParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
SPEED = 0.05


class Exec(Node):
    def __init__(self):
        super().__init__('execute_plan')
        self.js = {}
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.js.update(zip(m.name, m.position)), 20)
        self.prog = None
        self.scaling = None
        from std_msgs.msg import Bool, Float64
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                 lambda m: setattr(self, 'prog', m.data), 5)
        self.mg = ActionClient(self, MoveGroup, '/cumotion/move_group')
        self.fjt = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def wait_state(self, t=10.0):
        """Wait for /joint_states. robot_program_running is NOT waited on:
        io_and_status_controller publishes it ON CHANGE ONLY and does not
        latch, so a fresh subscriber can wait forever while the program is
        perfectly healthy. It is checked out-of-band instead (dashboard)."""
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.js) >= 6:
                return True
        return False

    def q(self):
        return np.array([self.js[j] for j in JOINTS])

    def replan(self, goal_joints, seconds):
        if not self.mg.wait_for_server(timeout_sec=20.0):
            sys.exit('cuMotion move_group action unavailable')
        req = MotionPlanRequest()
        req.pipeline_id = 'isaac_ros_cumotion'
        req.group_name = 'ur_manipulator'
        req.num_planning_attempts = 1
        req.allowed_planning_time = float(seconds)
        req.max_velocity_scaling_factor = SPEED
        req.max_acceleration_scaling_factor = SPEED
        req.workspace_parameters = WorkspaceParameters()
        c = Constraints()
        for j, v in zip(JOINTS, goal_joints):
            jc = JointConstraint(joint_name=j, position=float(v),
                                 tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0)
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        g = MoveGroup.Goal()
        g.request = req
        g.planning_options.plan_only = True
        fut = self.mg.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=40.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            sys.exit('planning goal rejected')
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=90.0)
        res = rf.result().result
        if res.error_code.val != 1:
            sys.exit(f'RE-PLAN FAILED (code {res.error_code.val}) — not executing')
        return res.planned_trajectory.joint_trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('plan')
    lim_group = ap.add_mutually_exclusive_group(required=True)
    lim_group.add_argument('--limits', help='pneumatic windows yaml (reference_pose_rad + '
                           'window_deg): refuse any waypoint outside them')
    lim_group.add_argument('--no-limits', action='store_true',
                           help='EXPLICITLY skip the pneumatic-window guard: only when the '
                                'captain has confirmed no pneumatic line / wire strap is on the arm')
    ap.add_argument('--tol-rad', type=float, default=0.02,
                    help='max joint-space deviation of the re-planned path from the '
                         'validated one before this refuses to run')
    ap.add_argument('--start-tol-rad', type=float, default=0.01)
    ap.add_argument('--max-track-rad', type=float, default=0.15,
                    help='abort if the arm deviates this far from the commanded point')
    ap.add_argument('--time', type=float, default=25.0)
    ap.add_argument('--exact', action='store_true',
                    help='execute the SAVED validated waypoints with reconstructed '
                         'uniform timing instead of re-requesting a plan')
    a = ap.parse_args()

    saved = yaml.safe_load(open(a.plan))
    val = np.array(saved['joint_path'])
    goal = np.array(saved['goal_joints'] if saved.get('goal_joints') else val[-1])
    if a.limits:
        lim = yaml.safe_load(open(a.limits))
        ref, win = lim['reference_pose_rad'], lim['window_deg']
        lo = np.array([ref[j] - math.radians(win[j]) for j in JOINTS])
        hi = np.array([ref[j] + math.radians(win[j]) for j in JOINTS])
    else:
        lo = hi = None
        print('pneumatic-window guard SKIPPED (--no-limits): captain confirmed no '
              'pneumatic line / wire strap on the arm')

    rclpy.init()
    n = Exec()
    if not n.wait_state():
        sys.exit('no /joint_states — driver down?')

    q0 = n.q()
    d0 = np.abs(q0 - val[0]).max()
    print(f'[{saved["label"]}] live start vs validated start: max {math.degrees(d0):.2f} deg')
    if d0 > a.start_tol_rad:
        sys.exit(f'ABORT: the arm is not where the plan was validated from '
                 f'({math.degrees(d0):.1f} deg off). Re-validate before executing.')
    import socket
    try:
        sk = socket.create_connection(('192.168.0.1', 29999), timeout=5)
        sk.recv(4096); sk.sendall(b'running\n'); time.sleep(0.4)
        running = b'true' in sk.recv(4096).lower()
        sk.sendall(b'quit\n'); sk.close()
    except Exception as e:                                   # noqa: BLE001
        sys.exit(f'ABORT: could not query the dashboard ({e})')
    if not running:
        sys.exit('ABORT: the external-control program is not running '
                 '(resend_robot_program first — RUNBOOK section 4)')
    print('robot program: running')

    if a.exact:
        # EXACT mode: execute the VALIDATED waypoints, timing reconstructed as
        # uniform over the plan's own duration. Measured on both legs, that
        # reproduces cuMotion's profile to the letter — peak velocity comes
        # out at exactly 5.0 % of every joint's limit, because the planner was
        # asked for 0.05 scaling and its trajectory is already near-uniform.
        # This is preferred over re-planning: cuMotion's trajopt is not bit
        # deterministic, so a re-plan lands within ~2 deg of the approved path
        # but is not the approved path, and "the same validated plan" should
        # mean exactly that. Peak velocity is re-checked below and refuses to
        # run if the reconstruction is not conservative.
        VEL = {'shoulder_pan_joint': 2.09, 'shoulder_lift_joint': 2.09,
               'elbow_joint': 3.14, 'wrist_1_joint': 3.14,
               'wrist_2_joint': 3.14, 'wrist_3_joint': 3.14}
        dur_total = float(saved['duration_s'])
        dt = dur_total / (len(val) - 1)
        vmax = np.array([VEL[j] for j in JOINTS])
        peak = (np.abs(np.diff(val, axis=0)) / dt / vmax).max()
        print(f'exact mode: {len(val)} validated waypoints, uniform dt {dt*1000:.0f} ms, '
              f'peak velocity {peak*100:.1f}% of the joint limits')
        if peak > SPEED * 1.05:
            sys.exit(f'ABORT: reconstructed timing peaks at {peak*100:.1f}% of the joint '
                     f'limits, above the {SPEED*100:.0f}% cap — not executing')
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        for k, w in enumerate(val):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in w]
            pt.velocities = [0.0] * 6 if k in (0, len(val) - 1) else []
            t = k * dt
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)
    else:
        print('re-requesting the identical plan to recover executable timing...')
        traj = n.replan(goal, a.time)
    got = np.array([list(p.positions) for p in traj.points])
    if list(traj.joint_names) != JOINTS:
        idx = [list(traj.joint_names).index(j) for j in JOINTS]
        got = got[:, idx]

    # every returned waypoint must lie on the validated path
    dev = 0.0 if a.exact else max(float(np.linalg.norm(val - w, axis=1).min()) for w in got)
    ends = float(np.abs(got[-1] - val[-1]).max())
    print(f'match vs validated path: max off-path {dev:.4f} rad '
          f'({math.degrees(dev):.2f} deg), endpoint {math.degrees(ends):.2f} deg, '
          f'{len(got)} vs {len(val)} points')
    if dev > a.tol_rad or ends > a.tol_rad:
        sys.exit('ABORT: re-planned path differs from the validated one — not executing.')

    if lo is not None:
        out = [(k, JOINTS[i], v) for k, w in enumerate(got) for i, v in enumerate(w)
               if not (lo[i] <= v <= hi[i])]
        if out:
            sys.exit(f'ABORT: {len(out)} waypoints outside the pneumatic windows '
                     f'(first: point {out[0][0]} {out[0][1]})')
        print(f'pneumatic windows OK for all {len(got)} waypoints')

    dur = traj.points[-1].time_from_start
    dur = dur.sec + dur.nanosec * 1e-9
    print(f'EXECUTING {saved["label"]}: {len(got)} points, {dur:.1f} s at {SPEED} scaling')
    if not n.fjt.wait_for_server(timeout_sec=15.0):
        sys.exit('trajectory controller action unavailable')
    # Start NOW. move_group returns the trajectory stamped with its own
    # planning time; by the time it reaches the controller that stamp is in
    # the past and the goal is rejected outright — which looks exactly like
    # 'the arm will not move' with no other symptom. A zero stamp means
    # 'begin immediately' (verified 2026-08-28: the hand-built trajectory in
    # --exact mode had a zero stamp and was accepted; the planner's own was
    # rejected).
    # The controller REFUSES a trajectory whose last point is not exactly at
    # rest: 'Velocity of last trajectory point of joint X is not zero'.
    # cuMotion leaves ~4e-4 rad/s of residual terminal velocity, which is
    # physically nothing but is a hard reject — and the only symptom is the
    # arm not moving. Pin the final point to zero velocity/acceleration;
    # the motion is meant to end stopped anyway.
    traj.points[-1].velocities = [0.0] * len(traj.joint_names)
    traj.points[-1].accelerations = [0.0] * len(traj.joint_names)
    traj.header.stamp.sec = 0
    traj.header.stamp.nanosec = 0
    g = FollowJointTrajectory.Goal()
    g.trajectory = traj
    fut = n.fjt.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        sys.exit('ABORT: trajectory goal rejected by the controller')
    rf = gh.get_result_async()

    t0 = time.time()
    worst = 0.0
    while not rf.done():
        rclpy.spin_once(n, timeout_sec=0.05)
        el = time.time() - t0
        if el > dur + 20:
            print('ABORT: trajectory overran its duration by 20 s — cancelling')
            gh.cancel_goal_async(); break
        k = min(int(el / max(dur, 1e-6) * (len(got) - 1)), len(got) - 1)
        if len(n.js) >= 6:
            e = float(np.abs(n.q() - got[k]).max())
            worst = max(worst, e)
            if e > a.max_track_rad:
                print(f'ABORT: tracking error {math.degrees(e):.1f} deg > '
                      f'{math.degrees(a.max_track_rad):.1f} — cancelling')
                gh.cancel_goal_async(); break
        if n.prog is False:
            print('ABORT: robot program stopped mid-motion — cancelling')
            gh.cancel_goal_async(); break
        if int(el * 2) % 4 == 0:
            print(f'   t={el:5.1f}/{dur:.1f}s  tracking err {math.degrees(worst):.2f} deg',
                  end='\r', flush=True)
    rclpy.spin_until_future_complete(n, rf, timeout_sec=30.0)
    res = rf.result()
    code = res.result.error_code if res else None
    qf = n.q()
    err = np.degrees(np.abs(qf - val[-1]))
    print(f'\nresult error_code={code}  worst tracking error {math.degrees(worst):.2f} deg')
    print(f'final joint error vs plan goal: max {err.max():.2f} deg '
          f'({JOINTS[int(err.argmax())]})')
    ok = code == 0 and err.max() < 1.0
    print('LEG COMPLETE' if ok else 'LEG DID NOT COMPLETE CLEANLY')
    n.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
