#!/usr/bin/env python3
"""
Eye-in-hand cardboard approach demo v1 (host side).

State machine:  DETECT -> COMPUTE -> CONFIRM -> APPROACH -> HOLD -> CONFIRM
                -> RETREAT, one single motion per leg.

Safety model (read cams/approach/README.md before running):
  * --dry-run is the DEFAULT: plans both legs against the live MoveIt and
    prints full summaries, moves NOTHING. Real motion needs explicit
    --execute AND the literal word `yes` + Enter at each CONFIRM gate.
  * Velocity and acceleration scaling are hard-capped at SPEED_CAP = 0.05
    in code; --speed can only go lower. The pendant slider caps on top.
  * Approach is PURE TRANSLATION: tool0 keeps its start orientation; the
    tool translates so the CAMERA stops --standoff (>= 0.25 m, refused
    below) short of the target along the camera->target ray.
  * Workspace whitelist boxes in base_link: targets and approach poses
    outside them are REFUSED, never clamped (derivation in README).
  * Aborts (no motion / stop planning) on: stale TF (> 1 s), target moved
    > 5 cm since the plan, planning failure, execution feedback timeout.

Motion path reuses the repo's proven conventions (motion_test.py, RUNBOOK):
plan via MoveIt's /move_action (same action RViz Plan uses; plan_only, so
planning never moves the arm), execute the planned trajectory through
/scaled_joint_trajectory_controller/follow_joint_trajectory with the same
accept/result/feedback checks; preflight mirrors RUNBOOK §3 (program
running, speed scaling > 0) and points at §4 (rearm watchdog) on rejection.

Usage:
  source /opt/ros/jazzy/setup.bash
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4        # domain 0 is the default
  python3 cams/approach/approach_demo.py                 # dry-run (default)
  python3 cams/approach/approach_demo.py --execute       # captain only
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped support)
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             RobotState, WorkspaceParameters)
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

SPEED_CAP = 0.05          # hard cap, velocity AND acceleration scaling
STANDOFF_FLOOR = 0.25     # m; --standoff below this is refused
TF_MAX_AGE_S = 1.0        # dynamic TF older than this aborts
TARGET_DRIFT_M = 0.05     # target moved more than this since plan => abort
FEEDBACK_TIMEOUT_S = 3.0  # no controller feedback for this long => abort
GROUP = 'ur_manipulator'
BASE = 'base_link'
TOOL = 'tool0'

# UR10 rated working radius is 1.30 m around the shoulder (~0.128 m above
# base_link). Goals beyond REACH_MAX from there cannot be sampled by IK —
# abort with a useful message instead of a generic planner FAILURE. A deep
# target is approached by RAISING --standoff (larger is always safer).
SHOULDER = np.array([0.0, 0.0, 0.128])
REACH_MAX = 1.25

# A pure translation must not rewind the arm. Bench finding 2026-08-18: OMPL
# RRTConnect happily returned a valid-but-wild plan (shoulder_lift +405 deg,
# wrist_2 +272 deg) for a 0.59 m translation because goal IK sampled a distant
# configuration. Hence (a) the default pipeline is Pilz LIN — a straight
# Cartesian line holding orientation, exactly the demo's contract — and
# (b) ANY plan whose largest per-joint excursion exceeds this is REFUSED.
MAX_JOINT_DELTA_DEG = 170.0

# ---------------------------------------------------------------------------
# Workspace whitelist (base_link frame, metres). REFUSE outside, never clamp.
# Derivation (measured live 2026-08-18, parked shelf-facing pose — README):
#   tool0 at (0.065, 0.248, 1.156), camera looking along base_link +Y at the
#   gondola; the nearest cardboard front face (blue yogurt tray, lower
#   board) measured at base_link (-0.022, +1.516, +0.855).
# TARGET_BOX: the shelf volume where believable cardboard sits — front faces
#   from y=1.40 back to 2.40 (rack depth), z from just under the lower pick
#   surface (~0.83) to above the top board.
# APPROACH_BOX: where tool0/the camera may be sent — y <= 1.35 keeps >= 0.15 m
#   clear of the nearest measured front face (1.516) and fully off the shelf
#   plane; z >= 0.80 stays above everything below the lower pick surface;
#   z <= 1.55 stays under the top-shelf overhang. The 1.40/1.35 gap means a
#   pose can never be both a target and an approach point.
TARGET_BOX = {'x': (-0.80, 1.00), 'y': (1.40, 2.40), 'z': (0.55, 1.70)}
APPROACH_BOX = {'x': (-0.60, 0.80), 'y': (-0.10, 1.35), 'z': (0.80, 1.55)}
# ---------------------------------------------------------------------------


class Abort(Exception):
    pass


def in_box(p, box):
    return all(box[k][0] <= v <= box[k][1]
               for k, v in zip(('x', 'y', 'z'), p))


def fmt(p):
    return '({:+.3f}, {:+.3f}, {:+.3f})'.format(*p)


class ApproachDemo(Node):
    def __init__(self, args):
        super().__init__('approach_demo')
        self.args = args
        self.joints = None            # dict name -> pos, plus recv time
        self.joints_t = 0.0
        self.program_running = None
        self.speed_scaling = None
        self.targets = []             # recent PointStamped from the bridge
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joints, 10)
        self.create_subscription(PointStamped, self.args.topic,
                                 self._on_target, 10)
        self.create_subscription(
            Bool, '/io_and_status_controller/robot_program_running',
            self._on_prog, 10)
        self.create_subscription(
            Float64, '/speed_scaling_state_broadcaster/speed_scaling',
            self._on_scale, 10)
        self.move_ac = ActionClient(self, MoveGroup, '/move_action')
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.traj_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def _on_joints(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.joints = d
            self.joints_t = time.time()

    def _on_target(self, m):
        self.targets.append((time.time(), m))
        self.targets = self.targets[-20:]

    def _on_prog(self, m):
        self.program_running = m.data

    def _on_scale(self, m):
        self.speed_scaling = m.data

    # -- plumbing -----------------------------------------------------------
    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def spin_until(self, pred, timeout, what):
        end = time.time() + timeout
        while not pred():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > end:
                raise Abort(f'timeout waiting for {what}')

    def lookup(self, target_frame, source_frame):
        """Latest TF, aborting if the dynamic chain is older than 1 s."""
        try:
            tf = self.tf_buf.lookup_transform(target_frame, source_frame,
                                              Time())
        except tf2_ros.TransformException as e:
            raise Abort(f'TF {target_frame}<-{source_frame} unavailable: {e}')
        stamp = Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds > 0:   # static TFs carry stamp 0 — no age
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if age > TF_MAX_AGE_S:
                raise Abort(f'TF {target_frame}<-{source_frame} stale '
                            f'({age:.2f} s > {TF_MAX_AGE_S} s)')
        return tf

    # -- preflight (RUNBOOK §3 conventions) ---------------------------------
    def preflight(self):
        self.spin_until(lambda: self.joints is not None, 10, '/joint_states')
        self.spin_for(2.0)   # give latched/slow topics a chance
        age = time.time() - self.joints_t
        if age > TF_MAX_AGE_S:
            raise Abort(f'/joint_states stale ({age:.1f} s)')
        probs = []
        if self.program_running is not True:
            probs.append(f'robot_program_running={self.program_running} '
                         '(RUNBOOK §4: rearm)')
        if not self.speed_scaling or self.speed_scaling <= 0:
            probs.append(f'speed_scaling={self.speed_scaling} '
                         '(RUNBOOK §6: set_speed_slider)')
        if probs:
            if self.args.execute:
                raise Abort('preflight failed: ' + '; '.join(probs))
            print('  [dry-run] preflight warnings: ' + '; '.join(probs))
        print(f'  joints fresh ({age:.2f} s), program_running='
              f'{self.program_running}, speed_scaling={self.speed_scaling}')

    # -- DETECT -------------------------------------------------------------
    def detect(self):
        n = self.args.detections
        print(f'[DETECT] waiting for {n} valid targets on {self.args.topic} '
              '(bridge publishes only when stable + arm still) ...')
        t0 = time.time()
        self.targets.clear()
        self.spin_until(lambda: len(self.targets) >= n,
                        self.args.detect_timeout, 'valid targets')
        recent = self.targets[-n:]
        now = time.time()
        if now - recent[0][0] > 20.0:
            raise Abort('target stream too slow/stale')
        pts = np.array([[m.point.x, m.point.y, m.point.z]
                        for _, m in recent])
        spread = np.linalg.norm(pts - pts.mean(axis=0), axis=1).max()
        if spread > 0.03:
            raise Abort(f'target spread {spread * 100:.1f} cm > 3 cm')
        ps = PointStamped()
        ps.header = recent[-1][1].header
        ps.point.x, ps.point.y, ps.point.z = pts.mean(axis=0).tolist()
        print(f'  target (camera {ps.header.frame_id}): '
              f'{fmt(pts.mean(axis=0))}  spread {spread * 100:.1f} cm '
              f'({time.time() - t0:.1f} s)')
        return ps

    # -- COMPUTE ------------------------------------------------------------
    def compute(self, target_cam):
        cam_frame = target_cam.header.frame_id
        tf_cam = self.lookup(BASE, cam_frame)
        tf_tool = self.lookup(BASE, TOOL)
        target_cam.header.stamp = tf_cam.header.stamp
        tb = tf2_geometry_msgs.do_transform_point(target_cam, tf_cam)
        target = np.array([tb.point.x, tb.point.y, tb.point.z])
        cam = np.array([tf_cam.transform.translation.x,
                        tf_cam.transform.translation.y,
                        tf_cam.transform.translation.z])
        tool = np.array([tf_tool.transform.translation.x,
                         tf_tool.transform.translation.y,
                         tf_tool.transform.translation.z])
        quat = tf_tool.transform.rotation      # kept EXACTLY (pure translation)

        ray = target - cam
        dist = np.linalg.norm(ray)
        if dist < self.args.standoff + 0.05:
            raise Abort(f'target only {dist:.2f} m away — already closer '
                        f'than standoff {self.args.standoff:.2f} m + margin')
        ray_u = ray / dist
        cam_dest = target - self.args.standoff * ray_u
        delta = cam_dest - cam
        tool_goal = tool + delta

        print(f'[COMPUTE] target  {BASE}: {fmt(target)}')
        print(f'          camera  now:    {fmt(cam)}   tool0 now: {fmt(tool)}')
        print(f'          camera  dest:   {fmt(cam_dest)}  '
              f'(standoff {self.args.standoff:.2f} m along ray, '
              f'ray len {dist:.2f} m)')
        print(f'          tool0   goal:   {fmt(tool_goal)}  '
              f'(pure translation {np.linalg.norm(delta):.3f} m)')

        for name, p, box in (('target', target, TARGET_BOX),
                             ('camera destination', cam_dest, APPROACH_BOX),
                             ('tool0 goal', tool_goal, APPROACH_BOX)):
            if not in_box(p, box):
                raise Abort(f'{name} {fmt(p)} outside whitelist box {box} — '
                            'REFUSED (bounds are never clamped)')
        print('          bounds: target + approach inside whitelist boxes')
        radial = np.linalg.norm(tool_goal - SHOULDER)
        if radial > REACH_MAX:
            raise Abort(
                f'tool0 goal is {radial:.2f} m from the shoulder — beyond '
                f'the {REACH_MAX:.2f} m reachable envelope. Deep target: '
                f'raise --standoff by at least {radial - REACH_MAX:.2f} m '
                '(larger standoff = closer goal = always safer) or '
                'reposition the arm first')
        print(f'          reach:  tool0 goal {radial:.2f} m from shoulder '
              f'(<= {REACH_MAX:.2f} m)')
        return {'target': target, 'tool_goal': tool_goal, 'quat': quat,
                'delta': delta}

    # -- planning (MoveIt /move_action, plan_only) --------------------------
    def _base_request(self):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = self.args.pipeline
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        speed = min(self.args.speed, SPEED_CAP)
        req.max_velocity_scaling_factor = speed
        req.max_acceleration_scaling_factor = speed
        wp = WorkspaceParameters()
        wp.header.frame_id = BASE
        wp.min_corner.x, wp.min_corner.y, wp.min_corner.z = (
            APPROACH_BOX['x'][0], APPROACH_BOX['y'][0], APPROACH_BOX['z'][0])
        wp.max_corner.x, wp.max_corner.y, wp.max_corner.z = (
            APPROACH_BOX['x'][1], APPROACH_BOX['y'][1], APPROACH_BOX['z'][1])
        req.workspace_parameters = wp
        return req

    def _plan(self, req, leg):
        if req.pipeline_id == 'pilz_industrial_motion_planner':
            # LIN = straight Cartesian line for the pose goal; PTP = direct
            # joint-space move for the joint-goal retreat.
            req.planner_id = ('PTP' if req.goal_constraints
                              and req.goal_constraints[0].joint_constraints
                              else 'LIN')
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = True    # NEVER let MoveIt execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        if not self.move_ac.wait_for_server(timeout_sec=10):
            raise Abort('/move_action server unavailable (ur10-moveit down?)')
        send = self.move_ac.send_goal_async(goal)
        self.spin_until(lambda: send.done(), 15, 'plan goal ack')
        gh = send.result()
        if gh is None or not gh.accepted:
            raise Abort(f'{leg}: plan goal rejected')
        res_fut = gh.get_result_async()
        self.spin_until(lambda: res_fut.done(), 30, 'plan result')
        res = res_fut.result().result
        if res.error_code.val != 1:
            raise Abort(f'{leg}: planning FAILED (MoveItErrorCode '
                        f'{res.error_code.val})')
        traj = res.planned_trajectory.joint_trajectory
        if not traj.points:
            raise Abort(f'{leg}: planner returned empty trajectory')
        deltas = np.degrees(np.abs(
            np.array(traj.points[-1].positions)
            - np.array(traj.points[0].positions)))
        if deltas.max() > MAX_JOINT_DELTA_DEG:
            worst = traj.joint_names[int(deltas.argmax())]
            raise Abort(
                f'{leg}: plan rewinds {worst} by {deltas.max():.0f} deg '
                f'(> {MAX_JOINT_DELTA_DEG:.0f}) — REFUSED. The planner '
                'sampled a distant IK configuration; use the default Pilz '
                'LIN pipeline or reposition the arm')
        return traj

    def solve_ik(self, comp):
        """IK for the approach pose, seeded with the CURRENT joints so the
        solver returns the nearby configuration — never a rewound one.
        (Bench 2026-08-18: pose-goal planning let the goal-state sampler
        pick distant IK configurations — OMPL then planned +405 deg
        shoulder rewinds, Pilz LIN just failed. Explicit seeded IK + a
        joint-space goal keeps the motion boring, which is the point.)"""
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise Abort('/compute_ik unavailable (ur10-moveit down?)')
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = TOOL
        req.ik_request.avoid_collisions = True
        seed = RobotState()
        seed.joint_state.name = JOINTS
        seed.joint_state.position = [self.joints[j] for j in JOINTS]
        req.ik_request.robot_state = seed
        ps = PoseStamped()
        ps.header.frame_id = BASE
        (ps.pose.position.x, ps.pose.position.y,
         ps.pose.position.z) = comp['tool_goal'].tolist()
        ps.pose.orientation = comp['quat']        # unchanged start orientation
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        self.spin_until(lambda: fut.done(), 15, 'IK solution')
        res = fut.result()
        if res.error_code.val != 1:
            raise Abort(f'IK failed for the approach pose (MoveItErrorCode '
                        f'{res.error_code.val}) — pose likely at the edge of '
                        'the envelope; raise --standoff')
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        goal_joints = [sol[j] for j in JOINTS]
        deltas = np.degrees(np.abs(np.array(goal_joints)
                                   - np.array(seed.joint_state.position)))
        if deltas.max() > MAX_JOINT_DELTA_DEG:
            worst = JOINTS[int(deltas.argmax())]
            raise Abort(f'IK solution rewinds {worst} by {deltas.max():.0f} '
                        f'deg (> {MAX_JOINT_DELTA_DEG:.0f}) — REFUSED; '
                        'reposition the arm or adjust --standoff')
        print(f'  IK: nearby configuration found (max joint delta '
              f'{deltas.max():.1f} deg)')
        return goal_joints

    def plan_pose(self, comp):
        goal_joints = self.solve_ik(comp)
        return self.plan_joints(goal_joints, leg='APPROACH')

    def plan_joints(self, goal_joints, start_joints=None, leg='RETREAT'):
        req = self._base_request()
        if start_joints is not None:
            rs = RobotState()
            rs.joint_state.name = JOINTS
            rs.joint_state.position = list(start_joints)
            req.start_state = rs
        else:
            req.start_state.is_diff = True    # plan from the live state
        cons = Constraints()
        for j, pos in zip(JOINTS, goal_joints):
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = pos
            jc.tolerance_above = jc.tolerance_below = 0.005
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        req.goal_constraints.append(cons)
        return self._plan(req, leg)

    # -- summaries + gates --------------------------------------------------
    def summarize(self, traj, leg, comp=None):
        first = np.array(traj.points[0].positions)
        last = np.array(traj.points[-1].positions)
        dur = traj.points[-1].time_from_start
        dur_s = dur.sec + dur.nanosec / 1e9
        print(f'--- {leg} PLAN SUMMARY '.ljust(60, '-'))
        if comp is not None:
            print(f'  target ({BASE}):     {fmt(comp["target"])}')
            print(f'  approach tool0 goal: {fmt(comp["tool_goal"])}  '
                  '(orientation: unchanged)')
            print(f'  cartesian travel:    '
                  f'{np.linalg.norm(comp["delta"]):.3f} m')
        print(f'  planner pipeline:    {self.args.pipeline}   '
              f'vel/acc scaling: {min(self.args.speed, SPEED_CAP):.3f}')
        print(f'  trajectory:          {len(traj.points)} points, '
              f'{dur_s:.1f} s')
        print('  per-joint deltas (deg):')
        for j, a, b in zip(traj.joint_names, first, last):
            print(f'    {j:22s} {math.degrees(a):+8.2f} -> '
                  f'{math.degrees(b):+8.2f}  (d {math.degrees(b - a):+6.2f})')
        print('-' * 60)

    def confirm(self, what):
        if not self.args.execute:
            print(f'[CONFIRM] dry-run: NOT executing {what} '
                  '(pass --execute for the real gate)')
            return False
        ans = input(f'[CONFIRM] type exactly "yes" + Enter to EXECUTE {what} '
                    '(anything else aborts): ')
        if ans.strip() != 'yes':
            raise Abort(f'user declined {what} (typed {ans!r})')
        return True

    # -- EXECUTE (captain only; the demo never reaches this in dry-run) -----
    def check_target_drift(self, comp):
        self.targets.clear()
        self.spin_for(5.0)
        if not self.targets:
            raise Abort('target stream went silent before execution')
        m = self.targets[-1][1]
        ps = PointStamped()
        ps.header.frame_id = m.header.frame_id
        ps.point = m.point
        tf_cam = self.lookup(BASE, m.header.frame_id)
        ps.header.stamp = tf_cam.header.stamp
        now_b = tf2_geometry_msgs.do_transform_point(ps, tf_cam)
        drift = np.linalg.norm(
            np.array([now_b.point.x, now_b.point.y, now_b.point.z])
            - comp['target'])
        if drift > TARGET_DRIFT_M:
            raise Abort(f'target moved {drift * 100:.1f} cm > '
                        f'{TARGET_DRIFT_M * 100:.0f} cm since plan')
        print(f'  target drift since plan: {drift * 100:.1f} cm (ok)')

    def execute_traj(self, traj, leg):
        """motion_test.py conventions: accept check, feedback watchdog,
        error_code==0, verify with live /joint_states afterwards."""
        self.preflight()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        if not self.traj_ac.wait_for_server(timeout_sec=10):
            raise Abort('trajectory action server unavailable')
        last_fb = {'t': time.time()}

        def fb_cb(_):
            last_fb['t'] = time.time()

        send = self.traj_ac.send_goal_async(goal, feedback_callback=fb_cb)
        self.spin_until(lambda: send.done(), 10, 'execute goal ack')
        gh = send.result()
        if gh is None or not gh.accepted:
            raise Abort(f'{leg}: goal REJECTED by controller (RUNBOOK §4: '
                        'external control likely down; let ur10-rearm fix it)')
        print(f'  {leg}: goal accepted, executing ...')
        dur = traj.points[-1].time_from_start
        deadline = time.time() + dur.sec + dur.nanosec / 1e9 + 10
        res_fut = gh.get_result_async()
        while not res_fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - last_fb['t'] > FEEDBACK_TIMEOUT_S:
                gh.cancel_goal_async()
                raise Abort(f'{leg}: no controller feedback for '
                            f'{FEEDBACK_TIMEOUT_S} s — goal cancelled')
            if time.time() > deadline:
                gh.cancel_goal_async()
                raise Abort(f'{leg}: execution overran plan duration — '
                            'goal cancelled')
        err = res_fut.result().result.error_code
        if err != 0:
            raise Abort(f'{leg}: controller error_code={err}')
        print(f'  {leg}: SUCCEEDED')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--standoff', type=float, default=0.30,
                    help='stop this short of the target (m, floor 0.25)')
    ap.add_argument('--speed', type=float, default=SPEED_CAP,
                    help=f'vel+acc scaling, hard-capped at {SPEED_CAP}')
    ap.add_argument('--execute', action='store_true',
                    help='REAL MOTION (captain only). Default is --dry-run.')
    ap.add_argument('--dry-run', action='store_true',
                    help='plan only (the default; flag kept for clarity)')
    ap.add_argument('--hold', type=float, default=3.0,
                    help='seconds to hold at the approach pose')
    ap.add_argument('--detections', type=int, default=5,
                    help='valid bridge targets required')
    ap.add_argument('--detect-timeout', type=float, default=120.0)
    ap.add_argument('--topic', default='/approach/target_camera')
    ap.add_argument('--pipeline', default='pilz_industrial_motion_planner',
                    choices=['pilz_industrial_motion_planner', 'ompl',
                             'chomp', 'isaac_ros_cumotion'],
                    help='pilz (LIN/PTP) default: predictable straight-line '
                         'motion; others may need the joint-delta guard')
    args = ap.parse_args()

    if args.standoff < STANDOFF_FLOOR:
        sys.exit(f'REFUSED: --standoff {args.standoff} < hard floor '
                 f'{STANDOFF_FLOOR} m (never clamped)')
    if args.speed > SPEED_CAP:
        print(f'--speed {args.speed} above hard cap; using {SPEED_CAP}')
    if args.execute and args.dry_run:
        sys.exit('choose one of --execute / --dry-run')

    rclpy.init()
    node = ApproachDemo(args)
    mode = 'EXECUTE (real motion)' if args.execute else 'DRY-RUN (plan only)'
    print(f'=== approach_demo v1 — {mode} ===')
    try:
        print('[PREFLIGHT]')
        node.preflight()
        start_joints = [node.joints[j] for j in JOINTS]
        print('  recorded start joints (deg): '
              + ', '.join(f'{math.degrees(p):+.2f}' for p in start_joints))

        target_cam = node.detect()
        comp = node.compute(target_cam)

        traj = node.plan_pose(comp)
        node.summarize(traj, 'APPROACH', comp)

        if node.confirm('APPROACH'):
            node.check_target_drift(comp)
            node.execute_traj(traj, 'APPROACH')
            print(f'[HOLD] {args.hold:.1f} s at approach pose')
            node.spin_for(args.hold)
            retreat = node.plan_joints(start_joints)
            node.summarize(retreat, 'RETREAT')
            if node.confirm('RETREAT'):
                node.execute_traj(retreat, 'RETREAT')
                node.spin_for(1.0)
                back = np.array([node.joints[j] for j in JOINTS])
                err = np.abs(back - np.array(start_joints)).max()
                print(f'  back at start, max joint error '
                      f'{math.degrees(err):.3f} deg')
        else:
            # dry-run: validate the retreat leg too, from the approach end
            end_state = list(traj.points[-1].positions)
            # trajectory joint order may differ from JOINTS; remap
            order = [traj.joint_names.index(j) for j in JOINTS]
            retreat = node.plan_joints(start_joints,
                                       start_joints=[end_state[i]
                                                     for i in order])
            node.summarize(retreat, 'RETREAT (from planned approach end)')
            print('[DONE] dry-run complete: both legs plan clean, '
                  'nothing moved')
    except Abort as e:
        print(f'\nABORT (safe, no motion commanded beyond this point): {e}')
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
