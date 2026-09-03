#!/usr/bin/env python3
"""
Plan-only test against the running planner (host, domain 0). NEVER moves the
arm: every request goes through MoveIt's /move_action with plan_only=True,
exactly like cams/approach/approach_demo.py's dry run. Execution is a
separate, captain-confirmed tool (see README "arm-execute").

What it does:
  * start = live joints (default) or --start-joints (plan a virtual A->B leg
    without moving the arm to A first);
  * goal  = --goal-joints, or --goal-pose x,y,z [+ --goal-rpy] resolved with
    seeded /compute_ik and the approach demo's 170 deg rewind guard;
  * pipeline isaac_ros_cumotion (default) or ompl/pilz for comparison;
  * FK of every trajectory point via /compute_fk -> tool0 polyline;
  * writes plans/<label>.yaml (joints, tool path, length, duration, planner)
    and, with --plot label1 label2 ..., a top-down + side figure comparing
    saved plans (e.g. "with-box" vs "box-removed"), optionally with the box
    outline --box xmin,xmax,ymin,ymax,zmin,zmax.

The MoveIt DisplayTrajectory on /display_planned_path is published by
move_group for every plan, so the MotionPlanning display in the ur10-moveit
RViz (RUNBOOK 8b) shows the last plan for screenshots.

  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 plan_test.py --label with-box \
      --start-joints ... --goal-joints ... --pipeline isaac_ros_cumotion
"""
import argparse
import datetime as dt
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MotionPlanRequest,
                             OrientationConstraint, PositionConstraint, RobotState)
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
GROUP, BASE, TOOL = 'ur_manipulator', 'base_link', 'tool0'
SPEED_CAP = 0.05
MAX_JOINT_DELTA_DEG = 170.0


class PlanTest(Node):
    def __init__(self, a):
        super().__init__('cellmap_plan_test')
        self.a = a
        self.joints = None
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.joints = [d[j] for j in JOINTS]

    def wait(self, pred, timeout, what):
        t0 = time.time()
        while not pred():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                sys.exit(f'timeout waiting for {what}')

    def call(self, cli, req, what, timeout=30):
        if not cli.wait_for_service(timeout_sec=10):
            sys.exit(f'{cli.srv_name} unavailable (ur10-moveit down?)')
        fut = cli.call_async(req)
        self.wait(fut.done, timeout, what)
        return fut.result()

    def fk_points(self, names, positions_list):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = [TOOL]
        out = []
        for pos in positions_list:
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(names)
            req.robot_state.joint_state.position = list(pos)
            res = self.call(self.fk, req, 'FK')
            if res.error_code.val != 1:
                sys.exit(f'FK failed ({res.error_code.val})')
            p = res.pose_stamped[0].pose.position
            out.append([p.x, p.y, p.z])
        return np.array(out)

    def sphere_floor(self, joints):
        """Lowest (z - radius - buffer) over the XRDF collision spheres at this
        configuration: negative = the arm dives below the mounting plane.
        Needed because MoveIt's IK (empty planning scene) happily returns
        elbow-DOWN branches for low tool poses on a table-mounted arm."""
        if not hasattr(self, '_spheres'):
            x = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', '..', '..',
                                                 'cumotion', 'ur10_map.xrdf')))
            self._spheres = x['geometry']['ur10e_collision_spheres']['spheres']
            self._buf = x['collision']['buffer_distance']
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = list(self._spheres.keys())
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(joints)
        res = self.call(self.fk, req, 'FK spheres')
        floor = 9.0
        for ps, link in zip(res.pose_stamped, res.fk_link_names):
            p, q = ps.pose.position, ps.pose.orientation
            Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            for sp in self._spheres[link]:
                c = np.array([p.x, p.y, p.z]) + Rm @ np.array(sp['center'])
                floor = min(floor, c[2] - sp['radius'] - self._buf[link])
        return floor

    def sweep_clearance(self, names, joint_path, box_str):
        """Min over trajectory points and XRDF spheres (+buffer) of the distance
        to an axis-aligned box in base_link. Negative = the ARM (not just the
        tool point) passes through the box. Ground truth for the synthetic
        rehearsal; for the real cell the box corners come from the map."""
        self.sphere_floor(joint_path[0])          # loads spheres/buffers
        b = [float(v) for v in box_str.split(',')]
        lo, hi = np.array(b[0::2]), np.array(b[1::2])
        best = {'min_clearance_m': 9.0, 'link': '', 'point': -1}
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = list(self._spheres.keys())
        for i, js in enumerate(joint_path):
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(names)
            req.robot_state.joint_state.position = list(js)
            res = self.call(self.fk, req, 'FK sweep')
            for ps, link in zip(res.pose_stamped, res.fk_link_names):
                p, q = ps.pose.position, ps.pose.orientation
                Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                for sp in self._spheres[link]:
                    c = np.array([p.x, p.y, p.z]) + Rm @ np.array(sp['center'])
                    d = np.linalg.norm(np.maximum(np.maximum(lo - c, 0), c - hi))
                    clr = float(d - sp['radius'] - self._buf[link])
                    if clr < best['min_clearance_m']:
                        best = {'min_clearance_m': clr, 'link': link, 'point': i}
        best['box'] = b
        return best

    def ik_joints(self, xyz, rpy_deg, seed):
        """Seeded IK; --seed-tries perturbed seeds, keep the solution with the
        smallest joint delta whose spheres stay >= --floor above the plane."""
        rng = np.random.default_rng(0)
        best = None
        for k in range(max(1, self.a.seed_tries)):
            s = np.array(seed) if k == 0 else np.array(seed) + rng.uniform(-1.2, 1.2, 6)
            sol = self._ik_once(xyz, rpy_deg, s.tolist(), seed)
            if sol is None:
                continue
            floor = self.sphere_floor(sol)
            delta = np.degrees(np.abs(np.array(sol) - np.array(seed))).max()
            if floor < self.a.floor:
                continue
            if best is None or delta < best[1]:
                best = (sol, delta, floor)
        if best is None:
            sys.exit(f'no IK solution for {xyz} keeps the arm above the mounting plane '
                     f'(tried {self.a.seed_tries} seeds) — choose a higher/closer pose')
        sol, delta, floor = best
        print(f'IK ok (max joint delta {delta:.1f} deg, lowest sphere {floor*100:+.1f} cm '
              f'above the plane): {np.round(sol, 3).tolist()}')
        return sol

    def _ik_once(self, xyz, rpy_deg, seed, ref):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = TOOL
        req.ik_request.avoid_collisions = True
        rs = RobotState()
        rs.joint_state.name = JOINTS
        rs.joint_state.position = list(seed)
        req.ik_request.robot_state = rs
        ps = PoseStamped()
        ps.header.frame_id = BASE
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
        if rpy_deg is None:
            # keep the seed's orientation
            fkp = self.call(self.fk, self._fk_req(seed), 'FK seed')
            ps.pose.orientation = fkp.pose_stamped[0].pose.orientation
        else:
            q = R.from_euler('xyz', rpy_deg, degrees=True).as_quat()
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = q
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 2
        res = self.call(self.ik, req, 'IK')
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        goal = [sol[j] for j in JOINTS]
        d = np.degrees(np.abs(np.array(goal) - np.array(ref)))
        if d.max() > MAX_JOINT_DELTA_DEG:
            return None                       # rewound branch — refused
        return goal

    def _fk_req(self, joints):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = [TOOL]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(joints)
        return req

    def pose_constraints(self, xyz, quat_xyzw):
        """tool0 pose goal: cuMotion then solves IK against ITS world (map +
        ground plane + buffers), instead of us handing it a joint config that
        MoveIt's empty-scene IK considered fine but the map does not."""
        cons = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = BASE
        pc.link_name = TOOL
        pc.weight = 1.0
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.005]
        bv.primitives.append(sp)
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        oc = OrientationConstraint()
        oc.header.frame_id = BASE
        oc.link_name = TOOL
        (oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w) = quat_xyzw
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = \
            oc.absolute_z_axis_tolerance = 0.02
        oc.weight = 1.0
        cons.position_constraints.append(pc)
        cons.orientation_constraints.append(oc)
        return cons

    def plan(self, start, goal, pose_goal=None):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = self.a.pipeline
        if self.a.pipeline == 'pilz_industrial_motion_planner':
            req.planner_id = 'PTP'
        elif self.a.planner_id:
            req.planner_id = self.a.planner_id
        req.num_planning_attempts = 3
        req.allowed_planning_time = self.a.time
        req.max_velocity_scaling_factor = SPEED_CAP
        req.max_acceleration_scaling_factor = SPEED_CAP
        rs = RobotState()
        rs.joint_state.name = JOINTS
        rs.joint_state.position = list(start)
        req.start_state = rs
        if pose_goal is not None:
            req.goal_constraints.append(self.pose_constraints(*pose_goal))
        else:
            cons = Constraints()
            for j, p in zip(JOINTS, goal):
                jc = JointConstraint()
                jc.joint_name, jc.position = j, float(p)
                jc.tolerance_above = jc.tolerance_below = 0.005
                jc.weight = 1.0
                cons.joint_constraints.append(jc)
            req.goal_constraints.append(cons)
        g = MoveGroup.Goal()
        g.request = req
        g.planning_options.plan_only = True          # never executes
        g.planning_options.look_around = False
        g.planning_options.replan = False
        if not self.move.wait_for_server(timeout_sec=10):
            sys.exit('/move_action unavailable (ur10-moveit down?)')
        t0 = time.time()
        send = self.move.send_goal_async(g)
        self.wait(send.done, 15, 'goal ack')
        gh = send.result()
        if gh is None or not gh.accepted:
            sys.exit('plan goal rejected')
        rf = gh.get_result_async()
        self.wait(rf.done, self.a.time + 60, 'plan result')
        res = rf.result().result
        dur = time.time() - t0
        return res, dur


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--label', default='plan')
    ap.add_argument('--pipeline', default='isaac_ros_cumotion',
                    help='isaac_ros_cumotion | ompl | pilz_industrial_motion_planner')
    ap.add_argument('--planner-id', default='')
    ap.add_argument('--time', type=float, default=10.0, help='allowed planning time s')
    ap.add_argument('--start-joints', help='6 rad, comma-separated (default: live)')
    ap.add_argument('--goal-joints', help='6 rad, comma-separated')
    ap.add_argument('--goal-pose', help='x,y,z m in base_link (orientation of the start)')
    ap.add_argument('--seed-tries', type=int, default=25)
    ap.add_argument('--floor', type=float, default=0.0,
                    help='m: every collision sphere (+buffer) must stay this far above z=0')
    ap.add_argument('--goal-delta', help='6 joint deltas rad added to the start (smoke test)')
    ap.add_argument('--goal-rpy', help='roll,pitch,yaw deg for --goal-pose')
    ap.add_argument('--pose-goal', action='store_true',
                    help='send --goal-pose as a POSE constraint (planner does IK against '
                         'its world) instead of pre-solving IK here')
    ap.add_argument('--start-pose', help='x,y,z m: IK from the live joints -> virtual start')
    ap.add_argument('--start-from', help='label of a saved plan: its END joints become the '
                    'start (a configuration cuMotion itself found collision-free)')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'plans'))
    ap.add_argument('--plot', nargs='*', help='labels to plot (no planning)')
    ap.add_argument('--check-box', help='xmin,xmax,ymin,ymax,zmin,zmax: sweep ALL collision '
                    'spheres (+buffer) of the saved plans (--recheck labels) or of this plan '
                    'through the trajectory and report the minimum clearance to that box')
    ap.add_argument('--recheck', nargs='*', help='labels to re-evaluate with --check-box')
    ap.add_argument('--box', help='xmin,xmax,ymin,ymax,zmin,zmax obstacle outline for the plot')
    ap.add_argument('--plot-out', default=None)
    ap.add_argument('--joint-limits', default=None,
                    help='pneumatic_limits.yaml: refuse any plan whose waypoints leave '
                         'the per-joint windows (independent of the planner\'s own limits)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    if a.plot is not None:
        return plot(a)
    if a.recheck is not None:
        rclpy.init()
        node = PlanTest(a)
        for lab in a.recheck:
            p = os.path.join(a.out, f'{lab}.yaml')
            rec = yaml.safe_load(open(p))
            if 'joint_path' not in rec:
                print(f'{lab}: no trajectory')
                continue
            c = node.sweep_clearance(rec['joint_names'], rec['joint_path'], a.check_box)
            rec['box_check'] = c
            with open(p, 'w') as f:
                yaml.safe_dump(rec, f, sort_keys=False)
            print(f"{lab}: min arm clearance to box {c['min_clearance_m']*100:+.1f} cm "
                  f"({c['link']} at point {c['point']}/{len(rec['joint_path'])}) -> "
                  f"{'SWEEPS THROUGH THE BOX' if c['min_clearance_m'] < 0 else 'clear'}")
        node.destroy_node()
        rclpy.shutdown()
        return

    rclpy.init()
    node = PlanTest(a)
    node.wait(lambda: node.joints is not None, 10, '/joint_states')
    live = list(node.joints)
    if a.start_from:
        prev = yaml.safe_load(open(os.path.join(a.out, f'{a.start_from}.yaml')))
        start = [float(v) for v in prev['joint_path'][-1]]
    else:
        start = ([float(v) for v in a.start_joints.split(',')] if a.start_joints
                 else node.ik_joints([float(v) for v in a.start_pose.split(',')], None, live)
                 if a.start_pose else live)
    pose_goal = None
    if a.goal_pose and a.pose_goal:
        xyz = [float(v) for v in a.goal_pose.split(',')]
        if a.goal_rpy:
            q = R.from_euler('xyz', [float(v) for v in a.goal_rpy.split(',')], degrees=True).as_quat()
        else:
            o = node.call(node.fk, node._fk_req(start), 'FK start').pose_stamped[0].pose.orientation
            q = [o.x, o.y, o.z, o.w]
        pose_goal = (xyz, [float(v) for v in q])
        goal = None
    elif a.goal_joints:
        goal = [float(v) for v in a.goal_joints.split(',')]
    elif a.goal_delta:
        goal = [s0 + float(d) for s0, d in zip(start, a.goal_delta.split(','))]
    elif a.goal_pose:
        rpy = [float(v) for v in a.goal_rpy.split(',')] if a.goal_rpy else None
        goal = node.ik_joints([float(v) for v in a.goal_pose.split(',')], rpy, start)
    else:
        sys.exit('--goal-joints or --goal-pose required')
    if pose_goal is not None:
        sp = node.fk_points(JOINTS, [start])[0]
        gp = np.array(pose_goal[0])
    else:
        sp, gp = node.fk_points(JOINTS, [start, goal])
    print(f'{a.label}: {a.pipeline}{" pose-goal" if pose_goal else ""}  start tool0 '
          f'{np.round(sp, 3).tolist()} -> goal tool0 {np.round(gp, 3).tolist()}  '
          f'(straight line {np.linalg.norm(gp-sp):.3f} m)')
    res, wall = node.plan(start, goal, pose_goal)
    rec = {'label': a.label, 'pipeline': a.pipeline, 'planner_id': a.planner_id,
           'when': dt.datetime.now().isoformat(timespec='seconds'),
           'error_code': int(res.error_code.val), 'planning_time_s': float(res.planning_time),
           'wall_s': float(wall), 'start_joints': [float(v) for v in start],
           'goal_joints': [float(v) for v in goal] if goal is not None else None,
           'pose_goal': [list(map(float, pose_goal[0])), list(map(float, pose_goal[1]))]
           if pose_goal else None,
           'start_tool0': sp.tolist(), 'goal_tool0': gp.tolist(),
           'straight_line_m': float(np.linalg.norm(gp - sp))}
    if res.error_code.val != 1:
        print(f'  planning FAILED: MoveItErrorCode {res.error_code.val} '
              f'(99999 = cuMotion: start/goal in collision incl. buffers, self-collision, or no path — see the planner log; -31 = no IK)')
    else:
        traj = res.planned_trajectory.joint_trajectory
        pts = [list(p.positions) for p in traj.points]
        dj = np.degrees(np.abs(np.array(pts[-1]) - np.array(pts[0])))
        if dj.max() > MAX_JOINT_DELTA_DEG:
            print(f'  REFUSED: plan rewinds {traj.joint_names[int(dj.argmax())]} by '
                  f'{dj.max():.0f} deg (> {MAX_JOINT_DELTA_DEG:.0f}) — distant IK branch')
            rec['error_code'] = -170
            pts = []
    if res.error_code.val == 1 and pts and a.joint_limits:
        # SECOND, INDEPENDENT GUARD on the pneumatic-line windows. The planner
        # already cannot exceed them (cuMotion reads the reduced limits from
        # ur10_pneumatic.urdf), but "the config we think we launched" and "the
        # config the planner actually used" are different claims, and on a real
        # arm the difference is the whole point. This re-checks EVERY waypoint
        # against the yaml and refuses the plan on any violation.
        import yaml as _y
        _lim = _y.safe_load(open(a.joint_limits))
        _ref, _win = _lim['reference_pose_rad'], _lim['window_deg']
        viol = []
        for k, wp in enumerate(pts):
            for jn, v in zip(traj.joint_names, wp):
                if jn not in _ref:
                    continue
                half = np.radians(_win[jn])
                lo, hi = _ref[jn] - half, _ref[jn] + half
                if not (lo <= v <= hi):
                    viol.append((k, jn, v, lo, hi))
        if viol:
            k, jn, v, lo, hi = viol[0]
            print(f'  REFUSED: waypoint {k} has {jn} = {v:.4f} rad outside the '
                  f'pneumatic window [{lo:.4f}, {hi:.4f}] ({len(viol)} violations)')
            rec['error_code'] = -171
            rec['limit_violations'] = len(viol)
            pts = []
        else:
            margins = {}
            for jn in traj.joint_names:
                if jn not in _ref:
                    continue
                col = [wp[list(traj.joint_names).index(jn)] for wp in pts]
                half = np.radians(_win[jn])
                lo, hi = _ref[jn] - half, _ref[jn] + half
                margins[jn] = float(np.degrees(min(min(c - lo for c in col),
                                                   min(hi - c for c in col))))
            rec['pneumatic_margin_deg'] = margins
            worst = min(margins, key=margins.get)
            print(f'  pneumatic limits OK: every waypoint inside; tightest margin '
                  f'{worst} {margins[worst]:.1f} deg from its window edge')

    if res.error_code.val == 1 and pts:
        path = node.fk_points(traj.joint_names, pts)
        seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
        tf = traj.points[-1].time_from_start
        gp = path[-1]
        rec['goal_tool0'] = gp.tolist()
        rec.update({'points': len(pts), 'duration_s': tf.sec + tf.nanosec * 1e-9,
                    'max_joint_delta_deg': float(dj.max()),
                    'tool_path_m': float(seg.sum()),
                    'detour_ratio': float(seg.sum() / max(np.linalg.norm(gp - sp), 1e-6)),
                    'max_dev_from_line_m': float(max_dev(path, sp, gp)),
                    'joint_names': list(traj.joint_names), 'joint_path': pts,
                    'tool_path': path.tolist()})
        print(f'  OK: {len(pts)} points, {rec["duration_s"]:.1f} s at {SPEED_CAP} scaling, '
              f'tool path {rec["tool_path_m"]:.3f} m = {rec["detour_ratio"]:.2f}x straight line, '
              f'max deviation from the line {rec["max_dev_from_line_m"]*100:.1f} cm, '
              f'planning {res.planning_time:.2f} s')
    p = os.path.join(a.out, f'{a.label}.yaml')
    with open(p, 'w') as f:
        yaml.safe_dump(rec, f, sort_keys=False)
    print('  ->', p)
    node.destroy_node()
    rclpy.shutdown()


def max_dev(path, sp, gp):
    d = gp - sp
    n = np.linalg.norm(d)
    if n < 1e-6:
        return float(np.linalg.norm(path - sp, axis=1).max())
    u = d / n
    v = path - sp
    return float(np.linalg.norm(v - np.outer(v @ u, u), axis=1).max())


def plot(a):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, (ax, az) = plt.subplots(1, 2, figsize=(12, 5.5))
    colors = ['#0e7d86', '#b23a3a', '#a06c14', '#2f7d4f', '#555']
    for i, lab in enumerate(a.plot):
        rec = yaml.safe_load(open(os.path.join(a.out, f'{lab}.yaml')))
        if 'tool_path' not in rec:
            print(f'{lab}: no trajectory (error {rec["error_code"]})')
            continue
        P = np.array(rec['tool_path'])
        lbl = (f'{lab} ({rec["pipeline"]}): {rec["tool_path_m"]:.2f} m, '
               f'{rec["detour_ratio"]:.2f}x, dev {rec["max_dev_from_line_m"]*100:.0f} cm')
        if 'box_check' in rec:
            m = rec['box_check']['min_clearance_m']
            lbl += (f', arm {"THROUGH box" if m < 0 else "clear"} ({m*100:+.0f} cm)')
        ax.plot(P[:, 0], P[:, 1], '-', color=colors[i % 5], lw=2, label=lbl)
        az.plot(P[:, 1], P[:, 2], '-', color=colors[i % 5], lw=2)
        for A in (ax,):
            A.plot(P[0, 0], P[0, 1], 'o', color=colors[i % 5])
            A.plot(P[-1, 0], P[-1, 1], 'x', color=colors[i % 5], mew=2)
        az.plot(P[0, 1], P[0, 2], 'o', color=colors[i % 5])
        az.plot(P[-1, 1], P[-1, 2], 'x', color=colors[i % 5], mew=2)
        sp, gp = np.array(rec['start_tool0']), np.array(rec['goal_tool0'])
        ax.plot([sp[0], gp[0]], [sp[1], gp[1]], ':', color='#999', lw=1)
        az.plot([sp[1], gp[1]], [sp[2], gp[2]], ':', color='#999', lw=1)
    if a.box:
        b = [float(v) for v in a.box.split(',')]
        ax.add_patch(Rectangle((b[0], b[2]), b[1]-b[0], b[3]-b[2], fill=True,
                               alpha=.25, color='#a06c14', label='box (measured)'))
        az.add_patch(Rectangle((b[2], b[4]), b[3]-b[2], b[5]-b[4], fill=True,
                               alpha=.25, color='#a06c14'))
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(fontsize=8, loc='best')
    ax.set_xlabel('x [m] base_link'); ax.set_ylabel('y [m]'); ax.set_title('tool0 path, top-down')
    az.set_aspect('equal'); az.grid(alpha=.3)
    az.set_xlabel('y [m]'); az.set_ylabel('z [m]'); az.set_title('tool0 path, side (y-z)')
    fig.tight_layout()
    out = a.plot_out or os.path.join(a.out, 'plans-' + '-vs-'.join(a.plot) + '.png')
    fig.savefig(out, dpi=130)
    print('plot ->', out)


if __name__ == '__main__':
    main()
