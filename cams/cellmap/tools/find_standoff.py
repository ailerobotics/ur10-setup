#!/usr/bin/env python3
"""
Find a JOINT configuration, inside the pneumatic windows, that parks the
camera at a standoff above a mapped AprilTag.

Why not just give cuMotion a Cartesian pose goal: with wrist_2 limited to
+-30 deg and wrist_3 to +-20 deg the reachable ORIENTATION family is narrow,
so almost any full 6-DoF pose is unreachable and cuMotion returns
INVERSE_KINEMATICS_FAILURE — which says nothing about whether a perfectly
good standoff exists. Searching in joint space instead keeps the limits
satisfied by construction and lets the orientation land wherever the windows
allow, scoring it on what actually matters: where the CAMERA is and whether
it looks at the tag.

Scores each sample on
  - distance from the camera to the desired standoff point above the tag
  - angle between the camera's optical axis and the direction to the tag
and reports the best configuration found. Uses /compute_fk, so it needs the
driver up; it commands nothing.

  python3 find_standoff.py --tag -0.402,-0.765,-0.053 --standoff 0.25
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class FK(Node):
    def __init__(self):
        super().__init__('find_standoff')
        self.cli = self.create_client(GetPositionFK, '/compute_fk')
        if not self.cli.wait_for_service(timeout_sec=15.0):
            sys.exit('/compute_fk unavailable (is MoveIt up?)')

    def fk(self, batch, link):
        """batch: list of 6-vectors -> list of (pos, R) for `link`."""
        out = []
        for q in batch:
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = [link]
            rs = RobotState()
            rs.joint_state = JointState(name=JOINTS, position=[float(v) for v in q])
            req.robot_state = rs
            fut = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            r = fut.result()
            if r is None or not r.pose_stamped:
                out.append(None)
                continue
            p = r.pose_stamped[0].pose
            from scipy.spatial.transform import Rotation as R
            out.append((np.array([p.position.x, p.position.y, p.position.z]),
                        R.from_quat([p.orientation.x, p.orientation.y,
                                     p.orientation.z, p.orientation.w]).as_matrix()))
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', required=True, help='x,y,z of the tag centre in base_link')
    ap.add_argument('--standoff', type=float, default=0.25,
                    help='m from the target along --dir')
    ap.add_argument('--dir', default='0,0,1',
                    help='unit direction from the target to the standoff point. '
                         '0,0,1 = above it (a tag flat on the table); 0,-1,0 = in front '
                         'of it toward the robot (a shelf face)')
    ap.add_argument('--limits', default=os.path.join(os.path.dirname(__file__),
                                                     '..', '..', '..', 'cumotion',
                                                     'pneumatic_limits.yaml'))
    ap.add_argument('--link', default='tool0')
    ap.add_argument('--samples', type=int, default=4000)
    ap.add_argument('--refine', type=int, default=600)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--inset-deg', type=float, default=2.0,
                    help='shrink every window by this before searching, so no solution '
                         'rides a limit edge')
    ap.add_argument('--min-height', type=float, default=None,
                    help='m above the tag that the link must stay (default: --standoff)')
    a = ap.parse_args()

    lim = yaml.safe_load(open(a.limits))
    ref, win = lim['reference_pose_rad'], lim['window_deg']
    # INSET the windows before searching. A solution that lands exactly on a
    # window edge is rejected by cuMotion as "exceeds limits" — the search
    # clips to [lo, hi] but floating point puts the value a fraction outside,
    # and the planner is (rightly) not forgiving about a safety limit. The
    # inset also leaves the executed trajectory a real margin rather than
    # riding the boundary.
    inset = np.radians(a.inset_deg)
    lo = np.array([ref[j] - np.radians(win[j]) for j in JOINTS]) + inset
    hi = np.array([ref[j] + np.radians(win[j]) for j in JOINTS]) - inset
    tag = np.array([float(v) for v in a.tag.split(',')])
    if a.min_height is None:
        a.min_height = a.standoff
    dvec = np.array([float(v) for v in a.dir.split(',')], float)
    dvec /= np.linalg.norm(dvec)
    target = tag + dvec * a.standoff

    rclpy.init()
    n = FK()
    rng = np.random.default_rng(a.seed)

    def score(batch):
        res = n.fk(batch, a.link)
        out = []
        for q, r in zip(batch, res):
            if r is None:
                out.append((1e9, None, None))
                continue
            p, R = r
            d = float(np.linalg.norm(p - target))
            axis = R[:, 2]                       # tool0 +Z, the camera's view direction
            to_tag = tag - p
            to_tag /= max(np.linalg.norm(to_tag), 1e-9)
            ang = float(np.degrees(np.arccos(np.clip(axis @ to_tag, -1, 1))))
            # HARD floor: never propose a standoff that dips towards the table.
            # Without this the search happily returns a configuration 12 cm
            # above the table surface — geometrically close to the tag, but
            # the 0.04 m world buffer then puts it in collision and the plan
            # is refused. Staying at or above the requested standoff height is
            # what "safe standoff" means; being a bit far is fine, being low
            # is not.
            # Keep the link on the standoff SIDE of the target: never let the
            # search creep past the surface it is approaching.
            along = float((p - tag) @ dvec)
            pen = 0.0 if along >= a.min_height else 10.0 * (a.min_height - along)
            out.append((d + 0.004 * ang + pen, d, ang))   # 1 cm ~ 2.5 deg of aim
        return out

    print(f'searching {a.samples} configs inside the pneumatic windows '
          f'for a {a.standoff:.2f} m standoff above {tag}')
    best = None
    B = 250
    for i in range(0, a.samples, B):
        batch = rng.uniform(lo, hi, size=(min(B, a.samples - i), 6))
        for q, (s, d, ang) in zip(batch, score(list(batch))):
            if best is None or s < best[0]:
                best = (s, q.copy(), d, ang)
        print(f'  {i+B:5d}/{a.samples}  best: {best[2]*100:.1f} cm from the standoff '
              f'point, aim {best[3]:.0f} deg off the tag', flush=True)

    step = (hi - lo) * 0.06
    for _ in range(a.refine // B):
        batch = np.clip(best[1] + rng.normal(0, step, size=(B, 6)), lo, hi)
        for q, (s, d, ang) in zip(batch, score(list(batch))):
            if s < best[0]:
                best = (s, q.copy(), d, ang)
        step *= 0.75
    s, q, d, ang = best
    print(f'\nBEST: {d*100:.1f} cm from the standoff point, camera axis {ang:.0f} deg '
          f'off the direction to the tag')
    print('joints (rad):', ','.join(f'{v:.4f}' for v in q))
    print('margins from window edges (deg):',
          ' '.join(f'{j.split("_joint")[0]}={min(np.degrees(v-l), np.degrees(h-v)):.0f}'
                   for j, v, l, h in zip(JOINTS, q, lo, hi)))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
