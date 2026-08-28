#!/usr/bin/env python3
"""
Write ur10_pneumatic.urdf: ur10_calibrated.urdf with the six arm joints
clamped to the pneumatic-line windows in pneumatic_limits.yaml.

Why a URDF and not a post-hoc check: cuMotion (cuRobo) reads its joint
limits from the URDF it is launched with, so tightening them here means the
PLANNER CANNOT PRODUCE an out-of-limit trajectory in the first place. A
check applied after planning would only tell us the plan was unusable —
and on the real arm, "we noticed afterwards" is not a safety property.
plan_test.py --joint-limits still re-verifies every waypoint independently,
because one guard is not a guard.

  python3 cumotion/make_limited_urdf.py            # windows around the LIVE pose
  python3 cumotion/make_limited_urdf.py --frozen   # around the pose recorded in the yaml
"""
import argparse
import datetime as dt
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def live_pose(timeout=10.0):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import time
    rclpy.init()
    n = Node('limited_urdf')
    got = {}
    n.create_subscription(JointState, '/joint_states',
                          lambda m: got.update(zip(m.name, m.position)), 10)
    t0 = time.time()
    while time.time() - t0 < timeout and not all(j in got for j in JOINTS):
        rclpy.spin_once(n, timeout_sec=0.1)
    rclpy.shutdown()
    if not all(j in got for j in JOINTS):
        sys.exit('could not read /joint_states (driver down?) — use --frozen')
    return {j: float(got[j]) for j in JOINTS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limits', default=os.path.join(HERE, 'pneumatic_limits.yaml'))
    ap.add_argument('--urdf', default=os.path.join(HERE, 'ur10_calibrated.urdf'))
    ap.add_argument('--out', default=os.path.join(HERE, 'ur10_pneumatic.urdf'))
    ap.add_argument('--xrdf-in', default=os.path.join(HERE, 'ur10_map.xrdf'))
    ap.add_argument('--xrdf-out', default=os.path.join(HERE, 'ur10_pneumatic.xrdf'))
    ap.add_argument('--frozen', action='store_true',
                    help='use the reference pose in the yaml instead of live /joint_states')
    a = ap.parse_args()

    cfg = yaml.safe_load(open(a.limits))
    win = cfg['window_deg']
    ref = cfg['reference_pose_rad'] if a.frozen else live_pose()
    if not a.frozen:
        drift = {j: math.degrees(abs(ref[j] - cfg['reference_pose_rad'][j])) for j in JOINTS}
        worst = max(drift, key=drift.get)
        print(f'live pose vs yaml reference: worst drift {worst} {drift[worst]:.1f} deg')
        if drift[worst] > 5.0:
            print('  NOTE: the arm has moved since the limits were written; the windows '
                  'below are around the LIVE pose. Re-confirm with the captain.')

    s = open(a.urdf).read()
    applied = {}
    for j in JOINTS:
        half = math.radians(win[j])
        lo, hi = ref[j] - half, ref[j] + half
        applied[j] = (lo, hi)
        pat = re.compile(r'(<joint name="' + re.escape(j) + r'" type="revolute">.*?<limit[^>]*?)'
                         r'lower="[-\d.eE+]+"([^>]*?)upper="[-\d.eE+]+"', re.S)
        s, n = pat.subn(lambda m: f'{m.group(1)}lower="{lo:.6f}"{m.group(2)}upper="{hi:.6f}"', s, count=1)
        if n != 1:
            sys.exit(f'could not rewrite limits for {j} (matched {n})')

    banner = (f'<!-- GENERATED {dt.datetime.now().isoformat(timespec="seconds")} by '
              f'make_limited_urdf.py from {os.path.basename(a.urdf)}.\n'
              f'     Joint limits REDUCED for the pneumatic-line wire strap '
              f'(pneumatic_limits.yaml).\n'
              f'     Do not edit by hand; do not use as the general robot description. -->\n')
    s = s.replace('<robot', banner + '<robot', 1)
    open(a.out, 'w').write(s)

    # The XRDF's default_joint_positions is cuRobo's RETRACT CONFIG: it seeds
    # IK and trajectory optimisation. The stock one sits at the robot's normal
    # home, which is far outside these windows, and cuMotion rejects it at
    # startup with 'Invalid RobotDescription ... violates c-space position
    # limits' (observed 2026-08-28). A planner whose seed is out of bounds is
    # not one to hand a real arm, so the retract config is moved to the
    # reference pose, which is inside every window by construction.
    xr = open(a.xrdf_in).read()
    for j in JOINTS:
        pat = re.compile(r'(^\s*' + re.escape(j) + r':\s*)[-\d.eE+]+', re.M)
        xr, n = pat.subn(lambda m: f'{m.group(1)}{ref[j]:.4f}', xr, count=1)
        if n != 1:
            sys.exit(f'could not rewrite default_joint_positions for {j}')
    # BASE-LINK WORLD BUFFER 0.02 -> 0.01, shoulder_link and upper_arm_link only.
    # The map legitimately contains the robot's own MOUNTING PEDESTAL — the
    # camera saw it, it is a real static structure, and it belongs in the map.
    # But the upper arm's root sphere lives inside it: measured on
    # cell-2026-08-28, that sphere (r+buffer 0.110) sees an ESDF distance of
    # 0.108, i.e. -2 mm, so the arm's PARKED pose is 'in collision' and every
    # plan fails with INVALID_INITIAL_CSPACE_POSITION before it starts. The
    # robot cannot meaningfully collide with the pedestal it is bolted to, the
    # overlap is 2 mm against a map whose own error is 1-2 cm, and these two
    # links never leave the base region. Every other sphere clears by >= 40 mm.
    # This narrows the buffer ONLY for those two links and ONLY in the
    # pneumatic XRDF; ur10_map.xrdf and ur10.xrdf are untouched.
    for link in ('shoulder_link', 'upper_arm_link'):
        pat = re.compile(r'(^collision:.*?buffer_distance:.*?^\s*' + link + r':\s*)0\.02',
                         re.S | re.M)
        xr, n = pat.subn(lambda m: f'{m.group(1)}0.01', xr, count=1)
        if n != 1:
            sys.exit(f'could not narrow the world buffer for {link} (matched {n})')

    xr = xr.replace('\ndefault_joint_positions:',
                    '\n# default_joint_positions REWRITTEN to the pneumatic reference pose by\n'
                    '# make_limited_urdf.py — the stock retract config is outside the windows\n'
                    '# and cuMotion refuses it as an invalid RobotDescription.\ndefault_joint_positions:', 1)
    open(a.xrdf_out, 'w').write(xr)

    print(f'wrote {a.out}')
    print(f'wrote {a.xrdf_out}  (retract config moved inside the windows)')
    print(f'{"joint":22s} {"window":>8}   {"min rad":>9} {"max rad":>9}   {"min deg":>8} {"max deg":>8}')
    for j in JOINTS:
        lo, hi = applied[j]
        print(f'{j:22s} {"+-"+str(win[j])+" deg":>8}   {lo:9.4f} {hi:9.4f}   '
              f'{math.degrees(lo):8.1f} {math.degrees(hi):8.1f}')


if __name__ == '__main__':
    main()
