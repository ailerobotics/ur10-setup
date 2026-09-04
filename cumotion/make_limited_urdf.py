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

  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cumotion/make_limited_urdf.py   # windows around the LIVE pose
  python3 cumotion/make_limited_urdf.py --frozen   # around the pose recorded in the yaml (no ROS)
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cumotion/make_limited_urdf.py \
      --write-limits cumotion/pneumatic_limits_live-<stamp>.yaml
      # also writes the windows AS APPLIED (reference = the live pose) in the
      # schema plan_test.py --joint-limits and execute_plan.py --limits read,
      # so the guards check the same windows the planner was given. Refuses to
      # overwrite an existing limits file (a MEASURED one must never be
      # replaced) unless --force is given.
The live read needs the rig's DDS transport (FASTDDS_BUILTIN_TRANSPORTS=UDPv4)
like every other ROS-talking command; without it discovery fails and the script
exits — do not fall back to --frozen for anything that will be executed.
"""
import argparse
import datetime as dt
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD_COLLISION_EXCLUDE = ['shoulder_link', 'upper_arm_link']
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
    ap.add_argument('--write-limits', default=None, metavar='YAML',
                    help='also write the applied windows (reference_pose_rad = the pose used '
                         'here, window_deg) for plan_test.py --joint-limits / execute_plan.py --limits')
    ap.add_argument('--force', action='store_true',
                    help='allow --write-limits to overwrite an existing file')
    a = ap.parse_args()
    if a.write_limits and os.path.exists(a.write_limits) and not a.force:
        sys.exit(f'refusing to overwrite {a.write_limits}: a limits file that already exists '
                 f'may be a MEASURED one. Write to a new path, or pass --force.')

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
        # CLAMP to the robot's real limits. A window can extend past the
        # mechanical range (+-120 deg around a pan of -246.8 deg reaches
        # -366.8 deg, outside the UR10's +-360), and a URDF that claims a
        # joint can go further than it can is a worse lie than the wide
        # limits we started from.
        m0 = re.search(r'<joint name="' + re.escape(j) + r'" type="revolute">.*?'
                       r'<limit[^>]*lower="([-\d.eE+]+)"[^>]*upper="([-\d.eE+]+)"',
                       s, re.S)
        if m0:
            stock_lo, stock_hi = float(m0.group(1)), float(m0.group(2))
            lo, hi = max(lo, stock_lo), min(hi, stock_hi)
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
    # WORLD-COLLISION EXCLUSION for shoulder_link and upper_arm_link
    # (captain-approved 2026-08-28). The map legitimately contains the robot's
    # own MOUNTING PEDESTAL, and the upper arm's root sphere lives inside it
    # (measured -2 mm on cell-2026-08-28), so the arm's own parked pose reads
    # as "in collision" and every plan dies with
    # INVALID_INITIAL_CSPACE_POSITION before it starts. The robot cannot
    # meaningfully collide with the structure it is bolted to.
    #
    # SCOPE, deliberately narrow: WORLD collision only, those two links only.
    # `collision:` and `self_collision:` share one sphere set in the stock
    # XRDF, so a separate world-only geometry set is built here that omits
    # those two links; self_collision keeps pointing at the FULL set, so the
    # arm is still checked against itself everywhere. Every other link keeps
    # world collision on with its normal buffer.
    xd = yaml.safe_load(xr)
    full = xd['collision']['geometry']
    world_set = full + '_world_only'
    spheres = {k: v for k, v in xd['geometry'][full]['spheres'].items()
               if k not in WORLD_COLLISION_EXCLUDE}
    dropped = [k for k in xd['geometry'][full]['spheres'] if k in WORLD_COLLISION_EXCLUDE]
    if sorted(dropped) != sorted(WORLD_COLLISION_EXCLUDE):
        sys.exit(f'expected to drop {WORLD_COLLISION_EXCLUDE}, dropped {dropped}')
    xd['geometry'][world_set] = {'spheres': spheres}
    xd['collision']['geometry'] = world_set
    xd['collision']['buffer_distance'] = {
        k: v for k, v in xd['collision']['buffer_distance'].items()
        if k not in WORLD_COLLISION_EXCLUDE}
    assert xd['self_collision']['geometry'] == full, 'self-collision must keep the FULL sphere set'
    for j in JOINTS:
        xd['default_joint_positions'][j] = round(float(ref[j]), 4)
    xr = yaml.safe_dump(xd, sort_keys=False, default_flow_style=None)
    xr = (f'# GENERATED {dt.datetime.now().isoformat(timespec="seconds")} by '
          f'make_limited_urdf.py from {os.path.basename(a.xrdf_in)}.\n'
          f'# - default_joint_positions moved to the pneumatic reference pose\n'
          f'#   (the stock retract config is outside the windows and cuMotion\n'
          f'#   rejects it as an invalid RobotDescription).\n'
          f'# - {", ".join(WORLD_COLLISION_EXCLUDE)} excluded from WORLD collision only,\n'
          f'#   via the generated "{world_set}" sphere set: the map contains the\n'
          f'#   robot\'s own mounting pedestal. SELF-collision still uses the full\n'
          f'#   set, so the arm is checked against itself everywhere.\n'
          f'# Do not edit by hand.\n') + xr

    open(a.xrdf_out, 'w').write(xr)

    if a.write_limits:
        hdr = (f'# GENERATED {dt.datetime.now().isoformat(timespec="seconds")} by '
               f'make_limited_urdf.py from {os.path.basename(a.limits)}.\n'
               f'# reference_pose_rad = the {"yaml reference" if a.frozen else "LIVE /joint_states"} '
               f'pose the windows in {os.path.basename(a.out)} were built around.\n'
               f'# Same schema as pneumatic_limits.yaml: feed to plan_test.py --joint-limits\n'
               f'# and execute_plan.py --limits so every guard checks THESE windows.\n')
        body = yaml.safe_dump({'reference_pose_rad': {j: round(float(ref[j]), 6) for j in JOINTS},
                               'window_deg': {j: win[j] for j in JOINTS}}, sort_keys=False)
        open(a.write_limits, 'w').write(hdr + body)

    print(f'wrote {a.out}')
    print(f'wrote {a.xrdf_out}  (retract config moved inside the windows)')
    if a.write_limits:
        print(f'wrote {a.write_limits}  (applied windows for plan_test/execute_plan guards)')
    print(f'{"joint":22s} {"window":>8}   {"min rad":>9} {"max rad":>9}   {"min deg":>8} {"max deg":>8}')
    for j in JOINTS:
        lo, hi = applied[j]
        print(f'{j:22s} {"+-"+str(win[j])+" deg":>8}   {lo:9.4f} {hi:9.4f}   '
              f'{math.degrees(lo):8.1f} {math.degrees(hi):8.1f}')


if __name__ == '__main__':
    main()
