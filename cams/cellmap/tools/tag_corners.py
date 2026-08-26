#!/usr/bin/env python3
"""
Tag pose in base_link from three corners touched by the arm (host, domain 0).

The captain freedrives a pointed tip to tag corners A, B, C (labels on the
printed sheet, tools/make_tags.py) and HOLDS STILL ~3 s on each; this tool
watches /joint_states + TF, detects each dwell and records the tip position.
No pendant transcription, no frame confusion: the pendant shows the TCP in
UR's `base` frame, which ur_description rotates 180 deg about Z relative to
ROS `base_link` (verified live 2026-08-26: TF base_link->base = yaw 180 deg),
and this robot's pendant TCP is offset ~0.5 m from the flange — two silent
ways to get a wrong anchor. Reading TF avoids both. A --pendant mode exists
for a cross-check and converts through the same live TF.

Which point is "the tip":
  --tip tcp    (default) the controller's active TCP (/tcp_pose_broadcaster/pose)
               — correct when the pendant TCP is set to the physical tip.
  --tip tool0  flange origin + --tip-offset x,y,z (m, in tool0) — use when the
               tip is a known offset (e.g. a pointer clamped to the flange).

Tag frame (same convention as the OpenCV solver in tag_anchor.py):
  origin = tag centre, x = A->B (right), y = D->A (up), z = x cross y (out of the
  printed face). A flat tag on the table must therefore report z ~ +Z base_link.

Checks (refuse on failure, tolerances in the args): |AB| and |BC| equal the
printed size, angle ABC = 90 deg, tag normal points up.

Usage (one terminal, then freedrive):
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 tag_corners.py --tags 0 1 --size 0.120
  # prints "captured A of tag 0 ..." at each dwell; writes anchor/tag<id>.yaml
  python3 tag_corners.py --tags 0 --pendant "x,y,z" "x,y,z" "x,y,z"   # mm, UR base
"""
import argparse
import datetime as dt
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

BASE = 'base_link'
TOOL = 'tool0'
UR_BASE = 'base'
CORNERS = 'ABC'


def tag_pose_from_corners(A, B, C, size, tol_mm, tol_deg):
    """Return (center, R 3x3, report dict); raise ValueError if the checks fail."""
    A, B, C = (np.asarray(p, float) for p in (A, B, C))
    ab, bc = B - A, C - B
    lab, lbc = np.linalg.norm(ab), np.linalg.norm(bc)
    ang = np.degrees(np.arccos(np.clip(ab @ bc / (lab * lbc), -1, 1)))
    x = ab / lab
    y_raw = -bc                     # D->A == B->C reversed
    y = y_raw - (y_raw @ x) * x
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    center = (A + C) / 2
    rep = {'AB_mm': float(lab * 1e3), 'BC_mm': float(lbc * 1e3),
           'angle_ABC_deg': float(ang), 'normal': [float(v) for v in z],
           'nominal_mm': float(size * 1e3)}
    problems = []
    for name, l in (('AB', lab), ('BC', lbc)):
        if abs(l - size) * 1e3 > tol_mm:
            problems.append(f'|{name}| = {l*1e3:.1f} mm, printed {size*1e3:.0f} '
                            f'(tol {tol_mm} mm) — wrong corner, wrong tip, or print scale')
    if abs(ang - 90) > tol_deg:
        problems.append(f'angle ABC = {ang:.2f} deg (tol {tol_deg}) — corners not on a square')
    rep['problems'] = problems
    if problems:
        raise ValueError('; '.join(problems))
    return center, np.column_stack([x, y, z]), rep


class CornerCapture(Node):
    def __init__(self, a):
        super().__init__('tag_corners')
        self.a = a
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.joints = None
        self.tcp = None
        self.create_subscription(JointState, '/joint_states', self._js, 50)
        self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self._tcp,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.window = []            # (t, tip_xyz)
        self.captures = []          # tip positions in base_link
        self.start_tip = None
        self.moved = False
        self.last_status = 0

    def _js(self, m):
        self.joints = m

    def _tcp(self, m):
        self.tcp = m

    def tf(self, target, source):
        t = self.tfb.lookup_transform(target, source, Time())
        q = t.transform.rotation
        p = t.transform.translation
        return np.array([p.x, p.y, p.z]), R.from_quat([q.x, q.y, q.z, q.w])

    def tip_position(self):
        """Tip in base_link, or None if inputs are missing."""
        if self.a.tip == 'tcp':
            if self.tcp is None:
                return None
            if self.tcp.header.frame_id != UR_BASE:
                raise RuntimeError(f'TCP pose frame is {self.tcp.header.frame_id!r}, expected {UR_BASE!r}')
            p_b, R_b = self.tf(BASE, UR_BASE)      # base_link <- base (180 deg yaw)
            pp = self.tcp.pose.position
            return p_b + R_b.apply([pp.x, pp.y, pp.z])
        p_t, R_t = self.tf(BASE, TOOL)
        return p_t + R_t.apply(self.a.tip_offset)

    def joints_still(self):
        if self.joints is None or not self.joints.velocity:
            return False
        return max(abs(v) for v in self.joints.velocity) < self.a.vmax

    def tick(self):
        try:
            tip = self.tip_position()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        if tip is None:
            return
        now = time.time()
        if self.start_tip is None:
            self.start_tip = tip
        if not self.moved and np.linalg.norm(tip - self.start_tip) > self.a.min_move:
            self.moved = True
            print(f'\narm moved {np.linalg.norm(tip - self.start_tip)*1e3:.0f} mm — '
                  'dwell detection ARMED', flush=True)
        self.window.append((now, tip))
        self.window = [(t, p) for t, p in self.window if now - t <= self.a.dwell]
        pts = np.array([p for _, p in self.window])
        spread = float(np.ptp(pts, axis=0).max()) if len(pts) > 5 else 1.0
        full = (self.window[-1][0] - self.window[0][0]) >= self.a.dwell * 0.95
        still = full and spread < self.a.still_mm * 1e-3 and self.joints_still()
        n = len(self.captures)
        tag_i, corner = divmod(n, len(CORNERS))
        if now - self.last_status > 1.0:
            self.last_status = now
            what = (f'waiting for corner {CORNERS[corner]} of tag {self.a.tags[tag_i]}'
                    if n < self.total else 'done')
            print(f'\r  tip {tip[0]:+.4f} {tip[1]:+.4f} {tip[2]:+.4f}  spread {spread*1e3:5.1f} mm '
                  f'{"STILL" if still else "     "}  [{what}]      ', end='', flush=True)
        if not (self.moved and still) or n >= self.total:
            return
        if self.captures and np.linalg.norm(tip - self.captures[-1]) < self.a.min_move:
            return                          # same spot as the last capture
        med = np.median(pts, axis=0)
        self.captures.append(med)
        print(f'\n\a>>> captured corner {CORNERS[corner]} of tag {self.a.tags[tag_i]}: '
              f'{med[0]:+.4f} {med[1]:+.4f} {med[2]:+.4f} (spread {spread*1e3:.1f} mm, '
              f'{len(pts)} samples) — move to the next corner', flush=True)
        self.window = []

    @property
    def total(self):
        return len(self.a.tags) * len(CORNERS)


def write_tag(a, tag_id, corners, out_dir, source):
    center, Rm, rep = tag_pose_from_corners(*corners, a.size, a.tol_mm, a.tol_deg)
    if rep['normal'][2] < 0.9:
        print(f'  WARNING tag {tag_id}: normal {np.round(rep["normal"], 3)} is not +Z '
              '(tag not flat on the table?) — continuing', file=sys.stderr)
    q = R.from_matrix(Rm).as_quat()
    d = {'tag_id': int(tag_id), 'family': '36h11', 'size_m': float(a.size),
         'frame_id': BASE, 'measured': dt.datetime.now().isoformat(timespec='seconds'),
         'source': source, 'tip': a.tip, 'tip_offset': [float(v) for v in a.tip_offset],
         'corners_base_link': {k: [float(v) for v in c] for k, c in zip(CORNERS, corners)},
         'center': [float(v) for v in center],
         'quaternion_xyzw': [float(v) for v in q],
         'rotation_rows': [[float(v) for v in row] for row in Rm],
         'checks': rep}
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'tag{tag_id}.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(d, f, sort_keys=False)
    print(f'tag {tag_id}: centre {np.round(center, 4)}  |AB| {rep["AB_mm"]:.1f} mm  '
          f'|BC| {rep["BC_mm"]:.1f} mm  angle {rep["angle_ABC_deg"]:.2f} deg  '
          f'normal {np.round(rep["normal"], 3)}\n  -> {path}')
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tags', type=int, nargs='+', default=[0])
    ap.add_argument('--size', type=float, default=0.120, help='printed black square, m')
    ap.add_argument('--tip', choices=['tcp', 'tool0'], default='tcp')
    ap.add_argument('--tip-offset', default='0,0,0', help='m, in tool0 (with --tip tool0)')
    ap.add_argument('--dwell', type=float, default=3.0, help='s of stillness per corner')
    ap.add_argument('--still-mm', type=float, default=1.5, help='max tip spread while still')
    ap.add_argument('--vmax', type=float, default=0.01, help='rad/s joint velocity = still')
    ap.add_argument('--min-move', type=float, default=0.03, help='m between captures')
    ap.add_argument('--tol-mm', type=float, default=4.0)
    ap.add_argument('--tol-deg', type=float, default=1.0)
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'anchor'))
    ap.add_argument('--pendant', nargs='+', metavar='X,Y,Z',
                    help='mm in UR `base` frame, 3 per tag, A B C order (no dwell capture)')
    a = ap.parse_args()
    a.tip_offset = np.array([float(v) for v in a.tip_offset.split(',')])

    rclpy.init()
    node = CornerCapture(a)
    try:
        if a.pendant:
            if len(a.pendant) != 3 * len(a.tags):
                sys.exit('need exactly 3 corners per tag')
            deadline = time.time() + 10
            while time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    p_b, R_b = node.tf(BASE, UR_BASE)
                    break
                except Exception:
                    continue
            else:
                sys.exit(f'no TF {BASE}->{UR_BASE} (driver down?)')
            pts = [p_b + R_b.apply(np.array([float(v) for v in s.split(',')]) / 1e3)
                   for s in a.pendant]
            for i, tid in enumerate(a.tags):
                write_tag(a, tid, pts[3 * i:3 * i + 3], a.out, 'pendant')
            return
        print(f'tags {a.tags}: touch corners A, B, C of each in order, hold still '
              f'{a.dwell:.0f} s per corner. tip = {a.tip}'
              + (f' + offset {a.tip_offset}' if a.tip == 'tool0' else ''))
        while rclpy.ok() and len(node.captures) < node.total:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.tick()
        print()
        for i, tid in enumerate(a.tags):
            write_tag(a, tid, node.captures[3 * i:3 * i + 3], a.out, 'dwell')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
