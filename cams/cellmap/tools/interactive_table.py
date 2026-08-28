#!/usr/bin/env python3
"""
Interactive table motion: stop on a PERCEIVED obstacle in the path, show it in
RViz, and auto-resume ONLY when the blocking region is seen to be EMPTY.

THE SAFETY POINT, and why this does not use the ESDF for the resume decision.
nvblox serves unobserved space as FREE (esdf_and_gradients_unobserved_value),
which is what lets cuMotion plan at all — but it means the fused map CANNOT
distinguish "I looked and it is empty" from "I have never looked there". A
resume gated on the ESDF would therefore restart the arm the moment the hand
moves somewhere the camera cannot see, which is exactly backwards.

So the gate reads the DEPTH IMAGE directly. For a path point P, projected into
the depth camera at range z along the ray:

  no valid depth at that pixel, or P outside the image  -> UNOBSERVED
  |d - z| <= clearance   (a surface right AT P)         -> OCCUPIED
  d >  z + clearance     (ray passes beyond P)          -> OBSERVED EMPTY
  d <  z - clearance     (something in FRONT of P, so
                          P is hidden behind it)        -> UNOBSERVED

Occlusion is deliberately UNOBSERVED and not OCCUPIED. Treating "hidden
behind a surface" as an obstacle would flag every path point beyond the table
and block constantly; treating it as clear would resume the arm into a region
it cannot see. Unknown is its own answer: it never blocks, and it never
resumes.

Only OBSERVED EMPTY counts as clear. UNOBSERVED never resumes the arm — it is
treated exactly like still-blocked. Clearing must also hold for --debounce
seconds so a single flickering frame cannot restart motion.

Backstops all remain: the static base map in the planner, the pneumatic joint
windows, the tracking-error guard, and the captain's e-stop. This layer stops
the arm EARLIER than contact; it does not replace anything.

  python3 interactive_table.py --limits cumotion/pneumatic_limits.yaml
  touch /tmp/fm-cell-map-v1/STOP    # stop
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from execute_plan import Exec, JOINTS, SPEED           # noqa: E402

BASE, TOOL, OPTICAL = 'base_link', 'tool0', 'camera_color_optical_frame'
UNOBSERVED, OCCUPIED, FREE = 0, 1, 2


def tf_to_T(t):
    from scipy.spatial.transform import Rotation as R
    q, v = t.transform.rotation, t.transform.translation
    M = np.eye(4)
    M[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M[:3, 3] = [v.x, v.y, v.z]
    return M


class NearGuard:
    """Near-field obstacle guard that is safe against the D435i BLIND ZONE.

    MEASURED on this camera (2026-08-28, 1280x720 aligned depth): the closest
    valid return is 240 mm; the 0.1st percentile is 243 mm; nothing valid
    exists below that. So anything nearer than ~0.24 m simply VANISHES from
    the depth image. The old path-point classifier called that UNOBSERVED,
    and UNOBSERVED never blocked — meaning a hand that came close enough
    would have gone INVISIBLE and the arm would have kept moving. That is the
    hole this class closes.

    Three rules, all on the RAW depth with arm pixels excluded via the
    segmenter's mask (so the arm never triggers on itself):

    1. STOP EARLY. Something NEW and nearer than --stop-range (default
       0.45 m, i.e. ~0.21 m above the measured 0.24 m floor) stops the arm
       while it is still comfortably inside the reliable sensing band. New
       means "closer than the rolling background by --novelty", which is what
       keeps the table — always ~0.34 m away and perfectly static — from
       triggering.
    2. DROPOUT IS AN OBSTACLE. Pixels that HAD valid near depth and now read
       invalid are treated as a possible object that has entered the blind
       zone, and stop the arm. Never the reverse: a dropout is never clear.
       Baseline invalid fraction on this camera is 0.62 %, so a real dropout
       stands well clear of the noise.
    3. RESUME ONLY ON POSITIVE EVIDENCE. The near field must be actively
       SEEN empty — enough valid pixels, no novelty, no dropout — for
       --debounce seconds. Missing data is never taken as clearance.
    """

    def __init__(self, node, a):
        self.a = a
        self.raw = None
        self.mask = None
        self.hist = []
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        node.create_subscription(Image, a.raw_depth, self._raw, q)
        node.create_subscription(Image, a.mask_topic, self._mask, q)

    def _raw(self, m):
        if m.encoding in ('16UC1', 'mono16'):
            self.raw = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            self.hist.append(self.raw)
            if len(self.hist) > self.a.bg_frames:
                self.hist.pop(0)

    def _mask(self, m):
        if m.encoding in ('16UC1', 'mono16'):
            self.mask = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)

    def ready(self):
        return self.raw is not None and len(self.hist) >= max(4, self.a.bg_frames // 2)

    def assess(self):
        """-> (verdict, detail) with verdict in {'CLEAR','NOVEL','DROPOUT','NODATA'}."""
        if not self.ready():
            return 'NODATA', 'no depth yet'
        cur = self.hist[-1].astype(np.int32)
        bg = np.median(np.stack(self.hist[:max(2, len(self.hist) // 2)]), axis=0).astype(np.int32)
        # The segmenter's mask marks KEEP with 65535 and the arm with 0 —
        # the opposite of the obvious reading. Getting this backwards marks
        # the whole image as "arm", leaves zero pixels to judge, and the guard
        # silently reports NODATA forever (i.e. it never stops anything).
        keep = (self.mask > 0) if (self.mask is not None
                                   and self.mask.shape == cur.shape) else np.ones_like(cur, bool)
        sel = keep
        stop_mm = int(self.a.stop_range * 1000)
        novel = sel & (cur > 0) & (bg > 0) & (cur < bg - self.a.novelty_mm) & (cur < stop_mm)
        n_novel = int(novel.sum())
        drop = sel & (cur == 0) & (bg > 0) & (bg < int(self.a.dropout_range * 1000))
        n_drop = int(drop.sum())
        near_valid = int((sel & (cur > 0) & (cur < stop_mm)).sum())
        if n_novel >= self.a.min_px:
            return 'NOVEL', f'{n_novel} px newly closer than {self.a.stop_range:.2f} m'
        if n_drop >= self.a.min_drop_px:
            return 'DROPOUT', (f'{n_drop} px lost valid near depth — possible object inside '
                               f'the {self.a.min_range:.2f} m blind zone')
        if near_valid < self.a.min_valid_px:
            return 'NODATA', f'only {near_valid} valid near pixels — cannot call it clear'
        return 'CLEAR', f'{near_valid} valid near px, no novelty, no dropout'


class Eyes:
    """Classifies base_link points as UNOBSERVED / OCCUPIED / FREE from the
    live masked depth image (the segmenter's output, so the arm itself is
    already removed and cannot block its own path)."""

    def __init__(self, node, a):
        self.n, self.a = node, a
        self.K = None
        self.depth = None
        self.stamp = None
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        node.create_subscription(CameraInfo, a.depth_info, self._info, 5)
        node.create_subscription(Image, a.depth_image, self._img, q)
        self.tfb = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfb, node, spin_thread=True)

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k, np.float64).reshape(3, 3)
            self.w, self.h = m.width, m.height

    def _img(self, m):
        if m.encoding not in ('16UC1', 'mono16'):
            return
        self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        self.stamp = m.header.stamp

    def ready(self):
        return self.K is not None and self.depth is not None

    def T_base_opt(self):
        # Split lookup, same reason as validate_tag1: only base_link->tool0
        # varies in time; the camera's own tree is a slow dynamic transform.
        try:
            bt = tf_to_T(self.tfb.lookup_transform(BASE, TOOL, Time()))
            to = tf_to_T(self.tfb.lookup_transform(TOOL, OPTICAL, Time()))
            return bt @ to
        except Exception:                                    # noqa: BLE001
            return None

    def classify(self, pts):
        """pts: (N,3) in base_link -> (N,) of UNOBSERVED/OCCUPIED/FREE."""
        out = np.full(len(pts), UNOBSERVED, np.uint8)
        if not self.ready():
            return out
        T = self.T_base_opt()
        if T is None:
            return out
        Ti = np.linalg.inv(T)
        P = (Ti @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
        z = P[:, 2]
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = fx * P[:, 0] / z + cx
            v = fy * P[:, 1] / z + cy
        m = self.a.margin_px
        ok = (z > 0.12) & (u >= m) & (v >= m) & (u < self.w - m) & (v < self.h - m)
        for i in np.flatnonzero(ok):
            ui, vi = int(u[i]), int(v[i])
            k = self.a.patch
            patch = self.depth[max(vi - k, 0):vi + k + 1, max(ui - k, 0):ui + k + 1]
            valid = patch[patch > 0]
            if valid.size < max(1, (2 * k + 1) ** 2 // 4):
                continue                                     # stays UNOBSERVED
            d = float(np.median(valid)) / 1000.0
            if abs(d - z[i]) <= self.a.clearance:
                out[i] = OCCUPIED          # a surface right at the path point
            elif d > z[i] + self.a.clearance:
                out[i] = FREE              # ray reaches past it: seen empty
            # else d < z - clearance: P is hidden behind something -> leave
            # UNOBSERVED. Not an obstacle we can claim, not clearance either.
        return out


def markers(pts, state, blocking, ns='live_avoid'):
    ma = MarkerArray()
    col = {'RUNNING': (0.15, 0.8, 0.2), 'BLOCKED': (0.95, 0.1, 0.1),
           'RESUMING': (0.98, 0.7, 0.1)}[state]
    if blocking is not None and len(blocking):
        m = Marker()
        m.header.frame_id = BASE
        m.ns, m.id, m.type, m.action = ns, 0, Marker.CUBE_LIST, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color = ColorRGBA(r=0.95, g=0.05, b=0.05, a=0.9)
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in blocking]
        ma.markers.append(m)
    if pts is not None and len(pts):
        m = Marker()
        m.header.frame_id = BASE
        m.ns, m.id, m.type, m.action = ns, 1, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.008
        m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.8)
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        ma.markers.append(m)
    t = Marker()
    t.header.frame_id = BASE
    t.ns, t.id, t.type, t.action = ns, 2, Marker.TEXT_VIEW_FACING, Marker.ADD
    t.scale.z = 0.10
    t.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0)
    t.pose.position = Point(x=0.0, y=0.0, z=1.35)
    t.pose.orientation.w = 1.0
    t.text = state if state != 'BLOCKED' else 'BLOCKED — obstacle in path'
    ma.markers.append(t)
    return ma


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limits', required=True)
    ap.add_argument('--plans', nargs='+',
                    default=['cams/cellmap/plans/live-tableB.yaml',
                             'cams/cellmap/plans/live-tableC.yaml'])
    ap.add_argument('--depth-image', default='/cumotion/camera_1/world_depth')
    ap.add_argument('--depth-info', default='/camera/depth/camera_info')
    ap.add_argument('--clearance', type=float, default=0.15,
                    help='m: a surface within this of a path point blocks')
    ap.add_argument('--lookahead', type=float, default=2.0, help='s of path checked ahead')
    ap.add_argument('--debounce', type=float, default=1.5,
                    help='s the region must stay OBSERVED-EMPTY before resuming')
    ap.add_argument('--patch', type=int, default=3, help='depth patch half-width (px)')
    ap.add_argument('--margin-px', type=int, default=8)
    ap.add_argument('--max-track-rad', type=float, default=0.08)
    ap.add_argument('--grace', type=float, default=1.0)
    ap.add_argument('--raw-depth', default='/camera/depth/image_mm')
    ap.add_argument('--mask-topic', default='/cumotion/camera_1/robot_mask')
    ap.add_argument('--min-range', type=float, default=0.24,
                    help='MEASURED D435i minimum valid depth (m) on this camera')
    ap.add_argument('--stop-range', type=float, default=0.45,
                    help='m: a NEW object nearer than this stops the arm — kept well '
                         'above --min-range so the obstacle is still clearly visible')
    ap.add_argument('--dropout-range', type=float, default=0.60,
                    help='m: background nearer than this, now invalid, counts as a dropout')
    ap.add_argument('--novelty', dest='novelty_mm', type=int, default=60,
                    help='mm a pixel must close vs the background to count as new')
    # NOISE FLOOR MEASURED on a static scene, 433 samples over 25 s:
    #   novelty  median 137  p95 431   max 1445 px
    #   dropout  median 1459 p95 3397  max 6481 px
    # Thresholds sit well above those maxima. For scale, a ~15 cm hand at
    # 0.40 m subtends roughly 68,000 px at this resolution and far more once
    # it reaches the blind zone, so these are still extremely sensitive to a
    # real hand while immune to speckle.
    ap.add_argument('--min-px', type=int, default=4000,
                    help='pixels of NOVELTY needed to stop (noise max was 1445)')
    ap.add_argument('--min-drop-px', type=int, default=20000,
                    help='pixels of near-field DROPOUT needed to stop (noise max was '
                         '6481; a hand in the blind zone blacks out far more)')
    ap.add_argument('--min-valid-px', type=int, default=2000,
                    help='valid near pixels required before the near field may be called '
                         'clear — positive evidence, not absence of evidence')
    ap.add_argument('--bg-frames', type=int, default=12)
    ap.add_argument('--stop-file', default='/tmp/fm-cell-map-v1/STOP')
    ap.add_argument('--go-file', default='/tmp/fm-cell-map-v1/GO',
                    help='the arm does not move until this file exists. Auto-resume is a '
                         'WITHIN-RUN behaviour: it may restart motion after an '
                         'obstacle-stop, but nothing may start a run by itself.')
    ap.add_argument('--no-go-gate', dest='go_gate', action='store_false',
                    help='skip the start gate (do not use for captain-supervised runs)')
    ap.add_argument('--settle', type=float, default=1.0)
    a = ap.parse_args()

    lim = yaml.safe_load(open(a.limits))
    ref, win = lim['reference_pose_rad'], lim['window_deg']
    lo = np.array([ref[j] - math.radians(win[j]) for j in JOINTS])
    hi = np.array([ref[j] + math.radians(win[j]) for j in JOINTS])
    legs = [yaml.safe_load(open(p)) for p in a.plans]
    if os.path.exists(a.stop_file):
        os.remove(a.stop_file)

    rclpy.init()
    n = Exec()
    eyes = Eyes(n, a)
    near = NearGuard(n, a)
    pub = n.create_publisher(MarkerArray, '/cellmap/live_avoid', 2)
    if not n.wait_state():
        sys.exit('no /joint_states')
    if not n.fjt.wait_for_server(timeout_sec=15.0):
        sys.exit('trajectory controller unavailable')
    t0 = time.time()
    while (not eyes.ready() or not near.ready()) and time.time() - t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not eyes.ready() or not near.ready():
        sys.exit('no depth image / camera_info — live avoidance would be blind')
    print(f'interactive: {len(legs)} legs, clearance {a.clearance} m, lookahead '
          f'{a.lookahead} s, resume debounce {a.debounce} s (OBSERVED-EMPTY only)')
    print(f'stop with: touch {a.stop_file}', flush=True)

    # HARD START GATE. The captain must authorise a run; this tool must never
    # put the arm in motion on its own. Auto-resume only ever restarts motion
    # INSIDE a run that was already authorised.
    if a.go_gate:
        print(f'\nREADY — arm holding at rest. Waiting for the go file: {a.go_file}',
              flush=True)
        pub.publish(markers(None, 'RUNNING', None))
        while not os.path.exists(a.go_file):
            if os.path.exists(a.stop_file):
                print('STOP seen while waiting for go — exiting without moving')
                n.destroy_node(); rclpy.shutdown(); return 0
            rclpy.spin_once(n, timeout_sec=0.1)
            time.sleep(0.1)
        print('go received — starting the authorised run', flush=True)

    state = 'RUNNING'
    blocking = np.zeros((0, 3))
    leg_i = 0
    passes = 0
    while True:
        if os.path.exists(a.stop_file):
            print('\nSTOP file seen — halting'); break
        leg = legs[leg_i % len(legs)]
        goal = np.array(leg['joint_path'][-1])
        tool = np.array(leg['tool_path'])
        label = leg['label']

        # plan fresh from wherever we are (after a block we are mid-leg)
        print(f'\n[{label}] planning from live state...', flush=True)
        try:
            traj = n.replan(goal, 25.0)
        except SystemExit as e:
            print(f'  planning failed: {e} — holding'); time.sleep(2.0); continue
        got = np.array([list(p.positions) for p in traj.points])
        if list(traj.joint_names) != JOINTS:
            idx = [list(traj.joint_names).index(j) for j in JOINTS]
            got = got[:, idx]
        bad = [(k, JOINTS[i]) for k, w in enumerate(got) for i, v in enumerate(w)
               if not (lo[i] <= v <= hi[i])]
        if bad:
            print(f'  ABORT: {len(bad)} waypoints outside the pneumatic windows'); break
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        traj.points[-1].velocities = [0.0] * len(traj.joint_names)
        traj.points[-1].accelerations = [0.0] * len(traj.joint_names)
        times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                          for p in traj.points])
        dur = float(times[-1])

        # tool path of THIS trajectory, for the lookahead test
        fk = np.array(leg['tool_path'])
        fk_t = np.linspace(0, dur, len(fk))

        g = FollowJointTrajectory.Goal(); g.trajectory = traj
        fut = n.fjt.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('  goal rejected'); break
        rf = gh.get_result_async()
        state = 'RUNNING'
        print(f'  [{label}] moving ({len(got)} pts, {dur:.1f} s)', flush=True)
        tstart = time.time()
        worst = 0.0
        blocked_now = False
        while not rf.done():
            rclpy.spin_once(n, timeout_sec=0.02)
            el = time.time() - tstart
            if os.path.exists(a.stop_file):
                gh.cancel_goal_async(); print('  STOP file — cancelling'); break
            # --- perceived-obstacle lookahead
            sel = (fk_t >= el) & (fk_t <= el + a.lookahead)
            ahead = fk[sel] if sel.any() else fk[-1:][None, 0].reshape(1, 3)
            verdict, detail = near.assess()
            if verdict in ('NOVEL', 'DROPOUT'):
                gh.cancel_goal_async()
                cls = eyes.classify(ahead)
                blocking = ahead[cls == OCCUPIED]
                if not len(blocking):
                    blocking = ahead[:1]      # mark where we stopped
                blocked_now = True
                block_reason = verdict
                print(f'  [{label}] BLOCKED ({verdict}) — {detail}; stopped before '
                      f'contact', flush=True)
                break
            if el > a.grace and len(n.js) >= 6:
                tq = min(max(el, times[0]), times[-1])
                exp = np.array([np.interp(tq, times, got[:, i]) for i in range(6)])
                e = float(np.abs(n.q() - exp).max())
                worst = max(worst, e)
                if e > a.max_track_rad:
                    gh.cancel_goal_async()
                    print(f'  [{label}] CONTACT GUARD: tracking {math.degrees(e):.1f} deg '
                          f'— stopping'); blocked_now = True; blocking = np.zeros((0, 3))
                    break
            pub.publish(markers(ahead, 'RUNNING', None))
        if not blocked_now:
            rclpy.spin_until_future_complete(n, rf, timeout_sec=20.0)
            err = float(np.degrees(np.abs(n.q() - goal).max()))
            passes += 1
            print(f'  [{label}] pass {passes} complete, worst tracking '
                  f'{math.degrees(worst):.2f} deg, final error {err:.2f} deg', flush=True)
            pub.publish(markers(None, 'RUNNING', None))
            leg_i += 1
            time.sleep(a.settle)
            continue

        # --- BLOCKED: wait for the region to be OBSERVED EMPTY, with debounce
        state = 'BLOCKED'
        clear_since = None
        while True:
            rclpy.spin_once(n, timeout_sec=0.05)
            if os.path.exists(a.stop_file):
                print('  STOP file — halting while blocked'); break
            verdict, detail = near.assess()
            cls = eyes.classify(blocking) if len(blocking) else np.array([FREE])
            n_free = int((cls == FREE).sum())
            n_unobs = int((cls == UNOBSERVED).sum())
            n_occ = int((cls == OCCUPIED).sum())
            # Resume needs BOTH: the near field positively seen empty (which
            # rules out an object sitting in the blind zone) AND no path point
            # still perceived occupied. NODATA never resumes.
            all_seen_clear = (verdict == 'CLEAR') and n_occ == 0
            if all_seen_clear:
                if clear_since is None:
                    clear_since = time.time()
                    print(f'  [{label}] region OBSERVED EMPTY — debouncing '
                          f'{a.debounce:.1f} s', flush=True)
                elif time.time() - clear_since >= a.debounce:
                    state = 'RESUMING'
                    pub.publish(markers(None, 'RESUMING', None))
                    print(f'  [{label}] RESUMING — region seen empty for '
                          f'{a.debounce:.1f} s', flush=True)
                    break
            else:
                if clear_since is not None:
                    print(f'  [{label}] not clear again (near={verdict}: {detail}; '
                          f'path occ {n_occ}, unobs {n_unobs}) — debounce reset', flush=True)
                clear_since = None
            pub.publish(markers(None, state, blocking))
            time.sleep(0.05)
        if os.path.exists(a.stop_file):
            break
        time.sleep(a.settle)

    pub.publish(markers(None, 'RUNNING', None))
    print(f'\ninteractive run finished: {passes} completed passes')
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
