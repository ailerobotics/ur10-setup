#!/usr/bin/env python3
"""
Loop home -> tag0 -> tag1 -> home on the real arm, planning EVERY leg fresh
with cuMotion against the static map.

Each leg is planned from the arm's live state, not replayed and never
reversed: a reversed trajectory is not a planned one, and the return leg
starts from wherever the arm actually ended up, which is the state the
planner must be given. Every leg passes the same gates as the one-shot
executor before it moves — waypoints inside the pneumatic windows, live
start where the plan says it starts, robot program running — and is watched
for tracking error while it runs.

STOPPING: touch the stop file (--stop-file, default
/tmp/fm-cell-map-v1/STOP) and the loop finishes nothing further — it is
checked before every leg and while each leg runs, so a stop lands within a
fraction of a second of the current leg ending, or cancels it outright with
--hard. Killing the process also stops it; the arm holds position.

  python3 loop_tags.py --limits cumotion/pneumatic_limits.yaml --cycles 0
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
from geometry_msgs.msg import WrenchStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from execute_plan import Exec, JOINTS, SPEED           # noqa: E402

HOME = [-4.3076, -1.3337, 1.5676, 2.9386, 4.3670, -1.5911]
TAG0 = [-2.2472, -1.1492, 1.8986, 3.5012, 4.6681, -1.6669]
TAG1 = [-2.5620, -0.7045, 1.1731, 3.6891, 4.4676, -1.5466]


class Force:
    """Rolling baseline of the estimated external wrench, and the deviation
    from it.

    This robot is a CB3 (PolyScope 3.15.8) with NO wrist force/torque
    sensor: the wrench is estimated from joint currents, carries a standing
    bias of ~13.6 N that cannot be zeroed (zero_ftsensor is e-Series only,
    it returns success=False here), and drifts with pose. So an ABSOLUTE
    force threshold is useless. What is usable is the DEVIATION from a
    short rolling baseline: a hand push shows up as a step against it,
    while the slow pose-dependent drift is tracked out.

    This is a ROS-side guard that trips well below the robot's own safety
    Force limit — which on CB3 cannot be set lower than 100 N and is the
    reason a light push does not stop the arm by itself.
    """

    def __init__(self, node, window=1.5):
        self.buf = []
        self.window = window
        self.peak = 0.0
        node.create_subscription(WrenchStamped,
                                 '/force_torque_sensor_broadcaster/wrench',
                                 self._cb, 20)

    def _cb(self, m):
        f = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])
        self.buf.append((time.time(), f))
        cut = time.time() - self.window
        while len(self.buf) > 2 and self.buf[0][0] < cut:
            self.buf.pop(0)

    def deviation(self):
        """N of force deviation from the rolling baseline."""
        if len(self.buf) < 10:
            return 0.0
        base = np.mean([f for _, f in self.buf[:len(self.buf) // 2]], axis=0)
        now = np.mean([f for _, f in self.buf[-3:]], axis=0)
        d = float(np.linalg.norm(now - base))
        self.peak = max(self.peak, d)
        return d


def dashboard_running(ip='192.168.0.1'):
    import socket
    try:
        s = socket.create_connection((ip, 29999), timeout=5)
        s.recv(4096); s.sendall(b'running\n'); time.sleep(0.4)
        r = s.recv(4096).lower(); s.sendall(b'quit\n'); s.close()
        return b'true' in r
    except Exception:                                        # noqa: BLE001
        return False


def run_leg(n, name, goal, lo, hi, a, force=None):
    """Plan fresh, gate, execute. Returns (ok, message)."""
    q0 = n.q()
    print(f'  [{name}] planning fresh from live state...', flush=True)
    traj = n.replan(goal, a.time)
    got = np.array([list(p.positions) for p in traj.points])
    if list(traj.joint_names) != JOINTS:
        idx = [list(traj.joint_names).index(j) for j in JOINTS]
        got = got[:, idx]
    if np.abs(got[0] - q0).max() > a.start_tol_rad:
        return False, f'plan starts {math.degrees(np.abs(got[0]-q0).max()):.2f} deg from the live pose'
    bad = [(k, JOINTS[i]) for k, w in enumerate(got) for i, v in enumerate(w)
           if not (lo[i] <= v <= hi[i])]
    if bad:
        return False, f'{len(bad)} waypoints outside the pneumatic windows (first {bad[0]})'
    if np.abs(got[-1] - np.array(goal)).max() > 0.02:
        return False, 'plan does not end at the requested goal'
    dur = traj.points[-1].time_from_start
    dur = dur.sec + dur.nanosec * 1e-9
    if not dashboard_running():
        return False, 'external-control program not running'
    print(f'  [{name}] {len(got)} pts, {dur:.1f} s at {SPEED} scaling — executing', flush=True)

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
    g = FollowJointTrajectory.Goal(); g.trajectory = traj
    fut = n.fjt.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return False, 'trajectory goal rejected by the controller'
    rf = gh.get_result_async()
    # Expected position must be INTERPOLATED along the trajectory's own
    # timestamps. Snapping to the nearest waypoint index looks like a
    # tracking error of up to a whole segment — with 1 s spacing at ~6 deg/s
    # that is >10 deg of pure artifact, which tripped the abort on a
    # perfectly healthy arm (2026-08-28). Interpolating gives the real error,
    # which on this arm runs a few tenths of a degree.
    times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                      for p in traj.points])
    t0 = time.time(); worst = 0.0
    while not rf.done():
        rclpy.spin_once(n, timeout_sec=0.05)
        el = time.time() - t0
        if a.hard and os.path.exists(a.stop_file):
            gh.cancel_goal_async(); return False, 'STOP file seen — leg cancelled mid-motion'
        if el > dur + 20:
            gh.cancel_goal_async(); return False, 'overran its duration by 20 s'
        if force is not None and a.max_force_n > 0 and el > a.grace:
            fd = force.deviation()
            if fd > a.max_force_n:
                gh.cancel_goal_async()
                return False, (f'external force deviation {fd:.1f} N exceeded '
                               f'{a.max_force_n:.1f} N — contact detected, stopped')
        if len(n.js) >= 6 and el > a.grace:
            tq = min(max(el, times[0]), times[-1])
            exp = np.array([np.interp(tq, times, got[:, i]) for i in range(6)])
            e = float(np.abs(n.q() - exp).max())
            worst = max(worst, e)
            if e > a.max_track_rad:
                gh.cancel_goal_async()
                return False, f'tracking error {math.degrees(e):.1f} deg exceeded the limit'
    rclpy.spin_until_future_complete(n, rf, timeout_sec=30.0)
    res = rf.result()
    code = res.result.error_code if res else None
    err = float(np.degrees(np.abs(n.q() - np.array(goal)).max()))
    if code != 0:
        return False, f'controller returned error_code {code}'
    if err > 1.0:
        return False, f'ended {err:.2f} deg from the goal'
    fpk = f', peak force dev {force.peak:.1f} N' if force is not None else ''
    if force is not None:
        force.peak = 0.0
    return True, (f'ok, {dur:.1f} s, worst tracking {math.degrees(worst):.2f} deg, '
                  f'final error {err:.2f} deg{fpk}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limits', required=True)
    ap.add_argument('--cycles', type=int, default=0, help='0 = until stopped')
    ap.add_argument('--stop-file', default='/tmp/fm-cell-map-v1/STOP')
    ap.add_argument('--hard', action='store_true',
                    help='also cancel a leg mid-motion when the stop file appears')
    ap.add_argument('--time', type=float, default=25.0)
    ap.add_argument('--start-tol-rad', type=float, default=0.02)
    # THRESHOLDS MEASURED, not guessed (2026-08-28, full clean cycle at 5 %):
    #   tracking error  0.32 - 2.9 deg     force deviation  27.8 - 151.8 N
    # (clean peaks seen across runs: 1.31, 2.18, 2.27, 2.51, 2.9 deg)
    # A hand push registered >= 8.6 deg of tracking error. So tracking error
    # separates cleanly (1.31 vs 8.6) and force does not: on this CB3 the
    # wrench is estimated from joint currents with no wrist sensor, and its
    # normal-motion swing — measured up to 151.8 N with NOTHING touching the
    # arm — buries any light push entirely. A 150 N threshold false-tripped a
    # clean cycle. Any threshold high enough to be quiet is far less sensitive
    # than the robot's own safety Force limit, so force is demoted to a gross
    # -contact backstop at 300 N and the real ROS-side guard is tracking
    # error. Sensitive contact detection on CB3 has to come from the pendant
    # safety limits and a correct payload, not from this signal.
    ap.add_argument('--max-track-rad', type=float, default=0.08,
                    help='rad of tracking error that stops the arm (default 0.08 = 4.6 '
                         'deg: 1.6x the measured clean maximum of 2.9 deg, about half '
                         'the >=8.6 deg a hand push produces). 0.05 was tried and '
                         'false-tripped a clean cycle.')
    ap.add_argument('--max-force-n', type=float, default=300.0,
                    help='N of force deviation from the rolling baseline — GROSS contact '
                         'backstop only (clean motion already reaches 152 N on this CB3); '
                         '0 disables it. See class Force.')
    ap.add_argument('--settle', type=float, default=1.0)
    ap.add_argument('--grace', type=float, default=1.0,
                    help='s at the start of a leg before tracking is judged (the '
                         'controller begins slightly after the goal is sent)')
    a = ap.parse_args()

    lim = yaml.safe_load(open(a.limits))
    ref, win = lim['reference_pose_rad'], lim['window_deg']
    lo = np.array([ref[j] - math.radians(win[j]) for j in JOINTS])
    hi = np.array([ref[j] + math.radians(win[j]) for j in JOINTS])
    for nm, cfg in (('HOME', HOME), ('TAG0', TAG0), ('TAG1', TAG1)):
        v = np.array(cfg)
        if not ((lo <= v) & (v <= hi)).all():
            sys.exit(f'{nm} is outside the pneumatic windows — refusing to start')

    if os.path.exists(a.stop_file):
        os.remove(a.stop_file)
    os.makedirs(os.path.dirname(a.stop_file), exist_ok=True)

    rclpy.init(); n = Exec()
    force = Force(n)
    if not n.wait_state():
        sys.exit('no /joint_states — driver down?')
    if not n.fjt.wait_for_server(timeout_sec=15.0):
        sys.exit('trajectory controller action unavailable')

    print(f'loop: home -> tag0 -> tag1 -> home at {SPEED} scaling, planning every leg '
          f'fresh.\nstop with:  touch {a.stop_file}', flush=True)

    print('\n[return] moving to HOME before starting the loop', flush=True)
    ok, msg = run_leg(n, 'return->home', HOME, lo, hi, a, force)
    print(f'  [return->home] {msg}', flush=True)
    if not ok:
        print('ABORTED before the loop started'); n.destroy_node(); rclpy.shutdown(); return 1

    cyc = 0
    t_start = time.time()
    while True:
        if os.path.exists(a.stop_file):
            print(f'\nSTOP file seen — stopping cleanly after {cyc} complete cycles',
                  flush=True)
            break
        if a.cycles and cyc >= a.cycles:
            print(f'\nreached {a.cycles} cycles', flush=True); break
        cyc += 1
        print(f'\n=== CYCLE {cyc} ===', flush=True)
        tc = time.time()
        for name, goal in (('home->tag0', TAG0), ('tag0->tag1', TAG1),
                           ('tag1->home', HOME)):
            if os.path.exists(a.stop_file) and not a.hard:
                print(f'  STOP file seen — not starting {name}', flush=True)
                cyc -= 1
                break
            ok, msg = run_leg(n, name, goal, lo, hi, a, force)
            print(f'  [{name}] {msg}', flush=True)
            if not ok:
                print(f'\nANOMALY on {name}: {msg}\nSTOPPING. Arm holds position.',
                      flush=True)
                n.destroy_node(); rclpy.shutdown(); return 2
            time.sleep(a.settle)
        else:
            print(f'=== CYCLE {cyc} COMPLETE in {time.time()-tc:.1f} s '
                  f'(total {time.time()-t_start:.0f} s) ===', flush=True)
            continue
        break
    print(f'\nloop finished: {cyc} complete cycles, arm at rest')
    n.destroy_node(); rclpy.shutdown(); return 0


if __name__ == '__main__':
    sys.exit(main())
