#!/usr/bin/env python3
"""
Tracking-loop evidence for cuVSLAM: record /visual_slam/tracking/odometry,
report loop closure (end pose vs start pose), path length, tracking-loss
gaps, and save a top-down plot + CSV + yaml.

Run INSIDE the scan container (it has the isaac_ros_visual_slam_interfaces
status message; on the host the status topic cannot be decoded, everything
else still works):
  docker exec -it ur10-cellmap-scan bash -lc 'source /opt/ros/jazzy/setup.bash && \
      python3 /cellmap/tools/loop_check.py --out /maps/loop-<date>'
Procedure: rest the camera on a marked spot (e.g. table edge against a tape
mark, lens forward), start the tool, wait for "start pose fixed", walk the
loop, put the camera back on the SAME spot, hold 3 s, Ctrl-C.
Pass = closure < a few cm with no gaps; a jump in the path or a gap > 0.5 s
means tracking was lost (see README "what good looks like").
"""
import argparse
import csv
import datetime as dt
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import yaml

try:                                    # container only
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None


def quat_angle_deg(q1, q2):
    d = abs(float(np.dot(q1, q2)))
    return float(np.degrees(2 * np.arccos(min(1.0, d))))


class LoopCheck(Node):
    def __init__(self, a):
        super().__init__('loop_check')
        self.a = a
        self.rows = []              # t, x, y, z, qx, qy, qz, qw
        self.bad_status = 0
        self.n_status = 0
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self._odom, 50)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status', self._st, 10)
        self.t0 = None
        self.last_print = 0

    def _st(self, m):
        self.n_status += 1
        # vo_state: 0 unknown, 1 success, 2 failed, 3 invalidated (cuVSLAM docs)
        if getattr(m, 'vo_state', 1) != 1:
            self.bad_status += 1

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
            print(f'first odometry in frame {m.header.frame_id!r}; start pose fixes after '
                  f'{self.a.settle} s of stillness', flush=True)
        self.rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if time.time() - self.last_print > 1.0:
            self.last_print = time.time()
            n = len(self.rows)
            print(f'\r  t={t-self.t0:6.1f}s  pos ({p.x:+.3f} {p.y:+.3f} {p.z:+.3f})  '
                  f'{n} samples  bad status {self.bad_status}/{self.n_status}     ',
                  end='', flush=True)

    def report(self):
        if len(self.rows) < 20:
            print('\nnot enough odometry (tracking never started?)')
            return None
        A = np.array(self.rows)
        t, P, Q = A[:, 0] - A[0, 0], A[:, 1:4], A[:, 4:8]
        s = self.a.settle
        start = P[t <= s]
        end = P[t >= t[-1] - s]
        ps, pe = np.median(start, axis=0), np.median(end, axis=0)
        qs, qe = Q[t <= s][-1], Q[-1]
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
        dts = np.diff(t)
        gaps = [(float(t[i]), float(dts[i])) for i in np.where(dts > self.a.gap)[0]]
        jumps = [(float(t[i]), float(seg[i])) for i in np.where(seg > self.a.jump)[0]]
        rep = {
            'recorded': dt.datetime.now().isoformat(timespec='seconds'),
            'duration_s': float(t[-1]), 'samples': int(len(t)),
            'rate_hz': float(len(t) / max(t[-1], 1e-6)),
            'path_length_m': float(seg.sum()),
            'closure_error_m': float(np.linalg.norm(pe - ps)),
            'closure_error_xyz_m': [float(v) for v in (pe - ps)],
            'closure_rot_deg': quat_angle_deg(qs, qe),
            'start_spread_mm': float(np.ptp(start, axis=0).max() * 1e3),
            'end_spread_mm': float(np.ptp(end, axis=0).max() * 1e3),
            'max_step_m': float(seg.max()),
            'gaps_over_%.2fs' % self.a.gap: gaps,
            'jumps_over_%.2fm' % self.a.jump: jumps,
            'bad_status_msgs': int(self.bad_status), 'status_msgs': int(self.n_status),
            'bbox_min_m': [float(v) for v in P.min(axis=0)],
            'bbox_max_m': [float(v) for v in P.max(axis=0)],
        }
        print('\n' + yaml.safe_dump(rep, sort_keys=False))
        out = self.a.out
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        with open(out + '.yaml', 'w') as f:
            yaml.safe_dump(rep, f, sort_keys=False)
        with open(out + '.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            w.writerows(self.rows)
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, (ax, az) = plt.subplots(1, 2, figsize=(11, 5),
                                         gridspec_kw={'width_ratios': [3, 2]})
            ax.plot(P[:, 0], P[:, 1], lw=1, color='#0e7d86')
            ax.plot(ps[0], ps[1], 'o', color='#2f7d4f', ms=9, label='start')
            ax.plot(pe[0], pe[1], 'x', color='#b23a3a', ms=11, mew=2.5, label='end')
            for tt, _ in jumps:
                i = int(np.searchsorted(t, tt))
                ax.plot(P[i, 0], P[i, 1], 's', color='#a06c14', ms=7)
            ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend()
            ax.set_xlabel('x [m] (map)'); ax.set_ylabel('y [m]')
            ax.set_title(f'cuVSLAM RGBD top-down — path {rep["path_length_m"]:.1f} m, '
                         f'closure {rep["closure_error_m"]*100:.1f} cm / {rep["closure_rot_deg"]:.1f}°')
            az.plot(t, P[:, 2], lw=1, color='#0e7d86'); az.grid(alpha=.3)
            az.set_xlabel('t [s]'); az.set_ylabel('z [m]'); az.set_title('height vs time')
            fig.tight_layout(); fig.savefig(out + '.png', dpi=130)
            print('plot ->', out + '.png')
        except Exception as e:      # noqa: BLE001
            print('no plot:', e)
        return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=f'/maps/loop-{dt.date.today().isoformat()}',
                    help='basename for .yaml/.csv/.png')
    ap.add_argument('--settle', type=float, default=3.0, help='s averaged at start/end')
    ap.add_argument('--gap', type=float, default=0.5, help='odometry gap = tracking loss')
    ap.add_argument('--jump', type=float, default=0.15, help='m per sample = pose jump')
    ap.add_argument('--duration', type=float, default=0, help='auto-stop after s (0 = Ctrl-C)')
    a = ap.parse_args()
    rclpy.init()
    node = LoopCheck(a)
    stop = {'v': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('v', True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('v', True))
    t_end = time.time() + a.duration if a.duration else None
    while rclpy.ok() and not stop['v'] and (t_end is None or time.time() < t_end):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.report()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
