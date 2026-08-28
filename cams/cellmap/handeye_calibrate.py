#!/usr/bin/env python3
"""
Eye-in-hand hand-eye calibration (AX = XB) for the wrist D435i — host, domain 0.

X = tool0 -> camera_color_optical_frame. Per sample: A-side = base_link -> tool0
from TF (UR driver, /joint_states), B-side = camera -> tag from an AprilTag
36h11 (id 0, printed size --tag-size) detected in the COLOUR image with the
colour intrinsics. The captain freedrives the arm to 15-20 varied poses that
all see the tag from 0.3-0.8 m; this tool takes a sample only when:

  * the arm is still: every joint |velocity| < --vmax for --dwell s, AND the
    gyro (if /camera/imu is alive) < 0.03 rad/s like the approach demo;
  * the tag is fully inside the frame with --margin px, 0.3-0.8 m away,
    mean reprojection error < --max-reproj px;
  * the tag pose is stable over --frames consecutive frames (spread < 2 mm);
  * the pose differs from the last accepted one (> 8 deg or > 4 cm) — a
    repeated pose adds nothing to AX = XB and hides the real residual.

Every accepted/rejected frame prints its reason, so the captain sees the
count grow and knows why a pose was not taken. After >= --min samples the
solution is re-printed after each new sample (Park, Daniilidis, Tsai,
Horaud, Andreff via cv2.calibrateHandEye) with:
  * method agreement (translation mm / rotation deg between methods),
  * residual = spread of the TAG pose in base_link across all samples when
    each is mapped through X (what the map will actually feel), and the
    pairwise AX=XB residual,
  * the delta against the current guess in wrist_rig.launch.py.
Target: < 1 deg and < 5 mm. Stop with --target N (auto) or Ctrl-C.

Output calib/tool0_to_camera_<date>.yaml holds BOTH tool0->camera_color_optical_frame
(what was solved) and tool0->camera_link (what the launch publishes, derived
through the driver's own static camera_link->optical TF so the RealSense's
internal tree keeps working), plus samples, residuals and the plot.

  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 handeye_calibrate.py --target 18
  python3 handeye_calibrate.py --solve-only calib/samples-2026-08-26.yaml
"""
import argparse
import datetime as dt
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState

BASE, TOOL = 'base_link', 'tool0'
CAM_LINK, OPTICAL = 'camera_link', 'camera_color_optical_frame'
METHODS = {'park': cv2.CALIB_HAND_EYE_PARK, 'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
           'tsai': cv2.CALIB_HAND_EYE_TSAI, 'horaud': cv2.CALIB_HAND_EYE_HORAUD,
           'andreff': cv2.CALIB_HAND_EYE_ANDREFF}


def T(p, Rm):
    M = np.eye(4)
    M[:3, :3] = Rm
    M[:3, 3] = p
    return M


def inv(M):
    Rt = M[:3, :3].T
    out = np.eye(4)
    out[:3, :3] = Rt
    out[:3, 3] = -Rt @ M[:3, 3]
    return out


def rot_deg(Rm):
    return float(np.degrees(R.from_matrix(Rm).magnitude()))


def tf_to_T(t):
    q, p = t.transform.rotation, t.transform.translation
    return T([p.x, p.y, p.z], R.from_quat([q.x, q.y, q.z, q.w]).as_matrix())


# ---------------------------------------------------------------- solving --
def solve(samples, methods=METHODS):
    """samples: list of dicts with T_base_tool (4x4) and T_cam_tag (4x4).
    Returns {method: {'X': 4x4, residuals...}}."""
    Rg = [np.asarray(s['T_base_tool'])[:3, :3] for s in samples]
    tg = [np.asarray(s['T_base_tool'])[:3, 3] for s in samples]
    Rt = [np.asarray(s['T_cam_tag'])[:3, :3] for s in samples]
    tt = [np.asarray(s['T_cam_tag'])[:3, 3] for s in samples]
    out = {}
    for name, m in methods.items():
        try:
            Rx, tx = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=m)
        except cv2.error as e:
            out[name] = {'error': str(e).splitlines()[-1]}
            continue
        X = T(tx.ravel(), Rx)
        out[name] = {'X': X, **residuals(samples, X)}
    return out


def residuals(samples, X):
    """Tag pose in base_link through X for every sample -> spread; plus
    pairwise AX = XB residuals."""
    tags = [np.asarray(s['T_base_tool']) @ X @ np.asarray(s['T_cam_tag']) for s in samples]
    ts = np.array([M[:3, 3] for M in tags])
    mean_t = ts.mean(axis=0)
    Rm = sum(M[:3, :3] for M in tags) / len(tags)
    U, _, Vt = np.linalg.svd(Rm)
    Rmean = U @ Vt
    dev_t = np.linalg.norm(ts - mean_t, axis=1)
    dev_r = np.array([rot_deg(M[:3, :3] @ Rmean.T) for M in tags])
    pair_t, pair_r = [], []
    n = len(samples)
    for i in range(n):
        for j in range(i + 1, n):
            A = inv(np.asarray(samples[i]['T_base_tool'])) @ np.asarray(samples[j]['T_base_tool'])
            B = np.asarray(samples[i]['T_cam_tag']) @ inv(np.asarray(samples[j]['T_cam_tag']))
            E = inv(A @ X) @ (X @ B)
            pair_t.append(np.linalg.norm(E[:3, 3]))
            pair_r.append(rot_deg(E[:3, :3]))
    return {'tag_in_base_mean': mean_t.tolist(),
            'tag_spread_rms_mm': float(np.sqrt((dev_t ** 2).mean()) * 1e3),
            'tag_spread_max_mm': float(dev_t.max() * 1e3),
            'tag_rot_rms_deg': float(np.sqrt((dev_r ** 2).mean())),
            'tag_rot_max_deg': float(dev_r.max()),
            'per_sample_mm': (dev_t * 1e3).round(1).tolist(),
            'per_sample_deg': dev_r.round(2).tolist(),
            'pair_rms_mm': float(np.sqrt(np.mean(np.square(pair_t))) * 1e3) if pair_t else None,
            'pair_rms_deg': float(np.sqrt(np.mean(np.square(pair_r)))) if pair_r else None}


def method_agreement(results):
    names = [n for n, r in results.items() if 'X' in r]
    rows = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            D = inv(results[a]['X']) @ results[b]['X']
            rows.append((a, b, float(np.linalg.norm(D[:3, 3]) * 1e3), rot_deg(D[:3, :3])))
    return rows


def report(results, guess=None, chosen='park'):
    lines = []
    for name, r in results.items():
        if 'X' not in r:
            lines.append(f'  {name:10s} FAILED: {r["error"]}')
            continue
        X = r['X']
        lines.append(f'  {name:10s} t={np.round(X[:3, 3] * 1e3, 1)} mm  '
                     f'rpy={np.round(R.from_matrix(X[:3, :3]).as_euler("xyz", degrees=True), 2)} deg  '
                     f'| tag spread rms {r["tag_spread_rms_mm"]:.1f} mm (max {r["tag_spread_max_mm"]:.1f}), '
                     f'rot rms {r["tag_rot_rms_deg"]:.2f} deg (max {r["tag_rot_max_deg"]:.2f}) '
                     f'| pairwise rms {r["pair_rms_mm"]:.1f} mm / {r["pair_rms_deg"]:.2f} deg')
    agr = method_agreement(results)
    if agr:
        worst = max(agr, key=lambda x: x[2])
        lines.append(f'  method agreement: worst pair {worst[0]}/{worst[1]} '
                     f'{worst[2]:.1f} mm / {worst[3]:.2f} deg; '
                     + ', '.join(f'{a}-{b} {t:.1f}mm/{r:.2f}deg' for a, b, t, r in agr))
    if guess is not None and 'X' in results.get(chosen, {}):
        D = inv(guess) @ results[chosen]['X']
        lines.append(f'  vs current guess (tool0->optical): {np.linalg.norm(D[:3, 3]) * 1e3:.1f} mm, '
                     f'{rot_deg(D[:3, :3]):.2f} deg  (guess t={np.round(guess[:3, 3] * 1e3, 1)} mm, '
                     f'{chosen} t={np.round(results[chosen]["X"][:3, 3] * 1e3, 1)} mm)')
    return '\n'.join(lines)


# ------------------------------------------------------------- collecting --
class HandEye(Node):
    def __init__(self, a):
        super().__init__('handeye_calibrate')
        self.a = a
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, a.info, self._info, qos)
        self.create_subscription(Image, a.image, self._image, qos)
        self.create_subscription(JointState, '/joint_states', self._js, 50)
        self.create_subscription(Imu, a.imu, self._imu, qos)
        self.K = self.D = None
        self.js = None
        self.still_since = None
        self.gyro = []                    # (t, |w|)
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        h = a.tag_size / 2
        self.objp = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        self.window = []                  # recent (T_cam_tag, corners) while still
        self.samples = []
        self.last_msg = ''
        self.last_print = 0
        self.n_img = 0
        self.T_cl_opt = None              # camera_link -> optical from the driver
        self.guess = None                 # tool0 -> optical, current launch guess
        self.stamp_offset = None

    # -- inputs
    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64) if m.d else np.zeros(5)
            self.w, self.h = m.width, m.height
            print(f'camera_info {m.width}x{m.height} frame {m.header.frame_id} '
                  f'fx {self.K[0, 0]:.1f} fy {self.K[1, 1]:.1f}')

    def _js(self, m):
        self.js = m
        still = bool(m.velocity) and max(abs(v) for v in m.velocity) < self.a.vmax
        now = time.time()
        if still:
            self.still_since = self.still_since or now
        else:
            self.still_since = None
            self.window = []

    def _imu(self, m):
        w = m.angular_velocity
        self.gyro.append((time.time(), float(np.linalg.norm([w.x, w.y, w.z]))))
        self.gyro = [g for g in self.gyro if time.time() - g[0] < 1.0]

    def lookup_static(self):
        if self.T_cl_opt is None:
            try:
                self.T_cl_opt = tf_to_T(self.tfb.lookup_transform(CAM_LINK, OPTICAL, Time()))
                self.guess = tf_to_T(self.tfb.lookup_transform(TOOL, OPTICAL, Time()))
                print(f'driver static {CAM_LINK}->{OPTICAL}: t={np.round(self.T_cl_opt[:3, 3], 4)}; '
                      f'current launch guess {TOOL}->{OPTICAL}: t={np.round(self.guess[:3, 3], 4)} '
                      f'rpy={np.round(R.from_matrix(self.guess[:3, :3]).as_euler("xyz", degrees=True), 1)}')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

    # -- gating helpers
    def status(self, s, force=False):
        if force or (time.time() - self.last_print > 0.5 and s != self.last_msg):
            self.last_print = time.time()
            self.last_msg = s
            print(f'\r  [{len(self.samples)}/{self.a.target} samples] {s:<90s}', end='', flush=True)

    def arm_still(self):
        if self.js is None:
            return False, 'no /joint_states'
        if not self.js.velocity:
            return False, '/joint_states has no velocities'
        if self.still_since is None:
            return False, 'arm moving'
        held = time.time() - self.still_since
        if held < self.a.dwell:
            return False, f'settling {held:.1f}/{self.a.dwell:.0f} s'
        if self.gyro:
            g = max(v for _, v in self.gyro)
            if g > 0.03:
                return False, f'gyro {g:.3f} rad/s > 0.03'
        return True, 'still'

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        if ids is None:
            return None, 'no tag in view'
        for c, i in zip(corners, ids.ravel()):
            if int(i) != self.a.tag_id:
                continue
            c = c[0].astype(np.float64)
            m = self.a.margin
            if (c[:, 0] < m).any() or (c[:, 1] < m).any() or \
               (c[:, 0] > self.w - m).any() or (c[:, 1] > self.h - m).any():
                return None, f'tag too close to the image border (< {m} px)'
            ok, rvec, tvec = cv2.solvePnP(self.objp, c, self.K, self.D,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None, 'PnP failed'
            proj, _ = cv2.projectPoints(self.objp, rvec, tvec, self.K, self.D)
            err = float(np.linalg.norm(proj.reshape(-1, 2) - c, axis=1).mean())
            rng = float(np.linalg.norm(tvec))
            side = float(np.linalg.norm(c[1] - c[0]))
            if err > self.a.max_reproj:
                return None, f'reprojection error {err:.2f} px > {self.a.max_reproj}'
            if not (self.a.min_range <= rng <= self.a.max_range):
                return None, f'tag at {rng:.2f} m, want {self.a.min_range}-{self.a.max_range}'
            return (T(tvec.ravel(), cv2.Rodrigues(rvec)[0]), c, err, rng, side), 'ok'
        return None, f'tag id {self.a.tag_id} not in view (saw {ids.ravel().tolist()})'

    # -- main per-image path
    def _image(self, m):
        if self.K is None:
            return
        self.n_img += 1
        self.lookup_static()
        if self.stamp_offset is None and self.n_img > 5:
            self.stamp_offset = time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            print(f'image stamp offset now-stamp = {self.stamp_offset * 1e3:.0f} ms')
        if m.encoding not in ('rgb8', 'bgr8'):
            self.status(f'unsupported encoding {m.encoding}')
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        det, why = self.detect(img)
        if det is None:
            self.window = []
            self.status(why)
            return
        T_cam_tag, corners, err, rng, side = det
        still, why = self.arm_still()
        if not still:
            self.window = []
            self.status(f'tag ok ({rng:.2f} m, {err:.2f} px) — {why}')
            return
        self.window.append(T_cam_tag)
        if len(self.window) < self.a.frames:
            self.status(f'tag ok ({rng:.2f} m, {err:.2f} px), still — averaging '
                        f'{len(self.window)}/{self.a.frames}')
            return
        ts = np.array([M[:3, 3] for M in self.window])
        spread = float(np.linalg.norm(ts - ts.mean(axis=0), axis=1).max())
        if spread > 0.002:
            self.window = self.window[-2:]
            self.status(f'tag pose jitter {spread * 1e3:.1f} mm > 2 mm — hold steadier')
            return
        Rmean = sum(M[:3, :3] for M in self.window) / len(self.window)
        U, _, Vt = np.linalg.svd(Rmean)
        T_cam_tag = T(np.median(ts, axis=0), U @ Vt)
        try:
            T_base_tool = tf_to_T(self.tfb.lookup_transform(BASE, TOOL, Time()))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.status(f'no TF {BASE}->{TOOL} ({type(e).__name__})')
            return
        if self.samples:
            D = inv(np.asarray(self.samples[-1]['T_base_tool'])) @ T_base_tool
            dt_, dr = np.linalg.norm(D[:3, 3]), rot_deg(D[:3, :3])
            if dt_ < 0.04 and dr < 8:
                self.status(f'same pose as the last sample ({dt_ * 1e3:.0f} mm, {dr:.1f} deg) — '
                            'move to a NEW pose')
                self.window = []
                return
        self.samples.append({
            'T_base_tool': T_base_tool.tolist(), 'T_cam_tag': T_cam_tag.tolist(),
            'joints': dict(zip(self.js.name, [float(v) for v in self.js.position])),
            'reproj_px': err, 'range_m': rng, 'tag_side_px': side,
            'when': dt.datetime.now().isoformat(timespec='seconds')})
        self.window = []
        self.still_since = None          # force a fresh dwell for the next one
        n = len(self.samples)
        print(f'\n\a>>> sample {n}/{self.a.target} ACCEPTED: tag at {rng:.2f} m, reproj {err:.2f} px, '
              f'tool0 at {np.round(T_base_tool[:3, 3], 3)} — move to the next pose', flush=True)
        self.save_samples()
        if n >= self.a.min:
            res = solve(self.samples)
            print(report(res, self.guess, self.a.method), flush=True)

    def save_samples(self):
        with open(self.a.samples_file, 'w') as f:
            yaml.safe_dump({'tag_id': self.a.tag_id, 'tag_size_m': self.a.tag_size,
                            'image': self.a.image, 'info': self.a.info,
                            'T_camera_link_to_optical': self.T_cl_opt.tolist() if self.T_cl_opt is not None else None,
                            'guess_tool0_to_optical': self.guess.tolist() if self.guess is not None else None,
                            'samples': self.samples}, f, sort_keys=False)


# ---------------------------------------------------------------- output --
def write_result(a, samples, results, T_cl_opt, guess, out_dir):
    res = results[a.method]
    if 'X' not in res:
        sys.exit(f'{a.method} failed: {res["error"]}')
    X = res['X']                                   # tool0 -> optical
    X_link = X @ inv(T_cl_opt)                     # tool0 -> camera_link
    q = R.from_matrix(X_link[:3, :3]).as_quat()
    agr = method_agreement(results)
    d = {'calibrated': dt.datetime.now().isoformat(timespec='seconds'),
         'method': a.method, 'samples': len(samples), 'tag_id': a.tag_id,
         'tag_size_m': a.tag_size,
         'tool0_to_camera_link': {
             'parent': TOOL, 'child': CAM_LINK,
             'xyz': [float(v) for v in X_link[:3, 3]],
             'quaternion_xyzw': [float(v) for v in q],
             'rpy_deg': [float(v) for v in R.from_matrix(X_link[:3, :3]).as_euler('xyz', degrees=True)]},
         'tool0_to_camera_color_optical_frame': {
             'parent': TOOL, 'child': OPTICAL,
             'xyz': [float(v) for v in X[:3, 3]],
             'quaternion_xyzw': [float(v) for v in R.from_matrix(X[:3, :3]).as_quat()],
             'matrix': X.tolist()},
         'camera_link_to_optical_used': T_cl_opt.tolist(),
         'residuals': {k: v for k, v in res.items() if k != 'X'},
         'all_methods': {n: ({'xyz': r['X'][:3, 3].tolist(),
                              'rpy_deg': R.from_matrix(r['X'][:3, :3]).as_euler('xyz', degrees=True).tolist(),
                              'tag_spread_rms_mm': r['tag_spread_rms_mm'],
                              'tag_rot_rms_deg': r['tag_rot_rms_deg']} if 'X' in r else r)
                         for n, r in results.items()},
         'method_agreement': [{'a': x[0], 'b': x[1], 'mm': x[2], 'deg': x[3]} for x in agr],
         'target': {'translation_mm': 5.0, 'rotation_deg': 1.0},
         'meets_target': bool(res['tag_spread_rms_mm'] < 5.0 and res['tag_rot_rms_deg'] < 1.0)}
    if guess is not None:
        D = inv(guess) @ X
        d['vs_previous_guess'] = {'translation_mm': float(np.linalg.norm(D[:3, 3]) * 1e3),
                                  'rotation_deg': rot_deg(D[:3, :3]),
                                  'guess_tool0_to_optical_xyz': guess[:3, 3].tolist()}
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'tool0_to_camera_{dt.date.today().isoformat()}.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(d, f, sort_keys=False)
    print(f'\n>>> {path}\n    tool0 -> camera_link: xyz {np.round(X_link[:3, 3], 4)} '
          f'rpy {np.round(d["tool0_to_camera_link"]["rpy_deg"], 2)} deg\n'
          f'    residual (tag spread through X): {res["tag_spread_rms_mm"]:.1f} mm rms / '
          f'{res["tag_rot_rms_deg"]:.2f} deg rms  -> target {"MET" if d["meets_target"] else "NOT met"}')
    plot(samples, results, a.method, path.replace('.yaml', '.png'))
    return path


def plot(samples, results, method, out):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    r = results[method]
    n = len(samples)
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    ax[0].bar(range(1, n + 1), r['per_sample_mm'], color='#0e7d86')
    ax[0].axhline(5, color='#b23a3a', ls='--', lw=1, label='5 mm target')
    ax[0].set_title(f'tag position deviation through X ({method})'); ax[0].set_xlabel('sample')
    ax[0].set_ylabel('mm'); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].bar(range(1, n + 1), r['per_sample_deg'], color='#a06c14')
    ax[1].axhline(1, color='#b23a3a', ls='--', lw=1, label='1 deg target')
    ax[1].set_title('tag rotation deviation'); ax[1].set_xlabel('sample'); ax[1].set_ylabel('deg')
    ax[1].legend(); ax[1].grid(alpha=.3)
    names = [k for k, v in results.items() if 'X' in v]
    ax[2].bar(names, [results[k]['tag_spread_rms_mm'] for k in names], color='#2f7d4f')
    ax[2].set_title('rms residual per method'); ax[2].set_ylabel('mm'); ax[2].grid(alpha=.3)
    for i, k in enumerate(names):
        ax[2].text(i, results[k]['tag_spread_rms_mm'], f'{results[k]["tag_rot_rms_deg"]:.2f}°',
                   ha='center', va='bottom', fontsize=8)
    fig.suptitle(f'hand-eye calibration, {n} samples')
    fig.tight_layout(); fig.savefig(out, dpi=130)
    print('    plot ->', out)


def main():
    sys.stdout.reconfigure(line_buffering=True)     # status lines through pipes/tee
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', default='/camera/rgb')
    ap.add_argument('--info', default='/camera/camera/color/camera_info')
    ap.add_argument('--imu', default='/camera/imu')
    ap.add_argument('--tag-id', type=int, default=0)
    ap.add_argument('--tag-size', type=float, default=0.120, help='printed black square, m')
    ap.add_argument('--target', type=int, default=18, help='stop after this many samples')
    ap.add_argument('--min', type=int, default=8, help='solve from this many samples on')
    ap.add_argument('--dwell', type=float, default=1.0)
    ap.add_argument('--vmax', type=float, default=0.01, help='rad/s per joint = still')
    ap.add_argument('--frames', type=int, default=10, help='frames averaged per sample')
    ap.add_argument('--margin', type=int, default=40, help='px from the image border')
    ap.add_argument('--max-reproj', type=float, default=1.0, help='px')
    ap.add_argument('--min-range', type=float, default=0.3)
    ap.add_argument('--max-range', type=float, default=0.8)
    ap.add_argument('--method', default='park', choices=list(METHODS))
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calib'))
    ap.add_argument('--solve-only', help='samples yaml: re-solve offline, no ROS')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    a.samples_file = os.path.join(a.out, f'samples-{dt.date.today().isoformat()}.yaml')

    if a.solve_only:
        d = yaml.safe_load(open(a.solve_only))
        samples = d['samples']
        guess = np.array(d['guess_tool0_to_optical']) if d.get('guess_tool0_to_optical') else None
        res = solve(samples)
        print(f'{len(samples)} samples from {a.solve_only}')
        print(report(res, guess, a.method))
        write_result(a, samples, res, np.array(d['T_camera_link_to_optical']), guess, a.out)
        return

    rclpy.init()
    node = HandEye(a)
    print(f'hand-eye: tag {a.tag_id} ({a.tag_size * 1e3:.0f} mm) on {a.image}; target {a.target} '
          f'poses, 0.3-0.8 m, still {a.dwell:.0f} s each. Ctrl-C ends early.')
    try:
        while rclpy.ok() and len(node.samples) < a.target:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    print()
    if len(node.samples) < 3:
        print('fewer than 3 samples — nothing to solve')
        return
    if node.T_cl_opt is None:
        sys.exit(f'never saw the static TF {CAM_LINK}->{OPTICAL} (driver down?)')
    res = solve(node.samples)
    print(report(res, node.guess, a.method))
    write_result(a, node.samples, res, node.T_cl_opt, node.guess, a.out)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
