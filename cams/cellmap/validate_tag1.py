#!/usr/bin/env python3
"""
Camera-only validation of the hand-eye calibration (host, domain 0).

Replaces the arm-measured corner-touch validation (tools/tag_corners.py):
on this cell the flange carries ONLY the camera — no gripper, no pointer —
so the pendant TCP is meaningless and there is nothing to touch corners
with (captain, 2026-08-27; the active TCP measured 533 mm off the flange,
a leftover that would have put every corner half a metre out).

Instead the camera validates itself against geometry the calibration cannot
fake. Every tag pose is carried into base_link through exactly the chain the
map will be built on:

  base_link -> tool0 (TF, /joint_states) -> camera_link (CALIBRATED)
            -> camera_color_optical_frame (RealSense static) -> tag (PnP)

Three independent checks, none of which needs a tape on the robot:

(a) CONSISTENCY. A static tag must land in the same base_link place from
    every viewpoint. Wander is the calibration error made visible: a 1.6 deg
    error in X swings the tag ~14 mm at 0.5 m range, so a position spread
    under 5 mm bounds the rotation error well inside the 1 deg guide. This is
    the check that actually decides, because it is differential — it cannot
    be satisfied by a wrong-but-consistent transform.
(b) SCALE. The base_link distance between the tag 0 and tag 1 centres,
    against the captain's tape measure (--expected-mm). Catches a scale or
    translation error that (a) is blind to. Accept < 10 mm.
(c) TABLE PLANE. Both tags lie flat on the same table, so they must come out
    at the same base_link z (< 5 mm) with parallel normals (< 1 deg). Catches
    a tilt error in X, which is the component the fronto-parallel calibration
    samples constrained worst.

  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 validate_tag1.py --expected-mm 512
  python3 validate_tag1.py --seconds 120 --out calib/validate-2026-08-27.yaml
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
from rclpy.duration import Duration
import tf2_ros
import yaml
from sensor_msgs.msg import CameraInfo, Image

BASE, TOOL, OPTICAL = 'base_link', 'tool0', 'camera_color_optical_frame'


def aruco_dict():
    if hasattr(cv2.aruco, 'Dictionary_get'):        # legacy first, see aruco_params
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def aruco_params():
    # PREFER the legacy constructor. OpenCV 4.6 exposes the 4.7-style
    # DetectorParameters() class too, but passing that object to 4.6's
    # detectMarkers SEGFAULTS (verified 2026-08-27). Only fall back to the
    # new-style constructor when the legacy name is genuinely gone.
    p = (cv2.aruco.DetectorParameters_create()
         if hasattr(cv2.aruco, 'DetectorParameters_create')
         else cv2.aruco.DetectorParameters())
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return p


def detect_all(gray, d, params):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(d, params).detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, d, parameters=params)


def tf_to_T(t):
    from scipy.spatial.transform import Rotation as R
    q, v = t.transform.rotation, t.transform.translation
    M = np.eye(4)
    M[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M[:3, 3] = [v.x, v.y, v.z]
    return M


def mean_rot_axis(vs):
    """Mean unit vector + per-sample angle to it (deg)."""
    m = np.mean(vs, axis=0)
    m /= np.linalg.norm(m)
    ang = np.degrees(np.arccos(np.clip(vs @ m, -1, 1)))
    return m, ang


class Validate(Node):
    def __init__(self, a):
        super().__init__('validate_tag1')
        self.a = a
        self.K = self.D = None
        self.dict, self.params = aruco_dict(), aruco_params()
        h = a.tag_size / 2.0
        self.objp = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        self.obs = {i: [] for i in a.tags}      # id -> list of dicts
        self.n_img = self.n_rej = 0
        self.t_last = 0.0
        self.why = {}
        self.tfb = tf2_ros.Buffer()
        # spin_thread=True gives the TF listener its OWN executor thread.
        # Without it the AprilTag corner refinement (1280x720, ~100 ms/frame)
        # saturates the single-threaded executor, /tf is only drained between
        # frames in ~400 ms bursts, and the buffer's newest base_link->tool0
        # ends up ~190 ms BEHIND the image stamp — so every stamped lookup
        # throws ExtrapolationException "into the future" and the sweep
        # silently collects nothing (verified 2026-08-27). Looking up at
        # "latest" instead would hide it but smear every pose by ~180 ms of
        # arm motion, which is exactly what this tool exists to measure.
        tf2_ros.TransformListener(self.tfb, self, spin_thread=True)
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, a.info, self._info, 5)
        self.create_subscription(Image, a.image, self._image, q)
        # Print on a TIMER, not at the end of _image: the frames that matter
        # most to the captain mid-sweep are the ones where the tag is NOT in
        # view, and those return early. A display that freezes exactly when
        # the captain has aimed away is worse than useless.
        self.create_timer(1.0, self.running)

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k, np.float64).reshape(3, 3)
            self.D = np.array(m.d, np.float64)
            self.w, self.h = m.width, m.height
            print(f'camera_info {m.width}x{m.height} frame {m.header.frame_id} '
                  f'fx {self.K[0,0]:.1f}')

    def _bump(self, why):
        self.n_rej += 1
        self.why[why] = self.why.get(why, 0) + 1

    def _image(self, m):
        if self.K is None:
            return
        self.n_img += 1
        # THROTTLE before doing any OpenCV work. AprilTag corner refinement on
        # 1280x720 costs ~100 ms; at the camera's 26 fps that is ~260% of one
        # core, the process saturates, and the TF listener starves however it
        # is threaded — its newest base_link->tool0 falls ~190 ms behind the
        # image stamps and every stamped lookup throws (verified 2026-08-27).
        # A validation sweep needs a few well-spread views per second, not 26.
        now = time.time()
        if now - self.t_last < 1.0 / self.a.max_fps:
            return
        self.t_last = now
        if m.encoding not in ('rgb8', 'bgr8'):
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if m.encoding == 'rgb8'
                            else cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_all(gray, self.dict, self.params)
        if ids is None:
            self._bump('no tag in view')
            return
        stamp = Time.from_msg(m.header.stamp)
        # SPLIT the chain deliberately. Only base_link->tool0 varies in time,
        # and it must be read AT THE IMAGE STAMP because the captain sweeps
        # the camera while it streams (reading "latest" would smear every
        # pose by the arm's motion over the ~150 ms stamp lag).
        # tool0->camera_color_optical_frame is constant by construction
        # (calibrated mount + the driver's own extrinsics), so it is read at
        # "latest". Reading the WHOLE chain at the stamp fails: something in
        # the camera's own tree is published as a slow DYNAMIC transform
        # rather than a static one, so its newest sample is 135-335 ms behind
        # the image and every lookup throws ExtrapolationException
        # (verified 2026-08-27 — this silently produced zero samples).
        try:
            # NO timeout= here. This runs inside the image callback on a
            # single-threaded executor, so a blocking wait starves the very
            # TF listener that would satisfy it: TF falls behind the images
            # and every later lookup throws ExtrapolationException for good
            # (verified 2026-08-27 — rejections pinned at ~6/s, the blocking
            # rate, after only 2 samples got through). The image stamp is
            # ~50 ms in the PAST of the newest TF, so a plain buffered
            # lookup succeeds immediately or fails immediately.
            T_base_tool = tf_to_T(self.tfb.lookup_transform(BASE, TOOL, stamp))
            T_tool_opt = tf_to_T(self.tfb.lookup_transform(TOOL, OPTICAL, Time()))
        except Exception as e:                                   # noqa: BLE001
            if os.environ.get('VT_DEBUG') and self.n_rej < 3:
                print(f'\nTF DEBUG stamp={stamp.nanoseconds/1e9:.3f} '
                      f'now={time.time():.3f}\n  {type(e).__name__}: {e}')
            self._bump(f'TF unavailable ({type(e).__name__})')
            return
        T_base_opt = T_base_tool @ T_tool_opt
        for c, i in zip(corners, ids.ravel()):
            i = int(i)
            if i not in self.obs:
                continue
            c = c[0].astype(np.float64)
            mg = self.a.margin
            if (c[:, 0] < mg).any() or (c[:, 1] < mg).any() or \
               (c[:, 0] > self.w - mg).any() or (c[:, 1] > self.h - mg).any():
                self._bump('tag at the image border')
                continue
            ok, rvec, tvec = cv2.solvePnP(self.objp, c, self.K, self.D,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                self._bump('PnP failed')
                continue
            proj, _ = cv2.projectPoints(self.objp, rvec, tvec, self.K, self.D)
            err = float(np.linalg.norm(proj.reshape(-1, 2) - c, axis=1).mean())
            rng = float(np.linalg.norm(tvec))
            if err > self.a.max_reproj:
                self._bump(f'reproj > {self.a.max_reproj} px')
                continue
            if not (self.a.min_range <= rng <= self.a.max_range):
                self._bump(f'range outside {self.a.min_range}-{self.a.max_range} m')
                continue
            Rct = cv2.Rodrigues(rvec)[0]
            T_cam_tag = np.eye(4)
            T_cam_tag[:3, :3], T_cam_tag[:3, 3] = Rct, tvec.ravel()
            T_base_tag = T_base_opt @ T_cam_tag
            n_cam = Rct[:, 2]
            obl = float(np.degrees(np.arccos(min(1.0, abs(n_cam @ np.array([0, 0, 1.0]))))))
            self.obs[i].append({'p': T_base_tag[:3, 3].copy(),
                                'n': T_base_tag[:3, 2].copy(),
                                'range': rng, 'obliquity': obl, 'reproj': err})

    # ---- statistics
    def stats(self, i):
        o = self.obs[i]
        if len(o) < 2:
            return None
        P = np.array([x['p'] for x in o])
        N = np.array([x['n'] for x in o])
        N = N / np.linalg.norm(N, axis=1, keepdims=True)
        mp = P.mean(axis=0)
        dev = np.linalg.norm(P - mp, axis=1) * 1e3
        mn, ang = mean_rot_axis(N)
        rr = np.array([x['range'] for x in o])
        ob = np.array([x['obliquity'] for x in o])
        return {'n': len(o), 'mean_p': mp, 'mean_n': mn,
                'pos_rms_mm': float(np.sqrt((dev ** 2).mean())),
                'pos_max_mm': float(dev.max()),
                'nrm_rms_deg': float(np.sqrt((ang ** 2).mean())),
                'nrm_max_deg': float(ang.max()),
                'range_min': float(rr.min()), 'range_max': float(rr.max()),
                'obl_min': float(ob.min()), 'obl_max': float(ob.max())}

    def running(self):
        parts = []
        for i in self.a.tags:
            s = self.stats(i)
            parts.append(f'tag{i} n/a' if s is None else
                         f'tag{i} n={s["n"]:4d} spread {s["pos_rms_mm"]:5.1f}mm rms '
                         f'(max {s["pos_max_mm"]:5.1f}) normal {s["nrm_rms_deg"]:4.2f}deg '
                         f'range {s["range_min"]:.2f}-{s["range_max"]:.2f} '
                         f'obliq {s["obl_min"]:.0f}-{s["obl_max"]:.0f}deg')
        top = sorted(self.why.items(), key=lambda kv: -kv[1])[:1]
        tail = f'  | rejected {self.n_rej}' + (f' ({top[0][0]})' if top else '')
        sys.stdout.write('\r  ' + '   '.join(parts) + tail + '   ')
        sys.stdout.flush()

    # ---- verdict
    def verdict(self):
        out = {'when': dt.datetime.now().isoformat(timespec='seconds'),
               'tags': {}, 'checks': {}, 'thresholds': {
                   'consistency_pos_rms_mm': self.a.tol_spread_mm,
                   'scale_mm': self.a.tol_scale_mm,
                   'plane_z_mm': self.a.tol_z_mm,
                   'plane_normal_deg': self.a.tol_normal_deg,
                   'guide_mm': 10.0, 'guide_deg': 1.0}}
        for i in self.a.tags:
            s = self.stats(i)
            if s:
                out['tags'][i] = {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                  for k, v in s.items()}
        lines, ok_all = [], True

        # (a) consistency, on the target tag
        s = self.stats(self.a.target_tag)
        if s is None:
            lines.append(f'(a) consistency : NO DATA for tag {self.a.target_tag}')
            ok_all = False
        else:
            ok = s['pos_rms_mm'] < self.a.tol_spread_mm
            ok_all &= ok
            implied = np.degrees(np.arctan2(s['pos_rms_mm'] * 1e-3,
                                            max(s['range_min'], 1e-6)))
            out['checks']['consistency'] = {
                'pos_rms_mm': s['pos_rms_mm'], 'pos_max_mm': s['pos_max_mm'],
                'normal_rms_deg': s['nrm_rms_deg'], 'normal_max_deg': s['nrm_max_deg'],
                'implied_rot_deg_at_min_range': float(implied), 'pass': bool(ok)}
            lines.append(
                f'(a) consistency : tag {self.a.target_tag} lands within '
                f'{s["pos_rms_mm"]:.1f} mm rms / {s["pos_max_mm"]:.1f} mm max over '
                f'{s["n"]} views ({s["range_min"]:.2f}-{s["range_max"]:.2f} m, '
                f'obliquity {s["obl_min"]:.0f}-{s["obl_max"]:.0f} deg); normal spread '
                f'{s["nrm_rms_deg"]:.2f} deg rms  -> {"PASS" if ok else "FAIL"} '
                f'(< {self.a.tol_spread_mm} mm)')
            if s['obl_max'] - s['obl_min'] < 20 or s['range_max'] - s['range_min'] < 0.15:
                lines.append('    WARNING: the sweep was not diverse enough (want '
                             '>20 deg of obliquity and >0.15 m of range span) — a '
                             'small spread from similar views does not bound much.')

        # (b) scale between the two tag centres
        a_id, b_id = self.a.tags[0], self.a.tags[-1]
        sa, sb = self.stats(a_id), self.stats(b_id)
        if sa and sb and a_id != b_id:
            dist = float(np.linalg.norm(sa['mean_p'] - sb['mean_p']) * 1e3)
            out['checks']['scale'] = {'measured_mm': dist,
                                      'expected_mm': self.a.expected_mm}
            if self.a.expected_mm:
                err = abs(dist - self.a.expected_mm)
                ok = err < self.a.tol_scale_mm
                ok_all &= ok
                out['checks']['scale'].update({'error_mm': err, 'pass': bool(ok)})
                lines.append(f'(b) scale       : tag{a_id}-tag{b_id} centres '
                             f'{dist:.1f} mm, tape {self.a.expected_mm:.1f} mm, '
                             f'error {err:.1f} mm -> {"PASS" if ok else "FAIL"} '
                             f'(< {self.a.tol_scale_mm} mm)')
            else:
                lines.append(f'(b) scale       : tag{a_id}-tag{b_id} centres '
                             f'{dist:.1f} mm — no tape measure given '
                             f'(--expected-mm), not judged')
        else:
            lines.append('(b) scale       : NO DATA (need both tags seen)')

        # (c) table plane
        if sa and sb and a_id != b_id:
            dz = abs(sa['mean_p'][2] - sb['mean_p'][2]) * 1e3
            dn = float(np.degrees(np.arccos(np.clip(sa['mean_n'] @ sb['mean_n'], -1, 1))))
            ok = dz < self.a.tol_z_mm and dn < self.a.tol_normal_deg
            ok_all &= ok
            out['checks']['plane'] = {'dz_mm': dz, 'normal_angle_deg': dn,
                                      'pass': bool(ok)}
            lines.append(f'(c) table plane : dz {dz:.1f} mm (< {self.a.tol_z_mm}), '
                         f'normals differ {dn:.2f} deg (< {self.a.tol_normal_deg}) '
                         f'-> {"PASS" if ok else "FAIL"}')
        else:
            lines.append('(c) table plane : NO DATA (need both tags seen)')

        out['pass'] = bool(ok_all)
        return out, lines


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', default='/camera/rgb')
    ap.add_argument('--info', default='/camera/camera/color/camera_info')
    ap.add_argument('--tags', type=int, nargs='+', default=[0, 1])
    ap.add_argument('--target-tag', type=int, default=1)
    ap.add_argument('--tag-size', type=float, default=0.120)
    ap.add_argument('--max-reproj', type=float, default=1.0, help='px')
    ap.add_argument('--min-range', type=float, default=0.3)
    ap.add_argument('--max-range', type=float, default=0.8)
    ap.add_argument('--margin', type=int, default=40, help='px from the image border')
    ap.add_argument('--max-fps', type=float, default=8.0,
                    help='frames/s actually processed; above ~10 the detector '
                         'saturates the process and starves the TF listener')
    ap.add_argument('--expected-mm', type=float, default=None,
                    help='captain tape measure, tag0 centre to tag1 centre (mm)')
    ap.add_argument('--tol-spread-mm', type=float, default=5.0)
    ap.add_argument('--tol-scale-mm', type=float, default=10.0)
    ap.add_argument('--tol-z-mm', type=float, default=5.0)
    ap.add_argument('--tol-normal-deg', type=float, default=1.0)
    ap.add_argument('--seconds', type=float, default=0, help='auto-stop (0 = Ctrl-C)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    if a.target_tag not in a.tags:
        sys.exit(f'--target-tag {a.target_tag} must be one of --tags {a.tags}')

    rclpy.init()
    n = Validate(a)
    print(f'validate: tags {a.tags} ({a.tag_size*1e3:.0f} mm) on {a.image}; target tag '
          f'{a.target_tag}; reproj <= {a.max_reproj} px, range {a.min_range}-{a.max_range} m.')
    print('Sweep the camera around the tags — many distances and SLANTED views. '
          'Ctrl-C when done.')
    t0 = time.time()
    try:
        while rclpy.ok() and (a.seconds <= 0 or time.time() - t0 < a.seconds):
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    print('\n')
    res, lines = n.verdict()
    print('\n'.join(lines))
    print(f'\nVERDICT: {"ACCEPT" if res["pass"] else "DIAGNOSE"}  '
          f'(guide: < 10 mm / 1 deg end-to-end)')
    if not res['pass']:
        print('  A spread that GROWS with viewing angle/range = rotation error in X;\n'
              '  a constant offset between tags = translation/scale error;\n'
              '  dz or non-parallel normals = tilt error (the component the\n'
              '  fronto-parallel calibration samples constrained worst).')
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calib',
                                f'validate-{dt.date.today().isoformat()}.yaml')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        yaml.safe_dump(res, f, default_flow_style=False, sort_keys=False)
    print(f'\nwrote {out}')
    n.destroy_node()
    rclpy.shutdown()
    return 0 if res['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
