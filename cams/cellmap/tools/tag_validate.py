#!/usr/bin/env python3
"""
Validate the hand-eye calibration with a SECOND landmark (host, domain 0).

Detects tag <id> (36h11) in the wrist camera's colour stream, solves its pose
(IPPE_SQUARE, refined corners, printed size) and carries it into base_link
through the live kinematic chain

  base_link -> tool0 (TF) -> camera_link (calibrated) -> camera_color_optical_frame

then compares with the same tag measured by the ARM (tools/tag_corners.py:
the TCP touched three corners). The disagreement is the end-to-end error of
the chain the map will be built on. Accept < 10 mm / 1 deg; otherwise
re-calibrate before scanning (README "Validation").

Keeps printing per frame (and a running median) so the captain can move the
arm around: a residual that GROWS with viewing angle or range points at a
rotation error in the calibration; a constant offset at a translation error.

  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 tag_validate.py --tag anchor/tag1.yaml
  python3 tag_validate.py --tag anchor/tag1.yaml --seconds 30 --out calib/validate-<date>.yaml
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
from scipy.spatial.transform import Rotation as R
import tf2_ros
import yaml
from sensor_msgs.msg import CameraInfo, Image

BASE = 'base_link'


def T(p, Rm):
    M = np.eye(4)
    M[:3, :3] = Rm
    M[:3, 3] = p
    return M


def load_tag(path):
    d = yaml.safe_load(open(path))
    if d.get('frame_id') != BASE:
        sys.exit(f'{path}: frame_id must be {BASE}')
    return d, T(np.array(d['center']), np.array(d['rotation_rows']))


class Validate(Node):
    def __init__(self, a, T_base_tag):
        super().__init__('tag_validate')
        self.a = a
        self.T_base_tag = T_base_tag
        self.tfb = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.K = self.D = None
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        h = a.tag_size / 2
        self.objp = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, a.info, self._info, qos)
        self.create_subscription(Image, a.image, self._image, qos)
        self.rows = []          # (range, dt_mm, dr_deg, dx, dy, dz)
        self.last_print = 0
        self.n_img = 0

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64) if m.d else np.zeros(5)
            self.frame = m.header.frame_id
            print(f'camera_info {m.width}x{m.height} frame {self.frame}')

    def _image(self, m):
        if self.K is None:
            return
        self.n_img += 1
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        det = None
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if int(i) == self.a.tag_id:
                    ok, rvec, tvec = cv2.solvePnP(self.objp, c[0].astype(np.float64), self.K,
                                                  self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        det = T(tvec.ravel(), cv2.Rodrigues(rvec)[0])
        if det is None:
            self.status(f'tag {self.a.tag_id} not in view')
            return
        try:
            t = self.tfb.lookup_transform(BASE, m.header.frame_id or self.frame,
                                          Time.from_msg(m.header.stamp),
                                          timeout=Duration(seconds=0.15))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.status(f'no TF {BASE}->camera at the image stamp ({type(e).__name__})')
            return
        q, p = t.transform.rotation, t.transform.translation
        T_base_cam = T([p.x, p.y, p.z], R.from_quat([q.x, q.y, q.z, q.w]).as_matrix())
        seen = T_base_cam @ det
        dp = seen[:3, 3] - self.T_base_tag[:3, 3]
        dr = np.degrees(R.from_matrix(seen[:3, :3] @ self.T_base_tag[:3, :3].T).magnitude())
        rng = float(np.linalg.norm(det[:3, 3]))
        self.rows.append((rng, float(np.linalg.norm(dp) * 1e3), float(dr), *(dp * 1e3)))
        med = np.median([r[1] for r in self.rows])
        medr = np.median([r[2] for r in self.rows])
        self.status(f'tag at {rng:.2f} m: camera-vs-arm {np.linalg.norm(dp)*1e3:5.1f} mm '
                    f'({dp[0]*1e3:+.0f},{dp[1]*1e3:+.0f},{dp[2]*1e3:+.0f}) {dr:.2f} deg | '
                    f'median of {len(self.rows)}: {med:.1f} mm / {medr:.2f} deg')

    def status(self, s):
        if time.time() - self.last_print > 0.3:
            self.last_print = time.time()
            print(f'\r  {s:<110s}', end='', flush=True)

    def summary(self):
        if not self.rows:
            print('\nno detections')
            return None
        A = np.array(self.rows)
        d = {'tag_id': self.a.tag_id, 'tag_file': self.a.tag, 'frames': int(len(A)),
             'when': dt.datetime.now().isoformat(timespec='seconds'),
             'translation_mm': {'median': float(np.median(A[:, 1])), 'p90': float(np.percentile(A[:, 1], 90)),
                                'max': float(A[:, 1].max())},
             'rotation_deg': {'median': float(np.median(A[:, 2])), 'p90': float(np.percentile(A[:, 2], 90)),
                              'max': float(A[:, 2].max())},
             'offset_xyz_mm_median': [float(np.median(A[:, k])) for k in (3, 4, 5)],
             'range_m': {'min': float(A[:, 0].min()), 'max': float(A[:, 0].max())},
             'accept': bool(np.median(A[:, 1]) < 10.0 and np.median(A[:, 2]) < 1.0)}
        print('\n' + yaml.safe_dump(d, sort_keys=False))
        if self.a.out:
            os.makedirs(os.path.dirname(self.a.out) or '.', exist_ok=True)
            with open(self.a.out, 'w') as f:
                yaml.safe_dump(d, f, sort_keys=False)
            print('->', self.a.out)
        return d


def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', required=True, help='tag<id>.yaml from tag_corners.py')
    ap.add_argument('--image', default='/camera/rgb')
    ap.add_argument('--info', default='/camera/camera/color/camera_info')
    ap.add_argument('--seconds', type=float, default=0, help='auto-stop (0 = Ctrl-C)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    tag, T_base_tag = load_tag(a.tag)
    a.tag_id, a.tag_size = tag['tag_id'], tag['size_m']
    rclpy.init()
    node = Validate(a, T_base_tag)
    print(f'validating tag {a.tag_id} ({a.tag_size*1e3:.0f} mm), arm-measured centre '
          f'{np.round(T_base_tag[:3, 3], 4)}; move the arm around while it watches ...')
    t_end = time.time() + a.seconds if a.seconds else None
    try:
        while rclpy.ok() and (t_end is None or time.time() < t_end):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.summary()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
