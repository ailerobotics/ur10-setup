#!/usr/bin/env python3
"""
Anchor the cuVSLAM map to the robot: publish static TF  base_link -> map.

  T_base_map = T_base_tag (arm-measured, tools/tag_corners.py)
               * inv( T_map_cam (cuVSLAM TF) * T_cam_tag (OpenCV PnP) )

Runs on the HOST (OpenCV 4.6 with aruco/AprilTag, rclpy, tf2), domain 0.
Detects the 36h11 tag in the handheld colour stream, solves the planar pose
(IPPE_SQUARE, 4 refined corners, printed size), looks up map ->
camera_color_optical_frame at the IMAGE STAMP, and averages --samples
estimates (median translation, chordal-mean rotation, 3-sigma outlier
rejection). Refuses to publish if the spread exceeds --max-spread-mm /
--max-spread-deg: a jittery anchor means the tracker was drifting or the tag
was too far — hold the camera 0.4-1.0 m from the tag, fronto-parallel-ish, still.

Why the anchor must exist BEFORE nvblox starts: nvblox integrates each depth
frame in base_link through this TF; frames integrated before it exists land
in the wrong place and cannot be un-integrated. scan procedure: run this,
see "published", THEN start the nvblox/segmenter stage.

After publishing it keeps detecting and prints the live residual between the
arm-measured tag pose and the tag as seen through the SLAM chain — that
number, read when the captain returns to the tag at the END of the walk, is
the map's registration error (drift after loop closure).

  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 tag_anchor.py --tag anchor/tag0.yaml
  python3 tag_anchor.py --replay anchor/anchor-2026-08-26.yaml    # re-publish only
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
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image

BASE = 'base_link'


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


def mean_rotation(rots):
    """Chordal L2 mean of rotation matrices (SVD projection)."""
    M = sum(rots) / len(rots)
    U, _, Vt = np.linalg.svd(M)
    Rm = U @ Vt
    if np.linalg.det(Rm) < 0:
        U[:, -1] *= -1
        Rm = U @ Vt
    return Rm


def load_tag(path):
    d = yaml.safe_load(open(path))
    if d.get('frame_id') != BASE:
        sys.exit(f'{path}: frame_id must be {BASE}')
    Rm = np.array(d['rotation_rows'])
    return d, T(np.array(d['center']), Rm)


class Anchor(Node):
    def __init__(self, a):
        super().__init__('tag_anchor')
        self.a = a
        self.tfb = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.static_pub = tf2_ros.StaticTransformBroadcaster(self)
        self.K = self.D = None
        self.samples = []       # T_base_map candidates
        self.published = None
        self.frame = None
        self.last_print = 0
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        h = a.tag_size / 2
        # A(TL) B(TR) C(BR) D(BL): x right, y up, z out of the face — the same
        # frame tag_corners.py builds from the touched corners.
        self.objp = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, a.info, self._info, qos)
        self.create_subscription(Image, a.image, self._image, qos)
        self.depth = None
        if a.depth:
            self.create_subscription(Image, a.depth, self._depth, qos)
        self.n_img = self.n_det = 0
        self.checks = []

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64) if m.d else np.zeros(5)
            self.frame = m.header.frame_id
            print(f'camera_info: {m.width}x{m.height} frame {self.frame} fx {self.K[0,0]:.1f}')

    def _depth(self, m):
        self.depth = m

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        if ids is None:
            return None
        for c, i in zip(corners, ids.ravel()):
            if int(i) == self.a.tag_id:
                ok, rvec, tvec = cv2.solvePnP(self.objp, c[0].astype(np.float64), self.K,
                                              self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    return T(tvec.ravel(), cv2.Rodrigues(rvec)[0]), c[0]
        return None

    def _image(self, m):
        if self.K is None:
            return
        self.n_img += 1
        buf = np.frombuffer(m.data, np.uint8)
        if m.encoding in ('rgb8', 'bgr8'):
            img = buf.reshape(m.height, m.width, 3)
            if m.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif m.encoding == 'mono8':
            img = buf.reshape(m.height, m.width)
        else:
            print('unsupported encoding', m.encoding)
            return
        det = self.detect(img)
        if det is None:
            self.status('no tag in view')
            return
        T_cam_tag, px = det
        self.n_det += 1
        rng = np.linalg.norm(T_cam_tag[:3, 3])
        try:
            t = self.tfb.lookup_transform(self.a.map_frame, m.header.frame_id or self.frame,
                                          Time.from_msg(m.header.stamp),
                                          timeout=Duration(seconds=0.15))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.status(f'tag seen at {rng:.2f} m but no TF {self.a.map_frame}->camera '
                        f'({type(e).__name__}: cuVSLAM tracking?)')
            return
        q, p = t.transform.rotation, t.transform.translation
        T_map_cam = T([p.x, p.y, p.z], R.from_quat([q.x, q.y, q.z, q.w]).as_matrix())
        T_map_tag = T_map_cam @ T_cam_tag
        if self.a.check_only:
            try:
                tb = self.tfb.lookup_transform(BASE, self.a.map_frame, Time(),
                                               timeout=Duration(seconds=0.2))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                self.status(f'no TF {BASE}->{self.a.map_frame} yet (anchor not published?)')
                return
            qb, pb = tb.transform.rotation, tb.transform.translation
            T_base_map = T([pb.x, pb.y, pb.z], R.from_quat([qb.x, qb.y, qb.z, qb.w]).as_matrix())
            seen = T_base_map @ T_map_tag
            dp = seen[:3, 3] - self.T_base_tag[:3, 3]
            dR = R.from_matrix(seen[:3, :3] @ self.T_base_tag[:3, :3].T).magnitude()
            self.checks.append(np.linalg.norm(dp))
            self.status(f'CHECK tag {self.a.tag_id}: seen-through-chain vs arm-measured '
                        f'{np.linalg.norm(dp)*1e3:5.1f} mm ({dp[0]*1e3:+.0f},{dp[1]*1e3:+.0f},'
                        f'{dp[2]*1e3:+.0f}) {np.degrees(dR):.2f} deg  [median of '
                        f'{len(self.checks)}: {np.median(self.checks)*1e3:.1f} mm]  tag at {rng:.2f} m')
            return
        if self.published is None:
            if rng > self.a.max_range:
                self.status(f'tag at {rng:.2f} m — come closer than {self.a.max_range} m')
                return
            self.samples.append(self.T_base_tag @ inv(T_map_tag))
            self.status(f'sample {len(self.samples)}/{self.a.samples}, tag at {rng:.2f} m')
            if len(self.samples) >= self.a.samples:
                self.solve()
        else:
            # live residual: tag through the SLAM chain vs arm-measured
            pred = self.published @ T_map_tag           # tag in base_link via chain
            dp = pred[:3, 3] - self.T_base_tag[:3, 3]
            dR = R.from_matrix(pred[:3, :3] @ self.T_base_tag[:3, :3].T).magnitude()
            self.status(f'residual {np.linalg.norm(dp)*1e3:5.1f} mm '
                        f'({dp[0]*1e3:+.0f},{dp[1]*1e3:+.0f},{dp[2]*1e3:+.0f}) '
                        f'{np.degrees(dR):.2f} deg  tag at {rng:.2f} m')
        if self.depth is not None and self.a.depth and self.n_det % 30 == 0:
            self.depth_check(px, T_cam_tag)

    def depth_check(self, px, T_cam_tag):
        """Aligned depth at the tag centre vs the PnP range: catches print-scale
        or intrinsics errors (both would scale every anchored pose)."""
        m = self.depth
        if m.encoding != '16UC1':
            return
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        c = px.mean(axis=0).astype(int)
        win = d[max(c[1]-3, 0):c[1]+4, max(c[0]-3, 0):c[0]+4].astype(float) / 1e3
        win = win[win > 0]
        if win.size:
            z_depth = float(np.median(win))
            z_pnp = float(T_cam_tag[2, 3])
            print(f'\n  depth check: aligned depth {z_depth:.3f} m vs PnP z {z_pnp:.3f} m '
                  f'({(z_depth/z_pnp-1)*100:+.1f} %)', flush=True)

    def status(self, s):
        if time.time() - self.last_print > 0.5:
            self.last_print = time.time()
            print(f'\r  [{self.n_det}/{self.n_img} frames with tag] {s}            ',
                  end='', flush=True)

    def solve(self):
        S = self.samples
        ts = np.array([M[:3, 3] for M in S])
        med = np.median(ts, axis=0)
        dev = np.linalg.norm(ts - med, axis=1)
        keep = dev <= max(3 * dev.std(), 1e-3)
        S = [M for M, k in zip(S, keep) if k]
        ts = ts[keep]
        Rm = mean_rotation([M[:3, :3] for M in S])
        p = np.median(ts, axis=0)
        ang = [np.degrees(R.from_matrix(M[:3, :3] @ Rm.T).magnitude()) for M in S]
        spread_mm = float(np.linalg.norm(ts - p, axis=1).max() * 1e3)
        spread_deg = float(max(ang))
        print(f'\n{len(S)} samples kept ({len(self.samples)-len(S)} rejected): '
              f'base_link->map t={np.round(p, 4)} spread {spread_mm:.1f} mm / {spread_deg:.2f} deg')
        if spread_mm > self.a.max_spread_mm or spread_deg > self.a.max_spread_deg:
            print(f'  REFUSED: spread above {self.a.max_spread_mm} mm / {self.a.max_spread_deg} deg '
                  '— hold still, closer, or restart cuVSLAM; retrying with fresh samples')
            self.samples = []
            return
        self.publish(T(p, Rm), {'samples': len(S), 'spread_mm': spread_mm,
                                'spread_deg': spread_deg})

    def publish(self, M, stats):
        q = R.from_matrix(M[:3, :3]).as_quat()
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = BASE
        ts.child_frame_id = self.a.map_frame
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = M[:3, 3]
        (ts.transform.rotation.x, ts.transform.rotation.y,
         ts.transform.rotation.z, ts.transform.rotation.w) = q
        self.static_pub.sendTransform(ts)
        self.published = M
        d = {'parent': BASE, 'child': self.a.map_frame,
             'published': dt.datetime.now().isoformat(timespec='seconds'),
             'tag_file': self.a.tag, 'tag_id': self.a.tag_id,
             'translation': [float(v) for v in M[:3, 3]],
             'quaternion_xyzw': [float(v) for v in q], **stats}
        if self.a.out:
            os.makedirs(os.path.dirname(self.a.out) or '.', exist_ok=True)
            with open(self.a.out, 'w') as f:
                yaml.safe_dump(d, f, sort_keys=False)
        print(f'\a>>> PUBLISHED static TF {BASE} -> {self.a.map_frame}: '
              f't={np.round(M[:3, 3], 4)} q={np.round(q, 4)}'
              + (f'  -> {self.a.out}' if self.a.out else '')
              + '\n    nvblox may start now. Keeping the TF alive; Ctrl-C ends it.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', help='tag<id>.yaml from tag_corners.py')
    ap.add_argument('--replay', help='anchor yaml written earlier: publish it, no detection')
    ap.add_argument('--image', default='/camera/color/image_raw')
    ap.add_argument('--info', default='/camera/color/camera_info')
    ap.add_argument('--depth', default='/camera/aligned_depth_to_color/image_raw',
                    help="'' to disable the depth consistency check")
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--samples', type=int, default=40)
    ap.add_argument('--max-range', type=float, default=1.2)
    ap.add_argument('--max-spread-mm', type=float, default=20.0)
    ap.add_argument('--max-spread-deg', type=float, default=1.0)
    ap.add_argument('--check-only', action='store_true',
                    help='do not publish: look the tag up THROUGH the already published '
                         'base_link->map chain and print the residual vs its arm-measured '
                         'pose (second-landmark sanity check, e.g. --tag anchor/tag1.yaml)')
    ap.add_argument('--out', default=None,
                    help='anchor yaml (default anchor/anchor-<date>.yaml next to the tag file)')
    a = ap.parse_args()

    rclpy.init()
    node = Anchor.__new__(Anchor)
    if a.replay:
        d = yaml.safe_load(open(a.replay))
        a.tag, a.tag_id, a.tag_size, a.out = d.get('tag_file'), d.get('tag_id'), 0.1, None
        a.map_frame = d['child']
        Node.__init__(node, 'tag_anchor')
        node.a = a
        node.static_pub = tf2_ros.StaticTransformBroadcaster(node)
        node.published = None
        node.publish(T(np.array(d['translation']),
                       R.from_quat(d['quaternion_xyzw']).as_matrix()), {})
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        return
    if not a.tag:
        sys.exit('--tag or --replay required')
    tag, T_base_tag = load_tag(a.tag)
    a.tag_id, a.tag_size = tag['tag_id'], tag['size_m']
    if a.out is None:
        a.out = os.path.join(os.path.dirname(os.path.abspath(a.tag)),
                             f'anchor-{dt.date.today().isoformat()}.yaml')
    node.__init__(a)
    node.T_base_tag = T_base_tag
    print(f'tag {a.tag_id} ({a.tag_size*1e3:.0f} mm) at base_link {np.round(T_base_tag[:3,3],4)}; '
          f'waiting for {a.image} + TF {a.map_frame}->camera ...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
