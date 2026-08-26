#!/usr/bin/env python3
"""
Synthetic depth camera for BENCH rehearsal of the map pipeline — no RealSense,
no cuVSLAM, no arm motion. Renders a table plane + axis-aligned boxes (in
base_link) into 16UC1 aligned-depth frames with the scan launch's topic/frame
contract, and publishes base_link -> camera_color_optical_frame TF directly
(standing in for base_link->map->odom->camera_link->optical).

Used 2026-08-26 to validate, before any physical step: nvblox integration in
base_link, save/load, the ESDF hand-off to cuMotion with the 0.04 m buffer,
and that plans detour around a box (tools/plan_test.py). It is NOT a
substitute for the real scan — depth is perfect here.

  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 fake_depth.py \
      --box 0.10,0.50,0.55,0.95,0.00,0.45 --orbit 1.6 --height 1.2 --duration 60
Scene: table plane at z=--table (default 0.0 = the robot's mounting plane),
boxes as xmin,xmax,ymin,ymax,zmin,zmax (repeatable). The camera orbits the
scene centre at --orbit radius, --height above base_link, looking at it.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

W, H = 640, 360
FX = FY = 460.0            # ~D435 colour intrinsics at 640x360
CX, CY = 320.0, 180.0
OPTICAL = 'camera_color_optical_frame'
BASE = 'base_link'


def look_at(eye, target):
    """Rotation of an OPTICAL frame (z forward, x right, y down) at eye looking at target."""
    z = target - eye
    z /= np.linalg.norm(z)
    up = np.array([0, 0, 1.0])
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0, 0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def render(eye, Rm, boxes, table_z, max_range=6.0):
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, float)], -1)   # optical rays
    dw = d @ Rm.T                                                              # world dirs
    t_best = np.full((H, W), np.inf)
    # table plane z = table_z (only hit from above)
    dz = dw[..., 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (table_z - eye[2]) / dz
    ok = (t > 0.05) & np.isfinite(t) & (dz < 0)
    t_best = np.where(ok, t, t_best)
    for b in boxes:
        bmin, bmax = np.array(b[0::2]), np.array(b[1::2])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (bmin - eye) / dw
            t2 = (bmax - eye) / dw
        tmin = np.max(np.minimum(t1, t2), axis=-1)
        tmax = np.min(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= tmin) & (tmax > 0.05)
        t = np.where(tmin > 0.05, tmin, np.inf)
        t = np.where(hit, t, np.inf)
        t_best = np.minimum(t_best, t)
    z = t_best * d[..., 2]                      # depth along the optical axis
    z[~np.isfinite(z) | (z > max_range)] = 0
    return (z * 1000).astype(np.uint16)


class FakeDepth(Node):
    def __init__(self, a):
        super().__init__('fake_depth')
        self.a = a
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_img = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', qos)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/aligned_depth_to_color/camera_info', qos)
        self.pub_cinfo = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)
        self.tfb = TransformBroadcaster(self)
        self.t0 = time.time()
        self.n = 0
        self.timer = self.create_timer(1.0 / a.rate, self.tick)
        c = np.mean([[(b[0]+b[1])/2, (b[2]+b[3])/2, (b[4]+b[5])/2] for b in a.boxes], axis=0) \
            if a.boxes else np.array([0.0, 0.6, 0.2])
        self.center = c
        print(f'orbiting {np.round(c,2)} r={a.orbit} h={a.height}; boxes={a.boxes}; table z={a.table}')

    def tick(self):
        el = time.time() - self.t0
        ang = 2 * math.pi * el / self.a.period + self.a.phase
        eye = self.center + np.array([self.a.orbit * math.cos(ang),
                                      self.a.orbit * math.sin(ang), 0])
        eye[2] = self.a.height
        Rm = look_at(eye, self.center)
        stamp = self.get_clock().now().to_msg()
        ts = TransformStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = BASE
        ts.child_frame_id = OPTICAL
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = eye
        q = R.from_matrix(Rm).as_quat()
        (ts.transform.rotation.x, ts.transform.rotation.y,
         ts.transform.rotation.z, ts.transform.rotation.w) = q
        self.tfb.sendTransform(ts)
        depth = render(eye, Rm, self.a.boxes, self.a.table)
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = OPTICAL
        img.height, img.width = H, W
        img.encoding = '16UC1'
        img.is_bigendian = 0
        img.step = W * 2
        img.data = depth.tobytes()
        info = CameraInfo()
        info.header = img.header
        info.height, info.width = H, W
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        info.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_img.publish(img)
        self.pub_info.publish(info)
        self.pub_cinfo.publish(info)
        self.n += 1
        if self.n % (self.a.rate * 5) == 0:
            print(f'  {self.n} frames, {el:.0f} s, cam at {np.round(eye,2)}', flush=True)
        if self.a.duration and el > self.a.duration:
            raise SystemExit


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--box', action='append', default=[],
                    help='xmin,xmax,ymin,ymax,zmin,zmax in base_link (repeatable)')
    ap.add_argument('--table', type=float, default=0.0)
    ap.add_argument('--orbit', type=float, default=1.6)
    ap.add_argument('--height', type=float, default=1.2)
    ap.add_argument('--period', type=float, default=40.0, help='s per orbit')
    ap.add_argument('--phase', type=float, default=0.0)
    ap.add_argument('--rate', type=int, default=10)
    ap.add_argument('--duration', type=float, default=0)
    a = ap.parse_args()
    a.boxes = [[float(v) for v in b.split(',')] for b in a.box]
    rclpy.init()
    node = FakeDepth(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
