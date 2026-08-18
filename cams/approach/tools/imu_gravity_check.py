#!/usr/bin/env python3
"""
Sanity-check the tool0->camera mount ROTATION against the IMU gravity vector.

With the arm PARKED (gyro still), the D435i accelerometer measures the
reaction to gravity: the mean accel vector points UP in the IMU frame.
Predicting "up" in that same frame through the TF chain
   base_link -> tool0 -> camera_link -> <imu frame>   (base_link +Z is up)
and comparing gives the angular residual of the mount-rotation guess in
wrist_rig.launch.py. Translation is NOT checked (gravity carries no lever
arm information) — fine, the demo stops >= 0.25 m short of everything.

Also prints the residual for the 4 cardinal rolls about the optical axis, so
a wrong `cam_roll_deg` guess is immediately obvious (a strapped-on camera is
most likely off by a multiple of 90 deg, or a few deg of strap slop).

Run (host, wrist_rig running with imu:=true, arm parked):
  ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    python3 cams/approach/tools/imu_gravity_check.py
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import tf2_ros
from sensor_msgs.msg import Imu

SAMPLE_S = 3.0
GYRO_STILL_RAD_S = 0.03    # same stillness bar as target_publisher.py


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Check(Node):
    def __init__(self):
        super().__init__('imu_gravity_check')
        self.acc = []
        self.gyr = []
        self.frame = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Imu, '/camera/imu', self._on_imu,
                                 qos_profile_sensor_data)

    def _on_imu(self, m):
        self.frame = m.header.frame_id
        self.acc.append([m.linear_acceleration.x, m.linear_acceleration.y,
                         m.linear_acceleration.z])
        self.gyr.append([m.angular_velocity.x, m.angular_velocity.y,
                         m.angular_velocity.z])


def main():
    rclpy.init()
    n = Check()
    print(f'sampling /camera/imu for {SAMPLE_S:.0f} s ...')
    end = time.time() + SAMPLE_S + 2.0
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)

    if len(n.acc) < 20:
        sys.exit(f'FAIL: only {len(n.acc)} IMU samples — wrist_rig running '
                 'with imu:=true?')
    gyr = np.linalg.norm(np.array(n.gyr), axis=1)
    if gyr.max() >= GYRO_STILL_RAD_S:
        sys.exit(f'FAIL: arm not still (max |w| {gyr.max():.3f} rad/s) — '
                 'park it first')
    up_meas = np.array(n.acc).mean(axis=0)
    g_mag = np.linalg.norm(up_meas)
    up_meas /= g_mag

    try:
        tf = n.buf.lookup_transform(n.frame, 'base_link', Time(),
                                    timeout=rclpy.duration.Duration(seconds=3))
    except tf2_ros.TransformException as e:
        sys.exit(f'FAIL: TF {n.frame}<-base_link: {e}')
    R = quat_to_mat(tf.transform.rotation)      # base_link -> imu frame
    up_pred = R @ np.array([0.0, 0.0, 1.0])

    res = math.degrees(math.acos(np.clip(np.dot(up_meas, up_pred), -1, 1)))
    print(f'imu frame:        {n.frame}   samples: {len(n.acc)}   '
          f'|g| = {g_mag:.2f} m/s^2')
    print(f'measured up:      {np.array2string(up_meas, precision=3)}')
    print(f'predicted up:     {np.array2string(up_pred, precision=3)}')
    print(f'ANGLE RESIDUAL:   {res:.1f} deg '
          f'({"OK for v1 rough extrinsics" if res < 15 else "SUSPICIOUS — check cam_roll_deg / mount guess"})')

    # residuals for cardinal rolls about the optical axis (imu frame ~ optical
    # frames share the forward axis on the D435i): rotate the PREDICTION by
    # extra roll about the camera forward axis expressed in the imu frame.
    try:
        tf_cl = n.buf.lookup_transform(n.frame, 'camera_link', Time(),
                                       timeout=rclpy.duration.Duration(seconds=2))
        fwd = quat_to_mat(tf_cl.transform.rotation) @ np.array([1.0, 0, 0])
        print('residual if cam_roll_deg were changed by:')
        for extra in (0, 90, 180, 270):
            a = math.radians(extra)
            K = np.array([[0, -fwd[2], fwd[1]],
                          [fwd[2], 0, -fwd[0]],
                          [-fwd[1], fwd[0], 0]])
            Rr = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)
            # extra roll of the camera w.r.t. the tool rotates measured-vs-
            # predicted the opposite way; compare against rolled prediction
            r = math.degrees(math.acos(np.clip(
                np.dot(up_meas, Rr @ up_pred), -1, 1)))
            print(f'  {extra:+4d} deg -> {r:6.1f} deg residual')
    except tf2_ros.TransformException:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
