#!/usr/bin/env python3
"""
Publish the calibrated tool0 -> <camera body> static TF from a result yaml.

WHY a script and not a hand-typed static_transform_publisher: the mount is
seven numbers (xyz + quaternion) and retyping them is exactly how a good
calibration becomes a bad map. This reads the file, so what is published is
what was solved.

WHEN it is needed: wrist_rig.launch.py normally publishes this, but the
multi-camera bring-up (zz_rigcams) deliberately runs with NO mount TF while a
calibration is stale. Validation still needs tool0 -> optical in TF, so this
fills the gap WITHOUT restarting anyone's camera nodes.

It publishes the parent/child NAMED IN THE FILE - not an assumed camera_link -
because the camera bring-up chooses its own frame prefix.

  python3 publish_mount.py calib/tool0_to_camera_2026-09-03.yaml
"""
import argparse
import sys

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_file')
    ap.add_argument('--key', default='tool0_to_camera_link')
    a = ap.parse_args()

    d = yaml.safe_load(open(a.yaml_file))
    m = d[a.key]
    src = d.get('source', {})

    rclpy.init()
    n = Node('mount_tf')
    b = StaticTransformBroadcaster(n)
    t = TransformStamped()
    t.header.stamp = n.get_clock().now().to_msg()
    t.header.frame_id = m['parent']
    t.child_frame_id = m['child']
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
        (float(v) for v in m['xyz'])
    q = [float(v) for v in m['quaternion_xyzw']]
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    b.sendTransform(t)
    print(f'publishing {m["parent"]} -> {m["child"]}  xyz={[round(v, 5) for v in m["xyz"]]}')
    if src.get('image_topic'):
        print(f'  solved on {src["image_topic"]} ({src.get("optical_frame")})')
    print('Ctrl-C to stop.')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
