#!/usr/bin/env python3
"""
Publish a saved cell map (.ply) as a coloured PointCloud2 in base_link.

Why not the nvblox mesh: with the camera unplugged the planner's nvblox is
serving the ESDF from a loaded map with meshing disabled, and turning
meshing on means restarting that node — which is the same node the motion
loop is planning against. Re-publishing the saved .ply from the host shows
the captain exactly the same geometry, costs the running loop nothing, and
keeps working when no nvblox is up at all.

Latched (transient local), so RViz can be started or restarted at any time
and still see the map.

  python3 publish_map_cloud.py ~/ur10/cumotion/maps/cell-2026-08-28.ply
"""
import argparse
import itertools
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_ply(path):
    f = open(path, 'r')
    n = None
    props = []
    for line in f:
        line = line.strip()
        if line.startswith('element vertex'):
            n = int(line.split()[-1])
        elif line.startswith('property'):
            props.append(line.split()[-1])
        elif line == 'end_header':
            break
    cols = {p: i for i, p in enumerate(props)}
    need = [cols['x'], cols['y'], cols['z']]
    rgb = [cols[c] for c in ('red', 'green', 'blue')] if 'red' in cols else None
    arr = np.loadtxt(itertools.islice(f, n),
                     usecols=need + (rgb or []))
    xyz = arr[:, :3]
    col = arr[:, 3:6].astype(np.uint8) if rgb else np.full((len(xyz), 3), 200, np.uint8)
    return xyz, col


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--topic', default='/cellmap/map_cloud')
    ap.add_argument('--frame', default='base_link')
    ap.add_argument('--stride', type=int, default=1)
    a = ap.parse_args()

    xyz, col = read_ply(a.ply)
    xyz, col = xyz[::a.stride], col[::a.stride]
    print(f'{a.ply}: publishing {len(xyz)} points on {a.topic} (frame {a.frame})')

    rclpy.init()
    n = Node('map_cloud')
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = n.create_publisher(PointCloud2, a.topic, qos)

    msg = PointCloud2()
    msg.header = Header(frame_id=a.frame)
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(xyz)
    msg.is_dense = True
    buf = bytearray()
    for (x, y, z), (r, g, b) in zip(xyz, col):
        packed = struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]
        buf += struct.pack('ffff', float(x), float(y), float(z), packed)
    msg.data = bytes(buf)

    def tick():
        msg.header.stamp = n.get_clock().now().to_msg()
        pub.publish(msg)
    n.create_timer(2.0, tick)
    tick()
    print('publishing — Ctrl-C to stop')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
