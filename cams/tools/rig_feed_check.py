#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Bounded liveness check for the rig's camera feeds: is each camera delivering
FRESH frames right now? Prints, per topic, frames received in the window, the
rate, and the age of the newest frame's header stamp against the wall clock,
then exits 0 only if every topic passed. Latched / stale frames and "process
alive" are NOT health on this rig, which is why this looks at stamps.

  ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    python3 cams/tools/rig_feed_check.py                 # all three raw colour feeds
  python3 cams/tools/rig_feed_check.py --topic /wrist/color_upright/image_raw
  python3 cams/tools/rig_feed_check.py --seconds 10 --min-hz 25 --max-age 0.5
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DEFAULT_TOPICS = ['/wrist/color/image_raw', '/shelf/color/image_raw',
                  '/table/color/image_raw']


class Check(Node):
    def __init__(self, topics):
        super().__init__('rig_feed_check')
        self.stats = {t: {'n': 0, 'last_stamp': None, 'last_wall': None} for t in topics}
        for t in topics:
            self.create_subscription(Image, t, lambda m, t=t: self._on(t, m),
                                     qos_profile_sensor_data)

    def _on(self, topic, msg):
        s = self.stats[topic]
        s['n'] += 1
        s['last_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s['last_wall'] = time.time()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', action='append', help='image topic (repeatable)')
    ap.add_argument('--seconds', type=float, default=5.0, help='listen window')
    ap.add_argument('--min-hz', type=float, default=20.0)
    ap.add_argument('--max-age', type=float, default=0.5,
                    help='max age (s) of the newest stamp vs wall clock at the end')
    args = ap.parse_args()
    topics = args.topic or DEFAULT_TOPICS

    rclpy.init()
    node = Check(topics)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    end = time.time()
    ok_all = True
    for t in topics:
        s = node.stats[t]
        hz = s['n'] / args.seconds
        age = (end - s['last_stamp']) if s['last_stamp'] is not None else float('inf')
        ok = hz >= args.min_hz and age <= args.max_age
        ok_all &= ok
        print(f"{'LIVE ' if ok else 'DEAD '} {t:40s} {s['n']:4d} frames in {args.seconds:.0f}s "
              f"= {hz:5.1f} Hz   newest stamp age {age:6.3f}s")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok_all else 1)


if __name__ == '__main__':
    main()
