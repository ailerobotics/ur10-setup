#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Count sensor_msgs/Image frames arriving on one or more topics, then exit.

Runs under the SYSTEM ROS 2 (/opt/ros/jazzy), deliberately in a different
process from Isaac Sim, so a pass proves the cross-process DDS path the real
detector graph uses actually delivers frames. Exit 0 iff every requested topic
received at least ``--min`` messages within ``--seconds``.

Prints, per topic, the count and the first frame's encoding + resolution so the
smoke test can also confirm the encoding the detector expects (rgb8 / 32FC1).
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Probe(Node):
    def __init__(self, topics):
        super().__init__("isaac_human_frame_probe")
        self.counts = {t: 0 for t in topics}
        self.first = {t: None for t in topics}
        for t in topics:
            self.create_subscription(
                Image, t, self._make_cb(t), qos_profile_sensor_data)

    def _make_cb(self, topic):
        def cb(msg):
            self.counts[topic] += 1
            if self.first[topic] is None:
                self.first[topic] = (msg.encoding, msg.width, msg.height)
        return cb


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topics", required=True,
                    help="comma-separated Image topics to watch")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--min", type=int, default=1,
                    help="minimum frames per topic to pass")
    args = ap.parse_args()

    topics = [t.strip() for t in args.topics.split(",") if t.strip()]
    rclpy.init()
    node = Probe(topics)
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
            if all(node.counts[t] >= args.min for t in topics):
                break
    finally:
        counts = dict(node.counts)
        first = dict(node.first)
        node.destroy_node()
        rclpy.shutdown()

    ok = True
    for t in topics:
        info = first[t]
        desc = (f"encoding={info[0]} {info[1]}x{info[2]}" if info else "no frames")
        status = "OK" if counts[t] >= args.min else "FAIL"
        print(f"[probe] {t}: {counts[t]} frames [{desc}] -> {status}", flush=True)
        if counts[t] < args.min:
            ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
