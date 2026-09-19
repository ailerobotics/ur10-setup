#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Replays the scripted HUMAN-safety scenario onto `/human_safety/state` in the REAL
camera wire format, for the dynamic-pick proofs. No cameras are ever opened (task
hard rule) — this stands in for `cams/human_safety`. A SILENCE phase stops
publishing so the gate/watcher see a heartbeat loss.

Run (task domain 83):
    ROS_DOMAIN_ID=83 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
        python3 human_state_publisher.py --ros-args -p scenario:=dynamic_pick_full
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scenarios  # noqa: E402


class HumanStatePublisher(Node):
    def __init__(self):
        super().__init__("dp_human_state_publisher")
        name = str(self.declare_parameter("scenario", "dynamic_pick_full").value)
        self.rate = float(self.declare_parameter("rate_hz", 12.0).value)
        topic = str(self.declare_parameter("state_topic", scenarios.sa.TOPIC_DEFAULT).value)
        self.phases = scenarios.HUMAN_SCENARIOS[name]()
        self.duration = scenarios.SCENARIO_DURATION[name]
        self.pub = self.create_publisher(String, topic, 10)
        self._t0 = time.monotonic()
        self._seq = 0
        self.done = False
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(f"human states '{name}' ({self.duration:.0f}s) on {topic}")

    def _active(self, rel):
        for s, e, spec in self.phases:
            if s <= rel < e:
                return spec
        return None

    def _tick(self):
        rel = time.monotonic() - self._t0
        if rel >= self.duration:
            if not self.done:
                self.get_logger().info("human scenario complete")
                self.done = True
            return
        spec = self._active(rel)
        if spec is None or spec.get("silence"):
            return
        self._seq += 1
        self.pub.publish(String(data=scenarios.human_state_json(spec, time.time(), self._seq)))


def main(args=None):
    rclpy.init(args=args)
    node = HumanStatePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
