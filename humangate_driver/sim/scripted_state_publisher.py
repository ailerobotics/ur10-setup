#!/usr/bin/env python3
"""
Replays a scripted camera-side scenario onto `/human_safety/state` in the REAL
wire format, for the end-to-end simulation proofs. No cameras are ever opened
(task hard rule) — this stands in for `cams/human_safety`.

It publishes at a fixed rate; during a SILENCE phase it stops publishing (so the
gate/watcher see a heartbeat loss); during a STALE phase it publishes a state
whose evidence stamp is frozen in the past. It exits when the scenario ends.

Run (task domain 79):
    ROS_DOMAIN_ID=79 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
        python3 scripted_state_publisher.py --ros-args -p scenario:=full
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scenarios  # noqa: E402


class Publisher(Node):
    def __init__(self):
        super().__init__("scripted_state_publisher")
        name = str(self.declare_parameter("scenario", "full").value)
        self.rate = float(self.declare_parameter("rate_hz", 10.0).value)
        topic = str(self.declare_parameter("state_topic", scenarios.sa.TOPIC_DEFAULT).value)
        self.phases = scenarios.SCENARIOS[name]()
        self.duration = max(e for _, e, _ in self.phases)
        self.pub = self.create_publisher(String, topic, 10)
        self._t0 = time.monotonic()
        self._seq = 0
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"publishing scenario '{name}' ({self.duration:.0f}s) on {topic} "
            f"at {self.rate:.0f} Hz")
        self.done = False

    def _active(self, rel):
        for s, e, spec in self.phases:
            if s <= rel < e:
                return spec
        return None

    def _tick(self):
        rel = time.monotonic() - self._t0
        if rel >= self.duration:
            if not self.done:
                self.get_logger().info("scenario complete")
                self.done = True
            return
        spec = self._active(rel)
        if spec is None or spec.get("silence"):
            return                                   # heartbeat gap
        self._seq += 1
        now_wall = time.time()
        if spec.get("stale"):
            data = scenarios.stale_state_json(now_wall, self._seq)
        else:
            data = scenarios.state_json_for(spec, now_wall, self._seq)
        self.pub.publish(String(data=data))


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
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
