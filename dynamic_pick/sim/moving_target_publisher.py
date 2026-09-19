#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Publishes the scripted MOVING pick target as `geometry_msgs/PointStamped` in the
base frame, for the dynamic-pick proofs. This stands in for the tray detector +
seeded-IK-to-base step; on the real cell the target arrives in a camera frame and
is transformed to base first. No camera is opened.

An "occluded" window publishes nothing, so the tracker coasts (OCCLUDED) and then
drops the target (LOST) — proving the arm holds rather than chasing a phantom.

Run (task domain 83):
    ROS_DOMAIN_ID=83 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
        python3 moving_target_publisher.py --ros-args -p scenario:=dynamic_pick_full
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scenarios  # noqa: E402


class MovingTargetPublisher(Node):
    def __init__(self):
        super().__init__("dp_moving_target_publisher")
        name = str(self.declare_parameter("scenario", "dynamic_pick_full").value)
        self.rate = float(self.declare_parameter("rate_hz", 7.0).value)   # ~ detector 6.7 Hz
        self.frame = str(self.declare_parameter("frame_id", "base_link").value)
        topic = str(self.declare_parameter("topic", "/dynamic_pick/target").value)
        self.waypoints = scenarios.TARGET_SCENARIOS[name]()
        self.duration = scenarios.SCENARIO_DURATION[name]
        self.pub = self.create_publisher(PointStamped, topic, 10)
        self._t0 = time.monotonic()
        self.done = False
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(f"moving target '{name}' on {topic} ({self.frame})")

    def _active(self, rel):
        spec = None
        for t, s in self.waypoints:
            if t <= rel:
                spec = s
            else:
                break
        return spec

    def _tick(self):
        rel = time.monotonic() - self._t0
        if rel >= self.duration:
            if not self.done:
                self.get_logger().info("target scenario complete")
                self.done = True
            return
        spec = self._active(rel)
        if spec is None or spec.get("occluded"):
            return                                # occlusion / lost window
        xyz = spec["xyz"]
        m = PointStamped()
        now = self.get_clock().now().to_msg()
        m.header.stamp = now
        m.header.frame_id = self.frame
        m.point.x, m.point.y, m.point.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MovingTargetPublisher()
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
