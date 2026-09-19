#!/usr/bin/env python3
"""Publish a scripted human-safety state timeline on /human_safety/state.

Stands in for cams/human_safety (opens NO camera). Emits the REAL committed wire
format (humangate.state_adapter.build_state_json) so the bridge's gate sees
exactly what the camera side would publish. A timeline is a comma-separated list
of ZONE:seconds phases, e.g. "CLEAR:6,STOP:4,CLEAR:8". CLEAR frames are fully
healthy (so a CLEAR + dwell reaches RUN); STOP/SLOW carry a plausible distance.

Runs on the caller's ROS_DOMAIN_ID (the sim uses 84).
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "humangate"))
import state_adapter as sa  # noqa: E402

_FRESH_CAM = [{"camera": "sim", "status": "fresh", "present_in_graph": True,
               "detector_alive": True, "positioned": True, "fresh": True,
               "frame_age": 0.0, "note": ""}]


def phase_json(zone, now_wall, seq):
    if zone == "CLEAR":
        return sa.build_state_json("CLEAR", stamp=now_wall, heartbeat_seq=seq,
                                   distance_status="none", cameras=_FRESH_CAM)
    dist = 1.0 if zone == "STOP" else 2.3
    return sa.build_state_json(zone, stamp=now_wall, evidence_stamp=now_wall,
                               heartbeat_seq=seq, distance_status="known",
                               nearest_distance_base=dist, any_person_visible=True,
                               cameras=_FRESH_CAM,
                               reasons=[f"nearest person id=1 at {dist:.2f} m from base"])


class GatePub(Node):
    def __init__(self, phases, rate):
        super().__init__("gate_state_pub")
        self.phases = phases          # list of (zone, dur)
        self.total = sum(d for _, d in phases)
        self.pub = self.create_publisher(String, sa.TOPIC_DEFAULT, 10)
        self.t0 = time.monotonic()
        self.seq = 0
        self.create_timer(1.0 / rate, self._tick)
        self.done = False

    def _zone_at(self, rel):
        acc = 0.0
        for z, d in self.phases:
            if acc <= rel < acc + d:
                return z
            acc += d
        return None

    def _tick(self):
        rel = time.monotonic() - self.t0
        if rel >= self.total:
            self.done = True
            return
        self.seq += 1
        self.pub.publish(String(data=phase_json(self._zone_at(rel), time.time(), self.seq)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sequence", default="CLEAR:8,STOP:5,CLEAR:8",
                    help="ZONE:sec,ZONE:sec,...")
    ap.add_argument("--rate", type=float, default=12.0)
    args = ap.parse_args()
    phases = []
    for tok in args.sequence.split(","):
        z, s = tok.split(":")
        phases.append((z.strip().upper(), float(s)))
    rclpy.init()
    node = GatePub(phases, args.rate)
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
