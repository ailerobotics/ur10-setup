#!/usr/bin/env python3
"""Record /joint_states + the bridge status for a fixed duration to JSONL.

Used by the mock/URSim e2e to prove, offline and deterministically, that the arm
moves only while the gate is RUN and holds otherwise, and that a starved command
stream leaves the arm stopped. SIM ONLY — opens no cameras, commands nothing.
"""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


class Recorder(Node):
    def __init__(self, out_path):
        super().__init__("mpc_e2e_recorder")
        self._fh = open(out_path, "w")
        self._t0 = time.monotonic()
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.create_subscription(String, "/mpc_gate_bridge/status", self._on_status, 10)
        self.get_logger().info(f"recording to {out_path}")

    def _on_js(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(j in m for j in _JOINTS):
            self._fh.write(json.dumps({
                "t": time.monotonic() - self._t0, "kind": "joint_states",
                "pos": [float(m[j]) for j in _JOINTS]}) + "\n")

    def _on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:                                # noqa: BLE001
            return
        d["t"] = time.monotonic() - self._t0
        d["kind"] = "status"
        self._fh.write(json.dumps(d) + "\n")

    def close(self):
        self._fh.flush()
        self._fh.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=45.0)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    rclpy.init()
    node = Recorder(args.out)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and (time.monotonic() - t0) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
