#!/usr/bin/env python3
"""Hold driver freedrive by publishing enable@10Hz until stopped (adjust helper).

Used during a step-replay 'adjust' pause so the captain can hand-move the arm.
Assumes freedrive_mode_controller is already active (step_replay --adjust-start
switches controllers). Publishes Bool(true) to the enable topic at 10 Hz; on
SIGTERM/SIGINT publishes Bool(false) and exits (arm HOLDS -- fail-safe).
"""
from __future__ import annotations
import signal
import sys
import time


def main():
    import rclpy
    from std_msgs.msg import Bool
    rclpy.init()
    node = rclpy.create_node("freedrive_hold")
    pub = node.create_publisher(Bool, "/freedrive_mode_controller/enable_freedrive_mode", 10)
    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
    node.get_logger().info("freedrive hold: publishing enable@10Hz (Ctrl-C to release)")
    try:
        while rclpy.ok() and not stop["v"]:
            pub.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.0)
    finally:
        pub.publish(Bool(data=False))
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
