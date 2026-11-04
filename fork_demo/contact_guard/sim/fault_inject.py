#!/usr/bin/env python3
"""Inject a synthetic contact on URSim (no contact physics of its own).

Publishes an out-of-band WrenchStamped on the force-torque topic after a delay,
plus a lateral wrist-torque step (a tip touch via the fork lever), so the live
contact-guard node trips during a stop-and-probe creep. Lazy rclpy; URSim-only.
"""
from __future__ import annotations

import argparse
import time


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plan", default=None)
    ap.add_argument("--inject-after-s", type=float, default=2.0)
    ap.add_argument("--topic", default="/force_torque_sensor_broadcaster/wrench")
    ap.add_argument("--torque-nm", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=50.0)
    a = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import WrenchStamped

    rclpy.init()
    nd = Node("fork_contact_fault_inject")
    pub = nd.create_publisher(WrenchStamped, a.topic, 10)
    t0 = time.time()
    msg = WrenchStamped()
    msg.header.frame_id = "tool0_controller"
    try:
        while rclpy.ok():
            if time.time() - t0 >= a.inject_after_s:
                # lateral prying torque step = a tip touch through the 0.38 m lever
                msg.wrench.torque.y = a.torque_nm
                pub.publish(msg)
            time.sleep(1.0 / a.hz)
    except KeyboardInterrupt:
        pass
    finally:
        nd.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
