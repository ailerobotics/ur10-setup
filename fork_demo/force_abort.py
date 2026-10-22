#!/usr/bin/env python3
"""Independent force-deviation abort watchdog for the real fork pick.

On this UR10 CB3 there is NO wrist F/T sensor: contact force is ESTIMATED from
joint currents (tens-of-newtons accuracy, ~152 N clean-motion swing). So this is
a COARSE gross-contact net, NOT a sensitive safety function -- the real
safety-rated stop is the pendant Force limit, which this code never touches.

Two layers, matching the project's existing mechanism
(cumotion/collision_force_limits.yaml, cams/cellmap/tools/collision_force.py):

  * pure logic (ForceMonitor) -- rolling baseline of the force magnitude from
    /force_torque_sensor_broadcaster, trips when the DEVIATION exceeds the repo
    backstop (default 200 N). Unit-tested offline, no ROS.
  * a ROS watcher (run as `python3 force_abort.py`) that subscribes to the wrench
    and CANCELS the running FollowJointTrajectory goal (cancel-all) the instant
    the monitor trips -- an independent process alongside execute_plan.py, so it
    can stop a goal it did not start (same pattern as humangate_driver).

The watchdog fails TOWARD stop: if the wrench feed goes stale it trips.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque


class ForceMonitor:
    """Trips when |force| deviates from a short rolling baseline by backstop_n.

    Pure: feed it (timestamp, fx, fy, fz) samples; ask `tripped`.
    """

    def __init__(self, backstop_n: float = 200.0, baseline_n: int = 25,
                 stale_s: float = 0.5):
        self.backstop_n = float(backstop_n)
        self.baseline_n = int(baseline_n)
        self.stale_s = float(stale_s)
        self._mags: deque = deque(maxlen=self.baseline_n)
        self.last_t: float | None = None
        self.last_reason = ""
        self.peak_dev = 0.0

    @staticmethod
    def magnitude(fx: float, fy: float, fz: float) -> float:
        return math.sqrt(fx * fx + fy * fy + fz * fz)

    def update(self, t: float, fx: float, fy: float, fz: float) -> bool:
        """Add a sample; return True if this sample trips the abort."""
        self.last_t = t
        mag = self.magnitude(fx, fy, fz)
        if len(self._mags) < self.baseline_n:
            # still learning the baseline; never trip, but record it
            self._mags.append(mag)
            return False
        baseline = sum(self._mags) / len(self._mags)
        dev = abs(mag - baseline)
        self.peak_dev = max(self.peak_dev, dev)
        self._mags.append(mag)
        if dev > self.backstop_n:
            self.last_reason = (f"force deviation {dev:.0f} N > {self.backstop_n:.0f} N "
                                f"(|F|={mag:.0f} N, baseline {baseline:.0f} N)")
            return True
        return False

    def is_stale(self, now: float) -> bool:
        if self.last_t is None:
            return False
        if now - self.last_t > self.stale_s:
            self.last_reason = (f"wrench feed stale ({now - self.last_t:.2f} s "
                                f"> {self.stale_s:.2f} s) -- fail toward stop")
            return True
        return False


def _load_backstop(force_cfg: str) -> float:
    try:
        import yaml  # noqa: PLC0415
        with open(force_cfg) as f:
            return float(yaml.safe_load(f)["ros_contact_force_backstop_n"])
    except (OSError, KeyError, ValueError, TypeError):
        return 200.0


def run_watchdog(backstop_n: float, controller: str, stale_s: float) -> int:
    """ROS watcher: cancel the running trajectory goal on a force trip."""
    import rclpy  # noqa: PLC0415
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import WrenchStamped

    rclpy.init()
    node = rclpy.create_node("fork_force_abort")
    mon = ForceMonitor(backstop_n=backstop_n, stale_s=stale_s)
    cancel_cli = node.create_client(
        CancelGoal, f"/{controller}/follow_joint_trajectory/_action/cancel_goal")
    state = {"tripped": False}

    def on_wrench(m):
        f = m.wrench.force
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if mon.update(t, f.x, f.y, f.z) and not state["tripped"]:
            state["tripped"] = True
            node.get_logger().error(f"FORCE ABORT: {mon.last_reason} -- cancelling")
            cancel_cli.call_async(CancelGoal.Request())  # cancel-all (zero uuid)

    node.create_subscription(WrenchStamped,
                             "/force_torque_sensor_broadcaster/wrench", on_wrench, 20)
    node.get_logger().info(
        f"force-abort watchdog armed: backstop {backstop_n:.0f} N, controller {controller}")
    try:
        while rclpy.ok() and not state["tripped"]:
            rclpy.spin_once(node, timeout_sec=0.1)
            if mon.is_stale(time.time()):
                node.get_logger().error(f"FORCE ABORT: {mon.last_reason} -- cancelling")
                cancel_cli.call_async(CancelGoal.Request())
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if state["tripped"] else 0


def main(argv=None) -> int:
    import os
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--force-cfg",
                   default=os.path.join(repo, "cumotion", "collision_force_limits.yaml"))
    p.add_argument("--backstop-n", type=float, default=None,
                   help="override the backstop (default: read from --force-cfg)")
    p.add_argument("--controller", default="scaled_joint_trajectory_controller")
    p.add_argument("--stale-s", type=float, default=0.5)
    args = p.parse_args(argv)
    backstop = args.backstop_n if args.backstop_n is not None else _load_backstop(args.force_cfg)
    return run_watchdog(backstop, args.controller, args.stale_s)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
