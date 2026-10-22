#!/usr/bin/env python3
"""Verify that /joint_states is a LIVE encoder feed, not a frozen republished packet.

A dropped RTDE link makes the UR driver republish the last packet forever at a
perfect 125 Hz (RUNBOOK.md section 10, "two things that will fool you"). Rate is
therefore NOT proof of liveness; encoder jitter is. A genuinely live, stationary
arm shows ~4-9 mdeg of per-joint jitter and a velocity that hovers around zero;
a frozen feed shows 0.000 mdeg range and a constant nonzero velocity that never
integrates into any position change.

Usable two ways:
  * CLI:  python3 fork_demo/check_joint_feed_live.py  -> prints PASS/FROZEN, exit 0/1
  * import: `from check_joint_feed_live import check_joint_feed_live`

Read-only: subscribes to /joint_states, commands nothing.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass


# A live stationary arm jitters at least this much (mdeg) over the sample window.
# Measured live values on this CB3 are 4-9 mdeg (cumotion/measured_joint_limits.yaml
# records a 4.8-6.2 mdeg startup gate). The frozen feed shows exactly 0.000.
MIN_LIVE_JITTER_MDEG = 1.0


@dataclass
class FeedVerdict:
    live: bool
    n_samples: int
    n_unique: int
    max_range_mdeg: float
    max_abs_velocity: float
    reason: str

    def __str__(self) -> str:
        tag = "LIVE" if self.live else "FROZEN"
        return (
            f"[joint-feed {tag}] samples={self.n_samples} unique_poses={self.n_unique} "
            f"max_jitter={self.max_range_mdeg:.3f} mdeg max|vel|={self.max_abs_velocity:.4f} rad/s "
            f"-- {self.reason}"
        )


def check_joint_feed_live(duration_s: float = 4.0, min_jitter_mdeg: float = MIN_LIVE_JITTER_MDEG) -> FeedVerdict:
    """Subscribe to /joint_states for `duration_s` and judge liveness by jitter.

    Imports rclpy lazily so this module is importable without a ROS environment.
    """
    import time

    import rclpy  # noqa: PLC0415
    from sensor_msgs.msg import JointState  # noqa: PLC0415

    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = rclpy.create_node("fork_demo_feed_check")
    samples: list[list[float]] = []
    vels: list[list[float]] = []

    def cb(msg: "JointState") -> None:
        samples.append(list(msg.position))
        vels.append(list(msg.velocity))

    node.create_subscription(JointState, "/joint_states", cb, 50)
    t0 = time.time()
    while time.time() - t0 < duration_s:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if owns_context:
        rclpy.shutdown()

    if not samples:
        return FeedVerdict(False, 0, 0, 0.0, 0.0, "no /joint_states messages received")

    n_joints = len(samples[0])
    max_range_mdeg = 0.0
    for j in range(n_joints):
        col = [s[j] for s in samples if len(s) > j]
        if col:
            rng_mdeg = (max(col) - min(col)) * 1000.0
            max_range_mdeg = max(max_range_mdeg, rng_mdeg)
    max_abs_vel = max((abs(v) for vv in vels for v in vv), default=0.0)
    n_unique = len({tuple(round(x, 9) for x in s) for s in samples})

    live = max_range_mdeg >= min_jitter_mdeg and n_unique > 1
    if live:
        reason = f"encoder jitter >= {min_jitter_mdeg} mdeg over {len(samples)} samples"
    else:
        reason = (
            "position frozen (0 jitter / single unique pose) -- stale republished RTDE packet; "
            "fix: systemctl --user restart ur10-driver"
        )
    return FeedVerdict(live, len(samples), n_unique, max_range_mdeg, max_abs_vel, reason)


def main(argv: list[str] | None = None) -> int:
    verdict = check_joint_feed_live()
    print(verdict)
    return 0 if verdict.live else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
