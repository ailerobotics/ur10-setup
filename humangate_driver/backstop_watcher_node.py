#!/usr/bin/env python3
"""
Independent backstop watcher — a SEPARATE PROCESS with its OWN timer and its OWN
copy of the camera-state heartbeat (task brief item 2; report §2, §5).

Running as its own process is the whole point: if the primary gate node hangs,
deadlocks, or its decide loop stalls, THIS process keeps ticking and can still
cancel the trajectory. It never reads the gate's internal state — only the two
public topics (`/human_safety/state`, `/human_gate/decision`) and the applied
scaling — so it is genuinely independent.

It cancels the active FollowJointTrajectory goal (arm holds) on ANY of:
  * its own camera-state heartbeat lost;
  * the gate's decision heartbeat lost (primary path dead);
  * the gate decided BACKSTOP;
  * a STOP demanded but the slider did not take effect within a deadline.

It runs the SAME arming interlock as the primary node, so it too cannot be
pointed at the real robot by configuration. Cancel is always fail-safe (it only
ever stops the arm), but the interlock keeps the whole change unreachable from
real hardware.

Run (isolated task domain 79), as its own process:
    ROS_DOMAIN_ID=79 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 backstop_watcher_node.py \
        --ros-args -p arm:=true -p robot_ip:=127.0.0.1
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "humangate"))

import state_adapter as sa  # noqa: E402
from arming_interlock import evaluate_arming  # noqa: E402
from driver_io import (  # noqa: E402
    FJT_ACTION_DEFAULT, ScalingMonitor, TrajectoryCanceller,
    enumerate_host_subnets)
from watcher_core import BackstopWatcher, WatcherParams  # noqa: E402

DECISION_TOPIC_DEFAULT = "/human_gate/decision"
STATUS_TOPIC_DEFAULT = "/human_gate/watcher_status"


class ArmingRefused(RuntimeError):
    pass


class BackstopWatcherNode(Node):
    def __init__(self):
        super().__init__("human_gate_backstop_watcher")

        arm = bool(self._p("arm", False))
        robot_ip = str(self._p("robot_ip", "127.0.0.1"))
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        phys, dock = enumerate_host_subnets()
        decision = evaluate_arming(arm=arm, ros_domain_id=domain, robot_ip=robot_ip,
                                   physical_cidrs=phys, docker_cidrs=dock)
        if not decision.armed:
            for r in decision.reasons:
                self.get_logger().error(f"ARMING REFUSED: {r}")
            raise ArmingRefused("; ".join(decision.reasons))

        params = WatcherParams(
            state_heartbeat_timeout_s=float(self._p("state_heartbeat_timeout_s", 0.5)),
            gate_heartbeat_timeout_s=float(self._p("gate_heartbeat_timeout_s", 0.5)),
            stop_effect_deadline_s=float(self._p("stop_effect_deadline_s", 0.7)),
            stop_scaling_threshold=float(self._p("stop_scaling_threshold", 0.05)),
            startup_grace_s=float(self._p("startup_grace_s", 3.0)),
            stop_strategy=str(self._p("stop_strategy", "cancel")),
            pendant_freeze_confirm_s=float(self._p("pendant_freeze_confirm_s", 2.5)))
        self.watcher = BackstopWatcher(params, start_time=time.monotonic())
        self.get_logger().warn(
            f"watcher STOP strategy = {params.stop_strategy}; effective stop-effect "
            f"deadline {self.watcher.effective_stop_effect_deadline_s:.2f}s (sized from "
            f"pendant_freeze_confirm_s={params.pendant_freeze_confirm_s:.2f}s under "
            f"pendant_pause). It must match the gate's stop_strategy.")

        action_ns = str(self._p("fjt_action", FJT_ACTION_DEFAULT))
        self.canceller = TrajectoryCanceller(self, action_ns)
        self.scaling = ScalingMonitor(self)

        self._last_state_arrival = None
        self._last_state_zone = None
        self._last_gate_arrival = None
        self._last_gate_action = None
        self._last_gate_demand = None
        self._last_gate_strategy = None
        self._last_cancel_t = 0.0
        self._last_misconfig_log_t = 0.0
        self._log_path = str(self._p("watcher_log", "")) or None
        if self._log_path:
            os.makedirs(os.path.dirname(os.path.abspath(self._log_path)), exist_ok=True)

        self.status_pub = self.create_publisher(
            String, str(self._p("watcher_status_topic", STATUS_TOPIC_DEFAULT)), 10)
        self.create_subscription(String, str(self._p("state_topic", sa.TOPIC_DEFAULT)),
                                 self._on_state, 10)
        self.create_subscription(String, str(self._p("decision_topic", DECISION_TOPIC_DEFAULT)),
                                 self._on_decision, 10)
        self.create_timer(1.0 / float(self._p("tick_hz", 25.0)), self._tick)
        self.canceller.available(3.0)
        self.get_logger().warn(
            "backstop watcher up (INDEPENDENT process). Cancels the trajectory on "
            "camera-state or gate heartbeat loss, on BACKSTOP, or on an ineffective "
            "STOP. SIMULATED arm only.")

    def _p(self, name, default):
        return self.declare_parameter(name, default).value

    def _on_state(self, msg):
        self._last_state_arrival = time.monotonic()
        try:
            self._last_state_zone = json.loads(msg.data).get("zone")
        except Exception:                            # noqa: BLE001
            self._last_state_zone = None             # malformed -> unknown (not CLEAR)

    def _on_decision(self, msg):
        self._last_gate_arrival = time.monotonic()
        try:
            d = json.loads(msg.data)
            self._last_gate_action = d.get("action")
            self._last_gate_demand = d.get("demand")
            self._last_gate_strategy = d.get("stop_strategy")
        except Exception:                            # noqa: BLE001
            self._last_gate_action = None
            self._last_gate_demand = None
            self._last_gate_strategy = None

    def _tick(self):
        now = time.monotonic()
        reason = self.watcher.update(
            now, last_state_arrival=self._last_state_arrival,
            last_gate_arrival=self._last_gate_arrival,
            state_zone=self._last_state_zone, gate_action=self._last_gate_action,
            gate_demand=self._last_gate_demand,
            gate_stop_strategy=self._last_gate_strategy,
            observed_scaling=self.scaling.value)

        # Surface a strategy/deadline misconfiguration LOUDLY and REPEATEDLY (never
        # silently): it never changes the safe cancel behaviour, but an operator who
        # armed pendant_pause on only one process must see it. Throttled so 25 Hz
        # does not flood the log, but it keeps repeating for as long as it holds.
        if self.watcher.misconfig and (now - self._last_misconfig_log_t) > 2.0:
            self.get_logger().error(f"WATCHER MISCONFIG: {self.watcher.misconfig}")
            self._last_misconfig_log_t = now

        self._publish_status(now, reason)

        if reason and (now - self._last_cancel_t) > 1.0:
            self.get_logger().error(f"BACKSTOP WATCHER CANCELLING TRAJECTORY: {reason}")
            self.canceller.cancel_all()
            self._last_cancel_t = now
            if self._log_path:
                with open(self._log_path, "a") as f:
                    f.write(json.dumps({"t_mono": now, "reason": reason,
                                        "misconfig": self.watcher.misconfig,
                                        "observed_scaling": self.scaling.value}) + "\n")

    def _publish_status(self, now, reason):
        self.status_pub.publish(String(data=json.dumps({
            "t_mono": round(now, 3),
            "stop_strategy": self.watcher.p.stop_strategy,
            "gate_stop_strategy": self._last_gate_strategy,
            "effective_stop_effect_deadline_s":
                round(self.watcher.effective_stop_effect_deadline_s, 3),
            "tripped": self.watcher.tripped,
            "cancel_reason": reason,
            "misconfig": self.watcher.misconfig,
            "observed_scaling": self.scaling.value,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BackstopWatcherNode()
        rclpy.spin(node)
    except ArmingRefused as e:
        print(f"backstop_watcher: ARMING REFUSED — not starting: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
