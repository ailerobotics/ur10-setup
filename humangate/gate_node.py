#!/usr/bin/env python3
"""
ROS 2 node for the robot-side human-safety gate — PLAN-ONLY (report §10 Phase 1).

It subscribes to the camera-side safety state, runs the decision core, publishes
its decision + reason on its OWN topic, and writes a structured decision log.
What it would command the robot goes to a RecorderPort — it is recorded, never
sent. There is deliberately no set_speed_slider client, no FollowJointTrajectory
action client, no dashboard client, and no publisher on any controller topic;
test_no_driver_interface.py enforces this.

Run it (isolated domain, per this task's brief):
    ROS_DOMAIN_ID=78 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
        python3 gate_node.py --ros-args -p decision_log:=/tmp/human_gate/decisions.jsonl

Transport note: the camera-side custom message is not committed yet, so the gate
consumes the state as a std_msgs/String JSON payload (state_adapter.py). When the
real message lands, swap the subscription type + parse function (one line each).
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gate_config as gate_config  # noqa: E402
import gate_core as gc  # noqa: E402
import state_adapter as sa  # noqa: E402
from command_port import RecorderPort  # noqa: E402
from driver_feedback import always_stopped  # noqa: E402

DECISION_TOPIC_DEFAULT = "/human_gate/decision"


class GateNode(Node):
    """Thin ROS wrapper around HumanGate. All logic lives in the pure core."""

    def __init__(self):
        super().__init__("human_gate")
        # thresholds: file first (report defaults), then any ROS-param overrides
        params_file = self._decl("params_file", gate_config.CONFIG_PATH)
        cfg = gate_config.load_or_default(params_file)
        cfg = gate_config.GateConfig(
            state_max_age_s=self._decl("state_max_age_s", cfg.state_max_age_s),
            heartbeat_timeout_s=self._decl("heartbeat_timeout_s", cfg.heartbeat_timeout_s),
            clock_skew_tolerance_s=self._decl("clock_skew_tolerance_s",
                                              cfg.clock_skew_tolerance_s),
            clear_dwell_s=self._decl("clear_dwell_s", cfg.clear_dwell_s),
            stop_radius_m=self._decl("stop_radius_m", cfg.stop_radius_m),
            slow_clear_hysteresis_m=self._decl("slow_clear_hysteresis_m",
                                               cfg.slow_clear_hysteresis_m),
            operating_speed_fraction=self._decl("operating_speed_fraction",
                                                cfg.operating_speed_fraction),
            slow_speed_fraction=self._decl("slow_speed_fraction", cfg.slow_speed_fraction),
            stop_speed_fraction=self._decl("stop_speed_fraction", cfg.stop_speed_fraction),
            backstop_deadline_s=self._decl("backstop_deadline_s", cfg.backstop_deadline_s),
            decide_rate_hz=self._decl("decide_rate_hz", cfg.decide_rate_hz),
        ).validate()
        self.cfg = cfg
        self.gate = gc.HumanGate(cfg)
        self.feedback = always_stopped()  # simulated; the real driver feedback is Phase 3

        state_topic = self._decl("state_topic", sa.TOPIC_DEFAULT)
        decision_topic = self._decl("decision_topic", DECISION_TOPIC_DEFAULT)
        log_path = self._decl("decision_log", "")
        cmd_log = self._decl("command_log", "")
        for p in (log_path, cmd_log):
            if p:
                os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
        self._decision_log = log_path or None
        self.port = RecorderPort(cmd_log or None)
        self._last_action = None

        self.pub = self.create_publisher(String, decision_topic, 10)
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.create_timer(1.0 / cfg.decide_rate_hz, self._on_tick)
        self.get_logger().info(
            f"human_gate up: sub {state_topic} (provisional String/JSON transport), "
            f"pub {decision_topic}, dwell {cfg.clear_dwell_s:.1f} s. PLAN-ONLY — "
            f"commands are recorded, never sent.")

    def _decl(self, name, default):
        return self.declare_parameter(name, default).value

    # --- subscriptions -----------------------------------------------------

    def _on_state(self, msg):
        try:
            state = sa.parse_string_msg(msg)
        except Exception as e:                       # noqa: BLE001
            # a malformed state is not a clearance: log and let the watchdog STOP
            self.get_logger().warn(f"ignoring malformed camera state: {e}")
            return
        self.gate.on_state(state, time.monotonic(), time.time())

    # --- decide loop -------------------------------------------------------

    def _on_tick(self):
        now = time.monotonic()
        self.feedback.set_now(now)
        d = self.gate.decide(now, time.time(), self.feedback.arm_stopped())
        self._publish(d, now)
        self._record_command(d, now)
        self._log_decision(d, now)

    def _publish(self, d: gc.Decision, now: float):
        import json
        payload = {
            "action": d.action, "reason": d.reason, "speed_fraction": d.speed_fraction,
            "demand": d.demand, "provenance": d.provenance,
            "clear_dwell_s": round(d.clear_dwell_s, 3), "healthy": d.healthy,
            "t_mono": round(now, 3),
        }
        self.pub.publish(String(data=json.dumps(payload)))

    def _record_command(self, d: gc.Decision, now: float):
        # only record on a change of intent, so the command log reads as events
        if d.action == self._last_action:
            return
        self._last_action = d.action
        if d.action == gc.BACKSTOP:
            self.port.cancel_and_hold(d.reason, now)
        else:
            self.port.set_speed_fraction(d.speed_fraction, d.reason, now)

    def _log_decision(self, d: gc.Decision, now: float):
        if not self._decision_log:
            return
        import json
        with open(self._decision_log, "a") as f:
            f.write(json.dumps({
                "t_mono": now, "action": d.action, "reason": d.reason,
                "speed_fraction": d.speed_fraction, "demand": d.demand,
                "provenance": d.provenance, "clear_dwell_s": d.clear_dwell_s,
                "healthy": d.healthy}) + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = GateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
