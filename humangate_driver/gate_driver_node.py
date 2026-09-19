#!/usr/bin/env python3
"""
Driver-backed human-safety gate node — SIMULATED arm only (report §10 Phase 3).

This wires the plan-only decision core (`humangate.gate_core.HumanGate`, unchanged)
to the real driver levers through `DriverCommandPort`:

  primary  = speed slider via `set_speed_slider`  (SLOW / soft-STOP / resume)
  backstop = cancel the active FollowJointTrajectory goal (arm holds)

It REFUSES TO START unless the arming interlock passes (explicit `arm:=true`, a
domain that is not 0/42, and a loopback / Docker-internal robot address that is
not 192.168.0.1 and not on a physical NIC). Real-robot arming is a later change;
no launch argument here can reach the real robot.

Sole-writer discipline (report §5 #16): it watches the applied scaling and, on a
change it did not command, goes to STOP/BACKSTOP and surfaces the conflict.

Fail toward zero (report §2, §5 #11): the gate core already emits STOP (slider 0)
on a lost camera-state heartbeat and BACKSTOP on a self-fault; on shutdown this
node writes the slider to 0. It NEVER restores the slider on its own — only an
explicit healthy CLEAR+dwell raises it back through a RUN decision.

Run (isolated task domain 79):
    ROS_DOMAIN_ID=79 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 gate_driver_node.py \
        --ros-args -p arm:=true -p robot_ip:=127.0.0.1 \
        -p decision_log:=/tmp/hg/decisions.jsonl
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "humangate"))

import gate_config as gate_config  # noqa: E402
import gate_core as gc  # noqa: E402
import state_adapter as sa  # noqa: E402
from arming_interlock import evaluate_arming  # noqa: E402
from driver_io import (  # noqa: E402
    FJT_ACTION_DEFAULT, JointMotionMonitor, ProgramMonitor, ScalingMonitor,
    SpeedSliderClient, TrajectoryCanceller, enumerate_host_subnets)
from driver_port import DriverCommandPort, DriverFeedbackFromTopics  # noqa: E402
from sole_writer import SoleWriterMonitor  # noqa: E402

DECISION_TOPIC_DEFAULT = "/human_gate/decision"


class ArmingRefused(RuntimeError):
    pass


class GateDriverNode(Node):
    def __init__(self):
        super().__init__("human_gate_driver")

        # ---- arming interlock (must pass before ANY driver client) -------- #
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
        self.get_logger().warn(
            f"ARMED for a SIMULATED arm: robot_ip={decision.resolved_ip} "
            f"({decision.classification}), domain={domain}. This must NEVER point "
            f"at real hardware (report §10 Phase 3).")

        # ---- gate config + core (unchanged plan-only core) ---------------- #
        cfg = gate_config.load_or_default(self._p("params_file", gate_config.CONFIG_PATH))
        # allow a per-node operating-fraction override (default keeps the file's)
        op = float(self._p("operating_speed_fraction", cfg.operating_speed_fraction))
        cfg = gate_config.GateConfig(**{**cfg.__dict__, "operating_speed_fraction": op}).validate()
        self.cfg = cfg
        self.gate = gc.HumanGate(cfg)

        # ---- driver I/O + port ------------------------------------------- #
        action_ns = str(self._p("fjt_action", FJT_ACTION_DEFAULT))
        stop_thresh = float(self._p("stop_scaling_threshold", 0.05))
        self.slider = SpeedSliderClient(self)
        self.scaling = ScalingMonitor(self)
        self.joints = JointMotionMonitor(self)
        self.program = ProgramMonitor(self)
        self.canceller = TrajectoryCanceller(self, action_ns)
        self.sole_writer = SoleWriterMonitor()
        self.feedback = DriverFeedbackFromTopics(self.joints, self.scaling, stop_thresh)
        self.port = DriverCommandPort(
            self.slider, self.scaling, self.canceller, self.sole_writer,
            logger=self.get_logger(),
            confirm_deadline_s=float(self._p("confirm_deadline_s", 0.5)),
            stop_scaling_threshold=stop_thresh)

        # A short, non-blocking availability check only — do NOT stall __init__
        # waiting for services, or the node will not start spinning and will miss
        # the first seconds of camera state (discovery). Service clients connect
        # lazily during spin; a call before the service is up is simply a no-op.
        for name, ok in (("set_speed_slider", self.slider.available(0.2)),
                         ("trajectory cancel", self.canceller.available(0.2))):
            if not ok:
                self.get_logger().warn(f"{name} service not up yet — will connect "
                                       "during spin")

        # ---- I/O topics + logs ------------------------------------------- #
        state_topic = str(self._p("state_topic", sa.TOPIC_DEFAULT))
        decision_topic = str(self._p("decision_topic", DECISION_TOPIC_DEFAULT))
        log_path = str(self._p("decision_log", ""))
        if log_path:
            os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
        self._decision_log = log_path or None
        self.pub = self.create_publisher(String, decision_topic, 10)
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.create_timer(1.0 / cfg.decide_rate_hz, self._tick)

        self._last_action = None
        self._last_fraction = None
        self._last_cancel_t = 0.0
        self._shutdown_zeroed = False
        self._last_cam_zone = None
        self._last_cam_health_ok = None
        self.get_logger().info(
            f"human_gate_driver up: sub {state_topic}, pub {decision_topic}, "
            f"FJT {action_ns}. Speed slider is the PRIMARY lever; trajectory "
            f"cancel is the BACKSTOP. SIMULATED arm only.")

    def _p(self, name, default):
        return self.declare_parameter(name, default).value

    # ---- subscriptions --------------------------------------------------- #
    def _on_state(self, msg):
        try:
            state = sa.parse_string_msg(msg)
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f"ignoring malformed camera state: {e}")
            return
        # keep the raw camera zone/health for the decision log so the analyzer can
        # see, e.g., a health fault reported alongside a CLEAR zone even when the
        # gate's own reason is masked by a latched backstop.
        self._last_cam_zone = state.zone
        self._last_cam_health_ok = bool(state.health_ok and not state.health_flags)
        self.gate.on_state(state, time.monotonic(), time.time())

    # ---- decide loop ----------------------------------------------------- #
    def _tick(self):
        now = time.monotonic()

        # sole-writer: a foreign slider change -> self-fault -> BACKSTOP.
        if self.scaling.value is not None:
            conflict = self.sole_writer.observe(self.scaling.value, now)
            if conflict:
                self.get_logger().error(f"SOLE-WRITER CONFLICT: {conflict}")
                self.gate.inject_self_fault(conflict)

        d = self.gate.decide(now, time.time(), self.feedback.arm_stopped())
        self._publish(d, now)
        self._apply(d, now)
        self.port.check_confirmation(now)
        self._log(d, now)

    def _apply(self, d, now):
        if d.action == gc.BACKSTOP:
            # rising edge, then re-assert cheaply while latched
            if self._last_action != gc.BACKSTOP or (now - self._last_cancel_t) > 1.0:
                self.port.cancel_and_hold(d.reason, now)
                self._last_cancel_t = now
            self._last_fraction = 0.0
        else:
            # RUN/SLOW/STOP: write the slider only when the target changes.
            if (self._last_fraction is None
                    or abs(d.speed_fraction - self._last_fraction) > 1e-6):
                self.port.set_speed_fraction(d.speed_fraction, d.reason, now)
                self._last_fraction = d.speed_fraction
        self._last_action = d.action

    def _publish(self, d, now):
        import json
        self.pub.publish(String(data=json.dumps({
            "action": d.action, "reason": d.reason,
            "speed_fraction": d.speed_fraction, "demand": d.demand,
            "provenance": d.provenance, "clear_dwell_s": round(d.clear_dwell_s, 3),
            "healthy": d.healthy, "t_mono": round(now, 3),
            "observed_scaling": self.scaling.value,
        })))

    def _log(self, d, now):
        if not self._decision_log:
            return
        import json
        with open(self._decision_log, "a") as f:
            f.write(json.dumps({
                "t_mono": now, "action": d.action, "reason": d.reason,
                "speed_fraction": d.speed_fraction, "demand": d.demand,
                "provenance": d.provenance, "clear_dwell_s": d.clear_dwell_s,
                "healthy": d.healthy, "observed_scaling": self.scaling.value,
                "arm_stopped": self.feedback.arm_stopped(),
                "cam_zone": self._last_cam_zone,
                "cam_health_ok": self._last_cam_health_ok}) + "\n")

    # ---- fail-to-zero on shutdown --------------------------------------- #
    def on_shutdown(self):
        if self._shutdown_zeroed:
            return
        self._shutdown_zeroed = True
        try:
            self.port.zero_slider("node shutdown — leaving slider at zero", time.monotonic())
            # give the async service call a moment to flush
            rclpy.spin_once(self, timeout_sec=0.2)
        except Exception:                            # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GateDriverNode()
        rclpy.spin(node)
    except ArmingRefused as e:
        print(f"human_gate_driver: ARMING REFUSED — not starting: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
