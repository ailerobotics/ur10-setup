#!/usr/bin/env python3
"""ROS 2 bridge: gate-mediated MPC command stream -> UR driver (SIM ONLY).

Streams a joint-position command every control tick to the driver's
``forward_position_controller`` (``std_msgs/Float64MultiArray`` on
``/forward_position_controller/commands``), which is the passthrough/forward
controller the UR ROS 2 driver offers for streamed control. It runs behind the
merged human-safety gate:

  * the camera-side safety state (``/human_safety/state``, the REAL committed
    ``std_msgs/String`` JSON contract) drives ``humangate.HumanGate``;
  * every tick, the gate Decision + the freshest MPC command go through the pure
    acceptance shim (``ros_bridge.stream_gate.GatedStreamController``): a command
    is forwarded ONLY while the gate says RUN, is dropped on STOP/SLOW/BACKSTOP,
    is refused if outside the measured joint windows, and is step-capped;
  * on HOLD the shim's last safe position is (re)published, so the arm holds
    whether the controller starves or is actively commanded.

SAFETY / ISOLATION (task hard rules):
  * SIM ONLY. Publishing is gated by the UNCHANGED arming interlock
    (humangate_driver/arming_interlock.py::evaluate_arming): it refuses unless
    ``arm:=true``, the ROS domain is not 0/42, and robot_ip is loopback or
    Docker-internal — never 192.168.0.1 / a physical NIC. The bridge targets
    127.0.0.1 (mock hardware / the URSim CB3 container), so it can never reach the
    real arm by configuration.
  * Runs on ROS_DOMAIN_ID=84 with FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (task domain).

The MPC command SOURCE is pluggable so the gate/stream/starvation behaviour can
be exercised without a GPU (``--command-source scripted``) or with the live
cuRobo MPC (``--command-source curobo``, imports curobo lazily).
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
import os
import subprocess
import sys
import time
from typing import List, Optional

# --- make the sibling packages importable ---------------------------------- #
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)                       # mpc_curobo/
_REPO = os.path.dirname(_PKG)                       # repo root
for p in (_PKG,
          os.path.join(_REPO, "humangate"),
          os.path.join(_REPO, "humangate_driver")):
    if p not in sys.path:
        sys.path.insert(0, p)

from robot.joint_windows import JOINT_ORDER, load_measured_windows  # noqa: E402
from ros_bridge.stream_gate import FORWARD, GatedStreamController    # noqa: E402
from arming_interlock import evaluate_arming                         # noqa: E402
from gate_config import GateConfig                                   # noqa: E402
from gate_core import HumanGate                                      # noqa: E402
import state_adapter                                                 # noqa: E402


def enumerate_host_subnets():
    """(physical_cidrs, docker_cidrs) for the interlock. Same rules as
    humangate_driver/driver_io.py; reimplemented here to avoid importing the UR
    service types that module pulls in. Fails safe to ([], [])."""
    docker_like = ("docker", "br-", "veth", "virbr", "cni", "flannel", "cali")
    physical, docker = [], []
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                             capture_output=True, text=True, timeout=5)
        for line in out.stdout.splitlines():
            parts = line.split()
            if len(parts) < 4 or parts[2] != "inet":
                continue
            ifname, cidr = parts[1], parts[3]
            if ifname == "lo" or cidr.startswith("127."):
                continue
            if any(ifname.startswith(pfx) for pfx in docker_like):
                docker.append(cidr)
            else:
                physical.append(cidr)
    except Exception:                                    # noqa: BLE001
        return [], []
    return physical, docker


# --------------------------------------------------------------------------- #
# MPC command sources
# --------------------------------------------------------------------------- #
class ScriptedCommandSource:
    """GPU-free stand-in for the MPC output: return a scripted GOAL joint vector
    (the shim step-caps the approach). The goal can be switched at runtime to
    emulate a moving target / a fresh re-target after a STOP."""

    def __init__(self, goal: List[float]):
        self._goal = list(goal)

    def set_goal(self, goal: List[float]) -> None:
        self._goal = list(goal)

    def command(self, measured: Optional[List[float]], now: float) -> Optional[List[float]]:
        return list(self._goal)


class CuroboMpcSource:
    """Live cuRobo MPC: each tick, re-optimize toward the current Cartesian goal
    and return the next joint position from the optimized action sequence.

    Imported lazily so the bridge (and its tests) load without curobo/torch on a
    host that has neither. Construction requires the GPU container.
    """

    def __init__(self, robot_cfg: str, scene_model: str, joint_names: List[str],
                 optimization_dt: float = 0.03, interpolation_steps: int = 4):
        import torch  # noqa
        from curobo.model_predictive_control import (
            ModelPredictiveControl, ModelPredictiveControlCfg)
        from curobo.types import GoalToolPose, JointState
        self._torch = torch
        self._JointState = JointState
        self._GoalToolPose = GoalToolPose
        cfg = ModelPredictiveControlCfg.create(
            robot=robot_cfg, scene_model=scene_model, use_cuda_graph=True,
            optimization_dt=optimization_dt, interpolation_steps=interpolation_steps)
        self._mpc = ModelPredictiveControl(cfg)
        self._joint_names = list(self._mpc.joint_names)
        start = self._JointState.from_position(
            self._mpc.default_joint_position.clone().unsqueeze(0),
            joint_names=self._joint_names)
        start.velocity = torch.zeros_like(start.position)
        start.acceleration = torch.zeros_like(start.position)
        self._mpc.setup(start)
        self._state = start
        self._seeded = False
        self._goal_pos = None      # stored goal pose, re-asserted every tick
        self._goal_quat = None
        self._tick = 0

    @property
    def joint_names(self) -> List[str]:
        return self._joint_names

    def set_goal_from_config(self, config) -> None:
        """Set the MPC goal to the FK pose (position AND orientation) of an
        in-window joint config, so the target is reachable by construction (a
        fixed arbitrary orientation is generally infeasible and stalls the MPC).

        Mirrors the proven prototype pattern: take the pose-dict TEMPLATE from the
        MPC's current state and overwrite position + orientation IN PLACE, then
        register it. Passing a different config's pose dict directly does NOT set
        the goal (observed: position_error stays 0)."""
        qa = self._JointState.from_position(
            self._torch.tensor([list(config)], device="cuda", dtype=self._torch.float32),
            joint_names=self._joint_names)
        link = self._mpc.tool_frames[0]
        gp = self._mpc.compute_kinematics(qa).tool_poses.to_dict()[link]
        self._goal_pos = gp.position.squeeze().detach().cpu().tolist()
        self._goal_quat = gp.quaternion.squeeze().detach().cpu().tolist()
        self._apply_goal(run_ik=True)

    def _apply_goal(self, run_ik: bool) -> None:
        """Re-assert the stored goal against the CURRENT MPC state template. This
        must run every tick: registering the goal once and then advancing the
        state goes stale (observed: position_error stays 0, no motion)."""
        if self._goal_pos is None:
            return
        link = self._mpc.tool_frames[0]
        poses = self._mpc.compute_kinematics(self._state).tool_poses.to_dict()
        for i in range(3):
            poses[link].position[..., i] = self._goal_pos[i]
        for i in range(4):
            poses[link].quaternion[..., i] = self._goal_quat[i]
        self._mpc.update_goal_tool_poses(
            self._GoalToolPose.from_poses(
                poses, ordered_tool_frames=self._mpc.tool_frames, num_goalset=1),
            run_ik=run_ik)

    def command(self, measured: Optional[List[float]], now: float) -> Optional[List[float]]:
        """Open-loop MPC streaming: seed the MPC state ONCE from the real start,
        then integrate the MPC's own model forward one control step per call (the
        proven prototype loop). The returned position is the freshest MPC action;
        the gate acceptance shim step-caps it before it reaches the controller.
        (A measured-feedback closed loop is a later refinement; resetting the MPC
        state to a lagging measured value every tick stalls progress.)"""
        if not self._seeded and measured is not None:
            st = self._JointState.from_position(
                self._torch.tensor([measured], device="cuda", dtype=self._torch.float32),
                joint_names=self._joint_names)
            st.velocity = self._torch.zeros_like(st.position)
            st.acceleration = self._torch.zeros_like(st.position)
            self._state = st
            self._seeded = True
        self._tick += 1
        self._apply_goal(run_ik=(self._tick % 40 == 1))   # re-assert goal each tick
        res = self._mpc.optimize_action_sequence(self._state)
        if res.action_sequence is None or res.action_sequence.position.shape[1] == 0:
            return None
        idx = res.action_sequence.position.shape[1] - 1   # horizon end (prototype loop)
        nxt = res.action_sequence.position[:, idx, :]
        self._state = self._JointState.from_position(nxt.clone(), joint_names=self._joint_names)
        self._state.velocity = res.action_sequence.velocity[:, idx, :]
        self._state.acceleration = res.action_sequence.acceleration[:, idx, :]
        return nxt.squeeze(0).detach().cpu().tolist()


# --------------------------------------------------------------------------- #
# ROS node
# --------------------------------------------------------------------------- #
def build_node(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                        ReentrantCallbackGroup)
    from std_msgs.msg import Float64MultiArray, String
    from sensor_msgs.msg import JointState as JointStateMsg

    limits_yaml = args.limits or os.path.join(_REPO, "cumotion", "measured_joint_limits.yaml")
    windows = load_measured_windows(limits_yaml)

    class MpcGateBridge(Node):
        def __init__(self):
            super().__init__("mpc_gate_bridge")

            # --- arming interlock (UNCHANGED module) ----------------------- #
            domain = os.environ.get("ROS_DOMAIN_ID", "unset")
            phys, dock = enumerate_host_subnets()
            decision = evaluate_arming(
                arm=args.arm, ros_domain_id=domain, robot_ip=args.robot_ip,
                physical_cidrs=phys, docker_cidrs=dock)
            self._armed = bool(decision)
            if not self._armed:
                self.get_logger().error(
                    "ARMING REFUSED — publishing DISABLED. reasons: "
                    + "; ".join(decision.reasons))
            else:
                self.get_logger().warn(
                    f"ARMED for SIM ({decision.classification} {decision.resolved_ip}, "
                    f"domain {domain}). Publishing to {args.command_topic}.")

            # --- gate + shim ---------------------------------------------- #
            self.gate = HumanGate(GateConfig())
            self.shim = GatedStreamController(
                windows=windows, joint_names=list(JOINT_ORDER),
                max_step_per_tick=args.max_step, operating_fraction=args.operating_fraction)
            self._measured: Optional[List[float]] = None
            self._seeded = False

            # --- MPC source ----------------------------------------------- #
            self.source = args._source

            # --- ROS wiring ----------------------------------------------- #
            # Subscriptions live in a reentrant group so state/joint intake keeps
            # running on other executor threads WHILE the timer runs a heavy MPC
            # solve — otherwise the solve blocks the gate's heartbeat and it STOPs.
            self._sub_cbg = ReentrantCallbackGroup()
            self._timer_cbg = MutuallyExclusiveCallbackGroup()
            self.pub = self.create_publisher(Float64MultiArray, args.command_topic, 10)
            self.create_subscription(String, args.state_topic, self._on_state, 10,
                                     callback_group=self._sub_cbg)
            self.create_subscription(JointStateMsg, args.joint_states_topic,
                                     self._on_joints, 10, callback_group=self._sub_cbg)
            self._log = self.create_publisher(String, "/mpc_gate_bridge/status", 10)
            self.period = 1.0 / args.rate
            self.create_timer(self.period, self._tick, callback_group=self._timer_cbg)
            self._ticks = 0
            self._forwarded = 0
            self.get_logger().info(
                f"mpc_gate_bridge up: rate={args.rate} Hz, source={args.command_source}")

        def _on_state(self, msg):
            try:
                cs = state_adapter.parse_string_msg(msg)
            except Exception as e:                       # noqa: BLE001
                self.get_logger().warn(f"bad human_safety state: {e}")
                return
            now = time.monotonic()
            self.gate.on_state(cs, now, now_wall=time.time())

        def _on_joints(self, msg):
            name_to_pos = dict(zip(msg.name, msg.position))
            if all(j in name_to_pos for j in JOINT_ORDER):
                self._measured = [float(name_to_pos[j]) for j in JOINT_ORDER]
                if not self._seeded:
                    self.shim.seed(self._measured)
                    self._seeded = True

        def _tick(self):
            self._ticks += 1
            now = time.monotonic()
            decision = self.gate.decide(now, time.time())
            mpc_cmd = None
            try:
                mpc_cmd = self.source.command(self._measured, now)
            except Exception as e:                       # noqa: BLE001
                self.get_logger().warn(f"MPC source error (holding): {e}")
            act = self.shim.step(decision.action, decision.speed_fraction,
                                 mpc_cmd, self._measured)
            # publish only when armed and the shim has been seeded from a real
            # /joint_states message; a HOLD still (re)publishes the hold position
            # so the controller actively holds. Dropping publication entirely is
            # the starvation path tested separately.
            if self._armed and self._seeded:
                self.pub.publish(Float64MultiArray(data=list(act.target_position)))
                if act.kind == FORWARD:
                    self._forwarded += 1
            if self._ticks % 5 == 0:   # ~10 Hz status: fine enough to correlate
                self._log.publish(String(data=json.dumps({
                    "tick": self._ticks, "gate": decision.action,
                    "shim": act.kind, "armed": self._armed,
                    "forwarded": self._forwarded,
                    "reason": act.reason, "window_ok": act.window_ok,
                })))

    return rclpy, MpcGateBridge


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arm", action="store_true",
                    help="explicitly arm (SIM). Default DISARMED (no publishing).")
    ap.add_argument("--robot-ip", default="127.0.0.1",
                    help="sim target (mock/URSim). Loopback/Docker only; never 192.168.0.1.")
    ap.add_argument("--command-topic", default="/forward_position_controller/commands")
    ap.add_argument("--state-topic", default="/human_safety/state")
    ap.add_argument("--joint-states-topic", default="/joint_states")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--max-step", type=float, default=0.01,
                    help="max per-joint rad change per tick at fraction 1.0")
    ap.add_argument("--operating-fraction", type=float, default=1.0)
    ap.add_argument("--limits", default=None)
    ap.add_argument("--command-source", choices=["scripted", "curobo"], default="scripted")
    ap.add_argument("--scripted-goal", default=None,
                    help="comma-separated 6 joint values for the scripted source")
    ap.add_argument("--robot-cfg", default=None, help="curobo robot config (curobo source)")
    ap.add_argument("--scene-model", default=None, help="curobo scene model (curobo source)")
    ap.add_argument("--mpc-goal-config", default=None,
                    help="curobo source: 6 joint values whose FK pose is the goal "
                         "(reachable by construction)")
    args = ap.parse_args(argv)

    # reference (park) config = default goal
    ref = [0.223385, -1.626713, 0.406275, 4.458966, 0.279341, 0.706048]
    if args.command_source == "scripted":
        goal = ref
        if args.scripted_goal:
            goal = [float(x) for x in args.scripted_goal.split(",")]
        args._source = ScriptedCommandSource(goal)
    else:
        src = CuroboMpcSource(
            robot_cfg=args.robot_cfg, scene_model=args.scene_model,
            joint_names=list(JOINT_ORDER))
        if args.mpc_goal_config:
            src.set_goal_from_config([float(x) for x in args.mpc_goal_config.split(",")])
        args._source = src

    rclpy, MpcGateBridge = build_node(args)
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = MpcGateBridge()
    # multi-threaded so subscription intake runs concurrently with the timer's
    # (possibly heavy) MPC solve — keeps the gate heartbeat alive.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
