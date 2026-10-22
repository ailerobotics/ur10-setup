#!/usr/bin/env python3
"""Scripted, conservative forklift tray-pick demo for the real UR10 cell.

Prepared under task ur10-fork-demo-readiness-v1. SAFE BY DEFAULT:

  * --dry-run (DEFAULT) computes and prints the base_link TCP waypoints for the
    full cycle and saves a 3D path plot. It touches no ROS and commands NO motion.
  * The execute path REFUSES to run unless ALL of these hold:
      1. the explicit flag  --i-am-at-the-estop  is passed,
      2. an interactive confirmation (type EXECUTE) is given,
      3. a fork TCP file is supplied with --tcp,
      4. the robot is RUNNING / NORMAL (dashboard),
      5. External Control is running AND a motion controller is active,
      6. /joint_states is verified LIVE (jitter gate -- rejects the frozen feed),
      7. every waypoint's seeded-IK joint solution is inside the measured joint
         windows (cumotion/measured_joint_limits.yaml),
    and only then dispatches ONE conservatively-timed trajectory.

As of the readiness check (2026-10-22) conditions 4-6 do NOT hold (program
STOPPED, controllers inactive, telemetry frozen) and no fork gripper/TCP exists,
so the execute path will refuse. See data/ur10-fork-demo-readiness-v1/report.md.

This mirrors the project's seeded-IK + joint-limit-guard approach
(AGENTS.md; cams/approach/approach_demo.py; execute_plan.py --limits) rather than
free MoveIt pose goals, which here sample distant IK rewinds.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_FORK_CFG = os.path.expanduser(
    "~/datasets/shelf-twin/grippers/forklift_S/isaac_sim_S_forklift_gripper.zip"
)
DEFAULT_LIMITS = os.path.join(REPO, "cumotion", "measured_joint_limits.yaml")
JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ----------------------------------------------------------------------------
# Fork geometry (from forklift_config.json; conservative fallbacks baked in so
# --dry-run works even without the kit mounted / unzipped).
# ----------------------------------------------------------------------------
@dataclass
class ForkGeometry:
    tine_z1: float = 0.427          # fork tip, m along tool Z from flange face
    fork_center_y: float = -0.038   # fork centreline offset on tool Y, m
    insert_travel: float = 0.38     # m the forks slide into the pocket
    approach_clearance: float = 0.08  # m standoff before the pocket mouth
    lift: float = 0.02              # m vertical lift (pallet; dock = 0.035)
    gripper_mass: float = 0.82      # kg
    tray_mass_full: float = 2.45    # kg

    @property
    def approach_back(self) -> float:
        return self.insert_travel + self.approach_clearance  # 0.46 m

    @classmethod
    def load(cls, path: str) -> "ForkGeometry":
        cfg = _load_fork_config(path)
        if not cfg:
            return cls()
        g = cfg.get("gripper", {})
        p = cfg.get("pallet", {})
        return cls(
            tine_z1=g.get("tine_z1", cls.tine_z1),
            fork_center_y=g.get("fork_center_y", cls.fork_center_y),
            insert_travel=p.get("insert_travel", cls.insert_travel),
            approach_clearance=p.get("approach_clearance", cls.approach_clearance),
            lift=p.get("lift", cls.lift),
            gripper_mass=g.get("mass", cls.gripper_mass),
            tray_mass_full=cfg.get("tray", {}).get("mass_full", cls.tray_mass_full),
        )


def _load_fork_config(path: str) -> dict:
    """Read forklift_config.json from a directory, a .json, or the kit .zip."""
    try:
        if path.endswith(".zip"):
            import zipfile

            with zipfile.ZipFile(path) as z:
                name = next(n for n in z.namelist() if n.endswith("forklift_config.json"))
                return json.loads(z.read(name))
        if os.path.isdir(path):
            path = os.path.join(path, "forklift_config.json")
        with open(path) as f:
            return json.load(f)
    except (OSError, StopIteration, ValueError):
        return {}


# ----------------------------------------------------------------------------
# Cell layout in base_link. Conservative provisional values -- must be confirmed
# against the real shelf/tray before any execution (report.md section 4).
# ----------------------------------------------------------------------------
@dataclass
class CellLayout:
    shelf_front_x: float = 0.95   # m ahead of base, pocket mouth plane
    pick_slot_z: float = 0.30     # m above base plane, tray/pallet underside
    pick_y: float = 0.0           # m lateral, pick location
    place_y: float = 0.55         # m lateral, place location (cart/dock)
    place_slot_z: float = 0.30
    carry_clear_z: float = 0.12   # extra height for the raised carry via-point


@dataclass
class Waypoint:
    name: str
    position: tuple  # (x, y, z) in base_link, m -- the TCP (fork pocket point)
    phase: str
    note: str = ""


@dataclass
class ForkPickPlan:
    geom: ForkGeometry
    cell: CellLayout
    waypoints: list = field(default_factory=list)

    # Fixed tool orientation for the whole pick: forks horizontal, tool Z along
    # +base_X (into the shelf), forks hanging down. Quaternion (x,y,z,w) of
    # base_link<-tool. Provisional -- refine with the real fork TCP.
    tool_quat_xyzw: tuple = (0.0, 0.70710678, 0.0, 0.70710678)

    def build(self) -> "ForkPickPlan":
        c, g = self.cell, self.geom
        # The TCP is the fork pocket-engagement point. "Insert" pose = TCP at the
        # pocket centre (tray at shelf_front_x). Approach = approach_back behind it.
        x_insert = c.shelf_front_x
        x_approach = x_insert - g.approach_back
        z_pick = c.pick_slot_z
        z_lift = z_pick + g.lift
        z_carry = max(z_lift, c.place_slot_z) + c.carry_clear_z
        wps = [
            Waypoint("HOME", (x_approach - 0.10, c.pick_y, z_carry), "home",
                     "safe staging pose above the approach line"),
            Waypoint("APPROACH", (x_approach, c.pick_y, z_pick), "approach",
                     f"{g.approach_back*1000:.0f} mm back along fork axis, fork tips clear of pocket"),
            Waypoint("INSERT", (x_insert, c.pick_y, z_pick), "insert",
                     f"forks slide {g.insert_travel*1000:.0f} mm into the pocket"),
            Waypoint("LIFT", (x_insert, c.pick_y, z_lift), "lift",
                     f"lift {g.lift*1000:.0f} mm; load rides on the forks"),
            Waypoint("EXTRACT", (x_approach, c.pick_y, z_lift), "extract",
                     f"retract {g.insert_travel*1000:.0f} mm, tray clear of shelf"),
            Waypoint("CARRY", (x_approach, (c.pick_y + c.place_y) / 2.0, z_carry), "carry",
                     "raised, laterally-interpolated via-point"),
            Waypoint("PLACE_APPROACH", (x_approach, c.place_y, c.place_slot_z + g.lift), "carry",
                     "above the place slot"),
            Waypoint("PLACE_INSERT", (x_insert, c.place_y, c.place_slot_z + g.lift), "place",
                     "forks over the place pocket"),
            Waypoint("PLACE_LOWER", (x_insert, c.place_y, c.place_slot_z), "place",
                     f"lower {g.lift*1000:.0f} mm, tray sets down"),
            Waypoint("PLACE_RETRACT", (x_approach, c.place_y, c.place_slot_z), "place",
                     "forks withdraw, cycle complete"),
        ]
        self.waypoints = wps
        return self


# ----------------------------------------------------------------------------
# Dry run: print + plot. No ROS.
# ----------------------------------------------------------------------------
def print_plan(plan: ForkPickPlan) -> None:
    g = plan.geom
    print("=" * 74)
    print("FORK TRAY-PICK DEMO -- DRY RUN (no motion, no ROS)")
    print("=" * 74)
    print(f"fork tip (tool Z)      : {g.tine_z1*1000:.0f} mm")
    print(f"fork centreline (tool Y): {g.fork_center_y*1000:.0f} mm")
    print(f"insert / clearance     : {g.insert_travel*1000:.0f} / {g.approach_clearance*1000:.0f} mm")
    print(f"lift                   : {g.lift*1000:.0f} mm")
    print(f"payload (grip+tray)    : {g.gripper_mass:.2f} + up to {g.tray_mass_full:.2f} kg "
          f"= {g.gripper_mass + g.tray_mass_full:.2f} kg (UR10 limit 10 kg)")
    print("-" * 74)
    print(f"{'#':>2} {'waypoint':<15} {'phase':<9} {'x':>7} {'y':>7} {'z':>7}   note")
    for i, w in enumerate(plan.waypoints):
        x, y, z = w.position
        print(f"{i:>2} {w.name:<15} {w.phase:<9} {x:7.3f} {y:7.3f} {z:7.3f}   {w.note}")
    print("-" * 74)
    print("Tool orientation (base<-tool quat x,y,z,w): "
          f"{tuple(round(q, 4) for q in plan.tool_quat_xyzw)}  [forks horizontal into shelf]")
    print("All positions are PROVISIONAL base_link TCP targets. Confirm against the real")
    print("shelf/tray and set a real fork TCP before execution (report.md section 4).")


def save_plot(plan: ForkPickPlan, path: str) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print(f"(matplotlib not available; skipping plot {path})")
        return False
    xs = [w.position[0] for w in plan.waypoints]
    ys = [w.position[1] for w in plan.waypoints]
    zs = [w.position[2] for w in plan.waypoints]
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(xs, ys, zs, "-o", color="tab:blue")
    for w in plan.waypoints:
        ax.text(*w.position, f" {w.name}", fontsize=7)
    ax.scatter([0], [0], [0], color="k", marker="s", s=40, label="base_link")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title("Fork tray-pick demo path (base_link TCP, PROVISIONAL)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"saved path plot -> {path}")
    return True


# ----------------------------------------------------------------------------
# Joint-window guard (shared schema with measured_joint_limits.yaml).
# ----------------------------------------------------------------------------
def load_joint_windows(limits_path: str) -> dict:
    import yaml  # noqa: PLC0415

    with open(limits_path) as f:
        data = yaml.safe_load(f)
    ref = data["reference_pose_rad"]
    win = data["window_deg"]
    return {j: (ref[j] - math.radians(win[j]), ref[j] + math.radians(win[j])) for j in ref}


def check_against_windows(joints: dict, windows: dict) -> list:
    violations = []
    for j, val in joints.items():
        if j in windows:
            lo, hi = windows[j]
            if not (lo <= val <= hi):
                violations.append(f"{j}={math.degrees(val):.1f} deg outside "
                                  f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
    return violations


# ----------------------------------------------------------------------------
# Execute path -- heavily gated. Imports ROS lazily. Not run in this task.
# ----------------------------------------------------------------------------
def _abort(msg: str) -> int:
    print(f"\nEXECUTE REFUSED: {msg}", file=sys.stderr)
    return 2


def execute(plan: ForkPickPlan, args) -> int:
    # Gate 1: explicit flag.
    if not args.i_am_at_the_estop:
        return _abort("--i-am-at-the-estop was not passed. Default is dry-run only.")
    # Gate 2: fork TCP file.
    if not args.tcp or not os.path.exists(args.tcp):
        return _abort("a real fork TCP file must be supplied with --tcp (none exists yet).")
    # Gate 3: interactive confirmation.
    print("\n*** You are asking to MOVE THE REAL UR10 with the fork gripper. ***")
    print("A person MUST be standing at the e-stop for the whole motion.")
    try:
        ans = input("Type EXECUTE (all caps) to proceed, anything else to abort: ").strip()
    except EOFError:
        return _abort("no interactive confirmation possible (non-tty).")
    if ans != "EXECUTE":
        return _abort("confirmation not given.")

    # Gate 4: robot RUNNING / NORMAL (dashboard).
    mode, safety, program = _dashboard_state(args.robot_ip)
    if mode != "RUNNING" or safety != "NORMAL":
        return _abort(f"robot not RUNNING/NORMAL (mode={mode}, safety={safety}).")

    # ROS checks (lazy import).
    try:
        import rclpy  # noqa: PLC0415, F401
    except ImportError:
        return _abort("rclpy unavailable; source /opt/ros/jazzy/setup.bash.")

    # Gate 5: External Control + motion controller active.
    if not _program_running():
        return _abort("External Control program not running (RUNBOOK section 4 to re-arm).")
    if not _motion_controller_active():
        return _abort("no motion controller active (RUNBOOK section 4).")

    # Gate 6: live feed.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from check_joint_feed_live import check_joint_feed_live  # noqa: PLC0415

    verdict = check_joint_feed_live()
    print(verdict)
    if not verdict.live:
        return _abort("/joint_states is frozen; restart ur10-driver and re-verify.")

    # Gate 7: seeded IK + joint-window guard for every waypoint.
    windows = load_joint_windows(args.limits)
    seed = _current_joints()
    resolved = []
    for w in plan.waypoints:
        joints = _compute_ik(w.position, plan.tool_quat_xyzw, seed)
        if joints is None:
            return _abort(f"no IK solution for waypoint {w.name}.")
        viol = check_against_windows(joints, windows)
        if viol:
            return _abort(f"waypoint {w.name} violates joint windows: {'; '.join(viol)}")
        resolved.append((w, joints))
        seed = joints
    print(f"\nAll {len(resolved)} waypoints resolved within measured joint windows.")
    print(f"Conservative speed cap: {args.max_speed_frac:.2f} slider / "
          f"{args.joint_speed_deg_s:.1f} deg/s per joint.")

    # Dispatch is intentionally a final explicit step so review can stop here.
    # Left as a guarded hand-off to the established executor rather than an
    # unreviewed inline trajectory send:
    print("\nNEXT (not auto-run): dispatch the resolved joint waypoints via "
          "execute_plan.py --exact --limits cumotion/measured_joint_limits.yaml")
    print("under the pendant speed slider, with a person at the e-stop. This script")
    print("stops at the validated, joint-limited plan on purpose.")
    return 0


def _dashboard_state(robot_ip: str):
    import socket

    try:
        with socket.create_connection((robot_ip, 29999), timeout=5) as s:
            s.recv(4096)
            out = b""
            for cmd in (b"robotmode\n", b"safetystatus\n", b"programState\n"):
                s.sendall(cmd)
                import time
                time.sleep(0.3)
                out += s.recv(4096)
        text = out.decode(errors="replace")
        mode = _grep(text, "Robotmode:")
        safety = _grep(text, "Safetystatus:")
        program = _grep(text, "")
        return mode, safety, program
    except OSError as e:
        return f"UNREACHABLE({e})", "UNKNOWN", "UNKNOWN"


def _grep(text: str, key: str) -> str:
    for line in text.splitlines():
        if key and line.startswith(key):
            return line.split(":", 1)[1].strip()
    return ""


def _program_running() -> bool:
    import rclpy
    from std_msgs.msg import Bool

    rclpy.init() if not rclpy.ok() else None
    node = rclpy.create_node("fork_demo_progcheck")
    got = {}

    def cb(m):
        got["v"] = m.data

    node.create_subscription(Bool, "/io_and_status_controller/robot_program_running", cb, 1)
    import time
    t0 = time.time()
    while time.time() - t0 < 3 and "v" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    return bool(got.get("v", False))


def _motion_controller_active() -> bool:
    import rclpy
    from controller_manager_msgs.srv import ListControllers

    node = rclpy.create_node("fork_demo_ctrlcheck")
    cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not cli.wait_for_service(timeout_sec=5):
        node.destroy_node()
        return False
    fut = cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
    res = fut.result()
    node.destroy_node()
    if not res:
        return False
    motion = {"scaled_joint_trajectory_controller", "joint_trajectory_controller"}
    return any(c.name in motion and c.state == "active" for c in res.controller)


def _current_joints() -> dict:
    import rclpy
    from sensor_msgs.msg import JointState

    node = rclpy.create_node("fork_demo_jointread")
    got = {}

    def cb(m):
        got.update(dict(zip(m.name, m.position)))

    node.create_subscription(JointState, "/joint_states", cb, 10)
    import time
    t0 = time.time()
    while time.time() - t0 < 3 and not got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    return {j: got.get(j, 0.0) for j in JOINT_ORDER}


def _compute_ik(position, quat_xyzw, seed) -> dict | None:
    """Seeded IK via MoveIt /compute_ik (joint-space, seeded -> predictable)."""
    import rclpy
    from moveit_msgs.srv import GetPositionIK

    node = rclpy.create_node("fork_demo_ik")
    cli = node.create_client(GetPositionIK, "/compute_ik")
    if not cli.wait_for_service(timeout_sec=5):
        node.destroy_node()
        return None
    req = GetPositionIK.Request()
    ik = req.ik_request
    ik.group_name = "ur_manipulator"
    ik.pose_stamped.header.frame_id = "base_link"
    ik.pose_stamped.pose.position.x = float(position[0])
    ik.pose_stamped.pose.position.y = float(position[1])
    ik.pose_stamped.pose.position.z = float(position[2])
    (ik.pose_stamped.pose.orientation.x,
     ik.pose_stamped.pose.orientation.y,
     ik.pose_stamped.pose.orientation.z,
     ik.pose_stamped.pose.orientation.w) = [float(q) for q in quat_xyzw]
    ik.robot_state.joint_state.name = list(seed.keys())
    ik.robot_state.joint_state.position = [float(v) for v in seed.values()]
    ik.timeout.sec = 1
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
    res = fut.result()
    node.destroy_node()
    if not res or res.error_code.val != 1:
        return None
    js = res.solution.joint_state
    sol = dict(zip(js.name, js.position))
    return {j: sol[j] for j in JOINT_ORDER if j in sol}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dry-run", action="store_true", help="print + plot the plan, no motion (DEFAULT)")
    p.add_argument("--i-am-at-the-estop", dest="i_am_at_the_estop", action="store_true",
                   help="REQUIRED to even attempt execution; asserts a person is at the e-stop")
    p.add_argument("--tcp", default=None, help="fork TCP file (required to execute)")
    p.add_argument("--fork-config", default=DEFAULT_FORK_CFG, help="forklift kit config/zip/dir")
    p.add_argument("--limits", default=DEFAULT_LIMITS, help="measured joint limits yaml")
    p.add_argument("--robot-ip", default="192.168.0.1")
    p.add_argument("--max-speed-frac", type=float, default=0.10, help="pendant speed slider cap")
    p.add_argument("--joint-speed-deg-s", type=float, default=10.0, help="per-joint speed cap")
    p.add_argument("--plot", default=os.path.join(REPO, "data", "ur10-fork-demo-readiness-v1", "fork_demo_path.png"))
    # cell layout overrides
    p.add_argument("--shelf-front-x", type=float, default=None)
    p.add_argument("--pick-slot-z", type=float, default=None)
    p.add_argument("--place-y", type=float, default=None)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    geom = ForkGeometry.load(args.fork_config)
    cell = CellLayout()
    if args.shelf_front_x is not None:
        cell.shelf_front_x = args.shelf_front_x
    if args.pick_slot_z is not None:
        cell.pick_slot_z = args.pick_slot_z
    if args.place_y is not None:
        cell.place_y = args.place_y
    plan = ForkPickPlan(geom=geom, cell=cell).build()

    if args.i_am_at_the_estop:
        return execute(plan, args)
    # default: dry run
    print_plan(plan)
    save_plot(plan, args.plot)
    print("\n(dry-run only. Pass --i-am-at-the-estop + --tcp to attempt a guarded execute.)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
