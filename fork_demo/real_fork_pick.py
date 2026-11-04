#!/usr/bin/env python3
"""Safety-bounded forklift tray pick-and-place for the REAL UR10 cell.

Task: ur10-fork-pick-real-v1. Picks the BRAVO Erdnussflips tray off the shelf
with the passive forklift gripper S and places it on the table on the other side
of the shelf, mirroring the Isaac sim prototype
(tray_perception configs/forklift_demo/erdnussflips_pick.yaml) but re-anchored on
the REAL, MEASURED tray-front and table-drop poses -- NEVER the twin's numbers.

SAFE BY DEFAULT. Three run modes:

  --dry-run        (DEFAULT) compute + print + plot the base_link TCP waypoints.
                   Touches no ROS, commands NO motion.
  --front-offset M replay the FULL path shifted M metres BACK along the fork
                   insertion axis (step-4 first execution: M=0.15 -> 150 mm in
                   front of the shelf, NO contact). Still real motion => fully
                   gated; labelled FRONT-DRY-RUN.
  (real pick)      --front-offset 0.0 : the contacting pick-and-place.

Both motion modes refuse to run unless EVERY gate in `execute()` passes, and the
actual trajectory dispatch is handed to the reviewed executor
(cams/cellmap/tools/execute_plan.py --exact --limits ...) with an independent
force-abort watchdog (fork_demo/force_abort.py) running alongside. This script
plans, validates and hands off; it never sends an unreviewed inline trajectory.

SAFETY THRESHOLDS (also in the report; see SafetyLimits below):
  * speed scaling <= 10 % on every leg, <= 5 % on the contact legs
    (insert / lift / extract / set-down / withdraw);
  * acceleration scaling capped (<= 5 %);
  * force-deviation abort at the repo backstop (cumotion/collision_force_limits.yaml,
    200 N) -- a COARSE CB3 net above the ~152 N clean-motion swing; the real
    safety-rated stop is the pendant Force limit, left UNTOUCHED;
  * a base_link workspace box keeps the forks clear of the shelf uprights and
    neighbouring trays -- every waypoint is checked, execution refuses on breach;
  * halt-on-tracking-error (execute_plan.py --max-track-rad), the effective
    ROS-side contact guard on this CB3.

The UR safety configuration is never read or written by this script.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fork_pick_demo import (  # noqa: E402  reuse the readiness-lane helpers
    ForkGeometry,
    JOINT_ORDER,
    check_against_windows,
    load_joint_windows,
)

DEFAULT_LIMITS = os.path.join(REPO, "cumotion", "measured_joint_limits.yaml")
DEFAULT_FORCE_CFG = os.path.join(REPO, "cumotion", "collision_force_limits.yaml")


# ---------------------------------------------------------------------------
# Safety limits -- all mandatory thresholds in one visible place (brief step 3).
# ---------------------------------------------------------------------------
@dataclass
class SafetyLimits:
    speed_frac_leg: float = 0.10        # <= 10 % scaling on every leg
    speed_frac_contact: float = 0.05    # <= 5 % on contact legs
    accel_frac: float = 0.05            # acceleration scaling cap
    max_track_rad: float = 0.15         # halt-on-tracking-error (execute_plan.py)
    force_backstop_n: float = 200.0     # loaded from collision_force_limits.yaml
    max_joint_step_deg: float = 20.0    # per-joint delta guard between waypoints
    # base_link workspace box (m). Provisional -- MUST be confirmed against the
    # real shelf uprights / neighbouring trays before any contact run.
    ws_xmin: float = 0.10
    ws_xmax: float = 1.35
    ws_ymin: float = -0.45
    ws_ymax: float = 1.25
    ws_zmin: float = -0.35
    ws_zmax: float = 1.00

    @classmethod
    def load(cls, force_cfg: str = DEFAULT_FORCE_CFG) -> "SafetyLimits":
        s = cls()
        try:
            import yaml  # noqa: PLC0415

            with open(force_cfg) as f:
                data = yaml.safe_load(f)
            s.force_backstop_n = float(data["ros_contact_force_backstop_n"])
        except (OSError, KeyError, ValueError, TypeError):
            pass
        return s

    def in_workspace(self, p) -> bool:
        x, y, z = p
        return (self.ws_xmin <= x <= self.ws_xmax
                and self.ws_ymin <= y <= self.ws_ymax
                and self.ws_zmin <= z <= self.ws_zmax)


# ---------------------------------------------------------------------------
# Relative motion offsets -- the SHAPE from the Isaac yaml, not its poses.
# ---------------------------------------------------------------------------
@dataclass
class MotionOffsets:
    approach_standoff: float = 0.12   # m back from the tray front (pre-approach)
    insert_travel: float = 0.28       # m the forks slide under the tray
    lift: float = 0.035               # m TCP lift on the dock (tray rises 23 mm)
    extract: float = 0.40             # m the loaded forks back out of the bay
    withdraw: float = 0.32            # m the empty forks back out after set-down
    retreat_up: float = 0.05          # m up above the placed load at the end
    carry_clear: float = 0.10         # m extra clearance for the raised transit
    set_down_clear: float = 0.06      # m above the table dock before set-down


# ---------------------------------------------------------------------------
# A measured TCP anchor pose in base_link (position + orientation quaternion).
# ---------------------------------------------------------------------------
@dataclass
class AnchorPose:
    position: tuple           # (x, y, z) base_link, m -- fork-tip centreline point
    quat_xyzw: tuple          # base_link <- tool orientation
    how_measured: str = "PROVISIONAL (placeholder; NOT a real measurement)"
    uncertainty_m: float = float("nan")
    uncertainty_deg: float = float("nan")

    def fwd(self) -> tuple:
        """Insertion axis: tool Z expressed in base, projected to horizontal."""
        v = _rotate(self.quat_xyzw, (0.0, 0.0, 1.0))
        vx, vy, _ = v
        n = math.hypot(vx, vy) or 1.0
        return (vx / n, vy / n, 0.0)


@dataclass
class Waypoint:
    name: str
    position: tuple
    phase: str
    leg_type: str   # "transit" (free-space, cuMotion re-plan) | "contact" (straight)
    speed_frac: float
    note: str = ""


# ---------------------------------------------------------------------------
# Quaternion helper (x, y, z, w) rotating a vector.
# ---------------------------------------------------------------------------
def _rotate(q, v) -> tuple:
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    # v' = v + w*t + cross(q.xyz, t)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return (rx, ry, rz)


def _add(p, v, s=1.0):
    return (p[0] + v[0] * s, p[1] + v[1] * s, p[2] + v[2] * s)


UP = (0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Build the pick-and-place waypoints from the two measured anchors + offsets.
# ---------------------------------------------------------------------------
@dataclass
class ForkPickPlan:
    pick: AnchorPose
    drop: AnchorPose
    off: MotionOffsets
    safety: SafetyLimits
    front_offset: float = 0.0     # m shift BACK along fwd (0 = real; 0.15 = front dry run)
    waypoints: list = field(default_factory=list)

    def build(self) -> "ForkPickPlan":
        o = self.off
        fwd = self.pick.fwd()
        dfwd = self.drop.fwd()
        P = self.pick.position
        D = self.drop.position
        sc_leg = self.safety.speed_frac_leg
        sc_con = self.safety.speed_frac_contact

        pre = _add(P, fwd, -o.approach_standoff)
        insert_start = P
        inserted = _add(P, fwd, o.insert_travel)
        lifted = _add(inserted, UP, o.lift)
        extracted = _add(_add(pre, UP, o.lift), (0, 0, 0))  # pre height + lift
        home = _add(pre, UP, o.carry_clear)

        # Place side (anchored on the measured table-drop pose D, fwd = dfwd).
        set_down = _add(D, UP, o.set_down_clear)
        lowered = D
        withdrawn = _add(D, dfwd, -o.withdraw)
        retreat = _add(withdrawn, UP, o.retreat_up)
        # Transit via-point: above the table drop at a safe clearance, reached
        # from `extracted`. The free-space transit leg itself is planned
        # collision-aware by cuMotion; this via-point just raises the goal clear.
        carry_over = _add(set_down, UP, o.carry_clear)

        wps = [
            Waypoint("HOME", home, "home", "transit", sc_leg,
                     "safe staging above the approach line"),
            Waypoint("PRE_APPROACH", pre, "approach", "transit", sc_leg,
                     f"{o.approach_standoff*1000:.0f} mm in front of the tray, fork height"),
            Waypoint("INSERT_START", insert_start, "insert", "contact", sc_con,
                     "fork tips at the tray front face"),
            Waypoint("INSERTED", inserted, "insert", "contact", sc_con,
                     f"forks slide {o.insert_travel*1000:.0f} mm under the tray"),
            Waypoint("LIFTED", lifted, "lift", "contact", sc_con,
                     f"lift {o.lift*1000:.0f} mm; tray rides the forks (~23 mm off the posts)"),
            Waypoint("EXTRACTED", extracted, "extract", "contact", sc_con,
                     f"retract {o.extract*1000:.0f} mm at the lifted height, tray clear of the bay"),
            Waypoint("CARRY_OVER", carry_over, "carry", "transit", sc_leg,
                     "raised transit to above the table drop (cuMotion free-space)"),
            Waypoint("SET_DOWN", set_down, "place", "transit", sc_leg,
                     f"{o.set_down_clear*1000:.0f} mm above the table dock"),
            Waypoint("LOWERED", lowered, "place", "contact", sc_con,
                     "lower; tray sets down on the table dock"),
            Waypoint("WITHDRAWN", withdrawn, "place", "contact", sc_con,
                     f"forks withdraw {o.withdraw*1000:.0f} mm, clear of the tray"),
            Waypoint("RETREAT", retreat, "retreat", "transit", sc_leg,
                     "up above the placed load, cycle complete"),
        ]
        if self.front_offset > 0.0:
            # Shift every waypoint BACK along the pick insertion axis so the forks
            # never reach the shelf/tray/table: a no-contact rehearsal of the path.
            wps = [Waypoint(w.name, _add(w.position, fwd, -self.front_offset),
                            w.phase, w.leg_type, w.speed_frac,
                            w.note + " [shifted back: no contact]") for w in wps]
        self.waypoints = wps
        return self

    def label(self) -> str:
        if self.front_offset > 0.0:
            return f"FRONT-DRY-RUN ({self.front_offset*1000:.0f} mm back, NO contact)"
        return "REAL PICK-AND-PLACE (contact)"


# ---------------------------------------------------------------------------
# Measured-pose loader. Refuses placeholder anchors on the execute path.
# ---------------------------------------------------------------------------
def load_anchors(path: str):
    import yaml  # noqa: PLC0415

    with open(path) as f:
        data = yaml.safe_load(f)

    def one(key) -> AnchorPose:
        d = data[key]
        return AnchorPose(
            position=tuple(float(v) for v in d["position_base"]),
            quat_xyzw=tuple(float(v) for v in d["quat_xyzw_base"]),
            how_measured=str(d.get("how_measured", "")),
            uncertainty_m=float(d.get("uncertainty_m", float("nan"))),
            uncertainty_deg=float(d.get("uncertainty_deg", float("nan"))),
        )

    return one("pick_front"), one("table_drop")


def provisional_anchors(fork_cfg_scene: dict | None = None):
    """Placeholder anchors so --dry-run works before the real poses are measured.

    Shelf ahead on +base_X at ~0.95 m (forklift_config scene_default); table on
    the far side is a stand-in. LOUDLY not a measurement -- execution refuses
    these (see execute()).
    """
    # tool Z -> +base_X, tool Y -> +base_Z (forks hang, camera up):
    # rotation mapping toolX->+Y, toolY->+Z, toolZ->+X. Quaternion (x,y,z,w):
    q = (0.5, 0.5, 0.5, 0.5)
    pick = AnchorPose(position=(0.95, 0.0, 0.30), quat_xyzw=q)
    drop = AnchorPose(position=(0.95, 0.70, -0.05), quat_xyzw=q)
    return pick, drop


# ---------------------------------------------------------------------------
# Dry run: print + plot. No ROS.
# ---------------------------------------------------------------------------
def print_plan(plan: ForkPickPlan, geom: ForkGeometry) -> None:
    s = plan.safety
    print("=" * 84)
    print(f"REAL FORK TRAY PICK-AND-PLACE -- {plan.label()} -- DRY RUN (no motion, no ROS)")
    print("=" * 84)
    print(f"pick anchor  : {tuple(round(v,3) for v in plan.pick.position)}  "
          f"[{plan.pick.how_measured}]")
    print(f"table anchor : {tuple(round(v,3) for v in plan.drop.position)}  "
          f"[{plan.drop.how_measured}]")
    print(f"payload      : gripper {geom.gripper_mass:.2f} kg + tray/product up to "
          f"{geom.tray_mass_full:.2f} kg")
    print("-" * 84)
    print("SAFETY LIMITS:")
    print(f"  speed <= {s.speed_frac_leg*100:.0f}% per leg, <= {s.speed_frac_contact*100:.0f}% on contact legs; "
          f"accel <= {s.accel_frac*100:.0f}%")
    print(f"  force-deviation backstop {s.force_backstop_n:.0f} N (ROS coarse net); "
          f"pendant Force limit UNTOUCHED")
    print(f"  tracking-error halt {math.degrees(s.max_track_rad):.1f} deg; "
          f"per-joint step guard {s.max_joint_step_deg:.0f} deg")
    print(f"  workspace box base_link x[{s.ws_xmin},{s.ws_xmax}] "
          f"y[{s.ws_ymin},{s.ws_ymax}] z[{s.ws_zmin},{s.ws_zmax}] m")
    print("-" * 84)
    print(f"{'#':>2} {'waypoint':<13} {'phase':<8} {'leg':<8} {'spd':>4} "
          f"{'x':>7} {'y':>7} {'z':>7}  in-box  note")
    for i, w in enumerate(plan.waypoints):
        x, y, z = w.position
        inbox = "ok" if s.in_workspace(w.position) else "OUT!"
        print(f"{i:>2} {w.name:<13} {w.phase:<8} {w.leg_type:<8} "
              f"{w.speed_frac*100:3.0f}% {x:7.3f} {y:7.3f} {z:7.3f}  {inbox:<6} {w.note}")
    print("-" * 84)
    breaches = [w.name for w in plan.waypoints if not s.in_workspace(w.position)]
    if breaches:
        print(f"WORKSPACE BREACH at: {', '.join(breaches)} -- would refuse to execute.")
    print("All positions are base_link TCP (fork-tip centreline) targets.")
    if "PROVISIONAL" in plan.pick.how_measured or "PROVISIONAL" in plan.drop.how_measured:
        print("*** PROVISIONAL anchors: supply measured poses with --poses before any run. ***")


def save_plot(plan: ForkPickPlan, path: str) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print(f"(matplotlib unavailable; skipping plot {path})")
        return False
    xs = [w.position[0] for w in plan.waypoints]
    ys = [w.position[1] for w in plan.waypoints]
    zs = [w.position[2] for w in plan.waypoints]
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    colors = {"contact": "tab:red", "transit": "tab:blue"}
    for i in range(len(plan.waypoints) - 1):
        seg = plan.waypoints[i + 1]
        ax.plot(xs[i:i + 2], ys[i:i + 2], zs[i:i + 2], "-o",
                color=colors.get(seg.leg_type, "gray"))
    for w in plan.waypoints:
        ax.text(*w.position, f" {w.name}", fontsize=7)
    ax.scatter([0], [0], [0], color="k", marker="s", s=40, label="base_link")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title(f"Fork pick-and-place -- {plan.label()}\n(red=contact leg, blue=transit)")
    ax.legend()
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    print(f"saved path plot -> {path}")
    return True


# ---------------------------------------------------------------------------
# Execute path -- heavily gated. Imports ROS lazily. NOT run before the gate.
# ---------------------------------------------------------------------------
def _abort(msg: str) -> int:
    print(f"\nEXECUTE REFUSED: {msg}", file=sys.stderr)
    return 2


def execute(plan: ForkPickPlan, args) -> int:
    s = plan.safety
    # Gate 0: measured anchors, never placeholders.
    for a in (plan.pick, plan.drop):
        if "PROVISIONAL" in a.how_measured or math.isnan(a.uncertainty_m):
            return _abort("anchors are PROVISIONAL -- supply measured --poses "
                          "(with how_measured + uncertainty) before executing.")
    # Gate 1: explicit flag.
    if not args.i_am_at_the_estop:
        return _abort("--i-am-at-the-estop not passed. Default is dry-run only.")
    # Gate 2: fork TCP file supplied (the robot-side TCP is verified separately
    # with fork_demo/read_tcp_payload.py; this records the expected offset).
    if not args.tcp or not os.path.exists(args.tcp):
        return _abort("a fork TCP file must be supplied with --tcp.")
    # Gate 3: workspace box holds for every waypoint.
    breaches = [w.name for w in plan.waypoints if not s.in_workspace(w.position)]
    if breaches:
        return _abort(f"workspace box breached at {', '.join(breaches)}.")
    # Gate 4: interactive confirmation.
    print(f"\n*** You are about to MOVE THE REAL UR10: {plan.label()} ***")
    print("A person MUST be at the e-stop for the entire motion.")
    try:
        if input("Type EXECUTE (all caps) to proceed: ").strip() != "EXECUTE":
            return _abort("confirmation not given.")
    except EOFError:
        return _abort("no interactive confirmation possible (non-tty).")

    # ROS checks (lazy import). These mirror fork_pick_demo.execute().
    try:
        import rclpy  # noqa: PLC0415, F401
    except ImportError:
        return _abort("rclpy unavailable; source /opt/ros/jazzy/setup.bash.")
    from fork_pick_demo import (  # noqa: PLC0415
        _compute_ik, _current_joints, _dashboard_state, _motion_controller_active,
        _program_running,
    )

    mode, safety_mode, _ = _dashboard_state(args.robot_ip)
    if mode != "RUNNING" or safety_mode != "NORMAL":
        return _abort(f"robot not RUNNING/NORMAL (mode={mode}, safety={safety_mode}).")
    if not _program_running():
        return _abort("External Control not running (RUNBOOK section 4 to re-arm).")
    if not _motion_controller_active():
        return _abort("no motion controller active (RUNBOOK section 4).")

    from check_joint_feed_live import check_joint_feed_live  # noqa: PLC0415
    verdict = check_joint_feed_live()
    print(verdict)
    if not verdict.live:
        return _abort("/joint_states is frozen; restart ur10-driver and re-verify.")

    # Resolve seeded IK per waypoint; enforce joint windows + per-joint step.
    windows = load_joint_windows(args.limits)
    seed = _current_joints()
    resolved = []
    for w in plan.waypoints:
        joints = _compute_ik(w.position, plan.pick.quat_xyzw, seed)
        if joints is None:
            return _abort(f"no IK for waypoint {w.name}.")
        viol = check_against_windows(joints, windows)
        if viol:
            return _abort(f"{w.name} violates joint windows: {'; '.join(viol)}")
        step = max(abs(math.degrees(joints[j] - seed[j])) for j in JOINT_ORDER)
        if step > s.max_joint_step_deg:
            return _abort(f"{w.name} needs a {step:.0f} deg joint step "
                          f"(> {s.max_joint_step_deg:.0f} deg guard).")
        resolved.append((w, joints))
        seed = joints

    print(f"\nAll {len(resolved)} waypoints resolved inside the measured joint windows "
          f"and the workspace box.")
    print("Per-leg speed policy (<= 10 % every leg, <= 5 % contact):")
    for w, _ in resolved:
        print(f"   {w.name:<13} {w.leg_type:<8} {w.speed_frac*100:3.0f}%")
    print(f"Force-deviation backstop: {s.force_backstop_n:.0f} N "
          f"(run fork_demo/force_abort.py alongside the dispatch).")
    print("\nNEXT (deliberate, separate hand-off -- this script stops here):")
    print("  1. save each leg's resolved joint_path as a plan yaml;")
    print("  2. dispatch via execute_plan.py --exact "
          f"--limits {os.path.relpath(args.limits, REPO)} "
          f"--max-track-rad {s.max_track_rad};")
    print("  3. with fork_demo/force_abort.py watching /force_torque_sensor_broadcaster")
    print("     and a person at the e-stop. Contact legs run at 5 % slider.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dry-run", action="store_true", help="print + plot, no motion (DEFAULT)")
    p.add_argument("--poses", default=None,
                   help="measured anchors yaml (pick_front + table_drop in base_link)")
    p.add_argument("--i-am-at-the-estop", dest="i_am_at_the_estop", action="store_true",
                   help="REQUIRED to attempt motion; asserts a person is at the e-stop")
    p.add_argument("--front-offset", type=float, default=0.0,
                   help="shift the path M metres BACK along fwd (0.15 = 150 mm front dry run)")
    p.add_argument("--tcp", default=None, help="fork TCP file (required to execute)")
    p.add_argument("--fork-config",
                   default=os.path.expanduser(
                       "~/datasets/shelf-twin/grippers/forklift_S/isaac_sim_S_forklift_gripper.zip"))
    p.add_argument("--limits", default=DEFAULT_LIMITS)
    p.add_argument("--force-cfg", default=DEFAULT_FORCE_CFG)
    p.add_argument("--robot-ip", default="192.168.0.1")
    p.add_argument("--plot", default=os.path.join(
        REPO, "data", "ur10-fork-pick-real-v1", "fork_pick_path.png"))
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    geom = ForkGeometry.load(args.fork_config)
    safety = SafetyLimits.load(args.force_cfg)
    # Offsets default to the Isaac erdnussflips_pick.yaml SHAPE (standoff 0.12,
    # insert 0.28, lift 0.035, extract 0.40, withdraw 0.32) -- the authoritative
    # reference for this demo, not the kit's generic pallet config.
    off = MotionOffsets()
    if args.poses and os.path.exists(args.poses):
        pick, drop = load_anchors(args.poses)
    else:
        pick, drop = provisional_anchors()

    plan = ForkPickPlan(pick=pick, drop=drop, off=off, safety=safety,
                        front_offset=args.front_offset).build()

    if args.i_am_at_the_estop:
        return execute(plan, args)
    print_plan(plan, geom)
    save_plot(plan, args.plot)
    print("\n(dry-run only. --front-offset 0.15 + --i-am-at-the-estop + --poses "
          "+ --tcp for the gated 150 mm no-contact rehearsal.)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
