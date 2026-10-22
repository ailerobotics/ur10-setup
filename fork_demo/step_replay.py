#!/usr/bin/env python3
"""Execute ONE step of the teach-002 replay on the REAL arm, captain-gated
(task ur10-fork-pick-real-v1). One invocation = one segment; nothing moves
without the relayed go (firstmate drives the invocations).

Per segment it:
  * reads the current measured joint pose;
  * for segment 0 (home->pre_grip) builds a DIRECT interpolation current->target
    (the arm sits at the teach END pose, not its start); for the rest it loads
    the segment's recorded joint path (plan_NN yaml from build_replay.py) and
    re-pins path[0] to the CURRENT pose so there is no start jump;
  * refuses if the current pose is not within --start-tol of the expected start
    (gross mismatch = arm not where the previous step left it);
  * launches fork_demo/force_abort.py as an INDEPENDENT watchdog;
  * dispatches via the PROVEN cams/cellmap/tools/execute_plan.py --exact
    (5% cap, re-checks peak velocity, joint-window guard against the teach
    envelope replay_joint_limits.yaml, tracking-error + program-stop abort);
  * reports measured end pose vs the taught target (joints + fork-tip).

Adjust (captain hand-corrects a pose at a pause):
  --adjust-start : freedrive ON (deactivate traj ctrl, activate freedrive, hold)
  --adjust-set LABEL : record the current pose as waypoint LABEL, freedrive OFF,
                       rebuild that segment's plan yaml.

NO motion goal is ever sent except through execute_plan.py under a segment go.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
EXECUTE_PLAN = os.path.join(REPO, "cams", "cellmap", "tools", "execute_plan.py")
FORCE_ABORT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "force_abort.py")
JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
VEL_LIMIT = {"shoulder_pan_joint": 2.09, "shoulder_lift_joint": 2.09, "elbow_joint": 3.14,
             "wrist_1_joint": 3.14, "wrist_2_joint": 3.14, "wrist_3_joint": 3.14}
FREEDRIVE = "freedrive_mode_controller"
TRAJ = "scaled_joint_trajectory_controller"
PEAK_TARGET = 0.045


# --------------------------------------------------------------------------- ROS reads
def read_state():
    """Return (joints dict in JO order, fork_tip xyz) from a short rclpy read."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import tf2_ros

    rclpy.init()
    n = Node("step_replay_read")
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    got = {}
    n.create_subscription(JointState, "/joint_states",
                          lambda m: got.update(dict(zip(m.name, m.position))), 20)
    t0 = time.time()
    while time.time() - t0 < 4 and not all(j in got for j in JOINT_ORDER):
        rclpy.spin_once(n, timeout_sec=0.1)
    joints = {j: got.get(j) for j in JOINT_ORDER}
    fork = None
    t1 = time.time()
    while time.time() - t1 < 3:
        try:
            tr = buf.lookup_transform("base_link", "tool0", rclpy.time.Time())
            t, r = tr.transform.translation, tr.transform.rotation
            # fork offset [0,-0.038,0.427] rotated by tool0
            from real_fork_pick import _rotate
            off = _rotate((r.x, r.y, r.z, r.w), (0.0, -0.038, 0.427))
            fork = [t.x + off[0], t.y + off[1], t.z + off[2]]
            break
        except Exception:  # noqa: BLE001
            rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.shutdown()
    return joints, fork


def switch_controllers(activate, deactivate):
    import rclpy
    from rclpy.node import Node
    from controller_manager_msgs.srv import SwitchController
    rclpy.init()
    n = Node("step_replay_switch")
    cli = n.create_client(SwitchController, "/controller_manager/switch_controller")
    ok = False
    if cli.wait_for_service(timeout_sec=10):
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=15)
        ok = bool(fut.result() and fut.result().ok)
    n.destroy_node(); rclpy.shutdown()
    return ok


# --------------------------------------------------------------------------- plan build
def interp(a, b, npts):
    return [[a[i] + (b[i] - a[i]) * k / (npts - 1) for i in range(6)] for k in range(npts)]


def seg_duration(path):
    # execute_plan.py uses uniform dt = duration/(n-1) and peak = worst/dt, so to
    # hold peak at PEAK_TARGET: duration = (worst/PEAK_TARGET) * (n-1).
    worst = 0.0
    for p, q in zip(path[:-1], path[1:]):
        for i, j in enumerate(JOINT_ORDER):
            worst = max(worst, abs(q[i] - p[i]) / VEL_LIMIT[j])
    dt = worst / PEAK_TARGET if worst else 0.1
    return max(round(dt * (len(path) - 1), 1), 2.0)


def build_temp_plan(run_dir, seg, cur_list, waypoints):
    """Return a temp plan yaml path: path[0]=current, rest = recorded (or direct)."""
    import yaml
    label = f"replay_{seg['index']:02d}_{seg['name']}"
    if seg["index"] == 0:
        target = waypoints["pre_grip"]["joints_mean"]
        path = interp(cur_list, target, 16)
    else:
        plan = yaml.safe_load(open(os.path.join(run_dir, seg["plan_yaml"])))
        rec = plan["joint_path"]
        path = [cur_list] + rec[1:]           # re-pin start to current (no jump)
    dur = seg_duration(path)
    goal = path[-1]
    out = {"label": label, "joint_path": path, "goal_joints": goal, "duration_s": float(dur)}
    tmp = os.path.join(run_dir, f"_live_{label}.yaml")
    yaml.safe_dump(out, open(tmp, "w"))
    return tmp, dur, goal


# --------------------------------------------------------------------------- run one step
def run_segment(args):
    plan = json.load(open(os.path.join(args.run_dir, "replay_plan.json")))
    seg = next((s for s in plan["segments"] if s["index"] == args.segment), None)
    if seg is None:
        print(f"no segment {args.segment}", file=sys.stderr); return 2
    lab = json.load(open(os.path.join(args.run_dir, "labelled_waypoints.json")))
    waypoints = {w["label"]: w for w in lab["waypoints"] if not w.get("segment")}

    joints, fork = read_state()
    if any(v is None for v in joints.values()):
        print("ABORT: could not read current joints", file=sys.stderr); return 2
    cur = [joints[j] for j in JOINT_ORDER]

    # expected start = recorded path[0] (or for seg 0, no check: direct from current)
    if seg["index"] != 0:
        import yaml
        rec0 = yaml.safe_load(open(os.path.join(args.run_dir, seg["plan_yaml"])))["joint_path"][0]
        d = max(abs(cur[i] - rec0[i]) for i in range(6))
        if d > args.start_tol_rad:
            print(f"ABORT: arm is {math.degrees(d):.1f} deg from the expected start of "
                  f"'{seg['name']}' (max tol {math.degrees(args.start_tol_rad):.1f}). "
                  f"Arm not where the previous step left it.", file=sys.stderr)
            return 2

    tmp, dur, goal = build_temp_plan(args.run_dir, seg, cur, waypoints)
    print(f"STEP {seg['index']} {seg['name']} [{'CONTACT' if seg['contact'] else 'free'}] "
          f"speed<=5%, {dur:.1f}s, {seg['n_points']} pts")
    print(f"  from fork {[round(v,3) for v in fork] if fork else '?'}")

    env = dict(os.environ, ROS_DOMAIN_ID="0", FASTDDS_BUILTIN_TRANSPORTS="UDPv4")
    watch = None
    if not args.no_force_watch:
        watch = subprocess.Popen([sys.executable, FORCE_ABORT, "--controller", TRAJ],
                                 env=env)
        time.sleep(1.0)
    cmd = [sys.executable, EXECUTE_PLAN, tmp, "--exact",
           "--limits", os.path.join(args.run_dir, "replay_joint_limits.yaml"),
           "--start-tol-rad", str(args.start_tol_rad),
           "--max-track-rad", str(args.max_track_rad)]
    print("  dispatch:", " ".join(cmd))
    rc = subprocess.call(cmd, env=env)
    if watch:
        watch.terminate()
        try:
            watch.wait(timeout=5)
        except subprocess.TimeoutExpired:
            watch.kill()

    joints2, fork2 = read_state()
    cur2 = [joints2[j] for j in JOINT_ORDER if joints2[j] is not None]
    jerr = max(abs(cur2[i] - goal[i]) for i in range(6)) if len(cur2) == 6 else float("nan")
    print(f"\nSTEP {seg['index']} result rc={rc}")
    print(f"  measured end fork {[round(v,3) for v in fork2] if fork2 else '?'}")
    print(f"  joint error vs taught target: {math.degrees(jerr):.2f} deg")
    nxt = next((s for s in plan["segments"] if s["index"] == args.segment + 1), None)
    if nxt:
        print(f"  NEXT pause: step {nxt['index']} {nxt['name']} "
              f"[{'CONTACT' if nxt['contact'] else 'free'}]")
    try:
        os.remove(tmp)
    except OSError:
        pass
    return 0 if rc == 0 else 1


def run_adjust(args):
    hold_py = os.path.join(os.path.dirname(os.path.abspath(__file__)), "freedrive_hold.py")
    pidfile = os.path.join(args.run_dir, ".freedrive_hold.pid")
    if args.adjust_start:
        ok = switch_controllers([FREEDRIVE], [TRAJ])
        if not ok:
            print("controller switch failed", file=sys.stderr); return 2
        env = dict(os.environ, ROS_DOMAIN_ID="0", FASTDDS_BUILTIN_TRANSPORTS="UDPv4")
        p = subprocess.Popen([sys.executable, hold_py], env=env)
        open(pidfile, "w").write(str(p.pid))
        print(f"FREEDRIVE ON (adjust), hold pid {p.pid}. Hand-move the arm; "
              f"then call --adjust-set LABEL to record + release.")
        return 0
    if args.adjust_set:
        if os.path.exists(pidfile):
            try:
                os.kill(int(open(pidfile).read().strip()), 15)
            except (OSError, ValueError):
                pass
            os.remove(pidfile)
            time.sleep(0.5)
        joints, fork = read_state()
        cur = [joints[j] for j in JOINT_ORDER]
        switch_controllers([TRAJ], [FREEDRIVE])
        # update the labelled waypoint + rebuild via build_replay
        lab_path = os.path.join(args.run_dir, "labelled_waypoints.json")
        lab = json.load(open(lab_path))
        found = False
        for w in lab["waypoints"]:
            if w.get("label") == args.adjust_set:
                w["joints_mean"] = cur
                w["fork_mean"] = fork
                w["adjusted"] = True
                found = True
        json.dump(lab, open(lab_path, "w"), indent=2)
        print(f"recorded current pose as '{args.adjust_set}' (found={found}); freedrive OFF.")
        print("re-run build_replay.py to regenerate the affected segment plan yaml.")
        return 0 if found else 2
    return 2


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-dir", default="/home/manishsaini/firstmate/data/ur10-fork-pick-real-v1/teach-002")
    p.add_argument("--segment", type=int)
    p.add_argument("--start-tol-rad", type=float, default=0.12, help="max start mismatch (~7deg)")
    p.add_argument("--max-track-rad", type=float, default=0.15)
    p.add_argument("--no-force-watch", action="store_true")
    p.add_argument("--adjust-start", action="store_true")
    p.add_argument("--adjust-set", default=None, help="record current pose as this waypoint label")
    a = p.parse_args(argv)
    if a.adjust_start or a.adjust_set:
        return run_adjust(a)
    if a.segment is None:
        print("give --segment N or --adjust-*", file=sys.stderr); return 2
    return run_segment(a)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
