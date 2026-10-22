#!/usr/bin/env python3
"""Freedrive-TEACH recorder for the real UR10 fork demo (task ur10-fork-pick-real-v1).

The captain hand-guides the arm in freedrive through the pick-and-place; this
records the path at full rate and auto-detects dwells (candidate waypoints) so
the motion can be replayed. Purely RECORDS the arm; it NEVER sends a trajectory
goal. The only arm-side action it can take -- and only behind --i-will-teach-now
-- is to put the controllers into a freedrive-SAFE state:

  FREEDRIVE-SAFE PATH (driver-native, chosen because this CB3 driver HAS
  ur_controllers/FreedriveModeController loaded and it keeps /joint_states live
  over the driver's own RTDE read, and because External Control stays UP so the
  headless auto-arm is not fought):
    1. deactivate EVERY joint-commanding controller
       (scaled/joint/passthrough trajectory, forward position/velocity/effort)
       -> no trajectory goal can reach the arm;
    2. activate freedrive_mode_controller;
    3. publish Bool(true) to /freedrive_mode_controller/enable_freedrive_mode at
       10 Hz (watchdog) -> the arm is hand-backdrivable. If this process dies the
       publish stops, freedrive disables, and the arm HOLDS (fail-safe).
  On exit it disables freedrive, deactivates the freedrive controller, and (with
  --restore-traj) re-activates scaled_joint_trajectory_controller for replay.

Without --i-will-teach-now it only records (use this if the captain instead holds
the pendant Freedrive/teach button with External Control stopped).

Fork TCP: logged as tool0 (base_link<-tool0 via tf2) composed with the sim kit
offset tcp_in_tool0 = [0,-0.038,0.427]. The pendant TCP/payload should ALSO be
set to the fork (TCP [0,-0.038,0.427]; payload gripper 0.82 kg + the carried
tray) so freedrive balances correctly -- but the logged fork pose does not depend
on that, it is computed from tool0.

Output (into --out-dir): teach_log.jsonl (every sample) + teach_dwells.json
(detected dwells) + teach_summary.txt.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

FORK_OFFSET_DEFAULT = (0.0, -0.038, 0.427)
JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TRAJ_CONTROLLERS = ["scaled_joint_trajectory_controller", "joint_trajectory_controller",
                    "passthrough_trajectory_controller", "forward_position_controller",
                    "forward_velocity_controller", "forward_effort_controller"]


def _rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


# ---------------------------------------------------------------------------
# PURE dwell detection -- unit-testable, no ROS.
# ---------------------------------------------------------------------------
def detect_dwells(samples, dwell_sec=2.0, move_tol_m=0.005):
    """samples: list of dicts with 't' (float s) and 'fork' (x,y,z).
    A dwell = a maximal run where every fork position stays within move_tol_m of
    the run's anchor and lasts >= dwell_sec. Returns list of dwell dicts.
    """
    dwells = []
    i = 0
    n = len(samples)
    while i < n:
        anchor = samples[i]["fork"]
        j = i + 1
        while j < n:
            p = samples[j]["fork"]
            d = math.sqrt(sum((p[k] - anchor[k]) ** 2 for k in range(3)))
            if d > move_tol_m:
                break
            j += 1
        dur = samples[j - 1]["t"] - samples[i]["t"]
        if dur >= dwell_sec and (j - i) >= 2:
            seg = samples[i:j]
            mean = tuple(sum(s["fork"][k] for s in seg) / len(seg) for k in range(3))
            mean_j = [sum(s["joints"][k] for s in seg) / len(seg) for k in range(6)] \
                if seg and "joints" in seg[0] else None
            dwells.append({
                "t_start": samples[i]["t"], "t_end": samples[j - 1]["t"],
                "duration_s": round(dur, 2), "n_samples": len(seg),
                "fork_mean": [round(v, 4) for v in mean],
                "joints_mean": [round(v, 5) for v in mean_j] if mean_j else None,
                "label": None,
            })
            i = j
        else:
            i += 1
    return dwells


# ---------------------------------------------------------------------------
# ROS recorder node.
# ---------------------------------------------------------------------------
def run(args) -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import Float64, Bool as BoolMsg  # speed scaling, program
    import tf2_ros
    from controller_manager_msgs.srv import SwitchController, ListControllers

    fork_off = tuple(float(x) for x in args.fork_offset.split(","))
    os.makedirs(args.out_dir, exist_ok=True)
    log_path = os.path.join(args.out_dir, "teach_log.jsonl")
    log_f = open(log_path, "w")

    rclpy.init()
    node = Node("teach_record")
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf, node)

    state = {"joints": None, "wrench": None, "speed": None, "prog": None,
             "last_pos": None, "last_change_t": time.time(), "frozen_warned": False}
    samples = []

    def on_js(m):
        d = dict(zip(m.name, m.position))
        if not all(j in d for j in JOINT_ORDER):
            return
        q = [d[j] for j in JOINT_ORDER]
        now = time.time()
        # fork tip = base<-tool0 * fork_offset
        fork = None
        tool0 = None
        try:
            tr = tf_buf.lookup_transform("base_link", "tool0", rclpy.time.Time())
            t = tr.transform.translation
            r = tr.transform.rotation
            tool0 = [t.x, t.y, t.z, r.x, r.y, r.z, r.w]
            off = _rotate((r.x, r.y, r.z, r.w), fork_off)
            fork = [t.x + off[0], t.y + off[1], t.z + off[2]]
        except Exception:  # noqa: BLE001  tf not ready yet
            pass
        # liveness: did the pose change?
        if state["last_pos"] is not None:
            moved = max(abs(q[k] - state["last_pos"][k]) for k in range(6))
            if moved > 1e-5:
                state["last_change_t"] = now
        state["last_pos"] = q
        rec = {
            "t": now, "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "joints": q, "tool0": tool0, "fork": fork,
            "wrench": state["wrench"], "speed_scaling": state["speed"],
            "program_running": state["prog"],
        }
        log_f.write(json.dumps(rec) + "\n")
        if fork is not None:
            samples.append({"t": now, "fork": fork, "joints": q})

    def on_wrench(m):
        f = m.wrench.force
        state["wrench"] = [f.x, f.y, f.z]

    def on_speed(m):
        state["speed"] = float(m.data)

    def on_prog(m):
        state["prog"] = bool(m.data)

    node.create_subscription(JointState, "/joint_states", on_js, 50)
    node.create_subscription(WrenchStamped,
                             "/force_torque_sensor_broadcaster/wrench", on_wrench, 20)
    # speed scaling topic carries Float64 on this driver's broadcaster
    try:
        node.create_subscription(Float64,
                                 "/speed_scaling_state_broadcaster/speed_scaling",
                                 on_speed, 10)
    except Exception:  # noqa: BLE001
        pass
    node.create_subscription(BoolMsg,
                             "/io_and_status_controller/robot_program_running", on_prog, 5)

    enable_pub = node.create_publisher(Bool, "/freedrive_mode_controller/enable_freedrive_mode", 10)

    def switch(activate, deactivate):
        cli = node.create_client(SwitchController, "/controller_manager/switch_controller")
        if not cli.wait_for_service(timeout_sec=10):
            node.get_logger().error("switch_controller service unavailable")
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15)
        return bool(fut.result() and fut.result().ok)

    teaching = args.i_will_teach_now
    if teaching:
        node.get_logger().info("FREEDRIVE-SAFE: deactivating trajectory controllers, "
                               "activating freedrive_mode_controller ...")
        if not switch(["freedrive_mode_controller"], TRAJ_CONTROLLERS):
            node.get_logger().error("controller switch failed -- NOT enabling freedrive")
            teaching = False

    stop = {"v": False}
    def _sig(*_):
        stop["v"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    print(f"[teach] recording -> {log_path}")
    print(f"[teach] freedrive {'ENABLED (driver)' if teaching else 'NOT managed (pendant teach mode)'}")
    print("[teach] Ctrl-C to stop; dwells printed live.")
    last_enable = 0.0
    last_dwell_report = 0
    t0 = time.time()
    while rclpy.ok() and not stop["v"]:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if teaching and now - last_enable > 0.1:  # 10 Hz freedrive watchdog
            enable_pub.publish(Bool(data=True))
            last_enable = now
        # staleness warning (only meaningful once the captain is moving)
        if now - state["last_change_t"] > 3.0 and now - t0 > 10 and not state["frozen_warned"]:
            node.get_logger().warn("joint feed has not changed in 3 s -- verify it is LIVE")
            state["frozen_warned"] = True
        elif now - state["last_change_t"] < 1.0:
            state["frozen_warned"] = False
        # live dwell reporting
        if len(samples) - last_dwell_report > 200:
            last_dwell_report = len(samples)
            dw = detect_dwells(samples, args.dwell_sec, args.move_tol_m)
            if dw:
                last = dw[-1]
                print(f"[teach] dwell #{len(dw)} {last['duration_s']}s @ fork "
                      f"{last['fork_mean']}", flush=True)

    # teardown
    if teaching:
        enable_pub.publish(Bool(data=False))
        time.sleep(0.3)
        node.get_logger().info("disabling freedrive, deactivating freedrive controller")
        restore = ["scaled_joint_trajectory_controller"] if args.restore_traj else []
        switch(restore, ["freedrive_mode_controller"])
    log_f.close()

    dwells = detect_dwells(samples, args.dwell_sec, args.move_tol_m)
    with open(os.path.join(args.out_dir, "teach_dwells.json"), "w") as f:
        json.dump({"dwell_sec": args.dwell_sec, "move_tol_m": args.move_tol_m,
                   "fork_offset": list(fork_off), "n_samples": len(samples),
                   "dwells": dwells}, f, indent=2)
    with open(os.path.join(args.out_dir, "teach_summary.txt"), "w") as f:
        f.write(f"samples: {len(samples)}\ndwells: {len(dwells)}\n")
        for i, d in enumerate(dwells):
            f.write(f"  dwell {i}: {d['duration_s']}s  fork {d['fork_mean']}  "
                    f"(label: {d['label']})\n")
    print(f"\n[teach] {len(samples)} samples, {len(dwells)} dwells detected.")
    for i, d in enumerate(dwells):
        print(f"  dwell {i}: {d['duration_s']}s  fork {d['fork_mean']}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--i-will-teach-now", action="store_true",
                   help="put controllers in the freedrive-safe state + enable driver freedrive")
    p.add_argument("--restore-traj", action="store_true",
                   help="re-activate scaled_joint_trajectory_controller on exit (for replay)")
    p.add_argument("--fork-offset", default="0,-0.038,0.427", help="tcp_in_tool0 x,y,z (m)")
    p.add_argument("--dwell-sec", type=float, default=2.0)
    p.add_argument("--move-tol-m", type=float, default=0.005)
    return p


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
