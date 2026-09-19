#!/usr/bin/env python3
"""cuRobo MPC reach-and-follow prototype for THIS cell's UR10 (SIM/EVAL ONLY).

Option (2)/candidate C3 of the human-dynamic-plan-s1 report: NVIDIA's reactive
follow-and-avoid MPC. Follows a scripted moving Cartesian target while avoiding a
world model (ground + representative shelf, then a moving capsule standing in for
a person), using the cell's MEASURED robot description and joint windows
(mpc_curobo/robot/ur10_cell_curobo.yml, built from cumotion/ur10_measured.*).

Measures, per scenario: control rate, warm solve time per step, peak GPU memory,
tracking error, collision margin, and behaviour when the target is unreachable or
the path is blocked.

Run inside the fm-plan-mpc-curobo-v1 container (holds the GPU):
    docker run --rm --gpus all -v <repo>:/repo:rw fm-plan-mpc-curobo-v1:latest \
        python3 /repo/mpc_curobo/mpc_prototype.py --scenario follow --out /repo/...json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import threading
import time

# baseline GPU memory BEFORE torch/cuda init, so we can attribute our footprint
# (we hold the fleet GPU lock, so only the constant ~2.2 GB voice service and this
# process use the card during a run).
def _gpu_used_mib():
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5)
        return float(out.stdout.strip().splitlines()[0])
    except Exception:                                    # noqa: BLE001
        return float("nan")


BASELINE_GPU_MIB = _gpu_used_mib()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world import cell_world  # noqa: E402

import torch  # noqa: E402
from curobo.model_predictive_control import (  # noqa: E402
    ModelPredictiveControl, ModelPredictiveControlCfg)
from curobo.types import GoalToolPose, JointState  # noqa: E402

ROBOT_DEFAULT = "/repo/mpc_curobo/robot/ur10_cell_curobo.yml"


class GpuSampler(threading.Thread):
    """Sample global GPU memory.used at ~5 Hz; keep the peak."""

    def __init__(self):
        super().__init__(daemon=True)
        self.peak = BASELINE_GPU_MIB
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            self.peak = max(self.peak, _gpu_used_mib())
            time.sleep(0.2)

    def stop(self):
        self._stop.set()


# --------------------------------------------------------------------------- #
# scenarios: target path + optional moving person
# --------------------------------------------------------------------------- #
def make_target_fn(scenario, center):
    """Return target(t)->xyz. center is the reachable pick-anchor TCP position."""
    cx, cy, cz = center
    if scenario == "unreachable":
        # far outside the UR10 reach and the joint windows: ~2.4 m out.
        return lambda t: (2.4, 0.0, 1.6)
    if scenario == "blocked":
        # a fixed reachable target offset in +y, on the far side of the person.
        return lambda t: (cx, cy + 0.30, cz)

    # follow / person: sweep the target side to side along y, sinusoidal, around
    # the reachable anchor centre (small amplitude so every pose is reachable).
    def target(t):
        y = cy + 0.25 * math.sin(2 * math.pi * t / 8.0)   # 8 s period, +-0.25 m
        z = cz + 0.08 * math.sin(2 * math.pi * t / 5.0)
        return (cx, y, z)
    return target


def make_person_fn(scenario, center):
    """Return person(t)->(x,y,z) pose origin, or None if no person moves.
    The person is placed nearer the base than the pick region so it crosses the
    space between the robot and the shelf."""
    cx, cy, cz = center
    if scenario == "person":
        # walk across the front of the cell from -0.9 to +0.9 m in y over 12 s,
        # at x just in front of the pick region.
        def person(t):
            y = -0.9 + 1.8 * min(max(t / 12.0, 0.0), 1.0)
            return (cx - 0.22, y, 0.0)
        return person
    if scenario == "blocked":
        # park the person directly between the robot and the target and hold.
        return lambda t: (cx - 0.10, cy + 0.15, 0.0)
    return None


def robot_spheres(mpc, state):
    """Return list of (x,y,z,r) collision spheres at this state, or [] if the
    kinematics result does not expose them under a known attribute."""
    res = mpc.compute_kinematics(state)
    for attr in ("robot_spheres", "link_spheres_tensor", "robot_spheres_tensor"):
        t = getattr(res, attr, None)
        if t is not None:
            arr = t.detach().reshape(-1, 4).cpu().tolist()
            return [(a[0], a[1], a[2], a[3]) for a in arr if a[3] > 0]
    return []


def advance(mpc, state):
    res = mpc.optimize_action_sequence(state)
    if res.action_sequence is None or res.action_sequence.position.shape[1] == 0:
        return state, res
    nxt = res.action_sequence.position[:, -1, :]
    ns = JointState.from_position(nxt.clone(), joint_names=mpc.joint_names)
    ns.velocity = res.action_sequence.velocity[:, -1, :]
    ns.acceleration = res.action_sequence.acceleration[:, -1, :]
    return ns, res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", choices=["follow", "person", "unreachable", "blocked"],
                    default="follow")
    ap.add_argument("--robot", default=ROBOT_DEFAULT)
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--optimization-dt", type=float, default=0.03)
    ap.add_argument("--interpolation-steps", type=int, default=4)
    ap.add_argument("--collision-activation", type=float, default=0.05,
                    help="obstacle activation distance (m) for the optimizer")
    ap.add_argument("--out", default=None, help="metrics JSON path")
    ap.add_argument("--trace", default=None, help="per-step JSONL trace path")
    ap.add_argument("--limits", default="/repo/cumotion/measured_joint_limits.yaml")
    args = ap.parse_args()

    # independent joint-window guard (recomputed from the measured YAML)
    from robot.joint_windows import load_measured_windows
    windows = load_measured_windows(args.limits)

    world = (cell_world.world_with_person() if args.scenario in ("person", "blocked")
             else cell_world.static_world())

    sampler = GpuSampler()
    sampler.start()

    cfg = ModelPredictiveControlCfg.create(
        robot=args.robot, scene_model=world, use_cuda_graph=True,
        optimization_dt=args.optimization_dt,
        interpolation_steps=args.interpolation_steps,
        optimizer_collision_activation_distance=args.collision_activation)
    mpc = ModelPredictiveControl(cfg)

    start = JointState.from_position(
        mpc.default_joint_position.clone().unsqueeze(0), joint_names=mpc.joint_names)
    start.velocity = torch.zeros_like(start.position)
    start.acceleration = torch.zeros_like(start.position)
    mpc.setup(start)

    link = mpc.tool_frames[0]
    # Derive a REACHABLE pick pose (position + tool orientation) by FK of the
    # in-window pick-anchor config, and track that orientation for every target.
    anchor = JointState.from_position(
        torch.tensor([cell_world.PICK_ANCHOR], device="cuda", dtype=torch.float32),
        joint_names=mpc.joint_names)
    anchor_pose = mpc.compute_kinematics(anchor).tool_poses.to_dict()[link]
    center = anchor_pose.position.squeeze().detach().cpu().tolist()
    start_quat = anchor_pose.quaternion.squeeze().detach().cpu().tolist()
    target_fn = make_target_fn(args.scenario, center)
    person_fn = make_person_fn(args.scenario, center)

    def set_goal(xyz, run_ik):
        poses = mpc.compute_kinematics(state).tool_poses.to_dict()
        poses[link].position[..., 0] = xyz[0]
        poses[link].position[..., 1] = xyz[1]
        poses[link].position[..., 2] = xyz[2]
        poses[link].quaternion[..., 0] = start_quat[0]
        poses[link].quaternion[..., 1] = start_quat[1]
        poses[link].quaternion[..., 2] = start_quat[2]
        poses[link].quaternion[..., 3] = start_quat[3]
        mpc.update_goal_tool_poses(
            GoalToolPose.from_poses(poses, ordered_tool_frames=mpc.tool_frames,
                                    num_goalset=1),
            run_ik=run_ik)

    state = start
    # initial goal (with IK) so the optimizer has a joint-space attractor
    set_goal(target_fn(0.0), run_ik=True)

    # warm-up (compile cuda graphs, warm-start the optimizer)
    for _ in range(8):
        state, _ = advance(mpc, state)
    torch.cuda.synchronize()

    # if this scenario has a person, confirm the obstacle is updatable up front
    # (fail loudly rather than silently ignoring the person — that masked a real
    # bug once: cuRobo only exposes CUBOIDS by name, so the person is a cuboid).
    if person_fn is not None:
        names = mpc.scene_collision_checker.get_obstacle_names()
        assert cell_world.PERSON in names, \
            f"person obstacle not updatable; scene has {names}"
        px, py, _ = person_fn(0.0)
        mpc.scene_collision_checker.update_obstacle_pose(
            cell_world.PERSON, _mk_pose(*cell_world.person_pose(px, py)[:3]))

    dt = args.optimization_dt
    solve_ms = []
    track_err = []
    min_clear = float("inf")
    min_person_clear = float("inf")
    window_viol = 0
    max_window_excess = 0.0
    trace = []
    t_run0 = time.time()
    for i in range(args.steps):
        t = i * dt
        # move target every step (pose tracking, cheap); refresh IK seed periodically
        set_goal(target_fn(t), run_ik=(i % 40 == 0))
        if person_fn is not None:
            px, py, _ = person_fn(t)
            pose = cell_world.person_pose(px, py)
            mpc.scene_collision_checker.update_obstacle_pose(
                cell_world.PERSON, _mk_pose(pose[0], pose[1], pose[2]))
            world["cuboid"][cell_world.PERSON]["pose"] = pose

        tic = time.time()
        state, res = advance(mpc, state)
        torch.cuda.synchronize()
        solve_ms.append(1e3 * (time.time() - tic))

        # metrics
        cur = mpc.compute_kinematics(state).tool_poses.to_dict()[link]
        cpos = cur.position.squeeze().detach().cpu().tolist()
        tx = target_fn(t)
        err = math.dist(cpos, tx)
        track_err.append(err)

        qpos = state.position.squeeze().detach().cpu().tolist()
        qd = {j: float(v) for j, v in zip(mpc.joint_names, qpos)}
        excess = max((max(lo - qd[j], qd[j] - hi, 0.0)
                      for j, (lo, hi) in windows.windows.items() if j in qd), default=0.0)
        max_window_excess = max(max_window_excess, excess)
        if not windows.within(qd):
            window_viol += 1

        sph = robot_spheres(mpc, state)
        if sph:
            min_clear = min(min_clear, cell_world.min_sphere_clearance(sph, world))
            if person_fn is not None:
                person_only = {"cuboid": {cell_world.PERSON: world["cuboid"][cell_world.PERSON]}}
                min_person_clear = min(min_person_clear,
                                       cell_world.min_sphere_clearance(sph, person_only))
        if args.trace is not None:
            trace.append({"t": t, "tcp": cpos, "target": list(tx), "err": err,
                          "solve_ms": solve_ms[-1],
                          "clear": (round(cell_world.min_sphere_clearance(sph, world), 4)
                                    if sph else None)})

    sampler.stop()
    wall = time.time() - t_run0

    warm = solve_ms  # already warmed up before the loop
    metrics = {
        "scenario": args.scenario,
        "curobo": "0.8.0 (Apache-2.0)",
        "robot": os.path.basename(args.robot),
        "steps": args.steps,
        "optimization_dt": dt,
        "interpolation_steps": args.interpolation_steps,
        "collision_activation_m": args.collision_activation,
        "control_rate_hz": round(1.0 / statistics.mean([s / 1e3 for s in warm]), 2),
        "solve_ms": {
            "mean": round(statistics.mean(warm), 3),
            "p50": round(statistics.median(warm), 3),
            "p95": round(sorted(warm)[int(0.95 * len(warm)) - 1], 3),
            "max": round(max(warm), 3),
        },
        "tracking_pos_err_m": {
            "mean": round(statistics.mean(track_err), 4),
            "final": round(track_err[-1], 4),
            "max": round(max(track_err), 4),
            "min": round(min(track_err), 4),
            # steady-state: second half, after the initial reach transient
            "steady_mean": round(statistics.mean(track_err[len(track_err) // 2:]), 4),
            "steady_max": round(max(track_err[len(track_err) // 2:]), 4),
        },
        "min_clearance_m": (round(min_clear, 4) if min_clear != float("inf") else None),
        "min_person_clearance_m": (round(min_person_clear, 4)
                                   if min_person_clear != float("inf") else None),
        "joint_window_violations": window_viol,
        "max_window_excess_rad": round(max_window_excess, 5),
        "gpu": {
            "baseline_used_mib": round(BASELINE_GPU_MIB, 1),
            "peak_used_mib_global": round(sampler.peak, 1),
            "est_process_mib": round(sampler.peak - BASELINE_GPU_MIB, 1),
            "torch_max_alloc_mib": round(torch.cuda.max_memory_allocated() / 2**20, 1),
        },
        "wall_s": round(wall, 2),
    }
    metrics["behaviour_note"] = _behaviour_note(args.scenario, metrics)
    print(json.dumps(metrics, indent=2))
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w") as fh:
            json.dump(metrics, fh, indent=2)
        print("wrote", args.out)
    if args.trace:
        with open(args.trace, "w") as fh:
            for r in trace:
                fh.write(json.dumps(r) + "\n")
        print("wrote", args.trace)


def _mk_pose(x, y, z):
    """Build a cuRobo Pose for an obstacle update, tolerant of API differences."""
    from curobo.types import Pose
    for maker in (
        lambda: Pose.from_list([x, y, z, 1.0, 0.0, 0.0, 0.0]),
        lambda: Pose(position=[x, y, z], quaternion=[1.0, 0.0, 0.0, 0.0]),
    ):
        try:
            return maker()
        except Exception:                                # noqa: BLE001
            continue
    from curobo.types import Pose as P
    return P.from_numpy([x, y, z], [1.0, 0.0, 0.0, 0.0])


def _behaviour_note(scenario, m):
    if scenario == "follow":
        return ("follows the sweeping target; report control rate + tracking error "
                "with the static shelf present.")
    if scenario == "person":
        return ("person capsule walks across; watch min_clearance stay >0 (MPC "
                "routes around) and tracking error rise while the person occludes.")
    if scenario == "unreachable":
        return ("target is 2.4 m out (outside reach + windows); expect tracking "
                "error to stay large, joint_window_violations == 0, arm stays bounded "
                "and does not thrash.")
    if scenario == "blocked":
        return ("person parked on the straight-line path; expect min_clearance to "
                "stay >0 (MPC does not drive through) and tracking error to remain "
                "elevated because the direct path is blocked.")
    return ""


if __name__ == "__main__":
    main()
