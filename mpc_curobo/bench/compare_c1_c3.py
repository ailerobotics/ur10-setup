#!/usr/bin/env python3
"""C1 (segmented re-plan) side of the option-1 vs option-2 comparison (SIM/EVAL).

Option (1) in the human-dynamic-plan-s1 report is "segmented re-plan + goal
preemption": plan a short leg to the freshest target with the cell's planner
(NVIDIA cuMotion), execute, preempt with a new plan when the target drifts. The
cell's cuMotion IS cuRobo's trajectory optimizer under the hood, so cuRobo's
``MotionPlanner.plan_pose`` on THIS host is a faithful, same-GPU stand-in for the
cell planner's *planning* cost (the report measured the cell's cuMotion at
~0.10-0.14 s solve / 0.25-0.48 s wall; this reproduces that number here).

This measures, for the SAME sweeping target as the MPC prototype's `follow`
scenario, the C1 re-target loop: replan a leg to the freshest target, "execute" a
short segment, replan. Records per-replan planning latency, success rate,
trajectory smoothness (jerk) and min collision clearance, and peak GPU. The MPC
(C3) side is mpc_curobo/mpc_prototype.py --scenario follow; the two JSONs are
merged into the comparison table by build_comparison.py.
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


def _gpu_used_mib():
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5)
        return float(out.stdout.strip().splitlines()[0])
    except Exception:                                    # noqa: BLE001
        return float("nan")


BASELINE_GPU_MIB = _gpu_used_mib()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from world import cell_world  # noqa: E402

import torch  # noqa: E402
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg  # noqa: E402
from curobo.types import GoalToolPose, JointState  # noqa: E402

ROBOT_DEFAULT = "/repo/mpc_curobo/robot/ur10_cell_curobo.yml"


class GpuSampler(threading.Thread):
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


def make_target_at(center):
    cx, cy, cz = center

    def target_at(t):
        return (cx, cy + 0.25 * math.sin(2 * math.pi * t / 8.0),
                cz + 0.08 * math.sin(2 * math.pi * t / 5.0))
    return target_at


def robot_spheres(planner, js):
    res = planner.compute_kinematics(js)
    for attr in ("robot_spheres", "link_spheres_tensor"):
        tt = getattr(res, attr, None)
        if tt is not None:
            arr = tt.detach().reshape(-1, 4).cpu().tolist()
            return [(a[0], a[1], a[2], a[3]) for a in arr if a[3] > 0]
    return []


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", default=ROBOT_DEFAULT)
    ap.add_argument("--replans", type=int, default=16, help="number of re-targets")
    ap.add_argument("--segment-s", type=float, default=0.5,
                    help="sim time 'executed' before preempting with a new plan")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    world = cell_world.static_world()
    sampler = GpuSampler(); sampler.start()

    cfg = MotionPlannerCfg.create(robot=args.robot, scene_model=world,
                                  num_ik_seeds=48, num_trajopt_seeds=12)
    planner = MotionPlanner(cfg)
    planner.warmup(enable_graph=True, num_warmup_iterations=5)

    q = JointState.from_position(
        planner.default_joint_state.position.unsqueeze(0), joint_names=planner.joint_names)
    link = planner.tool_frames[0]
    # reachable pick pose (position + orientation) from the in-window anchor, same
    # as the MPC prototype, so the two options track an identical, feasible target.
    anchor = JointState.from_position(
        torch.tensor([cell_world.PICK_ANCHOR], device="cuda", dtype=torch.float32),
        joint_names=planner.joint_names)
    anchor_pose = planner.compute_kinematics(anchor).tool_poses.to_dict()[link]
    center = anchor_pose.position.squeeze().detach().cpu().tolist()
    quat = anchor_pose.quaternion.squeeze().detach().cpu().tolist()
    target_at = make_target_at(center)

    plan_ms, successes, jerks, clears = [], 0, [], []
    interp_dt = planner.trajopt_solver.config.interpolation_dt \
        if hasattr(planner, "trajopt_solver") else 0.02

    t_sim = 0.0
    for i in range(args.replans):
        tx = target_at(t_sim)
        goal = GoalToolPose(
            tool_frames=planner.tool_frames,
            position=torch.tensor([[[[tx]]]], device="cuda", dtype=torch.float32),
            quaternion=torch.tensor([[[[quat]]]], device="cuda", dtype=torch.float32),
        )
        torch.cuda.synchronize()
        tic = time.time()
        result = planner.plan_pose(goal, q, max_attempts=6)
        torch.cuda.synchronize()
        plan_ms.append(1e3 * (time.time() - tic))

        ok = result is not None and bool(result.success.any())
        if ok:
            successes += 1
            interp = result.get_interpolated_plan()
            # trajectory smoothness: peak |jerk| if present, else peak |accel|
            for attr in ("jerk", "acceleration"):
                v = getattr(interp, attr, None)
                if v is not None:
                    jerks.append(float(v.abs().max().detach().cpu()))
                    break
            # min clearance sampled along the interpolated plan
            pos = interp.position.reshape(-1, interp.position.shape[-1])
            n = pos.shape[0]
            worst = float("inf")
            for k in range(0, n, max(1, n // 20)):
                js = JointState.from_position(pos[k].unsqueeze(0),
                                              joint_names=planner.joint_names)
                sph = robot_spheres(planner, js)
                if sph:
                    worst = min(worst, cell_world.min_sphere_clearance(sph, world))
            if worst != float("inf"):
                clears.append(worst)
            # "execute" a short segment: advance q to the waypoint at segment-s
            step = min(int(args.segment_s / interp_dt), n - 1)
            q = JointState.from_position(pos[step].unsqueeze(0),
                                         joint_names=planner.joint_names)
        t_sim += args.segment_s

    sampler.stop()
    metrics = {
        "option": "C1 segmented re-plan (cuRobo MotionPlanner == cell cuMotion trajopt)",
        "curobo": "0.8.0 (Apache-2.0)",
        "replans": args.replans,
        "segment_s": args.segment_s,
        "success_rate": round(successes / args.replans, 3),
        "replan_latency_ms": {
            "mean": round(statistics.mean(plan_ms), 2),
            "p50": round(statistics.median(plan_ms), 2),
            "p95": round(sorted(plan_ms)[int(0.95 * len(plan_ms)) - 1], 2),
            "max": round(max(plan_ms), 2),
        },
        "traj_peak_jerk_or_accel": (round(max(jerks), 2) if jerks else None),
        "min_clearance_m": (round(min(clears), 4) if clears else None),
        "gpu": {
            "baseline_used_mib": round(BASELINE_GPU_MIB, 1),
            "peak_used_mib_global": round(sampler.peak, 1),
            "est_process_mib": round(sampler.peak - BASELINE_GPU_MIB, 1),
        },
    }
    print(json.dumps(metrics, indent=2))
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w") as fh:
            json.dump(metrics, fh, indent=2)
        print("wrote", args.out)


if __name__ == "__main__":
    main()
