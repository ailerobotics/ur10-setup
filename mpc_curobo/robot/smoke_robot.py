#!/usr/bin/env python3
"""Smoke test: can cuRobo load THIS cell's UR10 and does MPC init + solve?

Run inside the container:
    docker run --rm --gpus all -v <repo>:/repo:ro fm-plan-mpc-curobo-v1:latest \
        python3 /repo/mpc_curobo/robot/smoke_robot.py
"""
import time

import torch

ROBOT = "/repo/mpc_curobo/robot/ur10_cell_curobo.yml"


def main():
    print("torch", torch.__version__, "cuda avail", torch.cuda.is_available())

    # --- 1. kinematics load + FK + joint limits ---------------------------- #
    from curobo.kinematics import Kinematics, KinematicsCfg
    from curobo.types import JointState
    kin = Kinematics(KinematicsCfg.from_robot_yaml_file(ROBOT))
    print("dof", kin.get_dof(), "joints", kin.joint_names, "tools", kin.tool_frames)
    lim = kin.get_joint_limits()
    # print the position limits cuRobo will enforce (these must sit within the
    # measured windows; position_limit_clip=0.025 pulls them a hair inside)
    try:
        pos = lim.position
        print("joint position limits (lower):", pos[0].cpu().tolist())
        print("joint position limits (upper):", pos[1].cpu().tolist())
    except Exception as e:                               # noqa: BLE001
        print("limits repr:", lim, "err", e)

    ref = [0.223385, -1.626713, 0.406275, 4.458966, 0.279341, 0.706048]
    q = torch.tensor([ref], device="cuda", dtype=torch.float32)
    st = kin.compute_kinematics(JointState.from_position(q, joint_names=kin.joint_names))
    ee = st.tool_poses.get_link_pose(kin.tool_frames[0])
    print("TCP @ reference pose: pos", ee.position.squeeze().cpu().tolist(),
          "quat", ee.quaternion.squeeze().cpu().tolist())

    # --- 2. MPC init + one solve ------------------------------------------ #
    from curobo.model_predictive_control import (
        ModelPredictiveControl, ModelPredictiveControlCfg)
    from curobo.types import GoalToolPose
    scene = {"cuboid": {
        "ground": {"dims": [3.0, 3.0, 0.1], "pose": [0.0, 0.0, -0.05, 1, 0, 0, 0]},
    }}
    t0 = time.time()
    cfg = ModelPredictiveControlCfg.create(
        robot=ROBOT, scene_model=scene, use_cuda_graph=True,
        optimization_dt=0.03, interpolation_steps=4)
    mpc = ModelPredictiveControl(cfg)
    print(f"MPC init {time.time()-t0:.2f}s; joints {mpc.joint_names}")

    start = JointState.from_position(
        mpc.default_joint_position.clone().unsqueeze(0), joint_names=mpc.joint_names)
    start.velocity = torch.zeros_like(start.position)
    start.acceleration = torch.zeros_like(start.position)
    mpc.setup(start)

    kin_r = mpc.compute_kinematics(start)
    poses = kin_r.tool_poses.to_dict()
    link = mpc.tool_frames[0]
    poses[link].position[..., 2] += 0.15  # goal 15 cm up
    mpc.update_goal_tool_poses(
        GoalToolPose.from_poses(poses, ordered_tool_frames=mpc.tool_frames, num_goalset=1),
        run_ik=True)

    # warm-up + timed steps
    state = start
    for _ in range(5):
        state = _advance(mpc, state, JointState)
    torch.cuda.synchronize()
    ts = []
    for _ in range(30):
        t = time.time()
        state = _advance(mpc, state, JointState)
        torch.cuda.synchronize()
        ts.append(time.time() - t)
    import statistics
    print(f"warm MPC solve: mean {1e3*statistics.mean(ts):.2f} ms  "
          f"p95 {1e3*sorted(ts)[int(0.95*len(ts))]:.2f} ms  "
          f"=> {1.0/statistics.mean(ts):.1f} Hz")
    print("peak GPU alloc MiB", torch.cuda.max_memory_allocated() / 2**20)
    print("SMOKE OK")


def _advance(mpc, state, JointState):
    res = mpc.optimize_action_sequence(state)
    if res.action_sequence is None or res.action_sequence.position.shape[1] == 0:
        return state
    nxt = res.action_sequence.position[:, -1, :]
    ns = JointState.from_position(nxt.clone(), joint_names=mpc.joint_names)
    ns.velocity = res.action_sequence.velocity[:, -1, :]
    ns.acceleration = res.action_sequence.acceleration[:, -1, :]
    return ns


if __name__ == "__main__":
    main()
