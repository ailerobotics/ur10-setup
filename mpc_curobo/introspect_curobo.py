#!/usr/bin/env python3
"""Print the exact cuRobo V2 API surface we build the prototype against.

Run inside the fm-plan-mpc-curobo-v1 container once it is built:

    docker run --rm --gpus all -v <repo>/mpc_curobo:/work fm-plan-mpc-curobo-v1:latest \
        python3 /work/introspect_curobo.py

It dumps: the installed cuRobo version, whether a UR10 robot config ships, the
MPC/MotionPlanner config signatures, the scene/world model format, and the robot
config YAML schema — so make_curobo_robot.py and cell_world.py target the real
schema, not a guess.
"""
import inspect
import sys


def hdr(t):
    print(f"\n=== {t} ===")


def main():
    out = {}
    import curobo

    hdr("version")
    ver = getattr(curobo, "__version__", "unknown")
    print("curobo", ver)
    out["version"] = str(ver)

    import torch
    print("torch", torch.__version__, "cuda", torch.version.cuda,
          "available", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("device", torch.cuda.get_device_name(0))

    from curobo.content import get_assets_path, get_configs_path  # type: ignore
    hdr("asset + config paths")
    ap = get_assets_path()
    cp = get_configs_path()
    print("assets", ap)
    print("configs", cp)

    import os
    hdr("robot configs shipped")
    robot_cfg_dir = os.path.join(str(cp), "robot")
    urs = []
    if os.path.isdir(robot_cfg_dir):
        for f in sorted(os.listdir(robot_cfg_dir)):
            if "ur" in f.lower():
                urs.append(f)
        print("UR-ish configs:", urs)
        print("(all):", sorted(os.listdir(robot_cfg_dir))[:40])
    out["ur_configs"] = urs

    hdr("scene/world configs shipped")
    for sub in ("scene", "world"):
        d = os.path.join(str(cp), sub)
        if os.path.isdir(d):
            print(sub, "->", sorted(os.listdir(d))[:40])

    hdr("MPCSolverCfg.create signature")
    from curobo.model_predictive_control import (
        ModelPredictiveControl,
        ModelPredictiveControlCfg,
    )
    try:
        print(inspect.signature(ModelPredictiveControlCfg.create))
    except Exception as e:
        print("create sig err:", e)
    print("MPC methods:", [m for m in dir(ModelPredictiveControl)
                           if not m.startswith("_")])

    hdr("MotionPlannerCfg.create signature (C1 comparison)")
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    try:
        print(inspect.signature(MotionPlannerCfg.create))
    except Exception as e:
        print("create sig err:", e)
    print("MotionPlanner methods:", [m for m in dir(MotionPlanner)
                                     if not m.startswith("_")])

    hdr("WorldConfig / obstacle types")
    try:
        from curobo.scene import WorldConfig  # type: ignore
        print("WorldConfig:", inspect.signature(WorldConfig.__init__))
    except Exception as e:
        print("WorldConfig import path differs:", e)
    # dump a bundled scene yml so we copy its exact schema
    for sub in ("scene", "world"):
        d = os.path.join(str(cp), sub)
        if os.path.isdir(d):
            for f in os.listdir(d):
                if f.endswith((".yml", ".yaml")):
                    p = os.path.join(d, f)
                    hdr(f"sample scene yml: {sub}/{f}")
                    with open(p) as fh:
                        print(fh.read()[:2500])
                    break
            break

    hdr("sample UR robot yml (schema)")
    if urs:
        p = os.path.join(robot_cfg_dir, urs[0])
        with open(p) as fh:
            print(fh.read()[:4000])

    print("\n=== DONE ===")


if __name__ == "__main__":
    sys.exit(main())
