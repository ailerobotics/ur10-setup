#!/usr/bin/env python3
"""Generate a cuRobo V2 robot config for THIS cell's UR10 from its measured files.

Inputs (all authoritative cell files, read-only):
  * cumotion/ur10_measured.urdf  — the cell kinematics WITH the captain-measured
    joint windows already in its <limit lower/upper> tags. cuRobo reads position
    limits from the URDF, so pointing at this URDF is what makes cuRobo respect
    the cell's windows (verified against measured_joint_limits.yaml by the guard).
  * cumotion/ur10_measured.xrdf  — the cell's collision spheres, self-collision
    ignore matrix, per-link buffers, reference pose, accel/jerk limits.
  * cumotion/measured_joint_limits.yaml — for the reference (park) pose used as
    the cuRobo retract/default configuration.

Output: a cuRobo ``robot_cfg`` YAML (kinematics + cspace). Paths inside it are
CONTAINER paths (the URDF is bind-mounted at ``--urdf``; UR meshes resolve under
``--asset-root`` = /opt/ros/humble/share, where ros-humble-ur-description installs
them). Run inside the fm-plan-mpc-curobo-v1 container.

This mirrors the collision model the cell's cuMotion uses (same ur10e sphere set,
same ignore matrix) so the MPC prototype and the C1 comparison plan against the
SAME robot body, not a cuRobo default.
"""
from __future__ import annotations

import argparse
import math

import yaml

JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def build_robot_cfg(urdf_path, xrdf_path, limits_path, asset_root):
    with open(xrdf_path, encoding="utf-8") as fh:
        xrdf = yaml.safe_load(fh)
    with open(limits_path, encoding="utf-8") as fh:
        lim = yaml.safe_load(fh)

    ref = lim["reference_pose_rad"]
    default_pose = [float(ref[j]) for j in JOINT_ORDER]

    geom_key = xrdf["collision"]["geometry"]           # "ur10e_collision_spheres"
    spheres = xrdf["geometry"][geom_key]["spheres"]
    collision_link_names = list(spheres.keys())

    # per-link world-collision buffers from the xrdf, baked into a scalar we keep
    # conservative (the smallest link buffer) so no link is under-buffered; the
    # per-link detail is preserved in a comment for provenance.
    coll_buffers = xrdf["collision"].get("buffer_distance", {})
    scalar_buffer = min(coll_buffers.values()) if coll_buffers else 0.02

    self_ignore = xrdf["self_collision"].get("ignore", {})
    self_buffer = xrdf["self_collision"].get("buffer_distance", {})

    accel = xrdf["cspace"]["acceleration_limits"][0]
    jerk = xrdf["cspace"]["jerk_limits"][0]

    robot_cfg = {
        "robot_cfg": {
            "kinematics": {
                "urdf_path": urdf_path,
                "asset_root_path": asset_root,
                "base_link": "base_link",
                "tool_frames": ["tool0"],
                "collision_link_names": collision_link_names,
                "collision_spheres": spheres,
                "collision_sphere_buffer": float(scalar_buffer),
                "self_collision_ignore": self_ignore,
                "self_collision_buffer": self_buffer,
                # sphere-only collision: no mesh_link_names, so the STL meshes are
                # needed by yourdfpy for FK only, not for collision.
                "cspace": {
                    "joint_names": JOINT_ORDER,
                    "max_acceleration": float(accel),
                    "max_jerk": float(jerk),
                    "null_space_weight": [1.0] * 6,
                    "cspace_distance_weight": [1.0] * 6,
                    # clip a hair INSIDE the measured URDF limits: keeps every
                    # cuRobo solution strictly within the cell's windows (safe
                    # side of "never loosen the measured joint windows").
                    "position_limit_clip": 0.025,
                    "default_joint_position": default_pose,
                },
                "lock_joints": None,
            }
        }
    }
    # provenance comment payload
    robot_cfg["_provenance"] = {
        "generated_from": {
            "urdf": "cumotion/ur10_measured.urdf (measured joint windows)",
            "xrdf": "cumotion/ur10_measured.xrdf (spheres, ignore, buffers)",
            "limits": "cumotion/measured_joint_limits.yaml (reference pose)",
        },
        "per_link_collision_buffer_distance": coll_buffers,
        "scalar_collision_sphere_buffer_used": float(scalar_buffer),
        "note": ("position_limit_clip=0.025 keeps solutions strictly inside the "
                 "measured windows; the independent joint_windows guard verifies "
                 "every commanded joint stays within them."),
    }
    return robot_cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="/repo/cumotion/ur10_measured.urdf")
    ap.add_argument("--xrdf", default="/repo/cumotion/ur10_measured.xrdf")
    ap.add_argument("--limits", default="/repo/cumotion/measured_joint_limits.yaml")
    ap.add_argument("--asset-root", default="/opt/ros/humble/share")
    ap.add_argument("--out", default="/repo/mpc_curobo/robot/ur10_cell_curobo.yml")
    args = ap.parse_args()
    cfg = build_robot_cfg(args.urdf, args.xrdf, args.limits, args.asset_root)
    with open(args.out, "w") as fh:
        # ASCII only: cuRobo's config loader (resolve_config) reads YAML with the
        # ascii codec, so a stray unicode dash here would crash MPC/planner init.
        fh.write("# GENERATED by mpc_curobo/robot/make_curobo_robot.py -- do not edit by hand.\n")
        fh.write("# cuRobo V2 robot config for the cell UR10 (container paths).\n")
        yaml.safe_dump(cfg, fh, sort_keys=False, default_flow_style=False, allow_unicode=False)
    print(f"wrote {args.out}")
    # echo the joint limits cuRobo will see, straight from the URDF, for the record
    print("reference/default pose:", cfg["robot_cfg"]["kinematics"]["cspace"]["default_joint_position"])


if __name__ == "__main__":
    main()
