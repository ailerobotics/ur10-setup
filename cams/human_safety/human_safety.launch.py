#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Bring up the human_safety graph: one detector node per requested camera
(in the torch venv), the fusion+safety node, an optional viewer, and a static
transform publisher for every fixed camera that has been calibrated in
``params/camera_poses.yaml``.

This launches ONLY the perception graph; the RealSense cameras themselves come
up via ``cams/rig_multicam.launch.py`` (see run_live.sh, which sets domain 77 +
UDPv4 and starts both). It never starts, stops or touches the robot.

  ros2 launch cams/human_safety/human_safety.launch.py \
      cams:=wrist,shelf,table viz:=true

Args:
  cams         comma list of roles (default wrist,shelf,table)
  config       path to human_safety.yaml
  poses        path to camera_poses.yaml (static extrinsics for fixed cams)
  viz          true/false — open the owner window (default true)
  venv_python  interpreter with torch for the detector nodes
  device       cuda|cpu override for the detectors
"""
import pathlib
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

HERE = pathlib.Path(__file__).resolve().parent
NODES = HERE / "nodes"
DEFAULT_VENV = str(pathlib.Path.home() / "venvs" / "human-safety" / "bin" / "python")


def _static_tf_procs(poses_path):
    """Publish a static base<-optical TF for every fixed camera whose calibration
    is VALID. The validity guard (calibrated, serial match, freshness, and a
    non-failed recheck) lives in human_safety.camera_poses so the launch and the
    offline tests share one code path; a rejected camera gets no TF and stays
    presence-only, and we log why."""
    sys.path.insert(0, str(HERE))
    from human_safety.camera_poses import load_with_reasons  # noqa: E402
    good, rejected = load_with_reasons(poses_path)
    for role, reason in rejected:
        print(f"[human_safety] camera '{role}' PRESENCE-ONLY: {reason}")
    procs = []
    for tf in good:
        print(f"[human_safety] camera '{tf.role}' ({tf.serial}) CALIBRATED: "
              f"publishing {tf.parent} -> {tf.child} from {tf.source}")
        t, q = tf.translation, tf.rotation
        procs.append(ExecuteProcess(
            cmd=["ros2", "run", "tf2_ros", "static_transform_publisher",
                 "--x", str(t[0]), "--y", str(t[1]), "--z", str(t[2]),
                 "--qx", str(q[0]), "--qy", str(q[1]), "--qz", str(q[2]),
                 "--qw", str(q[3]),
                 "--frame-id", tf.parent, "--child-frame-id", tf.child],
            output="screen", name=f"static_tf_{tf.role}"))
    return procs


def _setup(context, *args, **kwargs):
    cams = [c.strip() for c in
            LaunchConfiguration("cams").perform(context).split(",") if c.strip()]
    config = LaunchConfiguration("config").perform(context)
    poses = LaunchConfiguration("poses").perform(context)
    viz = LaunchConfiguration("viz").perform(context).lower() == "true"
    venv_python = LaunchConfiguration("venv_python").perform(context)
    device = LaunchConfiguration("device").perform(context)
    record = LaunchConfiguration("record").perform(context)

    procs = list(_static_tf_procs(poses))

    for role in cams:
        cmd = [venv_python, str(NODES / "detector_node.py"),
               "--config", config, "--role", role]
        if device:
            cmd += ["--device", device]
        procs.append(ExecuteProcess(cmd=cmd, output="screen",
                                    name=f"detector_{role}"))

    fusion_cmd = [sys.executable, str(NODES / "fusion_node.py"),
                  "--config", config, "--cams", ",".join(cams)]
    if record:
        fusion_cmd += ["--record", record]
    procs.append(ExecuteProcess(cmd=fusion_cmd, output="screen", name="fusion"))

    if viz:
        procs.append(ExecuteProcess(
            cmd=[sys.executable, str(NODES / "viz_node.py"), "--config", config],
            output="screen", name="viz"))
    return procs


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cams", default_value="wrist,shelf,table"),
        DeclareLaunchArgument("config", default_value=str(
            HERE / "params" / "human_safety.yaml")),
        DeclareLaunchArgument("poses", default_value=str(
            HERE / "params" / "camera_poses.yaml")),
        DeclareLaunchArgument("viz", default_value="true"),
        DeclareLaunchArgument("venv_python", default_value=DEFAULT_VENV),
        DeclareLaunchArgument("device", default_value=""),
        DeclareLaunchArgument("record", default_value="",
                              description="JSONL replay-log path for the fusion "
                                          "node (empty = no recording)"),
        OpaqueFunction(function=_setup),
    ])
