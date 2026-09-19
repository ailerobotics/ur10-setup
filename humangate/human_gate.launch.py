"""
Launch the robot-side human-safety gate (PLAN-ONLY, report §10 Phase 1).

The gate is a loose Python script (like cams/cellmap/tools/*), so it is launched
with ExecuteProcess exactly as live.launch.py launches map_ctl.py — no colcon
package/build is required for this phase.

Isolation: the whole feature runs on ROS_DOMAIN_ID=78 with
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (this task's brief). The env is applied to the
launched process here so a caller cannot forget it. Override with domain:=<n>.

  ros2 launch humangate/human_gate.launch.py \
      decision_log:=/tmp/human_gate/decisions.jsonl

NOTE: this phase consumes the camera-side state as a std_msgs/String JSON payload
(state_adapter.py) because the perception custom message is not committed yet.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(ctx, *args, **kwargs):
    lc = lambda k: LaunchConfiguration(k).perform(ctx)  # noqa: E731
    here = os.path.dirname(os.path.abspath(__file__))
    node = os.path.join(here, "gate_node.py")

    ros_args = ["--ros-args",
                "-p", f"params_file:={lc('params_file') or os.path.join(here, 'gate_params.yaml')}",
                "-p", f"state_topic:={lc('state_topic')}",
                "-p", f"decision_topic:={lc('decision_topic')}"]
    if lc("decision_log"):
        ros_args += ["-p", f"decision_log:={lc('decision_log')}"]
    if lc("command_log"):
        ros_args += ["-p", f"command_log:={lc('command_log')}"]

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = lc("domain")
    env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

    return [ExecuteProcess(
        cmd=["python3", node] + ros_args,
        name="human_gate", output="screen", additional_env=env,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("domain", default_value="78"),
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("state_topic", default_value="/human_safety/state"),
        DeclareLaunchArgument("decision_topic", default_value="/human_gate/decision"),
        DeclareLaunchArgument("decision_log", default_value=""),
        DeclareLaunchArgument("command_log", default_value=""),
        OpaqueFunction(function=_setup),
    ])
