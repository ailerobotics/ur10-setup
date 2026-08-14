#!/usr/bin/env bash
# Launch the UR ROS 2 driver against the robot.
# headless_mode:=true sends the external-control script to the robot directly,
# so no URCap is needed for first light. launch_rviz:=false (tty session, no display).
set -eo pipefail  # no -u: ROS setup.bash reads unset vars
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT_IP="${1:-192.168.0.1}"
UR_TYPE="$(cat "$HOME/ur10/ur_type.txt" 2>/dev/null || true)"
[ -n "$UR_TYPE" ] || { echo "ur_type.txt missing - run ur-detect.sh first"; exit 1; }

source /opt/ros/jazzy/setup.bash
exec ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:="$UR_TYPE" \
  robot_ip:="$ROBOT_IP" \
  reverse_ip:=192.168.0.2 \
  kinematics_params_file:="$HOME/ur10/ur10_calibration.yaml" \
  headless_mode:=true \
  launch_rviz:=false
