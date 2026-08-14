#!/usr/bin/env bash
# Run NVIDIA cuMotion (cuRobo) planner for the UR10 inside the Isaac ROS container.
#
# Why containerized: the host has no CUDA toolkit / NVIDIA CUDA apt repo, so the
# ros-jazzy-isaac-ros-cumotion deb is uninstallable natively — but the shelfbot
# Isaac image already ships it. --net=host joins the host DDS graph (domain 0),
# so move_group's cuMotion plugin (built on host) reaches the action server
# "cumotion/move_group" transparently. FASTDDS_BUILTIN_TRANSPORTS=UDPv4 avoids
# the container<->host shared-memory transport mismatch (same trick as the HIL
# bridge).
#
# read_esdf_world:=false  — no nvblox running; without this the planner waits on
#                           the ESDF service forever.
# add_ground_plane:=true  — free safety: cuMotion refuses plans that would sweep
#                           below the robot's mounting plane (MoveIt's own scene
#                           is empty, so this is the only "table" any planner
#                           knows about).
set -o pipefail
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"

exec docker run --rm --name ur10-cumotion \
  --net=host --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID=0 \
  -v "$HOME/ur10/cumotion:/cfg:ro" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
    cumotion_action_server.urdf_file_path:=/cfg/ur10_calibrated.urdf \
    cumotion_action_server.xrdf_file_path:=/cfg/ur10.xrdf \
    cumotion_action_server.read_esdf_world:=false \
    cumotion_action_server.update_esdf_on_request:=false \
    cumotion_action_server.add_ground_plane:=true \
    cumotion_action_server.time_dilation_factor:=0.5'
