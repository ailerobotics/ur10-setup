#!/usr/bin/env bash
# Start the cell-scan stack (scan.launch.py: robot segmenter + nvblox) inside
# the Isaac ROS image, on the ARM's DDS domain (0, UDPv4 — the cuMotion
# pattern). The wrist camera itself runs on the HOST
# (cams/approach/wrist_rig.launch.py, with the calibrated tool0->camera_link);
# this container only consumes its topics and the robot's TF.
#
# Preconditions it checks: a wrist_rig camera is streaming, the calibrated
# transform is what wrist_rig publishes (calib yaml exists), /joint_states is
# live (the segmenter and nvblox both need the arm's TF).
#
# Usage:
#   cams/cellmap/run_scan.sh                       # segmenter + nvblox
#   cams/cellmap/run_scan.sh segmenter:=false      # raw depth straight in (debug only)
# Any extra args are passed straight to `ros2 launch scan.launch.py`.
# Stop: Ctrl-C (or `docker stop ur10-cellmap-scan`). One instance at a time.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
NAME="${CELLMAP_CONTAINER:-ur10-cellmap-scan}"
MAPS="${CELLMAP_MAPS:-$HOME/ur10/cumotion/maps}"
mkdir -p "$MAPS"

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "run_scan.sh: container $NAME already running (docker stop $NAME first)" >&2
  exit 1
fi
if ! pgrep -f realsense2_camera_node >/dev/null; then
  echo "run_scan.sh: no realsense2_camera_node on the host — start the wrist camera first:" >&2
  echo "  ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 launch $ROOT/cams/approach/wrist_rig.launch.py" >&2
  exit 1
fi
if [[ " $* " != *" nocalib "* ]] && ! ls "$HERE"/calib/tool0_to_camera_*.yaml >/dev/null 2>&1; then
  echo "run_scan.sh: no cams/cellmap/calib/tool0_to_camera_*.yaml — run handeye_calibrate.py first" >&2
  echo "  (a map built on the old mount GUESS is off by several cm; pass 'nocalib' to override knowingly)" >&2
  exit 1
fi

ARGS=(); for a in "$@"; do [ "$a" = nocalib ] || ARGS+=("$a"); done
exec docker run --rm --name "$NAME" \
  --net=host --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID=0 \
  -v "$HERE:/cellmap:ro" \
  -v "$ROOT/cumotion:/cfg:ro" \
  -v "$MAPS:/maps" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec ros2 launch /cellmap/scan.launch.py "$@"' _ "${ARGS[@]}"
