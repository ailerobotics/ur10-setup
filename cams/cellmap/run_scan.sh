#!/usr/bin/env bash
# Start the handheld cell-scan stack (scan.launch.py) inside the Isaac ROS
# image, on the ARM's DDS domain (0, UDPv4 — the cuMotion pattern), with the
# D435i passed through to the container.
#
# Why the camera driver lives in the container too (and not on the host like
# cams/rig.launch.py): cuVSLAM/nvblox want the exact topic+frame layout of
# NVIDIA's RGBD launch and the same librealsense build that Isaac was tested
# against; keeping driver and consumers in one process tree also removes one
# DDS hop of 640x360 colour+depth at 30 Hz. USB passthrough: the RSUSB
# librealsense build talks libusb directly, so mounting /dev/bus/usb plus a
# cgroup rule for char-major 189 (USB) is all it needs — no --privileged,
# no udev inside the container (the host node is already 0666/plugdev).
#
# Usage:
#   cams/cellmap/run_scan.sh                       # step 1: camera + cuVSLAM only
#   cams/cellmap/run_scan.sh segmenter:=true nvblox:=true   # step 3: full scan
#   cams/cellmap/run_scan.sh camera:=false ...     # driver started elsewhere
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
if [[ " $* " != *" camera:=false "* ]] && pgrep -af realsense2_camera_node | grep -v pgrep >/dev/null; then
  echo "run_scan.sh: a realsense2_camera_node is already holding the camera on the host:" >&2
  pgrep -af realsense2_camera_node | grep -v pgrep >&2
  echo "  (wrist_rig / rig.launch still up? the camera cannot be opened twice)" >&2
  exit 1
fi

exec docker run --rm --name "$NAME" \
  --net=host --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID=0 \
  -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rmw' \
  -v "$HERE:/cellmap:ro" \
  -v "$ROOT/cumotion:/cfg:ro" \
  -v "$MAPS:/maps" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec ros2 launch /cellmap/scan.launch.py "$@"' _ "$@"
