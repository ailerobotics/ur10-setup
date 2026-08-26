#!/usr/bin/env bash
# RViz with the nvblox plugin — it only exists inside the Isaac image, and the
# host RViz cannot decode nvblox_msgs/Mesh. Runs rviz2 in a throwaway
# container on the workstation display (X socket + the session's xauth
# cookie; xhost is locked to the local user, the cookie is what lets root in
# the container through). NVIDIA GL via --gpus; falls back to llvmpipe if the
# driver capabilities are missing (slow but works).
#
#   cams/cellmap/run_rviz.sh                 # cellmap.rviz: robot + mesh + SLAM path + voxels
#   cams/cellmap/run_rviz.sh other.rviz
# Read-only viewer; safe alongside anything. Close the window to stop.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
CFG="${1:-/cellmap/cellmap.rviz}"
case "$CFG" in /cellmap/*) ;; /*) ;; *) CFG="/cellmap/$CFG" ;; esac
DISPLAY="${DISPLAY:-:0}"
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
if [ ! -f "$XAUTH" ]; then
  # session cookie of the running X session (rviz2 units use it too)
  XAUTH="$(ls -t /tmp/xauth_* 2>/dev/null | head -1)"
fi
[ -f "$XAUTH" ] || { echo "run_rviz.sh: no xauth cookie found (DISPLAY=$DISPLAY)" >&2; exit 1; }

exec docker run --rm --name "ur10-cellmap-rviz-$$" \
  --net=host --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID=0 \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/xauth \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$XAUTH:/tmp/xauth:ro" \
  -v "$HERE:/cellmap:ro" \
  -v "$ROOT/cumotion:/cfg:ro" \
  "$IMAGE" \
  bash -lc "source /opt/ros/jazzy/setup.bash && exec rviz2 -d '$CFG'"
