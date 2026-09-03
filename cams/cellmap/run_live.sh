#!/usr/bin/env bash
# LIVE collision avoidance: saved static map as base + live wrist-camera depth
# integrated on top, one ESDF feeding cuMotion (cams/cellmap/live.launch.py).
#
# Preconditions checked below: the wrist camera is streaming on the host with
# the CALIBRATED tool0->camera_link, and /joint_states is live (the segmenter
# and nvblox both need the arm's TF).
#
# Same container NAME as the default planner, so docker refuses to run both:
# they would both answer cumotion/move_group.
#   systemctl --user stop ur10-cumotion; docker stop ur10-cumotion
#
# Usage:  cams/cellmap/run_live.sh [<map.nvblx>] [extra launch args]
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"; ROOT="$(cd "$ROOT/.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
NAME="${CELLMAP_CONTAINER:-ur10-cumotion}"
HOST_MAPS="$HOME/ur10/cumotion/maps"
MAP="${1:-}"; [ $# -gt 0 ] && shift
if [ -z "$MAP" ]; then
  MAP="$(ls -t "$ROOT"/cumotion/maps/cell-*.nvblx "$HOST_MAPS"/cell-*.nvblx 2>/dev/null | head -1)"
  [ -n "$MAP" ] || { echo "run_live.sh: no cell-*.nvblx found" >&2; exit 2; }
fi
case "$MAP" in /*) ;; *) for d in "$ROOT/cumotion/maps" "$HOST_MAPS" "$PWD"; do
      [ -f "$d/$MAP" ] && { MAP="$d/$MAP"; break; }; done ;; esac
[ -f "$MAP" ] || { echo "run_live.sh: map not found: $MAP" >&2; exit 2; }

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "run_live.sh: container $NAME already running (docker stop $NAME first)" >&2; exit 1
fi
if ! pgrep -f realsense2_camera_node >/dev/null; then
  echo "run_live.sh: the wrist camera is NOT streaming — live avoidance would have no" >&2
  echo "  perception at all. Start it first (D435i pinned by serial):" >&2
  echo "  ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 launch \\" >&2
  echo "    $ROOT/cams/approach/wrist_rig.launch.py camera_params:=$ROOT/cams/cellmap/d435i_wrist_serial.yaml" >&2
  exit 1
fi
ls "$HERE"/calib/tool0_to_camera_*.yaml >/dev/null 2>&1 || {
  echo "run_live.sh: no calib/tool0_to_camera_*.yaml — live depth would land in the wrong" >&2
  echo "  place in base_link. Run handeye_calibrate.py first." >&2; exit 1; }

echo "run_live.sh: static base $MAP + live depth"
exec docker run --rm --name "$NAME" \
  --net=host --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e ROS_DOMAIN_ID=0 \
  -v "$HERE:/cellmap:ro" -v "$ROOT/cumotion:/cfg:ro" \
  -v "$(dirname "$MAP"):/mapdir" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec ros2 launch /cellmap/live.launch.py map:=/mapdir/'"$(basename "$MAP")"' "$@"' _ "$@"
