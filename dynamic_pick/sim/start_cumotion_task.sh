#!/usr/bin/env bash
# The task's OWN cuMotion planner container, on the ISOLATED task domain 83.
#
# This is a sibling of cumotion/start_cumotion_map.sh, but deliberately DIFFERENT
# where the task rules require it:
#   * ROS_DOMAIN_ID=83 (NOT 0) — never joins the live cell's DDS graph;
#   * a task-prefixed container name (NOT ur10-cumotion) so it can never be
#     confused with, or collide with, the cell's own planner;
#   * the cell's own start_cumotion*.sh is NEVER invoked (it bakes domain 0).
# It reuses the committed cams/cellmap/planner.launch.py (nvblox serves the saved
# map -> cuMotion read_esdf_world=true) READ-ONLY, so a leg is collision-aware
# through the nvblox ESDF (report §1). The isaac planner image is used read-only as
# a base. Uses --gpus all under the fleet GPU lock (the caller holds it).
#
# Usage:
#   dynamic_pick/sim/start_cumotion_task.sh [map.nvblx] [extra planner args...]
# with no map it picks the newest cell-*.nvblx or a cell-sim-*.nvblx (sim maps are
# fine — this is a sim-only proof). Stop it with:  docker rm -f <printed name>
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
CFG="$REPO/cumotion"
HOST_MAPS="$HOME/ur10/cumotion/maps"
CONT="fm-human-dynamic-pick-sim-v1-cumotion"

MAP="$1"; [ $# -gt 0 ] && shift
if [ -z "$MAP" ]; then
  MAP="$(ls -t "$HOST_MAPS"/cell-*.nvblx 2>/dev/null | head -1)"
  [ -n "$MAP" ] || MAP="$(ls -t "$HOST_MAPS"/cell-sim*.nvblx 2>/dev/null | head -1)"
  [ -n "$MAP" ] || { echo "start_cumotion_task.sh: no cell-*.nvblx in $HOST_MAPS" >&2; exit 2; }
fi
case "$MAP" in
  /*) ;;
  *) for d in "$HOST_MAPS" "$CFG/maps" "$PWD"; do [ -f "$d/$MAP" ] && { MAP="$d/$MAP"; break; }; done ;;
esac
[ -f "$MAP" ] || { echo "start_cumotion_task.sh: map not found: $MAP" >&2; exit 2; }
MAP="$(readlink -f "$MAP")"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONT"; then
  echo "start_cumotion_task.sh: removing a stale $CONT"; docker rm -f "$CONT" >/dev/null 2>&1
fi
echo "start_cumotion_task.sh: name=$CONT domain=83 map=$MAP ($(du -h "$MAP" | cut -f1))"

# --net=host is how cuMotion's move_group reaches its host-side clients (same as the
# cell's planner); DDS isolation is by ROS_DOMAIN_ID=83, so it never joins domain 0.
exec docker run --rm --name "$CONT" \
  --net=host --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID=83 \
  -v "$CFG:/cfg:ro" \
  -v "$REPO/cams/cellmap:/cellmap:ro" \
  -v "$(dirname "$MAP"):/mapdir:ro" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec ros2 launch /cellmap/planner.launch.py "$@"' _ \
  "map:=/mapdir/$(basename "$MAP")" "voxels:=false" "$@"
