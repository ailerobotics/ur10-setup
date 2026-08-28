#!/usr/bin/env bash
# cuMotion planner WITH the handheld cell map (opt-in sibling of
# start_cumotion.sh, whose default behaviour is untouched).
#
# Same container recipe as start_cumotion.sh (image, --net=host, --gpus,
# UDPv4 DDS, domain 0) and the SAME container name, so docker itself refuses
# to run both planners at once — they would both answer action
# cumotion/move_group and MoveIt would talk to whichever won the race.
# Stop the default one first:  systemctl --user stop ur10-cumotion
#
# Inside: cams/cellmap/planner.launch.py = nvblox (serves the saved .nvblx,
# no camera) -> load_map -> cuMotion read_esdf_world:=true with
# ur10_map.xrdf (0.04 m buffer). cuMotion only starts once the load reports
# success, so this can never block on the ESDF service with no map.
#
# Usage:
#   cumotion/start_cumotion_map.sh                      # newest cell-*.nvblx
#   cumotion/start_cumotion_map.sh cell-2026-08-26.nvblx [voxels:=false ...]
#   systemd-run --user --unit=ur10-cumotion-map -p Restart=on-failure -p RestartSec=15 \
#       ~/ur10/cumotion/start_cumotion_map.sh
# Maps are looked up in <repo>/cumotion/maps/ (committed, <50 MB) and
# ~/ur10/cumotion/maps/ (large ones, gitignored) — see cams/cellmap/README.md.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
HOST_MAPS="$HOME/ur10/cumotion/maps"
mkdir -p "$HOST_MAPS"

MAP="$1"; [ $# -gt 0 ] && shift
if [ -z "$MAP" ]; then
  MAP="$(ls -t "$HERE"/maps/cell-*.nvblx "$HOST_MAPS"/cell-*.nvblx 2>/dev/null | head -1)"
  [ -n "$MAP" ] || { echo "start_cumotion_map.sh: no cell-*.nvblx in $HERE/maps or $HOST_MAPS" >&2; exit 2; }
fi
case "$MAP" in
  /*) ;;                                   # absolute host path
  *)  for d in "$HERE/maps" "$HOST_MAPS" "$PWD"; do
        [ -f "$d/$MAP" ] && { MAP="$d/$MAP"; break; }
      done ;;
esac
[ -f "$MAP" ] || { echo "start_cumotion_map.sh: map not found: $MAP" >&2; exit 2; }
MAP="$(readlink -f "$MAP")"
echo "start_cumotion_map.sh: map $MAP ($(du -h "$MAP" | cut -f1)), xrdf ur10_map.xrdf (0.04 m buffer)"

if pgrep -f 'docker run --rm --name ur10-cumotion ' >/dev/null || docker ps --format '{{.Names}}' | grep -qx ur10-cumotion; then
  echo "start_cumotion_map.sh: container ur10-cumotion is already running — the default planner?" >&2
  echo "  systemctl --user stop ur10-cumotion   # then retry" >&2
  exit 1
fi

exec docker run --rm --name ur10-cumotion \
  --net=host --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID=0 \
  -v "$HERE:/cfg:ro" \
  -v "$ROOT/cams/cellmap:/cellmap:ro" \
  -v "$(dirname "$MAP"):/mapdir:ro" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec ros2 launch /cellmap/planner.launch.py "$@"' _ \
  "map:=/mapdir/$(basename "$MAP")" "$@"
