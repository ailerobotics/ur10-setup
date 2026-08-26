#!/usr/bin/env bash
# One scan session, in the order the frame chain demands (README "Scan"):
#
#   [run_scan.sh already running: camera + cuVSLAM]      <- started by you
#   1. tools/tag_anchor.py  -> waits for tag 0 in view, publishes base_link->map
#   2. mapper stage in the SAME container: robot segmenter + nvblox (base_link)
#   3. you walk; Enter here -> save_map (+ mesh .ply) -> stop the mapper stage
#
# The anchor gate (step 2 waits for step 1's yaml) is the whole point: nvblox
# frames integrated before base_link->map exists land in a wrong frame and
# cannot be un-integrated — the map would have to be thrown away.
#
# Usage:  cams/cellmap/scan_session.sh anchor/tag0.yaml [cell-YYYY-MM-DD]
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TAG="${1:?tag yaml from tools/tag_corners.py}"
NAME="${2:-cell-$(date +%F)}"
NAME="${NAME%.nvblx}"
CONT="${CELLMAP_CONTAINER:-ur10-cellmap-scan}"
MAPS="${CELLMAP_MAPS:-$HOME/ur10/cumotion/maps}"
ANCHOR="$(dirname "$(readlink -f "$TAG")")/anchor-$(date +%F).yaml"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash

docker ps --format '{{.Names}}' | grep -qx "$CONT" || { echo "scan_session: $CONT not running (run_scan.sh first)"; exit 1; }
rm -f "$ANCHOR"
echo "== 1/3 anchor: point the camera at tag 0 from 0.5-1.0 m and hold still"
python3 "$HERE/tools/tag_anchor.py" --tag "$TAG" --out "$ANCHOR" &
ANCHOR_PID=$!
trap 'kill $ANCHOR_PID 2>/dev/null' EXIT
while [ ! -s "$ANCHOR" ]; do
  sleep 0.5
  kill -0 $ANCHOR_PID 2>/dev/null || { echo "scan_session: tag_anchor exited without publishing"; exit 1; }
done
echo; echo "== anchor published ($ANCHOR)"

echo "== 2/3 mapper: robot segmenter + nvblox starting in $CONT"
docker exec -d "$CONT" bash -lc "source /opt/ros/jazzy/setup.bash && exec ros2 launch /cellmap/scan.launch.py camera:=false vslam:=false segmenter:=true nvblox:=true container:=cellmap_mapper > /tmp/mapper.log 2>&1"
for i in $(seq 1 60); do
  docker exec "$CONT" bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | grep -q /nvblox_node/save_map' && break
  sleep 1
done
echo "== MAPPING (nvblox up after ${i}s). Walk the cell now; the mesh grows in run_rviz.sh."
echo "   Finish by pointing at tag 0 again (the residual printed above is the map error)."
read -r -p "== press Enter when the walk is done -> save_map $NAME ... "
docker exec "$CONT" bash -lc "source /opt/ros/jazzy/setup.bash && python3 /cellmap/tools/map_ctl.py save /maps/$NAME.nvblx && python3 /cellmap/tools/map_ctl.py ply /maps/$NAME.ply"
ls -la "$MAPS/$NAME.nvblx" "$MAPS/$NAME.ply" 2>/dev/null
echo "== 3/3 saved. Stopping the mapper stage (camera + cuVSLAM keep running)."
docker exec "$CONT" bash -lc "pkill -INT -f 'container:=cellmap_mapper' || true"
echo "== done: $MAPS/$NAME.nvblx  (anchor: $ANCHOR)"
