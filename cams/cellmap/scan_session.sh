#!/usr/bin/env bash
# One scan session from the wrist camera:
#
#   [wrist_rig.launch.py streaming with the calibrated TF]   <- host, started by you
#   [run_scan.sh: robot segmenter + nvblox in base_link]     <- started by you
#   1. waits for nvblox, tells you to sweep (freedrive), shows the recipe
#   2. Enter -> save_map (+ mesh .ply) under ~/ur10/cumotion/maps/<name>.nvblx
#
# Usage:  cams/cellmap/scan_session.sh [cell-YYYY-MM-DD]
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${1:-cell-$(date +%F)}"
NAME="${NAME%.nvblx}"
CONT="${CELLMAP_CONTAINER:-ur10-cellmap-scan}"
MAPS="${CELLMAP_MAPS:-$HOME/ur10/cumotion/maps}"

docker ps --format '{{.Names}}' | grep -qx "$CONT" || { echo "scan_session: $CONT not running (run_scan.sh first)"; exit 1; }
for i in $(seq 1 60); do
  docker exec "$CONT" bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | grep -q /nvblox_node/save_map' && break
  sleep 1
done
echo "== MAPPING (nvblox up after ${i}s). Freedrive the sweep now; the mesh grows in run_rviz.sh."
cat <<'EOF'
   recipe (slow, ~1 min per item, keep surfaces 0.3-2 m from the lens):
     1. table top from three sides, camera looking down 45 deg
     2. every shelf face the arm can reach, left-to-right, two heights
     3. cell walls/columns within reach, low and high
     4. anything the arm could sweep through on the way (carts, boxes, pole)
   a surface that SMEARS while the arm moves = calibration or timestamp problem: stop, do not save.
EOF
read -r -p "== press Enter when the sweep is done -> save_map $NAME ... "
docker exec "$CONT" bash -lc "source /opt/ros/jazzy/setup.bash && python3 /cellmap/tools/map_ctl.py save /maps/$NAME.nvblx && python3 /cellmap/tools/map_ctl.py ply /maps/$NAME.ply"
ls -la "$MAPS/$NAME.nvblx" "$MAPS/$NAME.ply" 2>/dev/null
echo "== done: $MAPS/$NAME.nvblx  (< 50 MB -> copy into $HERE/../../cumotion/maps/ and commit)"
