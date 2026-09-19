#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Record ONE short depth bag from a fixed D415 for the dynamic-layer replay path.
# PERCEPTION ONLY — opens a camera, records, releases. Guards, per the task rules:
#   * refuse if ANY /dev/video* is busy (the owner may be running a live check)
#   * hold the camera at most --secs seconds (default 15, hard cap 60)
#   * release and confirm the devices are free afterwards
# NO GPU needed (driver + bag only), NO robot. The bag is depth+color+info from an
# UNCALIBRATED fixed camera, so replay of it is PROVISIONAL/presence-only (see
# extrinsics_guard.py) — realism, never a metric proof.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
NAME="fm-plan-nvblox-dynamic-v1-rec"
OUTDIR="$ROOT/data/plan-nvblox-dynamic-v1"
BAGDIR="$OUTDIR/fixed_d415_bag"
SECS="${1:-15}"
SERIAL="${2:-}"           # optional D415 serial; empty = first RealSense found
[ "$SECS" -gt 60 ] && { echo "refusing: $SECS s > 60 s cap"; exit 2; }
mkdir -p "$OUTDIR"; rm -rf "$BAGDIR"

echo "== camera free check (fuser /dev/video*) =="
if fuser /dev/video* 2>/dev/null; then
  echo "REFUSING: a /dev/video* device is busy — the owner may be running a live check. Try later."
  exit 3
fi

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1; }
trap cleanup EXIT

SER_ARG=""; [ -n "$SERIAL" ] && SER_ARG="serial_no:=$SERIAL"
echo "== bring up D415 depth+color ($SECS s) and record =="
docker run -d --name "$NAME" --privileged -v /dev:/dev \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e ROS_DOMAIN_ID=85 \
  -v "$OUTDIR:/out" "$IMAGE" sleep infinity >/dev/null || { echo "docker run failed"; exit 4; }
dexec() { docker exec "$NAME" bash -lc "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; $*"; }

docker exec -d "$NAME" bash -lc \
  "source /opt/ros/jazzy/setup.bash; ros2 launch realsense2_camera rs_launch.py \
     depth_module.depth_profile:=640x480x30 enable_color:=true enable_depth:=true \
     $SER_ARG pointcloud.enable:=false > /out/rec_rs.log 2>&1"
sleep 8   # driver bring-up
echo "== recording $SECS s =="
dexec "cd /out && timeout $((SECS+3)) ros2 bag record -o fixed_d415_bag \
        /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
        /camera/camera/color/image_raw /camera/camera/color/camera_info \
        > /out/rec_bag.log 2>&1" &
RECPID=$!
sleep "$SECS"
wait "$RECPID" 2>/dev/null || true
echo "== stopping driver + releasing camera =="
docker rm -f "$NAME" >/dev/null 2>&1
trap - EXIT
sleep 2
echo "== post-release camera free check =="
if fuser /dev/video* 2>/dev/null; then echo "WARNING: a device still busy after release"; else echo "cameras released OK"; fi
if [ -d "$BAGDIR" ]; then echo "RESULT: bag at $BAGDIR ($(du -sh "$BAGDIR" | cut -f1))"; else echo "RESULT: no bag written — see $OUTDIR/rec_*.log"; exit 1; fi
