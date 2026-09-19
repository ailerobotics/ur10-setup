#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# PROOF 2 — the map reaches the planner: an obstacle that APPEARS changes the
# cuMotion route (a detour), and after it LEAVES the route RETURNS. cuMotion plans
# against nvblox's LIVE dynamic ESDF, in this task's own container/domain.
#
# Throwaway container FROM the planner image (read-only base), private container
# network (no host net, no published ports). GPU only; the fleet GPU lock must be
# held by the CALLER. SIMULATION ONLY, PLAN-ONLY: cuMotion plans with
# plan_only=True; no driver, no controller, nothing is ever executed. Every
# planned waypoint is checked against cumotion/measured_joint_limits.yaml.
#
# Self-asserting: exits 0 iff route changed on appear AND returned on leave AND
# the ESDF gained-then-lost the obstacle AND all waypoints stayed within limits.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
NAME="fm-plan-nvblox-dynamic-v1-route"
DOMAIN=85
OUTDIR="$ROOT/data/plan-nvblox-dynamic-v1"
mkdir -p "$OUTDIR"
GPU_LOG="$OUTDIR/route_gpu.log"
RUN_LOG="$OUTDIR/route_run.log"
: > "$RUN_LOG"

cleanup() {
  [ -n "${SMPID:-}" ] && kill "$SMPID" 2>/dev/null
  docker rm -f "$NAME" >/dev/null 2>&1
}
trap cleanup EXIT

echo "== nvidia-smi before ==" | tee -a "$RUN_LOG"
nvidia-smi --query-gpu=memory.total,memory.used,memory.free --format=csv | tee -a "$RUN_LOG"
( while true; do nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits; sleep 1; done ) > "$GPU_LOG" 2>/dev/null &
SMPID=$!

docker run -d --name "$NAME" --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e ROS_DOMAIN_ID=$DOMAIN \
  -v "$ROOT/cams/cellmap/dynamic:/dynamic:ro" \
  -v "$ROOT/cams/cellmap:/cellmap:ro" \
  -v "$ROOT/cumotion:/cfg:ro" \
  -v "$OUTDIR:/out" \
  "$IMAGE" sleep infinity >/dev/null || { echo "docker run failed"; exit 3; }

dexec() { docker exec "$NAME" bash -lc "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; $*"; }

echo "== launch nvblox dynamic + cuMotion (read_esdf_world) ==" | tee -a "$RUN_LOG"
docker exec -d "$NAME" bash -lc \
  "source /opt/ros/jazzy/setup.bash; ros2 launch /dynamic/dynamic_live.launch.py \
     mapping_type:=dynamic with_cumotion:=true \
     urdf:=/cfg/ur10_calibrated.urdf xrdf:=/cfg/ur10_map.xrdf \
     > /out/route_nvblox.log 2>&1"

echo "== start synthetic depth (manual obstacle mode, default downward camera) ==" | tee -a "$RUN_LOG"
# Use the DEFAULT downward provisional camera — the exact view proven in the
# canonical ESDF proof to integrate obstacles in the clear volume (y0.5-0.9,
# z0-1.0). The low sweep keeps the planned TCP inside that volume, and the driver
# places the box on the planned path within it.
docker exec -d "$NAME" bash -lc \
  "source /opt/ros/jazzy/setup.bash; python3 /dynamic/synth_depth_pub.py \
     --manual --rate 10 > /out/route_synth.log 2>&1"

echo "== wait for cuMotion move_group action server ==" | tee -a "$RUN_LOG"
for i in $(seq 1 60); do
  if dexec "ros2 action list 2>/dev/null | grep -q /cumotion/move_group"; then
    echo "   cuMotion up after ${i}s" | tee -a "$RUN_LOG"; break; fi
  sleep 1
done

echo "== run route proof driver ==" | tee -a "$RUN_LOG"
dexec "python3 /dynamic/proof/route_proof_driver.py --out /out/route_proof.json" 2>&1 | tee -a "$RUN_LOG"
RC=${PIPESTATUS[0]}

PEAK=$(sort -n "$GPU_LOG" 2>/dev/null | tail -1)
echo "== peak GPU memory.used during run: ${PEAK:-?} MiB ==" | tee -a "$RUN_LOG"
echo "$PEAK" > "$OUTDIR/route_gpu_peak_mib.txt"
[ "$RC" -eq 0 ] && echo "RESULT: PASS — route detours on appear, returns on leave" | tee -a "$RUN_LOG" \
                || echo "RESULT: FAIL rc=$RC — see $OUTDIR/route_proof.json" | tee -a "$RUN_LOG"
exit "$RC"
