#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# PROOF 1 — the nvblox dynamic layer: an obstacle that APPEARS enters the ESDF the
# planner consults (get_esdf_and_gradient — the SAME service cuMotion queries),
# and DECAYS out of it after it LEAVES.
#
# Runs entirely in a throwaway container built FROM the planner image (read-only
# base), on the task's own ROS domain, on a PRIVATE container network (no host
# networking, no published ports) so it is fully isolated. GPU only; the fleet GPU
# lock must be held by the CALLER. SIMULATION ONLY: nvblox is fed synthetic depth
# (+ a synthetic segmentation mask when seg=on) from a fixed provisional camera
# pose; no robot, no live camera, nothing is ever commanded to move.
#
# Usage: run_dynamic_esdf_proof.sh [mode] [seg] [timeline] [duration] [tag]
#   mode:     nvblox mapping_type (default human_with_static_tsdf)
#   seg:      on|off   publish + consume a synthetic segmentation mask (default on)
#   timeline: default|box_only|person_only (default default)
#   duration: seconds (default 42)
#   tag:      output filename tag (default = mode)
# Self-asserting: exits 0 iff every required obstacle proved appear+decay.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
MODE="${1:-dynamic}"
SEG="${2:-off}"
TIMELINE="${3:-default}"
DURATION="${4:-42}"
TAG="${5:-$MODE}"
NAME="fm-plan-nvblox-dynamic-v1-esdf-$TAG"
DOMAIN=85
OUTDIR="$ROOT/data/plan-nvblox-dynamic-v1"
mkdir -p "$OUTDIR"
GPU_LOG="$OUTDIR/esdf_${TAG}_gpu.log"
RUN_LOG="$OUTDIR/esdf_${TAG}_run.log"
: > "$RUN_LOG"

cleanup() {
  [ -n "${SMPID:-}" ] && kill "$SMPID" 2>/dev/null
  docker rm -f "$NAME" >/dev/null 2>&1
}
trap cleanup EXIT

echo "== mode=$MODE seg=$SEG timeline=$TIMELINE dur=${DURATION}s ==" | tee -a "$RUN_LOG"
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

USE_SEG=false; [ "$SEG" = "on" ] && USE_SEG=true
echo "== launch nvblox ($MODE, use_segmentation=$USE_SEG) ==" | tee -a "$RUN_LOG"
docker exec -d "$NAME" bash -lc \
  "source /opt/ros/jazzy/setup.bash; ros2 launch /dynamic/dynamic_live.launch.py \
     mapping_type:=$MODE use_segmentation:=$USE_SEG with_cumotion:=false \
     > /out/esdf_${TAG}_nvblox.log 2>&1"

MASK_ARGS=""
[ "$SEG" = "on" ] && MASK_ARGS="--publish-mask --mask-labels person box"
echo "== start synthetic depth (+mask:$SEG) ==" | tee -a "$RUN_LOG"
docker exec -d "$NAME" bash -lc \
  "source /opt/ros/jazzy/setup.bash; python3 /dynamic/synth_depth_pub.py \
     --timeline $TIMELINE --rate 10 $MASK_ARGS > /out/esdf_${TAG}_synth.log 2>&1"

REQUIRE="box person"
[ "$TIMELINE" = "box_only" ] && REQUIRE="box"
[ "$TIMELINE" = "person_only" ] && REQUIRE="person"
echo "== run proof driver (duration=${DURATION}s) ==" | tee -a "$RUN_LOG"
dexec "python3 /dynamic/proof/esdf_proof_driver.py --duration $DURATION \
        --poll-hz 2 --out /out/esdf_${TAG}.json --require $REQUIRE" 2>&1 | tee -a "$RUN_LOG"
RC=${PIPESTATUS[0]}

PEAK=$(sort -n "$GPU_LOG" 2>/dev/null | tail -1)
echo "== peak GPU memory.used during run: ${PEAK:-?} MiB ==" | tee -a "$RUN_LOG"
echo "$PEAK" > "$OUTDIR/esdf_${TAG}_gpu_peak_mib.txt"
[ "$RC" -eq 0 ] && echo "RESULT: PASS ($MODE)" | tee -a "$RUN_LOG" \
                || echo "RESULT: FAIL rc=$RC ($MODE) — see $OUTDIR/esdf_${TAG}.json" | tee -a "$RUN_LOG"
exit "$RC"
