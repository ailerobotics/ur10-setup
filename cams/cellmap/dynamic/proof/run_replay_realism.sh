#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# REALISM (not a metric proof): replay the recorded fixed-D415 depth bag through
# the SAME nvblox dynamic layer and show it ingests REAL depth and populates the
# ESDF the planner consults. The fixed D415 is UNCALIBRATED, so the camera->base
# transform here is a clearly-labelled PROVISIONAL sim extrinsic and the result is
# presence-only: this demonstrates real-depth ingestion, it does NOT assert metric
# obstacle positions. Real metric use is gated by extrinsics_guard (refused until
# calibrate_static_camera.py writes a passing file). GPU; caller holds the lock.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64"
NAME="fm-plan-nvblox-dynamic-v1-replay"
DOMAIN=85
OUTDIR="$ROOT/data/plan-nvblox-dynamic-v1"
BAGDIR="$OUTDIR/fixed_d415_bag"
RUN_LOG="$OUTDIR/replay_run.log"; : > "$RUN_LOG"
[ -d "$BAGDIR" ] || { echo "no bag at $BAGDIR — run record_fixed_bag.sh first"; exit 2; }

cleanup() { [ -n "${SMPID:-}" ] && kill "$SMPID" 2>/dev/null; docker rm -f "$NAME" >/dev/null 2>&1; }
trap cleanup EXIT

( while true; do nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits; sleep 1; done ) > "$OUTDIR/replay_gpu.log" 2>/dev/null &
SMPID=$!

docker run -d --name "$NAME" --gpus all \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e ROS_DOMAIN_ID=$DOMAIN \
  -v "$ROOT/cams/cellmap/dynamic:/dynamic:ro" -v "$ROOT/cams/cellmap:/cellmap:ro" \
  -v "$ROOT/cumotion:/cfg:ro" -v "$OUTDIR:/out" \
  "$IMAGE" sleep infinity >/dev/null || { echo "docker run failed"; exit 3; }
dexec() { docker exec "$NAME" bash -lc "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; $*"; }

echo "== PROVISIONAL base_link->camera TF (uncalibrated D415, presence-only, sim time) ==" | tee -a "$RUN_LOG"
docker exec -d "$NAME" bash -lc "source /opt/ros/jazzy/setup.bash; \
  ros2 run tf2_ros static_transform_publisher --x 0 --y -1.6 --z 1.2 \
    --qx 0.8446 --qy 0 --qz 0 --qw 0.5354 \
    --frame-id base_link --child-frame-id camera_depth_optical_frame \
    --ros-args -p use_sim_time:=true > /out/replay_tf.log 2>&1"

echo "== nvblox dynamic fed by the RECORDED depth topic (use_sim_time) ==" | tee -a "$RUN_LOG"
docker exec -d "$NAME" bash -lc "source /opt/ros/jazzy/setup.bash; \
  ros2 launch /dynamic/dynamic_live.launch.py mapping_type:=dynamic with_cumotion:=false \
    use_sim_time:=true \
    depth_image:=/camera/camera/depth/image_rect_raw \
    depth_info:=/camera/camera/depth/camera_info > /out/replay_nvblox.log 2>&1"
sleep 6
echo "== replay the recorded bag (--clock so stamps drive the sim clock) ==" | tee -a "$RUN_LOG"
docker exec -d "$NAME" bash -lc "source /opt/ros/jazzy/setup.bash; \
  ros2 bag play /out/fixed_d415_bag --rate 1.0 --clock 200 \
    --qos-profile-overrides-path /dynamic/proof/replay_qos.yaml > /out/replay_play.log 2>&1"
sleep 12
echo "== probe the ESDF over the PROVISIONAL camera's view cone (realism) ==" | tee -a "$RUN_LOG"
# The fixed D415 is uncalibrated; with the provisional pose the real depth is
# integrated along that provisional view, which reaches BELOW/behind the arm
# workspace. Probe a volume that covers the provisional cone so the realism check
# sees the geometry the real depth produced (this is presence-only, not metric).
dexec "python3 /dynamic/esdf_probe.py --aabb-min -1.6 -1.7 -0.7 --aabb-size 3.2 3.6 2.8" | tee -a "$RUN_LOG"
RESULT_LINE=$(tail -1 "$RUN_LOG")
PEAK=$(sort -n "$OUTDIR/replay_gpu.log" 2>/dev/null | tail -1)
echo "== peak GPU during replay: ${PEAK:-?} MiB ==" | tee -a "$RUN_LOG"
python3 -c "
import json,sys
line='''$RESULT_LINE'''
try:
  r=json.loads(line); obs=r.get('observed',0)
  ok = obs>1000
  json.dump({'observed_voxels':obs,'occupied_voxels':r.get('occupied'),'peak_gpu_mib':'${PEAK:-}','provisional':True,'note':'presence-only; uncalibrated D415; realism not metric proof','ok':ok}, open('$OUTDIR/replay_realism.json','w'), indent=2)
  print('REPLAY REALISM', 'PASS (real depth ingested, ESDF populated)' if ok else 'INCONCLUSIVE', 'observed='+str(obs))
  sys.exit(0 if ok else 1)
except Exception as e:
  print('replay parse error', e); sys.exit(1)
"
