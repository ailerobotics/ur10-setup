#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Run the REAL human_safety detector graph against the Isaac human-detection
# scene, under ONE fleet GPU lock, and record ground truth + state + annotated
# frames for the offline analysis.
#
#   sim/isaac_human/run_session.sh [cams] [device]
#     cams    comma list of camera roles (default: wrist -- a SINGLE camera, the
#             measured-healthy set that keeps img->state latency under the 0.70 s
#             reaction budget on this shared 12 GB card). 2+ cameras run the
#             latency past the budget and pin the state at fail-safe STOP.
#     device  cuda (default) | cpu   -- detector compute device (SIM config only)
#
# SIMULATION ONLY. Domain 88, localhost only, FastDDS UDPv4. Touches no real
# robot, container or RealSense. Isaac uses its BUNDLED ROS 2 (isaac_env.sh); the
# detector graph + recorder use the SYSTEM ROS 2 (/opt/ros/jazzy); they meet over
# DDS on domain 88. Stops everything it starts, by PID, and reports.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
CAMS="${1:-wrist}"
DEVICE="${2:-cuda}"

_ncams=$(awk -F, '{c=0; for(i=1;i<=NF;i++) if($i!="") c++; print c}' <<<"$CAMS")
if [ "${_ncams:-1}" -gt 1 ]; then
  echo "[session] WARNING: $_ncams cameras requested ($CAMS). On this shared 12 GB" \
       "card only a SINGLE camera keeps img->state latency under the 0.70 s reaction" \
       "budget; 2+ cameras run the latency past the budget and pin the state at" \
       "fail-safe STOP, masking the distance behaviour. Proceeding as requested." >&2
fi
LOCK="${FLEET_GPU_LOCK:-$HOME/.cache/fleet-gpu.lock}"
VENV_PY="$HOME/venvs/human-safety/bin/python"

CFG="$HERE/params/human_safety.sim.yaml"
POSES="$HERE/params/camera_poses.sim.yaml"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUNDIR="${ISAAC_HUMAN_OUT:-$HOME/.cache/isaac_human_sim}/run_${STAMP}"
mkdir -p "$RUNDIR/frames"
GT_LOG="$RUNDIR/ground_truth.jsonl"
REC_LOG="$RUNDIR/record.jsonl"
ISAAC_LOG="$RUNDIR/isaac.log"
GRAPH_LOG="$RUNDIR/graph.log"
REC_RUN_LOG="$RUNDIR/recorder.log"
SETTLE="${SETTLE_SECONDS:-30}"        # render + detector warmup before motion
SESSION_CAP="${SESSION_CAP:-1140}"    # <20 min hard wall-clock cap (brief)

# shellcheck source=/dev/null
source "$HERE/preflight.sh"
echo "[session] preflight ..."
preflight_check || { echo "[session] preflight failed; not starting." >&2; exit 3; }

exec 9>"$LOCK"
echo "[session] acquiring GPU lock $LOCK ..."
flock -w 3600 9 || { echo "[session] could not acquire GPU lock." >&2; exit 4; }
echo "[session] GPU lock acquired. RUNDIR=$RUNDIR"

isaac_pid=""; graph_pid=""; rec_pid=""; wd_pid=""
cleanup() {
  echo "[session] tearing down (by PID) ..."
  [ -n "$rec_pid" ] && kill -INT "$rec_pid" 2>/dev/null || true
  sleep 2
  [ -n "$graph_pid" ] && kill -INT -"$graph_pid" 2>/dev/null || true
  [ -n "$isaac_pid" ] && kill -INT "$isaac_pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    a=0
    for p in "$rec_pid" "$graph_pid" "$isaac_pid"; do
      [ -n "$p" ] && kill -0 "$p" 2>/dev/null && a=1
    done
    [ "$a" = 0 ] && break
    sleep 0.5
  done
  [ -n "$rec_pid" ] && kill -9 "$rec_pid" 2>/dev/null || true
  [ -n "$graph_pid" ] && kill -9 -"$graph_pid" 2>/dev/null || true
  [ -n "$isaac_pid" ] && kill -9 "$isaac_pid" 2>/dev/null || true
  [ -n "$wd_pid" ] && kill -9 "$wd_pid" 2>/dev/null || true
  sleep 1
  echo "[session] teardown complete."
}
trap cleanup EXIT INT TERM

# 1. Isaac scene (bundled ROS 2).
(
  # shellcheck source=/dev/null
  source "$HERE/isaac_env.sh"
  exec "${CONDA_PREFIX}/bin/python" "$HERE/isaac_scene.py" \
    --cams "$CAMS" --gt-log "$GT_LOG" \
    --settle-seconds "$SETTLE" --max-seconds 300 --sim-hz "${SIM_HZ:-5.0}"
) >"$ISAAC_LOG" 2>&1 &
isaac_pid=$!
echo "[session] Isaac pid $isaac_pid; waiting for cameras to publish ..."
ram_watchdog "$isaac_pid" & wd_pid=$!

booted=0
for _ in $(seq 1 120); do
  kill -0 "$isaac_pid" 2>/dev/null || { echo "[session] Isaac died during boot; see $ISAAC_LOG" >&2; break; }
  if grep -q "path playback:" "$ISAAC_LOG" 2>/dev/null; then booted=1; break; fi
  # cameras wired is enough to start the graph; motion starts after SETTLE
  if grep -q "camera '.*' -> rgb" "$ISAAC_LOG" 2>/dev/null; then booted=1; break; fi
  sleep 2
done
[ "$booted" = 1 ] || { echo "[session] Isaac never published cameras." >&2; exit 5; }

# 2. detector graph (system ROS 2), in its own process group.
setsid bash -c "
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
         RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  exec ros2 launch '$REPO/cams/human_safety/human_safety.launch.py' \
    cams:='$CAMS' config:='$CFG' poses:='$POSES' viz:=false device:='$DEVICE'
" >"$GRAPH_LOG" 2>&1 &
graph_pid=$!
echo "[session] detector graph pgid $graph_pid (device=$DEVICE); loading models ..."

# 3. recorder (venv python, system ROS 2).
setsid bash -c "
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
         RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  exec '$VENV_PY' '$HERE/recorder_node.py' --config '$CFG' --cams '$CAMS' \
    --out '$REC_LOG' --frames-dir '$RUNDIR/frames' --seconds 400
" >"$REC_RUN_LOG" 2>&1 &
rec_pid=$!
echo "[session] recorder pid $rec_pid; log $REC_LOG"

# 4. wait for Isaac to finish the scripted path (it exits), within the cap.
t0=$(date +%s)
while kill -0 "$isaac_pid" 2>/dev/null; do
  now=$(date +%s)
  if [ $((now - t0)) -gt "$SESSION_CAP" ]; then
    echo "[session] session cap ${SESSION_CAP}s hit; stopping." >&2
    break
  fi
  avail=$(free -m | awk '/^Mem:/ {print $7}')
  if [ -n "$avail" ] && [ "$avail" -lt "${ABORT_RAM_MB:-3072}" ]; then
    echo "[session] RAM under ${ABORT_RAM_MB:-3072} MiB; aborting." >&2
    break
  fi
  sleep 3
done
echo "[session] Isaac path finished (or capped); letting recorder flush ..."
sleep 3

# cleanup() runs on EXIT (stops recorder, graph, isaac by PID).
echo "[session] RUNDIR=$RUNDIR"
echo "RUNDIR=$RUNDIR"   # last line: machine-readable for the caller
