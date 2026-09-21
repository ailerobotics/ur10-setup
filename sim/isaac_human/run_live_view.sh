#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# WATCH IT LIVE — run the human-detection scene AND stream the viewport over
# WebRTC so the owner can watch the person walk in and out from a phone/laptop
# on the tailnet. One command: preflight, GPU lock, Isaac (--livestream + the
# REAL detector graph exactly as run_session.sh), the owner's static browser
# client served on the tailnet IP, print the link, and on exit stop EVERYTHING
# it started BY PID and confirm the ports and GPU are released.
#
#   sim/isaac_human/run_live_view.sh [cams] [device]
#     cams    comma list of camera roles (default: wrist -- a SINGLE camera, the
#             measured-healthy set; 2+ cameras pin the state at fail-safe STOP on
#             this shared 12 GB card). See run_session.sh.
#     device  cuda (default) | cpu
#
#   Env: SESSION_SECS (default 1800, HARD MAX 1800 = 30 min), CLIENT_PORT
#        (default: first free at/above 8080, skipping the brief's forbidden set).
#
# SIMULATION ONLY. Domain 88, localhost-only ROS, FastDDS UDPv4, one fleet GPU
# lock. Touches no real robot, container or RealSense. The client is served from
# the owner's read-only webrtc_client (NOT copied into this repo).
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
CAMS="${1:-wrist}"
DEVICE="${2:-cuda}"

# The owner's static browser client — served in place, never copied (it carries a
# vendored vendor library). Literal path, read-only.
CLIENT_DIR="/home/manishsaini/workspaces/shelfbot/tools/webrtc_client"
LIVESTREAM_PORT=49100

LOCK="${FLEET_GPU_LOCK:-$HOME/.cache/fleet-gpu.lock}"
VENV_PY="$HOME/venvs/human-safety/bin/python"
CFG="$HERE/params/human_safety.sim.yaml"
POSES="$HERE/params/camera_poses.sim.yaml"

# Session length: default 30 min, hard cap 30 min.
SESSION_SECS="${SESSION_SECS:-1800}"
if [ "$SESSION_SECS" -gt 1800 ]; then
  echo "[live] SESSION_SECS=$SESSION_SECS exceeds the 30 min hard max; clamping to 1800." >&2
  SESSION_SECS=1800
fi
SETTLE="${SETTLE_SECONDS:-30}"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUNDIR="${ISAAC_HUMAN_OUT:-$HOME/.cache/isaac_human_sim}/liveview_${STAMP}"
mkdir -p "$RUNDIR/frames"
GT_LOG="$RUNDIR/ground_truth.jsonl"
REC_LOG="$RUNDIR/record.jsonl"
ISAAC_LOG="$RUNDIR/isaac.log"
GRAPH_LOG="$RUNDIR/graph.log"
REC_RUN_LOG="$RUNDIR/recorder.log"
CLIENT_LOG="$RUNDIR/client.log"
LISTEN_LOG="$RUNDIR/listeners.txt"

# --- preflight ------------------------------------------------------------- #
# shellcheck source=/dev/null
source "$HERE/preflight.sh"
echo "[live] preflight ..."
preflight_check || { echo "[live] preflight failed; not starting." >&2; exit 3; }

# Client dir must exist (read-only reference outside the repo).
[ -f "$CLIENT_DIR/index.html" ] || {
  echo "[live] BLOCKED: client page not found at $CLIENT_DIR/index.html" >&2; exit 3; }

# Tailnet IP (read at RUN TIME; do not copy the stale shelfbot default).
TAILNET_IP="$(tailscale ip -4 2>/dev/null | head -1 | tr -d '[:space:]')"
[ -n "$TAILNET_IP" ] || { echo "[live] BLOCKED: no tailnet IPv4 from 'tailscale ip -4'." >&2; exit 3; }
echo "[live] tailnet IP: $TAILNET_IP"

# Signalling port 49100 must be free.
if ss -ltn "( sport = :$LIVESTREAM_PORT )" 2>/dev/null | grep -q LISTEN; then
  echo "[live] BLOCKED: signalling port $LIVESTREAM_PORT already in use." >&2; exit 3
fi

# Client port: honour CLIENT_PORT if free+allowed, else pick the first free one at
# or above 8080 (skipping the brief's forbidden set). Tested on the tailnet IP.
CLIENT_PORT="$(python3 "$HERE/live_view.py" pick-port \
  --preferred "${CLIENT_PORT:-8080}" --bind "$TAILNET_IP")" || {
  echo "[live] BLOCKED: could not find a free client port." >&2; exit 3; }
echo "[live] client port: $CLIENT_PORT"

# --- GPU lock -------------------------------------------------------------- #
exec 9>"$LOCK"
echo "[live] acquiring GPU lock $LOCK ..."
flock -w 3600 9 || { echo "[live] could not acquire GPU lock." >&2; exit 4; }
echo "[live] GPU lock acquired. RUNDIR=$RUNDIR"

isaac_pid=""; graph_pid=""; rec_pid=""; wd_pid=""; client_pid=""
cleanup() {
  echo "[live] tearing down (by PID) ..."
  [ -n "$client_pid" ] && kill -INT "$client_pid" 2>/dev/null || true
  [ -n "$rec_pid" ] && kill -INT "$rec_pid" 2>/dev/null || true
  sleep 2
  [ -n "$graph_pid" ] && kill -INT -"$graph_pid" 2>/dev/null || true
  [ -n "$isaac_pid" ] && kill -INT "$isaac_pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    a=0
    for p in "$client_pid" "$rec_pid" "$graph_pid" "$isaac_pid"; do
      [ -n "$p" ] && kill -0 "$p" 2>/dev/null && a=1
    done
    [ "$a" = 0 ] && break
    sleep 0.5
  done
  [ -n "$client_pid" ] && kill -9 "$client_pid" 2>/dev/null || true
  [ -n "$rec_pid" ] && kill -9 "$rec_pid" 2>/dev/null || true
  [ -n "$graph_pid" ] && kill -9 -"$graph_pid" 2>/dev/null || true
  [ -n "$isaac_pid" ] && kill -9 "$isaac_pid" 2>/dev/null || true
  [ -n "$wd_pid" ] && kill -9 "$wd_pid" 2>/dev/null || true
  sleep 1
  # Confirm the ports we opened are released and the GPU is free of our procs.
  local p49 pcl
  p49="$(ss -ltn "( sport = :$LIVESTREAM_PORT )" 2>/dev/null | grep -c LISTEN || true)"
  pcl="$(ss -ltn "( sport = :$CLIENT_PORT )" 2>/dev/null | grep -c LISTEN || true)"
  echo "[live] port $LIVESTREAM_PORT listeners after teardown: ${p49:-0}"
  echo "[live] port $CLIENT_PORT listeners after teardown: ${pcl:-0}"
  echo "[live] GPU free after teardown: $(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ') MiB"
  echo "[live] teardown complete."
}
trap cleanup EXIT INT TERM

# --- 1. Isaac scene (bundled ROS 2) + livestream --------------------------- #
(
  # shellcheck source=/dev/null
  source "$HERE/isaac_env.sh"
  # Kit's WebRTC streamer drops NvStreamer-*.etli trace files in the CWD; run
  # from the run dir (under ~/.cache) so they never pollute the repo worktree.
  cd "$RUNDIR" || exit 7
  exec "${CONDA_PREFIX}/bin/python" "$HERE/isaac_scene.py" \
    --cams "$CAMS" --gt-log "$GT_LOG" \
    --settle-seconds "$SETTLE" --max-seconds "$SESSION_SECS" --sim-hz "${SIM_HZ:-5.0}" \
    --livestream --public-ip "$TAILNET_IP" --livestream-port "$LIVESTREAM_PORT" \
    --loop-walk --config "$CFG" --zone-beacon
) >"$ISAAC_LOG" 2>&1 &
isaac_pid=$!
echo "[live] Isaac pid $isaac_pid; waiting for cameras + livestream ..."
ram_watchdog "$isaac_pid" & wd_pid=$!

booted=0
for _ in $(seq 1 120); do
  kill -0 "$isaac_pid" 2>/dev/null || { echo "[live] Isaac died during boot; see $ISAAC_LOG" >&2; break; }
  if grep -q "camera '.*' -> rgb" "$ISAAC_LOG" 2>/dev/null; then booted=1; break; fi
  sleep 2
done
[ "$booted" = 1 ] || { echo "[live] Isaac never published cameras." >&2; exit 5; }

# --- 2. detector graph (system ROS 2), own process group ------------------- #
setsid bash -c "
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
         RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  exec ros2 launch '$REPO/cams/human_safety/human_safety.launch.py' \
    cams:='$CAMS' config:='$CFG' poses:='$POSES' viz:=false device:='$DEVICE'
" >"$GRAPH_LOG" 2>&1 &
graph_pid=$!
echo "[live] detector graph pgid $graph_pid (device=$DEVICE); loading models ..."

# --- 3. recorder (venv python, system ROS 2) ------------------------------- #
setsid bash -c "
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
         RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  exec '$VENV_PY' '$HERE/recorder_node.py' --config '$CFG' --cams '$CAMS' \
    --out '$REC_LOG' --frames-dir '$RUNDIR/frames' --seconds $((SESSION_SECS + 60))
" >"$REC_RUN_LOG" 2>&1 &
rec_pid=$!
echo "[live] recorder pid $rec_pid; log $REC_LOG"

# --- 4. static browser client, bound to the TAILNET IP only ---------------- #
python3 -m http.server "$CLIENT_PORT" --bind "$TAILNET_IP" --directory "$CLIENT_DIR" \
  >"$CLIENT_LOG" 2>&1 &
client_pid=$!
echo "[live] client server pid $client_pid (bind $TAILNET_IP:$CLIENT_PORT, dir $CLIENT_DIR)"

# Snapshot what is listening where (exposure reporting).
{ echo "# listeners during session (started $(date -u +%FT%TZ))"; ss -ltnup 2>/dev/null; } >"$LISTEN_LOG" || true

END_LOCAL="$(date -d "+${SESSION_SECS} seconds" +%H:%M 2>/dev/null || date +%H:%M)"
echo ""
echo "==============================================================="
echo " LIVE VIEW UP"
echo "   open  http://$TAILNET_IP:$CLIENT_PORT   (Tailscale on)"
echo "   session ends about $END_LOCAL local (${SESSION_SECS}s)"
echo "   listeners snapshot: $LISTEN_LOG"
echo "==============================================================="
echo ""

# --- 5. hold for the session, watching Isaac / cap / RAM ------------------- #
t0=$(date +%s)
while kill -0 "$isaac_pid" 2>/dev/null; do
  now=$(date +%s)
  if [ $((now - t0)) -gt "$SESSION_SECS" ]; then
    echo "[live] session length ${SESSION_SECS}s reached; stopping." >&2
    break
  fi
  avail=$(free -m | awk '/^Mem:/ {print $7}')
  if [ -n "$avail" ] && [ "$avail" -lt "${ABORT_RAM_MB:-3072}" ]; then
    echo "[live] RAM under ${ABORT_RAM_MB:-3072} MiB; aborting." >&2
    break
  fi
  sleep 3
done
echo "[live] session ending; cleanup() will stop everything by PID."
echo "RUNDIR=$RUNDIR"   # last line: machine-readable for the caller
