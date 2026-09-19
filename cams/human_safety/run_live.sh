#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# ONE command to run the human_safety layer live and record a replayable log.
# Brings up the RealSense cameras AND the perception graph on the ISOLATED
# domain 77 (this feature is the only task allowed to open the cameras), opens
# the owner window, and records every fusion cycle to a JSONL that replays as a
# regression test (human_safety/human_safety/replay.py).
#
# It NEVER touches the robot (driver/containers stay down). On exit it stops
# exactly the processes it started, BY PID, and confirms the cameras are
# released.
#
# Usage:
#   cams/human_safety/run_live.sh [cams] [record.jsonl]
#     cams        comma list of roles (default: all attached: wrist,shelf,table)
#     record      output log path (default: ./human_safety_<UTC>.jsonl)
#
# Walk-through recording (needs a person in view — the owner):
#   cams/human_safety/run_live.sh wrist,shelf,table walkthrough.jsonl
#   ... walk slowly toward the cell and back out, then Ctrl-C ...
#   then add walkthrough.jsonl under tests/data/ and assert it in test_replay.py.
# NB: no `set -u` — the ROS setup.bash references unbound vars under nounset.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
CAMS="${1:-wrist,shelf,table}"
RECORD="${2:-$PWD/human_safety_$(date -u +%Y%m%dT%H%M%SZ).jsonl}"
CFG="$HERE/params/human_safety.yaml"

export ROS_DOMAIN_ID=77
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# Read the owner's forwarded display; do NOT assume :0.
export DISPLAY="${DISPLAY:-}"

source /opt/ros/jazzy/setup.bash

PIDS=()
cleanup() {
  echo ""
  echo "[run_live] stopping processes I started (by PID) ..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  # give launch files time to shut their nodes down cleanly
  for _ in $(seq 1 20); do
    local_alive=0
    for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && local_alive=1; done
    [ "$local_alive" = 0 ] && break
    sleep 0.5
  done
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
  sleep 1
  # Confirm the camera devices are released (report only; never kill by name).
  if pgrep -a realsense2_camera_node >/dev/null 2>&1; then
    echo "[run_live] WARNING: a realsense2_camera_node is still running:" >&2
    pgrep -a realsense2_camera_node >&2
    echo "[run_live] (not owned by me if I started none; inspect before acting)" >&2
  else
    echo "[run_live] cameras released."
  fi
  echo "[run_live] recording saved: $RECORD"
}
trap cleanup EXIT INT TERM

echo "[run_live] domain 77, cams=$CAMS, DISPLAY=${DISPLAY:-unset}"
echo "[run_live] recording -> $RECORD"

# 1. cameras (their own launch; per cams/README.md). setsid so a clean SIGINT
#    reaches the whole launch tree.
setsid ros2 launch "$REPO/cams/rig_multicam.launch.py" \
  cams:="$CAMS" color_profile:=848x480x30 depth_profile:=848x480x30 &
PIDS+=($!)
echo "[run_live] cameras launching (pid ${PIDS[-1]}); waiting for enumeration ..."
sleep 12

# 2. perception graph: detectors (venv torch) + fusion(+record) + viz.
setsid ros2 launch "$HERE/human_safety.launch.py" \
  cams:="$CAMS" config:="$CFG" viz:=true record:="$RECORD" &
PIDS+=($!)
echo "[run_live] perception graph launching (pid ${PIDS[-1]})."
echo "[run_live] Ctrl-C to stop."

wait
