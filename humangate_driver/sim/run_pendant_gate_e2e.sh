#!/usr/bin/env bash
# End-to-end proof that the GATE (not just the raw dashboard) delivers the captain's
# ask with the PENDANT-PAUSE strategy: on a person the gate PAUSES a pendant-loaded
# External Control program so the arm freezes ON its planned path, and on a healthy
# CLEAR+dwell it PLAYS the program so the SAME trajectory continues to completion with
# no replan. It runs the real gate_driver_node (stop_strategy:=pendant_pause) + the
# independent backstop_watcher_node against URSim CB3, driven by scripted camera states
# in the REAL wire format, with the harmless trajectory_mover as the motion under test.
#
# Isolation (task brief): everything on ROS_DOMAIN_ID=82; URSim on its own docker
# network (gateway 192.168.56.1 = the URCap default host IP), no GPU, no published
# ports; teardown by PID + own container/network only. Never touches ur10-driver, the
# robot containers, the tray detector, or any process/container it did not create.
set -o pipefail
export ROS_DOMAIN_ID=82
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
export PLIB_HERE="$HERE"
export PLIB_NET=hg_pgate_net PLIB_CONT=hg_pgate_ursim
export PLIB_LOGDIR="${LOGDIR:-$REPO/data/human-gate-sim-v1/ursim-pendant-gate}"
mkdir -p "$PLIB_LOGDIR"
LOGDIR="$PLIB_LOGDIR"
source "$HERE/_ursim_pendant_lib.sh"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
  pendant_down
}
trap cleanup EXIT INT TERM

pendant_up || { echo "[pendant_gate_e2e] bring-up failed"; exit 3; }

echo "[pendant_gate_e2e] starting scripted publisher (pendant_pause_demo), gate (pendant_pause), watcher"
python3 "$HERE/scripted_state_publisher.py" --ros-args -p scenario:=pendant_pause_demo \
    > "$LOGDIR/publisher.log" 2>&1 &
PUB_PID=$!; PIDS+=("$PUB_PID")
sleep 1
# The SINGLE freeze-confirm knob (pendant_freeze_confirm_s) configures BOTH processes:
# the gate widens its own STOP->BACKSTOP deadline from it, and the watcher — given the
# same stop_strategy:=pendant_pause — sizes its own stop-effect deadline from it, so
# neither cancels the pause before it freezes past ~1.8 s (PENDANT_PAUSE_FINDING.md).
# No hand-tuned watcher stop_effect_deadline_s to forget.
python3 "$HERE/../gate_driver_node.py" --ros-args \
    -p arm:=true -p robot_ip:="$PLIB_URSIM_IP" \
    -p stop_strategy:=pendant_pause -p resume_max_pause_s:=120.0 \
    -p pendant_freeze_confirm_s:=2.5 \
    -p decision_log:="$LOGDIR/decisions.jsonl" > "$LOGDIR/gate.log" 2>&1 &
PIDS+=($!)
python3 "$HERE/../backstop_watcher_node.py" --ros-args \
    -p arm:=true -p robot_ip:="$PLIB_URSIM_IP" \
    -p stop_strategy:=pendant_pause -p pendant_freeze_confirm_s:=2.5 \
    -p watcher_log:="$LOGDIR/watcher.jsonl" > "$LOGDIR/watcher.log" 2>&1 &
PIDS+=($!)

echo "[pendant_gate_e2e] waiting for the gate to reach RUN before moving..."
for i in $(seq 1 20); do
  grep -q '"action": "RUN"' "$LOGDIR/decisions.jsonl" 2>/dev/null && break
  sleep 1
done
# MONOTONIC ramp so a real freeze is unambiguous (no cosine turnaround to mistake).
python3 "$HERE/trajectory_mover.py" --duration 20 --amp 0.4 --profile ramp \
    --log "$LOGDIR/mover.jsonl" --watch 120 > "$LOGDIR/mover.log" 2>&1 &
MOVER_PID=$!; PIDS+=("$MOVER_PID")

echo "[pendant_gate_e2e] running scenario (~80 s) + waiting for the mover goal to finish..."
for i in $(seq 1 95); do kill -0 "$PUB_PID" 2>/dev/null || break; sleep 1; done
for i in $(seq 1 90); do kill -0 "$MOVER_PID" 2>/dev/null || break; sleep 1; done
sleep 2

echo "[pendant_gate_e2e] analysing"
python3 "$HERE/assert_pendant_gate_e2e.py" \
    --decisions "$LOGDIR/decisions.jsonl" --mover "$LOGDIR/mover.jsonl" \
    --watcher "$LOGDIR/watcher.jsonl" | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
echo "[pendant_gate_e2e] logs in $LOGDIR (assert rc=$RC)"
exit $RC
