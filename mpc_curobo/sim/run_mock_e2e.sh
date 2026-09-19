#!/usr/bin/env bash
# End-to-end SIM proof of the gate-mediated MPC command stream against the UR
# driver's MOCK hardware + forward_position_controller (SIM ONLY, domain 84).
#
# Runs INSIDE the fm-plan-mpc-curobo-v1 container. Two phases:
#   1. GATING: gate CLEAR->STOP->CLEAR. The arm must move toward the goal only
#      while the gate is RUN and hold while it is STOP (a command arriving during
#      STOP must not move the arm).
#   2. STARVATION: gate held CLEAR (RUN) so the arm is moving, then the bridge is
#      KILLED BY PID mid-motion. The arm must END STOPPED (forward_position_
#      controller holds its last setpoint on a starved stream).
#
# Stops every process it starts BY PID (task hard rule). Writes recordings to
# /out/runs and runs the assertions.
set -o pipefail
export LANG=C.UTF-8 LC_ALL=C.UTF-8
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=84
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export HOME=/tmp

REPO=/repo
OUT=/out/runs
mkdir -p "$OUT"
BRIDGE=$REPO/mpc_curobo/ros_bridge/mpc_gate_bridge.py
GATEPUB=$REPO/mpc_curobo/sim/gate_state_pub.py
REC=$REPO/mpc_curobo/sim/record_run.py
# goals inside the measured windows. GATING drives to the reference/park pose;
# STARVATION drives to a different in-window config so the arm is moving when the
# stream is cut.
GOAL_PARK="0.223385,-1.626713,0.406275,4.458966,0.279341,0.706048"
GOAL_ALT="-0.8,-1.2,1.5,3.2,0.0,0.0"

PIDS=()
kill_pids() { for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null; done; sleep 2;
              for p in "${PIDS[@]:-}"; do kill -TERM "$p" 2>/dev/null; done; PIDS=(); }
trap kill_pids EXIT

# MODE=mock (default): UR driver fake hardware (mirrors commands to states).
# MODE=ursim: connect the driver to the CB3 URSim simulator over LOOPBACK — this
# container shares URSim's network namespace (docker run --network container:...),
# so robot_ip=127.0.0.1 reaches URSim and the arming interlock still sees loopback
# (never a physical NIC or 192.168.0.1). URSim needs a power-on + brake release.
MODE="${MODE:-mock}"
if [ "$MODE" = "ursim" ]; then
    echo "=== power on URSim CB3 (dashboard 127.0.0.1:29999) ==="
    for i in $(seq 1 40); do
        bash -c "exec 3<>/dev/tcp/127.0.0.1/30004" 2>/dev/null && break; sleep 2; done
    python3 - <<'PY'
import socket, time
def cmd(s, c):
    s.sendall((c + "\n").encode()); time.sleep(0.5)
    return s.recv(4096).decode(errors="replace").strip()
s = socket.create_connection(("127.0.0.1", 29999), timeout=10); time.sleep(0.5)
try: s.recv(4096)
except Exception: pass
cmd(s, "power on"); time.sleep(4); cmd(s, "brake release"); time.sleep(6)
print("  robotmode:", cmd(s, "robotmode"), "| safetymode:", cmd(s, "safetymode"))
s.close()
PY
    echo "=== launch UR10 driver -> URSim (127.0.0.1) + forward_position_controller ==="
    HW_ARGS="use_fake_hardware:=false"
else
    echo "=== launch UR10 fake (mock) hardware + forward_position_controller ==="
    HW_ARGS="use_fake_hardware:=true"
fi
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10 robot_ip:=127.0.0.1 $HW_ARGS \
    initial_joint_controller:=forward_position_controller \
    activate_joint_controller:=true headless_mode:=true launch_rviz:=false \
    > /out/runs/ur_launch_${MODE}.log 2>&1 &
LAUNCH_PID=$!; PIDS+=("$LAUNCH_PID")

echo "waiting for forward_position_controller active..."
for i in $(seq 1 30); do
    if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "forward_position_controller.*active"; then
        echo "controller active after ${i}0% of budget"; break; fi
    sleep 2
done
timeout 5 ros2 control list_controllers 2>/dev/null || true

run_phase() {
    local name=$1 seq=$2 out=$3 dur=$4 goal=$5 maxstep=$6 kill_bridge_at=${7:-0}
    echo "=== phase $name (seq=$seq dur=${dur}s step=$maxstep) ==="
    python3 "$REC" --out "$out" --duration "$dur" & local RECP=$!; PIDS+=("$RECP")
    python3 "$GATEPUB" --sequence "$seq" & local PUBP=$!; PIDS+=("$PUBP")
    # NOTE: --scripted-goal="$goal" (equals form) so a leading '-' in the goal is
    # not mistaken for a flag by argparse.
    python3 "$BRIDGE" --arm --command-source scripted --scripted-goal="$goal" \
        --robot-ip 127.0.0.1 --rate 50 --max-step "$maxstep" > /out/runs/bridge_$name.log 2>&1 & \
        local BRP=$!; PIDS+=("$BRP")
    if [ "$kill_bridge_at" != "0" ]; then
        sleep "$kill_bridge_at"
        echo ">>> STARVATION: killing bridge PID $BRP at t=${kill_bridge_at}s"
        kill -INT "$BRP" 2>/dev/null; sleep 1; kill -TERM "$BRP" 2>/dev/null
    fi
    wait "$RECP" 2>/dev/null
    kill -INT "$PUBP" 2>/dev/null; kill -INT "$BRP" 2>/dev/null; sleep 1
    kill -TERM "$PUBP" 2>/dev/null; kill -TERM "$BRP" 2>/dev/null
}

# Phase 1: gating + RESUME. Long initial CLEAR (DDS discovery + 2 s dwell -> RUN,
# arm moves partway to park), then SLOW (a person, arm HOLDS), then a tracked
# departure SLOW->CLEAR + dwell -> RUN RESUMES toward the freshest goal. A small
# step keeps the arm mid-motion across the SLOW so the resume is visible.
run_phase gating "CLEAR:6,SLOW:6,CLEAR:12" "$OUT/${MODE}_gating.jsonl" 26 "$GOAL_PARK" 0.006 0

# Phase 2: starvation. Hold CLEAR (=RUN) with a fresh far goal + slow step so the
# arm is still moving, then KILL the bridge by PID at 6 s and watch it hold.
run_phase starvation "CLEAR:22" "$OUT/${MODE}_starvation.jsonl" 22 "$GOAL_ALT" 0.004 6

kill_pids
trap - EXIT
echo "=== assertions ($MODE) ==="
python3 "$REPO/mpc_curobo/sim/assert_mock_e2e.py" --gating "$OUT/${MODE}_gating.jsonl" \
    --starvation "$OUT/${MODE}_starvation.jsonl"
