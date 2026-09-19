#!/usr/bin/env bash
# End-to-end SIM proof of the LIVE cuRobo MPC in the loop through the gate to the
# UR driver's mock hardware (SIM ONLY, domain 84, GPU). Closes the loop:
#   /joint_states -> cuRobo MPC solve -> gate acceptance shim -> forward_position_
#   controller -> mock hardware -> /joint_states.
# The gate goes CLEAR->STOP->CLEAR: the MPC-driven arm must move toward the
# Cartesian goal only while RUN and hold while STOP.
#
# Runs INSIDE the fm-plan-mpc-curobo-v1 container WITH --gpus. Hold the fleet GPU
# lock around the docker run. Stops every process by PID.
set -o pipefail
export LANG=C.UTF-8 LC_ALL=C.UTF-8
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=84 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 HOME=/tmp
REPO=/repo; OUT=/out/runs; mkdir -p "$OUT"
ROBOT=$REPO/mpc_curobo/robot/ur10_cell_curobo.yml
SCENE=$REPO/mpc_curobo/world/cell_scene.yml
# goal = FK pose of the in-window pick-anchor config (reachable by construction)
GOAL_CFG="0.0,-1.4,1.4,4.2,-1.4,0.0"

PIDS=()
cleanup(){ for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null; done; sleep 2;
           for p in "${PIDS[@]:-}"; do kill -TERM "$p" 2>/dev/null; done; }
trap cleanup EXIT

echo "=== launch UR10 mock + forward_position_controller ==="
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=127.0.0.1 \
    use_fake_hardware:=true initial_joint_controller:=forward_position_controller \
    activate_joint_controller:=true headless_mode:=true launch_rviz:=false \
    > "$OUT/ur_launch_live.log" 2>&1 &
PIDS+=("$!")
for i in $(seq 1 30); do
    timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "forward_position_controller.*active" && { echo "controller active"; break; }
    sleep 2
done

echo "=== home the arm to the in-window park pose (scripted bridge) ==="
# The mock starts at all-zeros (outside 3 joints' windows). Home to park first so
# the live MPC gets a valid in-window start (uses the proven scripted path).
PARK="0.223385,-1.626713,0.406275,4.458966,0.279341,0.706048"
python3 "$REPO/mpc_curobo/sim/gate_state_pub.py" --sequence "CLEAR:14" & HPUB=$!; PIDS+=("$HPUB")
python3 "$REPO/mpc_curobo/ros_bridge/mpc_gate_bridge.py" --arm --command-source scripted \
    --scripted-goal="$PARK" --robot-ip 127.0.0.1 --rate 50 --max-step 0.02 \
    > "$OUT/bridge_home.log" 2>&1 & HBR=$!; PIDS+=("$HBR")
sleep 14
kill -INT "$HPUB" "$HBR" 2>/dev/null; sleep 1; kill -TERM "$HPUB" "$HBR" 2>/dev/null

echo "=== start LIVE MPC bridge (cuRobo init takes ~15 s) ==="
python3 "$REPO/mpc_curobo/ros_bridge/mpc_gate_bridge.py" --arm --command-source curobo \
    --robot-cfg "$ROBOT" --scene-model "$SCENE" --mpc-goal-config "$GOAL_CFG" \
    --robot-ip 127.0.0.1 --rate 30 --max-step 0.02 > "$OUT/bridge_live.log" 2>&1 & PIDS+=("$!")
echo "waiting for bridge to finish cuRobo init..."
for i in $(seq 1 40); do
    grep -q "mpc_gate_bridge up" "$OUT/bridge_live.log" 2>/dev/null && { echo "bridge ready"; break; }
    sleep 2
done

echo "=== record + gate states (bridge already listening) ==="
python3 "$REPO/mpc_curobo/sim/record_run.py" --out "$OUT/live_mpc.jsonl" --duration 34 & PIDS+=("$!"); REC=$!
python3 "$REPO/mpc_curobo/sim/gate_state_pub.py" --sequence "CLEAR:14,STOP:8,CLEAR:10" & PIDS+=("$!")

wait "$REC" 2>/dev/null
cleanup; trap - EXIT
echo "=== assertion ==="
python3 "$REPO/mpc_curobo/sim/assert_live_mpc.py" --rec "$OUT/live_mpc.jsonl"
