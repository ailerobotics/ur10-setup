#!/usr/bin/env bash
# Contact-guard end-to-end proof against the Universal Robots CB3 SIMULATOR
# (URSim) in Docker, with the real ur_robot_driver connected to it (brief item 8:
# "a URSim CB3 integration proof of segments, cancel and back-off").
#
# What it proves on the simulator: the live contact-guard node (node.py) watching
# the driver's telemetry trips on an injected contact signature during a
# stop-and-probe segment creep and CANCELS the running FollowJointTrajectory goal
# through the existing cancel-all path (and records the reaction). URSim has no
# contact physics, so the contact is injected on the wrench topic (fault_inject.py);
# the stall/tracking paths are proven by the injected-fault unit tests and the
# mock E2E (run_mock_e2e.py, the in-session proof).
#
# Isolation (mirrors dynamic_pick/sim/run_ursim_e2e.sh): URSim on its OWN docker
# network, no GPU, reachable only by its docker-internal IP (no published ports);
# task-only ROS_DOMAIN_ID=86 (never 0/42 or another lane's); cleanup by PID and
# only the container/network THIS script created. NEVER touches the real cell
# (which is powered off) or any shared service.
set -u
export ROS_DOMAIN_ID=86
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$PKG/../.." && pwd)"
RUNDIR="${RUNDIR:-/home/manishsaini/firstmate/data/ur10-pick-contact-safety-v1/ursim}"
mkdir -p "$RUNDIR"

NET=fm-ucs-ursim-net
SUBNET=192.168.98.0/24
URSIM_IP=192.168.98.2
CONT=fm-ur10-pick-contact-safety-v1-ursim
IMAGE=universalrobots/ursim_cb3:3.15.8
PLAN="${PLAN:-/home/manishsaini/firstmate/data/ur10-fork-pick-real-v1/teach-002/plan_01_insert_forks.yaml}"
TRIP_OUT="$RUNDIR/trip.json"

PIDS=(); LAUNCH_PID=""
cleanup() {
  echo "[ucs-ursim] cleanup (by PID; own container/network only)"
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  [ -n "$LAUNCH_PID" ] && kill -INT -"$LAUNCH_PID" 2>/dev/null
  sleep 3
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
  docker rm -f "$CONT" >/dev/null 2>&1
  docker network rm "$NET" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo "[ucs-ursim] BLOCKED: image $IMAGE not present."
  echo "            Run the in-session proof instead: python3 $HERE/run_mock_e2e.py"
  exit 3; }

docker network inspect "$NET" >/dev/null 2>&1 || \
  docker network create --subnet="$SUBNET" "$NET" >/dev/null
docker rm -f "$CONT" >/dev/null 2>&1
echo "[ucs-ursim] starting URSim UR10 at $URSIM_IP (no GPU, own network)"
docker run --rm -d --name "$CONT" --net "$NET" --ip "$URSIM_IP" \
  -e ROBOT_MODEL=UR10 "$IMAGE" >/dev/null

echo "[ucs-ursim] waiting for the dashboard + RTDE on $URSIM_IP ..."
for i in $(seq 1 60); do
  (exec 3<>"/dev/tcp/$URSIM_IP/29999") 2>/dev/null && { exec 3>&-; break; }
  sleep 2
done

# bring up the driver pointed at URSim's internal IP (arming interlock: never a
# physical NIC / 192.168.0.1), play External Control, start the guard node, then
# dispatch a stop-and-probe segment and inject a contact mid-creep.
echo "[ucs-ursim] bringing up ur_robot_driver -> $URSIM_IP"
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 \
  robot_ip:="$URSIM_IP" headless_mode:=true launch_rviz:=false \
  >"$RUNDIR/driver.log" 2>&1 &
LAUNCH_PID=$!
sleep 15

echo "[ucs-ursim] starting the contact-guard node (probe mode) -> $TRIP_OUT"
PYTHONPATH="$REPO" python3 "$PKG/node.py" --plan "$PLAN" --leg insert_forks \
  --contact --out "$TRIP_OUT" >"$RUNDIR/guard.log" 2>&1 &
PIDS+=($!)
sleep 3

echo "[ucs-ursim] dispatching the segment + injecting a contact wrench mid-creep"
PYTHONPATH="$REPO" python3 "$HERE/fault_inject.py" --plan "$PLAN" \
  --inject-after-s 2.0 >"$RUNDIR/inject.log" 2>&1 &
PIDS+=($!)
# run execute_plan --exact on the segment (the real dispatcher)
timeout 60 ros2 run --prefix "" true 2>/dev/null || true
python3 "$REPO/cams/cellmap/tools/execute_plan.py" "$PLAN" \
  --limits /home/manishsaini/firstmate/data/ur10-fork-pick-real-v1/teach-002/replay_joint_limits.yaml \
  --exact >"$RUNDIR/execute.log" 2>&1 || true
sleep 3

echo "[ucs-ursim] asserting the guard tripped and cancelled the goal"
PYTHONPATH="$REPO" python3 "$HERE/assert_ursim.py" --trip "$TRIP_OUT" \
  --execute-log "$RUNDIR/execute.log"
RC=$?
echo "[ucs-ursim] result rc=$RC (logs in $RUNDIR)"
exit $RC
