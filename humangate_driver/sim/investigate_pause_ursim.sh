#!/usr/bin/env bash
# Investigate whether dashboard PAUSE/PLAY freezes-and-continues a running
# scaled-JTC trajectory on CB3. TWO modes (the pendant mode was added by the
# pendant-pause task and EXTENDS — does not replace — the original headless probe):
#
#   MODE=headless (default) — the driver runs HEADLESS (this cell today,
#     ur-bringup.sh: headless_mode:=true): no dashboard-loaded program, so pause has
#     nothing to pause. Runs on ROS_DOMAIN_ID=81. This reproduces PAUSE_RESUME_FINDING.md.
#
#   MODE=pendant — URSim has the External Control URCap installed and an External
#     Control .urp LOADED and PLAYING on the (simulated) pendant, and the driver runs
#     NON-headless (headless_mode:=false). This is the "load the program on the pendant
#     and pause it" test. Runs on ROS_DOMAIN_ID=82 (the pendant-pause task's domain).
#     The URCap's built-in DEFAULT host IP is 192.168.56.1, so the URSim network's
#     gateway MUST be 192.168.56.1 (subnet 192.168.56.0/24) for the loaded program to
#     connect back to the driver with no installation edit; reverse_ip:=192.168.56.1.
#
# ACTION (pendant only; passed to the probe): pause_play (default), pause_only,
#   foreign_play, stop_instead, pause_no_program — see _pause_probe.py.
#
# Isolation (task brief): URSim on its OWN docker network, no GPU, reachable only via
# its docker-internal IP (no published ports); teardown by PID + own container/network
# only. It never touches ur10-driver or any process or container it did not create.
set -o pipefail

MODE="${MODE:-headless}"
ACTION="${ACTION:-pause_play}"
DURATION="${DURATION:-40}"
AMP="${AMP:-2.0}"

if [ "$MODE" = "pendant" ]; then
  export ROS_DOMAIN_ID=82
  NET=hg_pendant_net; SUBNET=192.168.56.0/24; URSIM_IP=192.168.56.101; GATEWAY=192.168.56.1
  CONT=hg_pendant_ursim
  PAUSE_HOLD="${PAUSE_HOLD:-5}"
  LOGSUB=ursim-pendant-pause
else
  export ROS_DOMAIN_ID=81
  NET=hg_pause_net; SUBNET=192.168.99.0/24; URSIM_IP=192.168.99.2
  CONT=hg_pause_ursim
  PAUSE_HOLD="${PAUSE_HOLD:-8}"
  LOGSUB=ursim-pause
fi
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
LOGDIR="${LOGDIR:-$REPO/data/human-gate-sim-v1/$LOGSUB}"
mkdir -p "$LOGDIR"
IMAGE=universalrobots/ursim_cb3:3.15.8

# Pendant mode needs a writable copy of the URCap + the External Control program set
# (URSim mutates /ursim/programs), so we never let it touch the committed originals.
WORK=""
LAUNCH_PID=""
cleanup() {
  echo "[investigate_pause:$MODE] cleanup (driver by process group; own container/network only)"
  [ -n "$LAUNCH_PID" ] && kill -INT -"$LAUNCH_PID" 2>/dev/null && sleep 4
  [ -n "$LAUNCH_PID" ] && kill -9 -"$LAUNCH_PID" 2>/dev/null
  docker rm -f "$CONT" >/dev/null 2>&1
  docker network rm "$NET" >/dev/null 2>&1
  [ -n "$WORK" ] && rm -rf "$WORK"
  echo "[investigate_pause:$MODE] removed container $CONT and network $NET"
}
trap cleanup EXIT INT TERM

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[investigate_pause:$MODE] BLOCKED: image $IMAGE not present."; exit 3
fi

DOCKER_MOUNTS=()
if [ "$MODE" = "pendant" ]; then
  URCAP_SRC="$(ls /opt/ros/*/share/ur_robot_driver/resources/externalcontrol-*.urcap 2>/dev/null | head -1)"
  if [ -z "$URCAP_SRC" ]; then
    echo "[investigate_pause:pendant] BLOCKED: External Control URCap not found in ur_robot_driver resources."; exit 3
  fi
  WORK="$(mktemp -d /tmp/hg_pendant.XXXXXX)"
  mkdir -p "$WORK/urcaps" "$WORK/programs"
  cp "$URCAP_SRC" "$WORK/urcaps/$(basename "${URCAP_SRC%.urcap}").jar"
  cp "$HERE/ursim_pendant/programs/"* "$WORK/programs/"
  DOCKER_MOUNTS=(-v "$WORK/urcaps:/urcaps" -v "$WORK/programs:/ursim/programs")
  echo "[investigate_pause:pendant] URCap: $URCAP_SRC ; programs: $HERE/ursim_pendant/programs"
fi

docker network inspect "$NET" >/dev/null 2>&1 || \
  docker network create --subnet="$SUBNET" "$NET" >/dev/null
docker rm -f "$CONT" >/dev/null 2>&1
echo "[investigate_pause:$MODE] starting URSim ($IMAGE) at $URSIM_IP (no GPU, own network $SUBNET)"
docker run --rm -d --name "$CONT" --net "$NET" --ip "$URSIM_IP" \
  -e ROBOT_MODEL=UR10 "${DOCKER_MOUNTS[@]}" "$IMAGE" >/dev/null

echo "[investigate_pause:$MODE] waiting for the simulator dashboard + RTDE..."
for i in $(seq 1 60); do
  if bash -c "exec 3<>/dev/tcp/$URSIM_IP/29999" 2>/dev/null && \
     bash -c "exec 3<>/dev/tcp/$URSIM_IP/30004" 2>/dev/null; then break; fi
  sleep 2
done

echo "[investigate_pause:$MODE] powering on + releasing brakes"
python3 - "$URSIM_IP" <<'PY'
import socket, sys, time
ip = sys.argv[1]
def cmd(s, c):
    s.sendall((c+"\n").encode()); time.sleep(0.5); return s.recv(4096).decode(errors="replace").strip()
s = socket.create_connection((ip, 29999), timeout=10); time.sleep(0.5)
try: s.recv(4096)
except Exception: pass
cmd(s, "power on"); time.sleep(4); cmd(s, "brake release"); time.sleep(6)
print("  robotmode:", cmd(s, "robotmode"), "| safetymode:", cmd(s, "safetymode"))
s.close()
PY

if [ "$MODE" = "pendant" ]; then
  echo "[investigate_pause:pendant] launching ur_robot_driver -> $URSIM_IP (NON-headless, reverse_ip=$GATEWAY)"
  setsid ros2 launch ur_robot_driver ur10.launch.py robot_ip:="$URSIM_IP" \
      reverse_ip:="$GATEWAY" use_mock_hardware:=false headless_mode:=false launch_rviz:=false \
      controller_spawner_timeout:=60 > "$LOGDIR/driver.log" 2>&1 &
else
  echo "[investigate_pause:headless] launching ur_robot_driver -> $URSIM_IP (HEADLESS, matches ur-bringup.sh)"
  setsid ros2 launch ur_robot_driver ur10.launch.py robot_ip:="$URSIM_IP" \
      use_mock_hardware:=false headless_mode:=true launch_rviz:=false \
      controller_spawner_timeout:=60 > "$LOGDIR/driver.log" 2>&1 &
fi
LAUNCH_PID=$!

echo "[investigate_pause:$MODE] waiting for /speed_scaling_state_broadcaster/speed_scaling..."
ok=0
for i in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -q "/speed_scaling_state_broadcaster/speed_scaling"; then ok=1; break; fi
  sleep 2
done
if [ "$ok" != "1" ]; then
  echo "[investigate_pause:$MODE] FAILED: speed scaling broadcaster never came up"; tail -30 "$LOGDIR/driver.log"; exit 1
fi
echo "[investigate_pause:$MODE] driver + broadcasters up; dashboard services:"
ros2 service list 2>/dev/null | grep -i dashboard | tee "$LOGDIR/dashboard_services.txt" >/dev/null
sleep 3

if [ "$MODE" = "pendant" ]; then
  echo "[investigate_pause:pendant] loading + playing the External Control program (as an operator would)"
  python3 - "$URSIM_IP" <<'PY'
import socket, sys, time
ip = sys.argv[1]
def cmd(s, c, w=1.2):
    s.sendall((c+"\n").encode()); time.sleep(w); return s.recv(4096).decode(errors="replace").strip()
s = socket.create_connection((ip, 29999), timeout=10); time.sleep(0.6)
try: s.recv(4096)
except Exception: pass
print("  load:", cmd(s, "load external_control.urp"))
print("  play:", cmd(s, "play", 2.0)); time.sleep(1)
print("  state:", cmd(s, "programState"))
s.close()
PY
  echo "[investigate_pause:pendant] waiting for the External Control reverse connection..."
  for i in $(seq 1 45); do
    grep -q "Robot connected to reverse interface" "$LOGDIR/driver.log" 2>/dev/null && { echo "  reverse interface connected after ${i}x2s"; break; }
    sleep 2
  done
fi

echo "[investigate_pause:$MODE] running pause/resume probe (action=$ACTION, pause-hold=${PAUSE_HOLD}s)..."
timeout $(python3 -c "print(int(${DURATION}*2 + ${PAUSE_HOLD} + 120))") \
  python3 "$HERE/_pause_probe.py" --mode "$MODE" --action "$ACTION" \
    --pause-hold "$PAUSE_HOLD" --duration "$DURATION" --amp "$AMP" \
    2>"$LOGDIR/probe.err" | tee "$LOGDIR/pause_investigation.txt"
PRC=${PIPESTATUS[0]}
echo "[investigate_pause:$MODE] --- driver.log tail ---" >> "$LOGDIR/pause_investigation.txt"
tail -40 "$LOGDIR/driver.log" >> "$LOGDIR/pause_investigation.txt"

echo "[investigate_pause:$MODE] analysing (assert the finding still holds)"
python3 "$HERE/assert_pause_investigation.py" --file "$LOGDIR/pause_investigation.txt" --mode "$MODE" \
    | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
echo "[investigate_pause:$MODE] logs in $LOGDIR (probe rc=$PRC, assert rc=$RC)"
exit $RC
