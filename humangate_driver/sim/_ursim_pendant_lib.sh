#!/usr/bin/env bash
# Shared helpers to bring a PENDANT-mode URSim CB3 up and down: URSim with the
# External Control URCap installed and the External Control .urp LOADED + PLAYING on
# the (simulated) pendant, and the ur_robot_driver connected NON-headless. Sourced by
# the pendant pause investigator and the pendant gate e2e so the ~40-line bring-up
# lives in exactly one place.
#
# Isolation (task brief): URSim on its OWN docker network with gateway 192.168.56.1
# (the URCap's built-in default host IP), no GPU, reachable only via its
# docker-internal IP (no published ports); driver reverse_ip:=192.168.56.1; everything
# on ROS_DOMAIN_ID=82. Teardown is by PID (driver by process group) + own
# container/network/temp-dir only — it never touches anything it did not create.
#
# Caller must set: PLIB_HERE (the sim/ dir), PLIB_LOGDIR. May override PLIB_NET /
# PLIB_CONT / PLIB_SUBNET / PLIB_URSIM_IP / PLIB_GATEWAY / PLIB_IMAGE. The bring-up
# exports PLIB_LAUNCH_PID (driver process-group leader) and PLIB_WORK (temp mounts).

PLIB_SUBNET="${PLIB_SUBNET:-192.168.56.0/24}"
PLIB_URSIM_IP="${PLIB_URSIM_IP:-192.168.56.101}"
PLIB_GATEWAY="${PLIB_GATEWAY:-192.168.56.1}"
PLIB_IMAGE="${PLIB_IMAGE:-universalrobots/ursim_cb3:3.15.8}"
PLIB_LAUNCH_PID=""
PLIB_WORK=""

pendant_down() {
  echo "[pendant_lib] teardown (driver by process group; own container/network only)"
  [ -n "$PLIB_LAUNCH_PID" ] && kill -INT -"$PLIB_LAUNCH_PID" 2>/dev/null && sleep 4
  [ -n "$PLIB_LAUNCH_PID" ] && kill -9 -"$PLIB_LAUNCH_PID" 2>/dev/null
  docker rm -f "$PLIB_CONT" >/dev/null 2>&1
  docker network rm "$PLIB_NET" >/dev/null 2>&1
  [ -n "$PLIB_WORK" ] && rm -rf "$PLIB_WORK"
  echo "[pendant_lib] removed container $PLIB_CONT and network $PLIB_NET"
}

# pendant_up -> 0 on a driver connected to a playing External Control program, else nonzero.
pendant_up() {
  if ! docker image inspect "$PLIB_IMAGE" >/dev/null 2>&1; then
    echo "[pendant_lib] BLOCKED: image $PLIB_IMAGE not present."; return 3
  fi
  local urcap_src
  urcap_src="$(ls /opt/ros/*/share/ur_robot_driver/resources/externalcontrol-*.urcap 2>/dev/null | head -1)"
  if [ -z "$urcap_src" ]; then
    echo "[pendant_lib] BLOCKED: External Control URCap not found in ur_robot_driver resources."; return 3
  fi
  PLIB_WORK="$(mktemp -d /tmp/hg_pendant.XXXXXX)"
  mkdir -p "$PLIB_WORK/urcaps" "$PLIB_WORK/programs"
  cp "$urcap_src" "$PLIB_WORK/urcaps/$(basename "${urcap_src%.urcap}").jar"
  cp "$PLIB_HERE/ursim_pendant/programs/"* "$PLIB_WORK/programs/"

  docker network inspect "$PLIB_NET" >/dev/null 2>&1 || \
    docker network create --subnet="$PLIB_SUBNET" "$PLIB_NET" >/dev/null
  docker rm -f "$PLIB_CONT" >/dev/null 2>&1
  echo "[pendant_lib] starting URSim ($PLIB_IMAGE) at $PLIB_URSIM_IP (no GPU, own network $PLIB_SUBNET)"
  docker run --rm -d --name "$PLIB_CONT" --net "$PLIB_NET" --ip "$PLIB_URSIM_IP" \
    -e ROBOT_MODEL=UR10 -v "$PLIB_WORK/urcaps:/urcaps" -v "$PLIB_WORK/programs:/ursim/programs" \
    "$PLIB_IMAGE" >/dev/null

  echo "[pendant_lib] waiting for the simulator dashboard + RTDE..."
  local i
  for i in $(seq 1 60); do
    if bash -c "exec 3<>/dev/tcp/$PLIB_URSIM_IP/29999" 2>/dev/null && \
       bash -c "exec 3<>/dev/tcp/$PLIB_URSIM_IP/30004" 2>/dev/null; then break; fi
    sleep 2
  done
  python3 - "$PLIB_URSIM_IP" <<'PY'
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

  echo "[pendant_lib] launching ur_robot_driver -> $PLIB_URSIM_IP (NON-headless, reverse_ip=$PLIB_GATEWAY)"
  setsid ros2 launch ur_robot_driver ur10.launch.py robot_ip:="$PLIB_URSIM_IP" \
      reverse_ip:="$PLIB_GATEWAY" use_mock_hardware:=false headless_mode:=false launch_rviz:=false \
      controller_spawner_timeout:=60 > "$PLIB_LOGDIR/driver.log" 2>&1 &
  PLIB_LAUNCH_PID=$!

  echo "[pendant_lib] waiting for /speed_scaling_state_broadcaster/speed_scaling..."
  local ok=0
  for i in $(seq 1 50); do
    ros2 topic list 2>/dev/null | grep -q "/speed_scaling_state_broadcaster/speed_scaling" && { ok=1; break; }
    sleep 2
  done
  [ "$ok" != "1" ] && { echo "[pendant_lib] FAILED: speed scaling never came up"; tail -30 "$PLIB_LOGDIR/driver.log"; return 1; }
  sleep 3

  echo "[pendant_lib] loading + playing the External Control program (as an operator would)"
  python3 - "$PLIB_URSIM_IP" <<'PY'
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
  echo "[pendant_lib] waiting for the External Control reverse connection..."
  for i in $(seq 1 45); do
    grep -q "Robot connected to reverse interface" "$PLIB_LOGDIR/driver.log" 2>/dev/null && { echo "  connected after ${i}x2s"; return 0; }
    sleep 2
  done
  echo "[pendant_lib] FAILED: reverse interface never connected"; return 1
}
