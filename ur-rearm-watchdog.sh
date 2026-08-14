#!/usr/bin/env bash
# Auto-rearm watchdog for the UR10 external-control program.
#
# On a CB3, the ExternalControl program dies whenever the teach button is held
# (hand-guiding), the network blips, or the pendant interferes. Every death
# deactivates the trajectory controllers, and the next MoveIt Execute or
# trajectory goal is rejected. This watchdog detects the death and re-arms
# automatically:
#
#   program_running false for >5 s  AND  robot RUNNING + safety NORMAL
#     -> resend_robot_program + reactivate controllers, then 20 s cooldown
#
# Resending does NOT move the arm; the program only holds position. Runs as
# systemd user unit "ur10-rearm" (see RUNBOOK §4).
set -o pipefail  # no -u: ROS setup.bash reads unset vars
source /opt/ros/jazzy/setup.bash
ROBOT_IP="${1:-192.168.0.1}"

down_since=0
while true; do
  sleep 2
  running=$(timeout 6 ros2 topic echo /io_and_status_controller/robot_program_running --once 2>/dev/null | awk '/data:/{print $2}')
  if [ "$running" = "true" ] || [ -z "$running" ]; then
    down_since=0
    continue
  fi

  now=$(date +%s)
  [ "$down_since" -eq 0 ] && { down_since=$now; continue; }
  [ $((now - down_since)) -lt 5 ] && continue

  # program down >5s — only act if the robot itself is healthy
  modes=$( { sleep 0.7; printf 'robotmode\nsafetymode\nquit\n'; sleep 1.2; } | timeout 6 nc "$ROBOT_IP" 29999 2>/dev/null )
  if ! grep -q "Robotmode: RUNNING" <<<"$modes" || ! grep -q "Safetymode: NORMAL" <<<"$modes"; then
    echo "[rearm] program down but robot not ready ($(grep -oE 'Robotmode: [A-Z_]+|Safetymode: [A-Z_]+' <<<"$modes" | tr '\n' ' ')) — waiting"
    down_since=$now
    continue
  fi

  echo "[rearm] external control down >5s, robot healthy — re-arming"
  # Orphaned-program guard: if the ROBOT believes a program is running while the
  # driver sees none (no reverse connection), that stale instance swallows the
  # resend. Stop it first — halts only the program, never moves the arm.
  if { sleep 0.7; printf 'running\nquit\n'; sleep 1; } | timeout 6 nc "$ROBOT_IP" 29999 2>/dev/null | grep -q "Program running: true"; then
    echo "[rearm] orphaned program on controller — stopping it first"
    { sleep 0.7; printf 'stop\nquit\n'; sleep 1; } | timeout 6 nc "$ROBOT_IP" 29999 >/dev/null 2>&1
    sleep 2
  fi
  timeout 12 ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}" >/dev/null 2>&1
  sleep 4
  timeout 12 ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [scaled_joint_trajectory_controller, friction_model_controller], strictness: 1}" >/dev/null 2>&1
  verify=$(timeout 6 ros2 topic echo /io_and_status_controller/robot_program_running --once 2>/dev/null | awk '/data:/{print $2}')
  echo "[rearm] result: program_running=$verify"
  down_since=0
  sleep 20   # cooldown — never spam the robot
done
