# UR10 Runbook — recurring commands

Every command you'll run again and again, in the order you usually need them.
(Also rendered on `report.html`.)

Robot: `192.168.0.1` (CB3) · PC: `192.168.0.2` on `enp5s0` · ROS 2 Jazzy

> Every `ros2` command needs `source /opt/ros/jazzy/setup.bash` first.

## 1 · Is the robot reachable?

```bash
# ALWAYS bind the interface — plain ping false-positives via wifi
ping -I enp5s0 -c3 192.168.0.1

# Robot state via dashboard (no ROS needed)
printf 'robotmode\nsafetymode\nprogramState\nquit\n' | nc -w5 192.168.0.1 29999

# All UR ports in one look
~/ur10/ur-detect.sh
```

## 2 · Driver service (systemd user unit)

```bash
# start (after PC reboot — the unit is transient and does not autostart)
systemd-run --user --unit=ur10-driver -p Restart=on-failure -p RestartSec=5 ~/ur10/ur-bringup.sh

systemctl --user status  ur10-driver     # health
systemctl --user restart ur10-driver     # ALWAYS after rebooting the robot controller
systemctl --user stop    ur10-driver
journalctl --user -u ur10-driver -f      # live logs
```

## 3 · Health checklist (run after any (re)connect)

```bash
ping -I enp5s0 -c2 192.168.0.1                       # 1. link
ss -tn | grep ':50001'                               # 2. command channel — must show ESTAB
ros2 topic hz /joint_states                          # 3. ~125 Hz
ros2 topic echo --once /io_and_status_controller/robot_program_running   # 4. data: true
ros2 topic echo --once /speed_scaling_state_broadcaster/speed_scaling    # 5. > 0
```

## 4 · The #1 fix — robot reconnected / motion ignored

Cable was replugged, robot rebooted, or pendant popped "check IP/connection"?
The external-control script on the robot is dead. Re-arm it (no motion caused):

```bash
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}"
```

If controllers went inactive too:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [scaled_joint_trajectory_controller, friction_model_controller], strictness: 1}"
```

Robot controller itself was rebooted? The driver never survives that:

```bash
systemctl --user restart ur10-driver
```

## 5 · Wedged CB3 controller (dashboard ignores everything)

Symptoms: `running` → false, `play` → "Failed to execute", speed pinned at 0.
Reboot the controller from the pendant, then init remotely:

```bash
printf 'power on\nquit\n'      | nc 192.168.0.1 29999   # wait ~6 s
printf 'brake release\nquit\n' | nc 192.168.0.1 29999   # ~15 s to RUNNING
```

## 6 · Motion test

```bash
# VISIBLE test — wrist_1 tilts the whole wrist block ~8.6°; use this to see it move
source /opt/ros/jazzy/setup.bash && python3 ~/ur10/motion_test_visible.py

# minimal tests — wrist_3 = flange rotation, near-INVISIBLE without a tool mounted
source /opt/ros/jazzy/setup.bash && python3 ~/ur10/motion_test.py    # 0.05 rad
source /opt/ros/jazzy/setup.bash && python3 ~/ur10/nudge_wrist3.py   # 0.10 rad
```

Speed slider must be nonzero first:

```bash
ros2 service call /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction "{speed_slider_fraction: 0.3}"
```

## 7 · Controllers

```bash
# ros2controlcli is not installed — use the service:
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
```

## 8 · One-time / rare

```bash
# Re-extract kinematics calibration from the arm
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.0.1 target_filename:=$HOME/ur10/ur10_calibration.yaml

# Reinstall driver stack (keeps the Isaac-repo pin)
sudo bash ~/ur10/install-ur-driver.sh

# PC network profile (already persistent; for reference)
nmcli connection add type ethernet con-name ur10 ifname enp5s0 \
  ipv4.method manual ipv4.addresses "192.168.0.2/24,169.254.101.1/16" \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect yes connection.autoconnect-priority 10
```
