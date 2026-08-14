# UR10 — ROS 2 Setup & Operations

CB3-generation UR10 (PolyScope 3.15.8) direct-cabled to this workstation, driven by
`ur_robot_driver` on ROS 2 Jazzy. First brought up and motion-verified **2026-08-13**.

```
UR10 CB3 controller ───ethernet cable─── enp5s0 (this PC)
   192.168.0.1/24                        192.168.0.2/24  (NM profile "ur10")
        │
        └── dials back to PC: 50001 (commands) · 50003 (trajectories) · 50004 (script)
        └── PC dials robot:   29999 (dashboard) · 30001-30003 (primary/RT) · 30004 (RTDE, 125 Hz)
```

## Network

| Side | Setting |
|---|---|
| Robot (pendant → Setup Robot → Network) | Static `192.168.0.1` / `255.255.255.0` / gw `192.168.0.2` |
| PC (`nmcli` profile `ur10` on `enp5s0`) | `192.168.0.2/24` + link-local fallback, `ipv4.never-default yes` |

- **Test reachability with `ping -I enp5s0 192.168.0.1`** — a plain `ping` false-positives
  through wifi (a building device also answers on 192.168.0.1 via the office router).
- The pendant's network page saying **"not connected" is cosmetic** on a direct cable (its
  check wants internet/DNS). Judge the link by data, not by that page.
- Don't press **Apply** on the pendant network page casually — it restarts the robot's
  network stack and kills the running control script every time.

## Software

- `ros-jazzy-ur` 3.8.0 stack, **pinned to packages.ros.org** by
  `/etc/apt/preferences.d/99-ur-from-ros2.pref`. Reason: the NVIDIA Isaac apt repo overrides
  most `ros-jazzy-ur*` packages with a fake `99.2.0` version but not `ur-description`, so an
  unpinned install mixes incompatible halves. Keep the pin when upgrading.
- Kinematics calibration extracted from this specific arm: `~/ur10/ur10_calibration.yaml`
  (re-extract with `ros2 launch ur_calibration calibration_correction.launch.py
  robot_ip:=192.168.0.1 target_filename:=$HOME/ur10/ur10_calibration.yaml`).
- `ros2 control` CLI verb is not installed; query controllers via
  `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"`.

## Running the driver

The driver runs as a **transient systemd user service** so it survives every terminal/session:

```bash
# start (after PC reboot, or if not running)
systemd-run --user --unit=ur10-driver -p Restart=on-failure -p RestartSec=5 ~/ur10/ur-bringup.sh

systemctl --user status  ur10-driver   # health
systemctl --user restart ur10-driver   # ALWAYS after rebooting the robot controller
systemctl --user stop    ur10-driver
journalctl --user -u ur10-driver -f    # live logs
```

`ur-bringup.sh` launches `ur_control.launch.py` with: `ur_type:=ur10 robot_ip:=192.168.0.1
reverse_ip:=192.168.0.2 kinematics_params_file:=~/ur10/ur10_calibration.yaml
headless_mode:=true launch_rviz:=false`. Headless mode = no URCap needed on the pendant.
`reverse_ip` is pinned because this PC is multi-homed (wifi/tailscale/docker).

Transient means it does **not** autostart after a PC reboot — rerun the `systemd-run` line,
or promote it to a permanent unit file (+ `loginctl enable-linger`) if autostart is wanted.

## Health checklist

```bash
ping -I enp5s0 -c2 192.168.0.1                      # link
ss -tn | grep ':50001'                              # command channel (must show ESTAB)
ros2 topic hz /joint_states                         # ~125 Hz
ros2 topic echo --once /speed_scaling_state_broadcaster/speed_scaling   # > 0
printf 'running\nquit\n' | nc 192.168.0.1 29999     # "Program running: true"
```

## Moving the arm

Speed slider must be nonzero (pendant slider, or
`ros2 service call /io_and_status_controller/set_speed_slider ur_msgs/srv/SetSpeedSliderFraction "{speed_slider_fraction: 0.3}"`).

Minimal verified example — `nudge_wrist3.py` in this folder rotates wrist_3 by +0.1 rad and
back over 8 s via the `scaled_joint_trajectory_controller` FollowJointTrajectory action:

```bash
source /opt/ros/jazzy/setup.bash && python3 ~/ur10/nudge_wrist3.py
```

First motion 2026-08-13: `error_code=0`, return repeatability 8e-5 rad.

## Recovery playbook

| Symptom | Fix |
|---|---|
| Commands ignored; no ESTAB on port 50001; pendant popped "check IP/connection" | Control script died (network blip). `ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}"` then reactivate: `ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [scaled_joint_trajectory_controller, friction_model_controller], strictness: 1}"` |
| **Robot controller was rebooted**; driver logs spam "Background reading is not running"; scaling reads 0; slider writes no-op | The driver never survives a robot reboot. `systemctl --user restart ur10-driver` |
| **Controller wedged**: dashboard `running`→false, `play`→"Failed to execute", injected scripts (even popups) don't run, speed pinned at 0 | Reboot the controller from the pendant. Then remote-init works: `printf 'power on\nquit\n' | nc 192.168.0.1 29999`, wait ~6 s, `printf 'brake release\nquit\n' | nc 192.168.0.1 29999` (~15 s to RUNNING) |
| Controllers inactive after a rough start (spawner died at launch) | `ros2 run controller_manager spawner --controller-manager-timeout 20 <names from log>` |
| Second driver instance errors with "speed_slider_mask … controlled by another RTDE client" | Only one driver may run. `ps -eo pid,cmd | grep '[r]os2_control_node'` and kill the extra. |

## Files here

| File | Purpose |
|---|---|
| `RUNBOOK.md` | **The recurring commands** — start here day-to-day |
| `report.html` | Commissioning report (open in a browser; mirrors the runbook) |
| `ur-bringup.sh` | Launch the driver (used by the systemd unit) |
| `ur-detect.sh` | Ping + dashboard probe; detects CB3 vs e-Series, writes `ur_type.txt` |
| `install-ur-driver.sh` | Pinned apt install — defeats the Isaac-repo 99.x override |
| `ur10_calibration.yaml` | This arm's factory kinematics (extracted) |
| `motion_test.py` | Self-verifying motion test (±0.05 rad, asserts return < 0.005 rad) |
| `nudge_wrist3.py` | First-light motion script (±0.10 rad) |
| `driver.log` | Log from first-light day (local only, gitignored) |
