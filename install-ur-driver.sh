#!/usr/bin/env bash
# Install the Universal Robots ROS 2 driver stack for Jazzy.
#
# Why the pin: this machine has NVIDIA's Isaac ROS apt repo enabled, which ships
# ros-jazzy-ur* as version "99.2.0-0noble". The 99.x is NVIDIA's force-win version
# convention, not a real upstream release. Meanwhile ros-jazzy-ur-description is NOT
# overridden there and would come from packages.ros.org at 3.5.1 -- so an unpinned
# install mixes an Isaac driver with an OSRF description. We pin the whole ur* stack
# to packages.ros.org so it installs as one coherent upstream set (3.8.0).
set -euo pipefail

echo "==> Pinning ros-jazzy-ur* to packages.ros.org"
tee /etc/apt/preferences.d/99-ur-from-ros2.pref >/dev/null <<'EOF'
Package: ros-jazzy-ur*
Pin: origin packages.ros.org
Pin-Priority: 1001
EOF

echo "==> apt update"
apt-get update -qq

echo "==> Verifying the pin resolved to the upstream build (expect 3.x, not 99.x)"
apt-cache policy ros-jazzy-ur-robot-driver | sed -n '1,4p'

echo "==> Installing ros-jazzy-ur"
DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-ur

echo
echo "==> Installed versions:"
dpkg-query -W -f='${Package} ${Version}\n' \
  'ros-jazzy-ur' 'ros-jazzy-ur-robot-driver' 'ros-jazzy-ur-description' \
  'ros-jazzy-ur-calibration' 'ros-jazzy-ur-moveit-config' 2>/dev/null || true

echo
echo "==> Firewall: allowing the robot to connect back (driver listens on 50001-50004)"
# ufw is active on this machine; the UR external-control architecture has the ROBOT
# open TCP connections to the PC, so inbound from the robot must be allowed.
ufw allow in on enp5s0 from 192.168.0.1 comment 'UR10 robot -> ur_robot_driver' || true
ufw status | grep -A2 "192.168.0.1" || true

echo
echo "DONE."
