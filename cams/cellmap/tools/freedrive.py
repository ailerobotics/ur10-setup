#!/usr/bin/env python3
"""
Freedrive the UR10 from the workstation — host, domain 0.

WHY this exists: the pendant's own Freedrive button requires a hand on the
teach pendant, and holding that button KILLS the external-control program
(RUNBOOK §4), which drops /joint_states mid-capture — exactly what a hand-eye
calibration must not lose. Driving freedrive through the ROS
`freedrive_mode_controller` keeps the driver's RTDE/primary connections up, so
TF and /joint_states stay live while the captain hand-guides the arm.

Two things are needed and BOTH are easy to get wrong:

  1. The controller must own the command interfaces, so
     `scaled_joint_trajectory_controller` has to be deactivated in the SAME
     switch_controller call. Activating freedrive while the trajectory
     controller still holds the interfaces fails silently-ish and the arm
     stays rigid.
  2. `enable_freedrive_mode` is a DEADMAN, not a latch: the controller drops
     back out of freedrive if it stops hearing True. So this publishes at
     --rate Hz for as long as it runs, and sends one False on the way out.

Preconditions (checked, with the exact remedy printed):
  * robot RUNNING + NORMAL — read from the DASHBOARD (port 29999), never from
    /io_and_status_controller/robot_mode: those topics are latched-only and
    happily serve a stale value forever (observed 2026-09-03: topic said
    POWER_OFF/ROBOT_EMERGENCY_STOP while the robot was IDLE/NORMAL).
  * /joint_states genuinely LIVE — verified by encoder JITTER, not by rate.
    A dropped RTDE link keeps publishing at 125 Hz with frozen values
    (observed twice); zero jitter across a second of samples means the feed
    is stale and everything downstream would be silently wrong.

  python3 freedrive.py on      # switch + hold enable until Ctrl-C
  python3 freedrive.py off     # drop freedrive, hand control back
"""
import argparse
import socket
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController

ROBOT_IP = '192.168.0.1'
FREEDRIVE = 'freedrive_mode_controller'
TRAJ = 'scaled_joint_trajectory_controller'
ENABLE_TOPIC = f'/{FREEDRIVE}/enable_freedrive_mode'


def dashboard(*cmds, ip=ROBOT_IP, timeout=5.0):
    """Ask the robot itself. The dashboard is the only non-latched state source."""
    out = []
    s = socket.create_connection((ip, 29999), timeout=timeout)
    s.settimeout(timeout)
    try:
        s.recv(4096)
        for c in cmds:
            s.sendall((c + '\n').encode())
            out.append(s.recv(4096).decode(errors='replace').strip())
    finally:
        s.close()
    return out


def check_robot_ready():
    try:
        mode, safety = dashboard('robotmode', 'safetystatus')
    except OSError as e:
        return False, f'dashboard unreachable ({e}) - check `ping -I enp5s0 192.168.0.1`'
    if 'RUNNING' not in mode:
        return False, (f'{mode}; the arm is not started. At the pendant: power on and '
                       'RELEASE BRAKES (or `power on` + `brake release` on port 29999), '
                       'then `systemctl --user restart ur10-driver` - a mode change drops RTDE.')
    if 'NORMAL' not in safety:
        return False, f'{safety}; clear the stop at the pendant before freedrive.'
    return True, f'{mode}, {safety}'


def check_joints_live(node, secs=2.0, discovery=10.0):
    """Live means JITTERING. Rate alone is not evidence: a dead RTDE link
    republishes the last packet at a perfect 125 Hz.

    The measuring window only starts once the FIRST message lands. DDS matching
    takes a second or two, and a fixed window that begins at subscribe time
    reports 'driver down' for a perfectly healthy 125 Hz feed."""
    buf = []
    sub = node.create_subscription(
        JointState, '/joint_states', lambda m: buf.append(list(m.position)), 50)
    t0 = time.time()
    while not buf and time.time() - t0 < discovery:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not buf:
        node.destroy_subscription(sub)
        return False, (f'no /joint_states within {discovery:.0f}s of subscribing - '
                       'driver down? `systemctl --user status ur10-driver`')
    buf.clear()                                   # measure only the settled window
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if len(buf) < 20:
        return False, f'only {len(buf)} /joint_states in {secs}s - feed stalling'
    jitter = np.degrees(np.ptp(np.array(buf), axis=0)).max() * 1000.0
    if jitter <= 0.0:
        return False, ('/joint_states is FROZEN (0.000 mdeg jitter over '
                       f'{len(buf)} samples) - the RTDE link is down and the values are '
                       'stale. Fix: `systemctl --user restart ur10-driver`.')
    return True, f'{len(buf)} samples, {jitter:.3f} mdeg jitter'


def switch(node, activate, deactivate):
    cli = node.create_client(SwitchController, '/controller_manager/switch_controller')
    if not cli.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('controller_manager/switch_controller not available')
    req = SwitchController.Request()
    req.activate_controllers = activate
    req.deactivate_controllers = deactivate
    req.strictness = SwitchController.Request.BEST_EFFORT
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    if fut.result() is None or not fut.result().ok:
        raise RuntimeError(f'switch_controller failed (activate={activate} deactivate={deactivate})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('action', choices=['on', 'off'])
    ap.add_argument('--rate', type=float, default=4.0, help='Hz for the deadman enable')
    a = ap.parse_args()

    rclpy.init()
    node = Node('freedrive_ctl')
    pub = node.create_publisher(Bool, ENABLE_TOPIC, 10)

    if a.action == 'off':
        for _ in range(5):
            pub.publish(Bool(data=False))
            time.sleep(0.05)
        switch(node, [TRAJ], [FREEDRIVE])
        print('freedrive OFF - scaled_joint_trajectory_controller active, arm holds position')
        rclpy.shutdown()
        return 0

    ok, why = check_robot_ready()
    print(f'robot: {why}')
    if not ok:
        print('REFUSING to engage freedrive.', file=sys.stderr)
        rclpy.shutdown()
        return 2
    ok, why = check_joints_live(node)
    print(f'joint feed: {why}')
    if not ok:
        print('REFUSING to engage freedrive on a stale feed.', file=sys.stderr)
        rclpy.shutdown()
        return 2

    switch(node, [FREEDRIVE], [TRAJ])
    print(f'freedrive ON - holding enable at {a.rate} Hz. Ctrl-C to release.')
    print('the arm is now hand-guidable; expect dashboard robotmode BACKDRIVE while it is moved.')
    period = 1.0 / a.rate
    try:
        while rclpy.ok():
            pub.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            pub.publish(Bool(data=False))
            time.sleep(0.05)
        try:
            switch(node, [TRAJ], [FREEDRIVE])
            print('\nfreedrive released, trajectory controller back')
        except Exception as e:  # noqa: BLE001 - report, never leave the user guessing
            print(f'\nWARNING: could not restore {TRAJ}: {e}', file=sys.stderr)
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
