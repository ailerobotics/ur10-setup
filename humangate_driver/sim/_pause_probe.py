#!/usr/bin/env python3
"""Helper for investigate_pause_ursim.sh: does dashboard PAUSE/PLAY give a true
freeze-and-continue of a running scaled-JTC trajectory on a HEADLESS CB3 driver?

Not a standalone tool — invoked by the shell wrapper once URSim + the headless
ur_robot_driver are up (task: "pause and resume the robot program"). It:

  1. queries the dashboard for the loaded program / program state (headless has no
     dashboard-loaded .urp — the driver injects the external-control script over
     the primary interface, which is why pause has nothing to pause);
  2. sends ONE long, slow, MONOTONIC-pan trajectory to the
     scaled_joint_trajectory_controller (monotonic so a real freeze is
     unambiguous: pan advances steadily and stops dead if paused);
  3. calls /dashboard_client/pause and measures the dashboard reply, the time
     until joint motion actually freezes (if it does), whether /joint_states keeps
     flowing (driver/reverse connection alive), and whether the
     FollowJointTrajectory goal stays active or aborts;
  4. holds paused, then calls /dashboard_client/play and measures whether the SAME
     goal continues the SAME trajectory from where it froze and ultimately
     succeeds;
  5. prints a machine-readable RESULT: line the wrapper/asserter consume.

No cameras, no real robot — the arming interlock keeps the whole package off real
hardware; this probe is a measurement tool, not the gate."""
import argparse
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_dashboard_msgs.srv import GetLoadedProgram, GetProgramState, IsProgramRunning

J = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FJT = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
REST_RAD_S = 0.02        # below this joint speed the arm is "frozen"


class Probe(Node):
    def __init__(self):
        super().__init__('pause_probe')
        self.js = {}
        self.js_count = 0
        self.scaling = None
        self.prog = None
        self.prog_changes = []
        self.create_subscription(JointState, '/joint_states', self._js, 50)
        self.create_subscription(
            Float64, '/speed_scaling_state_broadcaster/speed_scaling',
            lambda m: setattr(self, 'scaling', m.data), 10)
        self.create_subscription(
            Bool, '/io_and_status_controller/robot_program_running',
            self._prog, 5)
        self.pause_cli = self.create_client(Trigger, '/dashboard_client/pause')
        self.play_cli = self.create_client(Trigger, '/dashboard_client/play')
        self.state_cli = self.create_client(GetProgramState, '/dashboard_client/program_state')
        self.loaded_cli = self.create_client(GetLoadedProgram, '/dashboard_client/get_loaded_program')
        self.running_cli = self.create_client(IsProgramRunning, '/dashboard_client/program_running')
        self.fjt = ActionClient(self, FollowJointTrajectory, FJT)

    def _js(self, m):
        self.js.update(zip(m.name, m.position))
        self.js_count += 1

    def _prog(self, m):
        if self.prog != m.data:
            self.prog_changes.append((time.monotonic(), bool(m.data)))
        self.prog = bool(m.data)

    def spin(self, t):
        end = time.monotonic() + t
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def q0(self):
        return self.js.get(J[0])

    def trig(self, cli, name, timeout=6.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return {'ok': False, 'err': f'{name} service absent'}
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'err': f'{name} timed out'}
        r = fut.result()
        return {'ok': bool(r.success), 'msg': r.message}

    def query(self, cli, req, name, timeout=6.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return {'err': f'{name} absent'}
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return {'err': f'{name} timed out'}
        return fut.result()

    def dashboard_status(self):
        st = self.query(self.state_cli, GetProgramState.Request(), 'program_state')
        ld = self.query(self.loaded_cli, GetLoadedProgram.Request(), 'get_loaded_program')
        rn = self.query(self.running_cli, IsProgramRunning.Request(), 'program_running')
        return {
            'program_state': getattr(getattr(st, 'state', None), 'state', None) if not isinstance(st, dict) else st,
            'loaded_program': getattr(ld, 'program_name', None) if not isinstance(ld, dict) else ld,
            'program_running': getattr(rn, 'program_running', None) if not isinstance(rn, dict) else rn,
        }

    def measure_speed(self, window=0.4):
        p0 = dict(self.js)
        t0 = time.monotonic()
        self.spin(window)
        dt = time.monotonic() - t0
        p1 = dict(self.js)
        common = set(p0) & set(p1)
        if not common or dt <= 0:
            return None
        return max(abs(p1[j] - p0[j]) for j in common) / dt


def build_ramp(q0pan, base, duration=40.0, amp=2.0, npts=80):
    """A MONOTONIC pan ramp with a trapezoidal velocity profile (ramp up, long
    CONSTANT-VELOCITY middle, ramp down). Mid-trajectory the arm moves at a
    clearly-non-zero speed, so a real pause shows as pan flat-lining dead — never
    confused with a sweep turning point or a slow eased tail."""
    tr = JointTrajectory()
    tr.joint_names = J
    a = 0.15                                   # fraction of the ramp spent easing
    vmax = 1.0 / (1.0 - a)                     # peak normalised velocity (area = 1)

    def pos_frac(fr):
        if fr <= a:
            return vmax * fr * fr / (2 * a)
        if fr < 1 - a:
            return vmax * (a / 2 + (fr - a))
        u = fr - (1 - a)
        return vmax * (a / 2 + (1 - 2 * a) + (u - u * u / (2 * a)))

    for k in range(npts + 1):
        fr = k / npts
        pt = JointTrajectoryPoint()
        p = list(base)
        p[0] = q0pan + amp * pos_frac(fr)
        pt.positions = [float(v) for v in p]
        t = fr * duration
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        tr.points.append(pt)
    tr.points[0].velocities = [0.0] * 6
    tr.points[-1].velocities = [0.0] * 6
    tr.points[-1].accelerations = [0.0] * 6
    tr.header.stamp.sec = 0
    tr.header.stamp.nanosec = 0
    return tr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pause-hold', type=float, default=8.0)
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--amp', type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    n = Probe()
    out = {}

    for _ in range(40):
        n.spin(0.5)
        if len(n.js) >= 6:
            break
    if len(n.js) < 6:
        print('RESULT ' + json.dumps({'error': 'no joint_states'})); return 1
    if not n.fjt.wait_for_server(timeout_sec=15.0):
        print('RESULT ' + json.dumps({'error': 'FJT unavailable'})); return 1

    out['dashboard_before_motion'] = n.dashboard_status()
    base = [n.js.get(j, 0.0) for j in J]
    q0pan = base[0]

    g = FollowJointTrajectory.Goal()
    g.trajectory = build_ramp(q0pan, base, a.duration, a.amp)
    sgf = n.fjt.send_goal_async(g)
    rclpy.spin_until_future_complete(n, sgf, timeout_sec=15.0)
    gh = sgf.result()
    if gh is None or not gh.accepted:
        print('RESULT ' + json.dumps({'error': 'goal rejected'})); return 1
    result_fut = gh.get_result_async()
    out['goal_accepted'] = True

    # let it reach the constant-velocity middle and confirm it is clearly moving
    n.spin(8.0)
    out['dashboard_during_motion'] = n.dashboard_status()
    out['speed_before_pause'] = round(n.measure_speed() or 0.0, 5)
    out['clearly_moving_before_pause'] = (out['speed_before_pause'] > 2 * REST_RAD_S)
    pan_at_pause = n.q0()
    out['pan_at_pause'] = round(pan_at_pause, 5)
    out['scaling_before_pause'] = n.scaling

    # ---- PAUSE ---------------------------------------------------------- #
    # A TRUE freeze = pan stops advancing (delta < 0.01 rad over a 1.5 s window)
    # within a few seconds of the pause call. Measured on pan advance, not a speed
    # threshold, so an eased tail cannot masquerade as a freeze.
    t_pause = time.monotonic()
    out['pause_call'] = n.trig(n.pause_cli, 'pause')
    freeze_t = None
    while time.monotonic() - t_pause < 4.0:
        p0 = n.q0()
        n.spin(1.5)
        if abs(n.q0() - p0) < 0.01:
            freeze_t = time.monotonic() - t_pause
            break
    out['froze_after_pause'] = freeze_t is not None
    out['time_to_freeze_s'] = round(freeze_t, 3) if freeze_t is not None else None
    out['goal_aborted_by_pause'] = result_fut.done()

    js_at_pause = n.js_count
    pan_frozen = n.q0()
    n.spin(a.pause_hold)
    out['pan_drift_during_pause'] = round(abs(n.q0() - pan_frozen), 5)
    out['joint_states_flowing_during_pause'] = (n.js_count - js_at_pause) > 5
    out['dashboard_during_pause'] = n.dashboard_status()
    out['goal_done_during_pause'] = result_fut.done()

    # ---- PLAY ----------------------------------------------------------- #
    out['play_call'] = n.trig(n.play_cli, 'play')
    n.spin(3.0)
    out['speed_after_play'] = round(n.measure_speed() or 0.0, 5)
    out['pan_advanced_since_pause'] = round(n.q0() - pan_at_pause, 5)

    cap = min(a.duration + 15.0, 80.0)
    t_wait = time.monotonic()
    while not result_fut.done() and time.monotonic() - t_wait < cap:
        n.spin(0.5)
    if result_fut.done():
        res = result_fut.result()
        out['final_goal_result_code'] = res.result.error_code if res else None
        out['final_goal_succeeded'] = bool(res and res.result.error_code == 0)
    else:
        out['final_goal_result_code'] = None
        out['final_goal_succeeded'] = False
        gh.cancel_goal_async(); n.spin(1.0)

    # interpretation for the reader
    out['pause_executed'] = bool(out['pause_call'].get('ok'))
    out['play_executed'] = bool(out['play_call'].get('ok'))
    out['true_freeze_and_continue'] = (
        out['pause_executed'] and out['froze_after_pause'] and out['play_executed']
        and out['final_goal_succeeded'])
    print('RESULT ' + json.dumps(out))
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
