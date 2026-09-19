#!/usr/bin/env python3
"""Helper for investigate_pause_ursim.sh: does dashboard PAUSE/PLAY give a true
freeze-and-continue of a running scaled-JTC trajectory on CB3?

Two modes, one probe (the pendant mode was added by the pendant-pause task and
extends — does not replace — the original headless probe):

  * ``--mode headless`` (default): the driver injects the external-control script
    over the primary interface, so there is NO dashboard-loaded PolyScope program.
    Dashboard pause/play have nothing to act on. This is how ``ur-bringup.sh`` runs
    the cell today; the finding is that pause/play are rejected (PAUSE_RESUME_FINDING.md).
  * ``--mode pendant``: URSim has the External Control URCap installed and an
    External Control ``.urp`` program LOADED and PLAYING on the (simulated) pendant
    (see the shell wrapper). Now dashboard pause acts on a real loaded program.
    This is the "load the program on the pendant and pause it" test.

Not a standalone tool — invoked by the shell wrapper once URSim + the driver are
up (task: "pause and resume the robot program"). It:

  1. (pendant) waits for a PLAYING program + reverse connection + an ACTIVE
     scaled_joint_trajectory_controller before doing anything, and queries the
     dashboard loaded-program / program state;
  2. sends ONE long, slow, MONOTONIC-pan trajectory FROM the current pose (zero
     initial path error) to the scaled_joint_trajectory_controller (monotonic so a
     real freeze is unambiguous: pan advances steadily and stops dead if paused);
  3. runs the chosen ``--action`` against the moving arm and measures the dashboard
     reply, the time until motion freezes, the stopping distance, whether
     /joint_states keeps flowing (reverse connection alive), the applied speed
     scaling during the hold, the JTC controller state, and whether the
     FollowJointTrajectory goal stays active or aborts;
  4. holds, then (for pause_play / foreign_play) calls play and measures whether
     the SAME goal continues the SAME trajectory from where it froze and succeeds;
  5. prints a machine-readable RESULT: line the wrapper/asserter consume.

Actions (``--action``):
  * ``pause_play``        pause, hold, play — the freeze-and-continue test;
  * ``pause_only``        pause and hold, never play (does it stay frozen / does the
                          goal time out?) — durability + "gate dies while paused";
  * ``foreign_play``      pause, then play WITHOUT the gate wanting resume — models
                          someone pressing play on the pendant while STOP is demanded;
  * ``stop_instead``      dashboard ``stop`` (not pause) — models the external-control
                          program being stopped rather than paused (terminal);
  * ``pause_no_program``  stop the program first, then pause — models pause commanded
                          when nothing is running.

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

try:
    from controller_manager_msgs.srv import ListControllers
except Exception:                                    # noqa: BLE001
    ListControllers = None

J = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FJT = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
JTC_NAME = 'scaled_joint_trajectory_controller'
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
        self.stop_cli = self.create_client(Trigger, '/dashboard_client/stop')
        self.state_cli = self.create_client(GetProgramState, '/dashboard_client/program_state')
        self.loaded_cli = self.create_client(GetLoadedProgram, '/dashboard_client/get_loaded_program')
        self.running_cli = self.create_client(IsProgramRunning, '/dashboard_client/program_running')
        self.lc_cli = (self.create_client(ListControllers, '/controller_manager/list_controllers')
                       if ListControllers is not None else None)
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

    def jtc_state(self):
        """State string of the scaled_joint_trajectory_controller, or None."""
        if self.lc_cli is None or not self.lc_cli.wait_for_service(timeout_sec=3.0):
            return None
        fut = self.lc_cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return None
        for c in fut.result().controller:
            if c.name == JTC_NAME:
                return c.state
        return 'absent'

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

    def wait_playing_active(self, timeout=90.0):
        """Pendant precondition: PLAYING program + reverse connection + ACTIVE JTC.
        Presses play once, then waits. Returns True when the JTC is active."""
        self.trig(self.play_cli, 'play')
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(1.0)
            if len(self.js) >= 6 and self.jtc_state() == 'active':
                return True
        return False


def build_ramp(q0pan, base, duration=40.0, amp=2.0, npts=80):
    """A MONOTONIC pan ramp with a trapezoidal velocity profile (ramp up, long
    CONSTANT-VELOCITY middle, ramp down). Mid-trajectory the arm moves at a
    clearly-non-zero speed, so a real pause shows as pan flat-lining dead — never
    confused with a sweep turning point or a slow eased tail. Point 0 is the
    current pose, so there is zero initial path error."""
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
    ap.add_argument('--mode', choices=['headless', 'pendant'], default='headless')
    ap.add_argument('--action',
                    choices=['pause_play', 'pause_only', 'foreign_play',
                             'stop_instead', 'pause_no_program'],
                    default='pause_play')
    ap.add_argument('--pause-hold', type=float, default=8.0)
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--amp', type=float, default=2.0)
    ap.add_argument('--settle', type=float, default=8.0)
    a = ap.parse_args()

    rclpy.init()
    n = Probe()
    out = {'mode': a.mode, 'action': a.action, 'pause_hold': a.pause_hold}

    for _ in range(40):
        n.spin(0.5)
        if len(n.js) >= 6:
            break
    if len(n.js) < 6:
        print('RESULT ' + json.dumps({**out, 'error': 'no joint_states'})); return 1

    # Pendant precondition: a PLAYING program with the reverse interface up and the
    # JTC active. Headless has neither a loaded program nor this gating.
    if a.mode == 'pendant':
        if not n.wait_playing_active(timeout=90.0):
            print('RESULT ' + json.dumps({**out, 'error': 'pendant: JTC never active '
                                          '(program not playing / reverse not connected)',
                                          'jtc_state': n.jtc_state(),
                                          'dashboard': n.dashboard_status()}))
            return 1
        out['jtc_state_before'] = n.jtc_state()

    if not n.fjt.wait_for_server(timeout_sec=15.0):
        print('RESULT ' + json.dumps({**out, 'error': 'FJT unavailable'})); return 1

    out['dashboard_before_motion'] = n.dashboard_status()
    base = [n.js.get(j, 0.0) for j in J]
    q0pan = base[0]

    g = FollowJointTrajectory.Goal()
    g.trajectory = build_ramp(q0pan, base, a.duration, a.amp)
    sgf = n.fjt.send_goal_async(g)
    rclpy.spin_until_future_complete(n, sgf, timeout_sec=15.0)
    gh = sgf.result()
    if gh is None or not gh.accepted:
        print('RESULT ' + json.dumps({**out, 'error': 'goal rejected'})); return 1
    result_fut = gh.get_result_async()
    out['goal_accepted'] = True

    # let it reach the constant-velocity middle and confirm it is clearly moving
    n.spin(a.settle)
    out['dashboard_during_motion'] = n.dashboard_status()
    out['speed_before_pause'] = round(n.measure_speed() or 0.0, 5)
    out['clearly_moving_before_pause'] = (out['speed_before_pause'] > 2 * REST_RAD_S)
    pan_at_pause = n.q0()
    out['pan_at_pause'] = round(pan_at_pause, 5)
    out['scaling_before_pause'] = n.scaling
    out['jtc_state_before_pause'] = n.jtc_state()

    # ---- the STOP action ------------------------------------------------ #
    # pause_no_program: stop first so nothing is running when we pause.
    if a.action == 'pause_no_program':
        out['pre_stop_call'] = n.trig(n.stop_cli, 'stop')
        n.spin(2.0)

    t_pause = time.monotonic()
    if a.action == 'stop_instead':
        out['pause_call'] = n.trig(n.stop_cli, 'stop')       # dashboard STOP, not pause
    else:
        out['pause_call'] = n.trig(n.pause_cli, 'pause')

    # A TRUE freeze = pan stops advancing (delta < 0.01 rad over a 1.5 s window)
    # within a few seconds of the pause call. Measured on pan advance, not a speed
    # threshold, so an eased tail cannot masquerade as a freeze.
    freeze_t = None
    while time.monotonic() - t_pause < 5.0:
        p0 = n.q0()
        n.spin(1.5)
        if abs(n.q0() - p0) < 0.01:
            freeze_t = time.monotonic() - t_pause
            break
    out['froze_after_pause'] = freeze_t is not None
    out['time_to_freeze_s'] = round(freeze_t, 3) if freeze_t is not None else None
    out['stopping_distance_rad'] = round(abs(n.q0() - pan_at_pause), 5)
    out['goal_aborted_by_pause'] = result_fut.done()
    out['goal_code_at_pause'] = (result_fut.result().result.error_code
                                 if result_fut.done() else None)

    # ---- HOLD ----------------------------------------------------------- #
    js_at_pause = n.js_count
    pan_frozen = n.q0()
    scal_samples = []
    held = 0.0
    step = min(2.0, max(0.5, a.pause_hold))
    while held < a.pause_hold:
        n.spin(step)
        held += step
        if n.scaling is not None:
            scal_samples.append(n.scaling)
    out['pan_drift_during_pause'] = round(abs(n.q0() - pan_frozen), 5)
    out['joint_states_flowing_during_pause'] = (n.js_count - js_at_pause) > 5
    out['scaling_during_pause_min'] = min(scal_samples) if scal_samples else None
    out['scaling_during_pause_last'] = n.scaling
    out['jtc_state_during_pause'] = n.jtc_state()
    out['dashboard_during_pause'] = n.dashboard_status()
    out['goal_done_during_pause'] = result_fut.done()
    out['goal_code_during_pause'] = (result_fut.result().result.error_code
                                     if result_fut.done() else None)

    # ---- PLAY / resume -------------------------------------------------- #
    if a.action in ('pause_play', 'foreign_play'):
        out['play_call'] = n.trig(n.play_cli, 'play')
        n.spin(3.0)
        out['speed_after_play'] = round(n.measure_speed() or 0.0, 5)
        out['pan_advanced_since_pause'] = round(n.q0() - pan_at_pause, 5)
    else:
        # pause_only / stop_instead / pause_no_program: never resume.
        out['play_call'] = {'ok': False, 'msg': 'not attempted (action=%s)' % a.action}
        out['speed_after_play'] = 0.0
        out['pan_advanced_since_pause'] = round(n.q0() - pan_at_pause, 5)

    cap = a.duration * 2 + a.pause_hold + 20.0
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

    out['jtc_state_after'] = n.jtc_state()
    out['dashboard_after'] = n.dashboard_status()

    # interpretation for the reader. A TRUE freeze-and-continue = pause executed,
    # the arm froze on its path, the SAME goal handle SURVIVED the pause (never
    # aborted, never resent), motion RESUMED on play, and that same goal ran to
    # SUCCESS (code 0) with no replan. The definitive signal is `final_goal_succeeded`
    # on the one goal handle held throughout — not how far the pan happened to have
    # advanced at the single sample taken 3 s after play.
    out['pause_executed'] = bool(out['pause_call'].get('ok'))
    out['play_executed'] = bool(out['play_call'].get('ok'))
    out['motion_resumed_after_play'] = (out.get('speed_after_play', 0.0) > 2 * REST_RAD_S
                                        or out.get('pan_advanced_since_pause', 0.0) > 0.02)
    out['true_freeze_and_continue'] = (
        a.action == 'pause_play'
        and out['pause_executed'] and out['froze_after_pause']
        and out['goal_aborted_by_pause'] is False
        and out['play_executed'] and out['motion_resumed_after_play']
        and out['final_goal_succeeded'])
    print('RESULT ' + json.dumps(out))
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
