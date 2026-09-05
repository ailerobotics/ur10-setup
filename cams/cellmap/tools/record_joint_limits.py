#!/usr/bin/env python3
"""
Record the UR10's REAL per-joint travel while the captain hand-sweeps each
joint in freedrive, and write a limits file the cuMotion pneumatic flow
consumes. Host, domain 0.

WHY this exists: cumotion/pneumatic_limits.yaml pins its windows to a
reference PARK POSE and its own header says it MUST be regenerated when the
arm is re-parked (make_limited_urdf.py re-reads the pose). The arm has been
re-parked, so those windows no longer bracket where the arm actually is, and
execute_plan.py --limits would abort a perfectly good plan. Rather than guess
a new window, the captain drives each joint by hand to its real physical
extreme and this tool records what is actually there.

SAFETY — this tool NEVER commands motion. It only listens to /joint_states.
Freedrive is enabled separately and out of band (tools/freedrive.py on) and is
driven by the CAPTAIN'S HAND. This process is a passive recorder.

WHAT IT DOES
  * Verifies the feed is genuinely LIVE by encoder JITTER, never by rate: a
    dropped RTDE link republishes the last packet at a perfect 125 Hz with
    frozen values (recorded twice in this cell — see cams/cellmap/README.md
    "Freedrive from the workstation"). Live looks like ~4-9 mdeg of jitter at
    rest. It REFUSES to record a frozen feed, and flags a mid-session freeze
    live in the table so a wasted sweep is caught immediately.
  * Tracks a single global per-joint [min, max] across the whole session (the
    captain sweeps the joints in sequence, so global extremes are exactly the
    per-joint travel); prints a live table at --rate Hz so the pane can be
    watched.
  * On Ctrl-C (SIGINT) writes the limits file in the EXACT format
    make_limited_urdf.py and execute_plan.py read (reference_pose_rad +
    window_deg), with each window = measured [min, max] shrunk by --margin-deg,
    encoded as reference = the measured MIDPOINT and window_deg = half the
    measured range minus the margin. That makes the effective window exactly
    [measured_min + margin, measured_max - margin]. The raw measured extremes,
    the margin, the date and the jitter evidence are recorded in the file.

  IMPORTANT for make_limited_urdf.py: these are ABSOLUTE measured joint
  positions, not a window around the live pose, so it MUST be run --frozen
  (which uses reference_pose_rad from the file). The default live mode would
  re-centre the windows on wherever the arm happens to be.

USAGE
  # read-only dry run: verify the feed is live and print current values, write
  # NOTHING (arm still, no freedrive) — the Phase A acceptance check
  python3 record_joint_limits.py --check

  # record a captain sweep; Ctrl-C to stop and write the file
  python3 record_joint_limits.py --out cumotion/measured_joint_limits.yaml
"""
import argparse
import collections
import copy
import datetime as dt
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Order the consumers (make_limited_urdf.py, execute_plan.py) use.
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

HERE = os.path.dirname(os.path.abspath(__file__))
# repo root is three levels up from cams/cellmap/tools/
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
DEFAULT_OUT = os.path.join(REPO, 'cumotion', 'measured_joint_limits.yaml')


def jitter_mdeg(samples):
    """Peak-to-peak per joint across a set of position rows, in millidegrees,
    taking the worst joint. This is the liveness signal: a frozen feed
    republishes one packet and this is exactly 0."""
    if len(samples) < 2:
        return 0.0
    arr = np.asarray(samples, dtype=float)
    return float(np.degrees(np.ptp(arr, axis=0)).max() * 1000.0)


def update_extremes(mins, maxs, sample):
    """Fold one 6-vector sample into running per-joint min/max (in place).
    mins/maxs are lists (or None to initialise). Returns (mins, maxs)."""
    s = list(sample)
    if mins is None:
        return list(s), list(s)
    for i, v in enumerate(s):
        if v < mins[i]:
            mins[i] = v
        if v > maxs[i]:
            maxs[i] = v
    return mins, maxs


def build_limits(mins_rad, maxs_rad, margin_deg, jitter_note, method,
                 date=None):
    """Build the limits-file dict in the consumers' format.

    window = reference_pose_rad +- radians(window_deg). To make the effective
    window exactly [min + margin, max - margin] we set the reference to the
    measured MIDPOINT and window_deg to half the measured range minus the
    margin. Round-trips through execute_plan.py's lo/hi computation and
    make_limited_urdf.py --frozen by construction.

    Raises ValueError if any joint's measured range is <= 2*margin (the shrink
    would invert the window) — the caller decides what to do with that.
    """
    date = date or dt.date.today().isoformat()
    ref = {}
    win = {}
    measured_rad = {}
    measured_deg = {}
    degenerate = []
    for i, j in enumerate(JOINTS):
        lo, hi = float(mins_rad[i]), float(maxs_rad[i])
        mid = 0.5 * (lo + hi)
        half_deg = math.degrees(0.5 * (hi - lo))
        w = half_deg - margin_deg
        if w <= 0.0:
            degenerate.append((j, hi - lo))
        ref[j] = round(mid, 6)
        win[j] = round(w, 4)
        measured_rad[j] = {'min': round(lo, 6), 'max': round(hi, 6)}
        measured_deg[j] = {'min': round(math.degrees(lo), 3),
                           'max': round(math.degrees(hi), 3)}
    if degenerate:
        names = ', '.join(f'{j} ({math.degrees(r):.1f} deg)' for j, r in degenerate)
        raise ValueError(
            f'measured range <= 2*margin ({2*margin_deg:.1f} deg) for: {names}; '
            'the margin would invert the window. Re-sweep those joints or lower '
            '--margin-deg.')
    return {
        'reference_pose_rad': ref,
        'window_deg': win,
        # Provenance — additive keys the consumers ignore (they read only
        # reference_pose_rad and window_deg). Kept structured so the file can
        # be regenerated and audited.
        'measured': {
            'source': 'captain freedrive hand-sweep',
            'method': method,
            'date': date,
            'margin_deg': margin_deg,
            'jitter_evidence': jitter_note,
            'extremes_rad': measured_rad,
            'extremes_deg': measured_deg,
        },
    }


HEADER = """\
# Measured UR10 joint limits — captain freedrive hand-sweep.
#
# HOW: the captain hand-drove each joint to its real physical extreme in ROS
# freedrive (tools/freedrive.py) while tools/record_joint_limits.py recorded
# the running per-joint [min, max] from /joint_states. No motion was ever
# commanded; the feed was verified LIVE by encoder jitter, not by rate (a
# dropped RTDE link republishes a frozen packet at a perfect 125 Hz).
#
# FORMAT — identical to cumotion/pneumatic_limits.yaml so the same consumers
# read it: make_limited_urdf.py (limits -> limited URDF/XRDF) and
# execute_plan.py --limits (per-waypoint window guard). Each window is
#   reference_pose_rad +- radians(window_deg)
# and here reference is the MIDPOINT of the measured [min, max] and window_deg
# is half the measured range minus the safety margin, so the effective window
# is exactly [measured_min + margin, measured_max - margin]. The raw measured
# extremes and the margin are under `measured:` below.
#
# make_limited_urdf.py MUST be run with --frozen against this file: these are
# ABSOLUTE measured joint positions, not a window around the live pose, so the
# default (live-pose) mode would centre the windows in the wrong place.
#
# REGENERATE whenever the line is re-routed or the arm re-parked — the real
# reachable travel changes. Re-run the recorder (see cams/cellmap/README.md
# "Measuring real joint limits").
#
"""


def write_limits_file(path, limits):
    body = yaml.safe_dump(limits, sort_keys=False, default_flow_style=False)
    header = HEADER
    if limits.get('declared'):
        notes = ['#',
                 '# CAPTAIN-DECLARED OVERRIDES (NOT hand-swept) — a window below was',
                 '# widened by captain decision to cover a known-required travel the sweep',
                 '# did not reach. Provenance for each is under `declared:`:']
        for d in limits['declared']:
            w = d['window_deg']
            ex = d.get('extended_from_measured_raw_deg')
            exs = (f'; extended from measured raw {ex["min"]}..{ex["max"]} deg'
                   if ex else '')
            notes.append(f'#   {d["joint"]} = {w["min"]}..{w["max"]} deg, declared '
                         f'{d["date"]} — {d["reason"]}{exs}')
        notes.append('#')
        header = HEADER + '\n'.join(notes) + '\n'
    with open(path, 'w') as f:
        f.write(header)
        f.write(body)


def normalize_joint(name):
    """Accept 'shoulder_lift' or 'shoulder_lift_joint'; return the full name."""
    name = name.strip()
    if name in JOINTS:
        return name
    if name + '_joint' in JOINTS:
        return name + '_joint'
    raise ValueError(f'unknown joint {name!r} (expected one of {JOINTS})')


def merge_limits(existing, session_mins, session_maxs, joints, jitter_note, date=None):
    """Targeted re-sweep merge: EXTEND only `joints` to cover the new session
    extremes, keep every other joint exactly as it was.

    The re-swept joints' raw window becomes the UNION of the file's existing
    measured extremes and this session's extremes (min of mins, max of maxs) —
    a re-sweep only ever widens, so a partial re-sweep can never silently
    narrow a joint that was already measured further. Windows are recomputed
    from those merged extremes using the FILE's own margin, so all joints keep
    one consistent margin. Untouched joints (reference, window, provenance) are
    copied through byte-for-byte. The merge is recorded under
    measured.resweeps so the history survives.

    Raises ValueError on an unknown joint, a missing/foreign existing file, or a
    merged range <= 2*margin (which would invert the window).
    """
    date = date or dt.date.today().isoformat()
    if 'reference_pose_rad' not in existing or 'window_deg' not in existing \
            or 'measured' not in existing:
        raise ValueError('the file to merge into is not a measured-limits file '
                         '(missing reference_pose_rad / window_deg / measured)')
    margin = float(existing['measured']['margin_deg'])
    out = copy.deepcopy(existing)
    ex_rad = out['measured'].setdefault('extremes_rad', {})
    ex_deg = out['measured'].setdefault('extremes_deg', {})
    changed = {}
    for j in joints:
        i = JOINTS.index(j)
        old_lo = float(ex_rad[j]['min'])
        old_hi = float(ex_rad[j]['max'])
        lo = min(old_lo, float(session_mins[i]))
        hi = max(old_hi, float(session_maxs[i]))
        half_deg = math.degrees(0.5 * (hi - lo))
        w = half_deg - margin
        if w <= 0.0:
            raise ValueError(f'{j}: merged range {math.degrees(hi-lo):.1f} deg <= 2*margin '
                             f'({2*margin:.1f}); the margin would invert the window')
        out['reference_pose_rad'][j] = round(0.5 * (lo + hi), 6)
        out['window_deg'][j] = round(w, 4)
        ex_rad[j] = {'min': round(lo, 6), 'max': round(hi, 6)}
        ex_deg[j] = {'min': round(math.degrees(lo), 3), 'max': round(math.degrees(hi), 3)}
        changed[j] = {'old_deg': [round(math.degrees(old_lo), 3), round(math.degrees(old_hi), 3)],
                      'new_deg': ex_deg[j]}
    out['measured'].setdefault('resweeps', []).append({
        'date': date,
        'joints': list(joints),
        'jitter_evidence': jitter_note,
        'margin_deg': margin,
        'changed': changed,
    })
    return out, changed


def declare_limits(existing, joint, tmin_rad, tmax_rad, slack_deg, reason, date=None):
    """CAPTAIN-DECLARED override for ONE joint: set its window to
    [travel_min - slack, travel_max + slack] to COVER a known-required travel
    the hand-sweep did not reach, WITHOUT re-measuring.

    This is NOT a measurement. It is a deliberate, reasoned widening the captain
    takes responsibility for (e.g. shoulder_lift must cover a validated demo
    plan whose swing exceeds what the freedrive sweep reached). It is recorded
    separately from `measured:`, under a top-level `declared:` list, so the
    provenance of every joint's window stays unambiguous — "5 measured, 1
    declared" is readable straight from the file.

    ONLY EVER WIDENS. The applied window is the UNION of the joint's current
    window and [travel_min - slack, travel_max + slack], so a one-sided need
    (extend the floor to cover a low demo swing) keeps the measured bound on the
    other side instead of collapsing it. A declaration can therefore never
    silently narrow a joint's reach.

    Only the named joint's reference_pose_rad + window_deg change; every other
    joint (reference, window, and the whole `measured:` block) is copied through
    byte-for-byte. The named joint's own measured.extremes_* are LEFT AS
    MEASURED — they still document what the hand-sweep actually reached; the
    `declared:` entry records the prior window, what it was widened to, the
    slack, and why. The consumers (make_limited_urdf.py, execute_plan.py) read
    only reference_pose_rad + window_deg, so they pick up the declared window and
    ignore the extra provenance.

    Raises ValueError on a missing/foreign file, an unknown joint, or travel
    min >= max.
    """
    date = date or dt.date.today().isoformat()
    if 'reference_pose_rad' not in existing or 'window_deg' not in existing \
            or 'measured' not in existing:
        raise ValueError('the file to declare into is not a measured-limits file '
                         '(missing reference_pose_rad / window_deg / measured)')
    if joint not in JOINTS:
        raise ValueError(f'unknown joint {joint!r} (expected one of {JOINTS})')
    tmin, tmax = float(tmin_rad), float(tmax_rad)
    if tmin >= tmax:
        raise ValueError(f'declared travel min ({math.degrees(tmin):.3f} deg) must be '
                         f'below max ({math.degrees(tmax):.3f} deg)')
    slack = math.radians(slack_deg)
    out = copy.deepcopy(existing)
    # current effective window of this joint (what "measured bound" means here)
    cur_ref = float(out['reference_pose_rad'][joint])
    cur_half = math.radians(float(out['window_deg'][joint]))
    cur_lo, cur_hi = cur_ref - cur_half, cur_ref + cur_half
    # UNION — widen only the side(s) the required travel needs
    lo = min(cur_lo, tmin - slack)
    hi = max(cur_hi, tmax + slack)
    half_deg = math.degrees(0.5 * (hi - lo))
    prior = out['measured'].get('extremes_deg', {}).get(joint)
    prior_deg = dict(prior) if prior else None    # a COPY, not a YAML alias
    out['reference_pose_rad'][joint] = round(0.5 * (lo + hi), 6)
    out['window_deg'][joint] = round(half_deg, 4)
    entry = {
        'joint': joint,
        'source': 'captain-declared (NOT hand-swept)',
        'date': date,
        'reason': reason,
        'covers_travel_deg': {'min': round(math.degrees(tmin), 3),
                              'max': round(math.degrees(tmax), 3)},
        'slack_deg': slack_deg,
        'prior_window_deg': {'min': round(math.degrees(cur_lo), 3),
                             'max': round(math.degrees(cur_hi), 3)},
        'window_deg': {'min': round(math.degrees(lo), 3),
                       'max': round(math.degrees(hi), 3)},
        'extended_from_measured_raw_deg': prior_deg,
    }
    # idempotent: a re-declare of the same joint replaces the prior entry
    out['declared'] = [d for d in out.get('declared', []) if d.get('joint') != joint]
    out['declared'].append(entry)
    return out, entry


class Recorder(Node):
    def __init__(self):
        super().__init__('record_joint_limits')
        self.latest = None            # last full 6-vector (JOINTS order)
        self.recent = collections.deque(maxlen=400)  # (t, 6-vec) for freeze check
        self.create_subscription(JointState, '/joint_states', self._cb, 50)

    def _cb(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(j in m for j in JOINTS):
            v = [float(m[j]) for j in JOINTS]
            self.latest = v
            self.recent.append((time.time(), v))

    def gate_live(self, secs, discovery, min_jitter_mdeg):
        """Block until the feed is proven live by jitter, or fail with the
        exact remedy. The measuring window starts only once the first message
        lands (DDS matching takes a second or two)."""
        buf = []
        t0 = time.time()
        while not self.recent and time.time() - t0 < discovery:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.recent:
            return False, (f'no /joint_states within {discovery:.0f}s of subscribing — '
                           'driver down? `systemctl --user status ur10-driver`'), 0.0
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None:
                buf.append(self.latest)
        if len(buf) < 20:
            return False, f'only {len(buf)} /joint_states in {secs:.0f}s — feed stalling', 0.0
        j = jitter_mdeg(buf)
        if j <= min_jitter_mdeg:
            return False, (f'/joint_states is FROZEN ({j:.3f} mdeg jitter over {len(buf)} '
                           'samples <= threshold) — the RTDE link is down and the values '
                           'are stale. Fix: `systemctl --user restart ur10-driver`. '
                           'REFUSING to record a frozen feed.'), j
        return True, f'{len(buf)} samples, {j:.3f} mdeg jitter', j

    def recent_jitter_mdeg(self, window_s=1.0):
        """Jitter over roughly the last window_s of samples — for the live
        freeze indicator in the table."""
        if not self.recent:
            return None
        now = time.time()
        rows = [v for (t, v) in self.recent if now - t <= window_s]
        return jitter_mdeg(rows) if len(rows) >= 2 else None


def print_table(mins, maxs, latest, recent_j, min_jitter_mdeg, elapsed):
    cur = latest if latest is not None else [float('nan')] * 6
    if recent_j is None:
        feed = 'feed: (waiting)'
    elif recent_j <= min_jitter_mdeg:
        feed = f'feed: !! FROZEN ({recent_j:.3f} mdeg) — STOP, do not trust this !!'
    else:
        feed = f'feed: LIVE ({recent_j:.2f} mdeg jitter)'
    lines = [f'  {feed}   elapsed {elapsed:6.1f}s   (Ctrl-C to stop and write)',
             f'  {"joint":20s} {"current":>10} {"min":>10} {"max":>10} {"range":>9}  (deg)']
    for i, j in enumerate(JOINTS):
        lo = math.degrees(mins[i]) if mins else float('nan')
        hi = math.degrees(maxs[i]) if maxs else float('nan')
        rng = (hi - lo) if mins else float('nan')
        lines.append(f'  {j:20s} {math.degrees(cur[i]):10.2f} {lo:10.2f} '
                     f'{hi:10.2f} {rng:9.2f}')
    # redraw in place: move cursor up over the previous block
    sys.stdout.write('\033[2K' + '\n\033[2K'.join(lines) + '\n')
    sys.stdout.write(f'\033[{len(lines)}A')
    sys.stdout.flush()


def _do_declare(a):
    """--declare: a pure file edit — no ROS, no recording. Applies a
    captain-declared window override to ONE joint in the existing --out file."""
    if not a.reason.strip():
        print('--declare requires --reason (why the window is widened by decision)',
              file=sys.stderr)
        return 2
    try:
        jspec, rng = a.declare.split('=', 1)
        joint = normalize_joint(jspec)
        parts = [p for p in rng.split(',') if p.strip()]
        if len(parts) != 2:
            raise ValueError('need exactly MIN,MAX')
        tmin_d, tmax_d = float(parts[0]), float(parts[1])
    except ValueError as e:
        print(f'--declare expects JOINT=MIN,MAX in deg '
              f'(e.g. shoulder_lift=-174.2,-12.2): {e}', file=sys.stderr)
        return 2
    if not os.path.exists(a.out):
        print(f'--declare target {a.out} does not exist; nothing to declare into',
              file=sys.stderr)
        return 2
    try:
        existing = yaml.safe_load(open(a.out))
    except Exception as e:                                   # noqa: BLE001
        print(f'--declare target {a.out} is not readable YAML: {e}', file=sys.stderr)
        return 2
    try:
        limits, entry = declare_limits(existing, joint, math.radians(tmin_d),
                                       math.radians(tmax_d), a.declare_slack_deg,
                                       a.reason.strip())
    except ValueError as e:
        print(f'REFUSING to declare: {e}', file=sys.stderr)
        return 1
    write_limits_file(a.out, limits)
    w = entry['window_deg']
    print(f'declared override for {joint} written to {a.out}')
    print(f'  window {w["min"]}..{w["max"]} deg (covers travel '
          f'{entry["covers_travel_deg"]["min"]}..{entry["covers_travel_deg"]["max"]} '
          f'+- {a.declare_slack_deg} deg slack)')
    print(f'  source: {entry["source"]}; reason: {entry["reason"]}')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=DEFAULT_OUT,
                    help=f'limits file to write on Ctrl-C (default: {DEFAULT_OUT})')
    ap.add_argument('--margin-deg', type=float, default=5.0,
                    help='shrink each measured [min,max] by this many deg on each '
                         'side (default 5)')
    ap.add_argument('--rate', type=float, default=2.0, help='live-table refresh Hz')
    ap.add_argument('--jitter-secs', type=float, default=2.0,
                    help='window for the startup liveness (jitter) gate')
    ap.add_argument('--discovery-secs', type=float, default=10.0,
                    help='how long to wait for the first /joint_states')
    ap.add_argument('--min-jitter-mdeg', type=float, default=0.0,
                    help='refuse the feed if worst-joint jitter is <= this over the '
                         'gate window (default 0.0 = refuse only a truly frozen feed, '
                         'matching freedrive.py)')
    ap.add_argument('--check', action='store_true',
                    help='READ-ONLY: verify the feed is live, print current values, '
                         'write nothing, exit. For the arm-still dry run.')
    ap.add_argument('--merge', action='store_true',
                    help='TARGETED RE-SWEEP: merge this session into the EXISTING --out '
                         'file, updating ONLY the joints named in --joints (their windows '
                         'are EXTENDED to cover the new extremes; every other joint is kept '
                         'byte-for-byte). Requires --joints.')
    ap.add_argument('--joints', default='',
                    help='merge mode only: comma-separated joints to update, short or full '
                         'name (e.g. "shoulder_lift,shoulder_pan").')
    ap.add_argument('--declare', default=None, metavar='JOINT=MIN,MAX',
                    help='CAPTAIN-DECLARED override (no rig, no recording): widen ONE joint '
                         "in the EXISTING --out file to the window [MIN-slack, MAX+slack] deg "
                         'so it COVERS a known-required travel the hand-sweep did not reach. '
                         'MIN,MAX are the travel to cover, in deg (e.g. '
                         'shoulder_lift=-174.2,-12.2). Records provenance under `declared:` '
                         '(source=captain-declared, NOT hand-swept); every other joint stays '
                         'measured, byte-identical. Requires --reason.')
    ap.add_argument('--reason', default='',
                    help='declare mode only: why this window is widened by decision.')
    ap.add_argument('--declare-slack-deg', type=float, default=1.0,
                    help='declare mode only: slack added on each side of the covered travel '
                         '(default 1.0).')
    a = ap.parse_args(argv)

    if a.declare:
        return _do_declare(a)

    merge_joints = None
    existing_for_merge = None
    if a.merge:
        if not a.joints.strip():
            print('--merge requires --joints (which joints were re-swept)', file=sys.stderr)
            return 2
        try:
            merge_joints = [normalize_joint(x) for x in a.joints.split(',') if x.strip()]
        except ValueError as e:
            print(str(e), file=sys.stderr)
            return 2
        if not os.path.exists(a.out):
            print(f'--merge target {a.out} does not exist; nothing to merge into',
                  file=sys.stderr)
            return 2
        try:
            existing_for_merge = yaml.safe_load(open(a.out))
            for j in merge_joints:                       # prove it is a measured-limits file
                existing_for_merge['measured']['extremes_rad'][j]
        except Exception as e:                           # noqa: BLE001
            print(f'--merge target {a.out} is not a valid measured-limits file: {e}',
                  file=sys.stderr)
            return 2
        print(f'MERGE mode: re-sweeping {merge_joints}; every other joint in {a.out} '
              'is kept exactly as-is.')

    rclpy.init()
    node = Recorder()
    try:
        ok, why, jit = node.gate_live(a.jitter_secs, a.discovery_secs, a.min_jitter_mdeg)
        print(f'joint feed: {why}')
        if not ok:
            print('REFUSING: not a live feed.', file=sys.stderr)
            return 2
        cur = node.latest
        print('current joint values (deg): ' +
              '  '.join(f'{j.split("_joint")[0]}={math.degrees(cur[i]):.2f}'
                        for i, j in enumerate(JOINTS)))
        if a.check:
            print('--check: live feed verified, current values sane above. '
                  'Nothing recorded, nothing written.')
            return 0

        jitter_note = f'startup gate: {why}'
        print(f'\nRECORDING. Sweep each joint by hand to its physical limit. '
              f'Margin {a.margin_deg:.1f} deg. Ctrl-C to stop and write {a.out}\n')
        mins = maxs = None
        worst_recent = jit
        period = 1.0 / a.rate
        t_start = time.time()
        next_draw = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.latest is not None:
                    mins, maxs = update_extremes(mins, maxs, node.latest)
                now = time.time()
                if now >= next_draw:
                    rj = node.recent_jitter_mdeg()
                    if rj is not None and rj > a.min_jitter_mdeg:
                        worst_recent = min(worst_recent, rj) if worst_recent else rj
                    print_table(mins, maxs, node.latest, rj, a.min_jitter_mdeg,
                                now - t_start)
                    next_draw = now + period
        except KeyboardInterrupt:
            pass

        # move the cursor below the live table before final output
        sys.stdout.write('\n' * (len(JOINTS) + 3))
        sys.stdout.flush()
        if mins is None:
            print('no samples recorded — nothing written.', file=sys.stderr)
            return 1

        jitter_note += f'; min recent jitter while recording: {worst_recent:.3f} mdeg'
        if a.merge:
            try:
                limits, changed = merge_limits(existing_for_merge, mins, maxs,
                                               merge_joints, jitter_note)
            except ValueError as e:
                print(f'REFUSING to merge: {e}', file=sys.stderr)
                return 1
            write_limits_file(a.out, limits)
            print(f'merged re-swept joints into {a.out}')
            for j, c in changed.items():
                print(f'  {j}: window extremes {c["old_deg"]} -> '
                      f'[{c["new_deg"]["min"]}, {c["new_deg"]["max"]}] deg')
        else:
            try:
                limits = build_limits(mins, maxs, a.margin_deg, jitter_note,
                                      method='global per-joint [min,max] over the session')
            except ValueError as e:
                print(f'REFUSING to write: {e}', file=sys.stderr)
                return 1
            write_limits_file(a.out, limits)
            print(f'wrote {a.out}')
        # unified summary from the final written file (works for both modes)
        ref, win = limits['reference_pose_rad'], limits['window_deg']
        exd = limits['measured']['extremes_deg']
        print(f'{"joint":20s} {"raw min":>9} {"raw max":>9} {"win lo":>9} {"win hi":>9}  (deg)')
        for j in JOINTS:
            wlo = math.degrees(ref[j] - math.radians(win[j]))
            whi = math.degrees(ref[j] + math.radians(win[j]))
            print(f'{j:20s} {exd[j]["min"]:9.2f} {exd[j]["max"]:9.2f} {wlo:9.2f} {whi:9.2f}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
