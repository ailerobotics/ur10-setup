#!/usr/bin/env python3
"""
Offline tests for record_joint_limits.py — NO live rig, NO ROS spin.

Covers, per the task's acceptance:
  * frozen-feed refusal (the jitter gate's exact predicate)
  * running per-joint min/max tracking on a synthetic /joint_states trace
  * margin application (reference = midpoint, window_deg = half-range - margin)
  * the degenerate case (range <= 2*margin is refused, never inverted)
  * the written file round-tripping through BOTH consumers' own parsers:
      - execute_plan.py's lo/hi window arithmetic
      - make_limited_urdf.py --frozen, run for real against the committed
        ur10_calibrated.urdf / ur10_map.xrdf (frozen mode needs no ROS)
  * --merge after --declare: a re-sweep can only widen, never narrow a
    declared bound, and the `declared:` entry always describes the applied window
  * a multi-line --reason still yields a parseable file
  * --declare (and a clean refusal to record) with NO ROS stack importable

Run standalone: python3 test_record_joint_limits.py
Or under pytest: pytest test_record_joint_limits.py
"""
import math
import os
import re
import subprocess
import sys
import tempfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import record_joint_limits as rec  # noqa: E402

REPO = rec.REPO
CUMOTION = os.path.join(REPO, 'cumotion')
JOINTS = rec.JOINTS


# --- the jitter gate predicate (frozen-feed refusal) -----------------------

def test_frozen_feed_has_zero_jitter():
    frozen = [[0.1, -1.3, 1.5, 2.9, 4.3, -1.5]] * 200   # one packet, republished
    assert rec.jitter_mdeg(frozen) == 0.0
    # gate_live refuses when jitter <= min_jitter_mdeg; default threshold 0.0
    assert rec.jitter_mdeg(frozen) <= 0.0, 'frozen feed must be refused'


def test_live_feed_has_positive_jitter():
    base = [0.1, -1.3, 1.5, 2.9, 4.3, -1.5]
    live = []
    for k in range(200):
        wobble = math.radians(0.006) * (1 if k % 2 else -1)   # ~6 mdeg
        live.append([b + wobble for b in base])
    j = rec.jitter_mdeg(live)
    assert j > 0.0, 'live feed must pass the gate'
    assert 5.0 < j < 20.0, f'expected ~12 mdeg ptp, got {j:.3f}'


# --- running min/max tracking ----------------------------------------------

def test_update_extremes_tracks_global_min_max():
    # a synthetic sweep: each joint ramps up then down through a distinct range
    trace = []
    for k in range(-50, 51):
        f = k / 50.0                       # -1 .. 1
        trace.append([f * 2.0,             # shoulder_pan +-2.0
                      -1.3 + f * 0.8,       # shoulder_lift around -1.3
                      f * 1.4,
                      f * 2.5,
                      f * 2.0,
                      f * 3.0])
    mins = maxs = None
    for s in trace:
        mins, maxs = rec.update_extremes(mins, maxs, s)
    assert math.isclose(mins[0], -2.0, abs_tol=1e-9)
    assert math.isclose(maxs[0], 2.0, abs_tol=1e-9)
    assert math.isclose(mins[1], -1.3 - 0.8, abs_tol=1e-9)
    assert math.isclose(maxs[1], -1.3 + 0.8, abs_tol=1e-9)
    assert math.isclose(maxs[5], 3.0, abs_tol=1e-9)


def test_update_extremes_first_sample_seeds_both():
    s = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    mins, maxs = rec.update_extremes(None, None, s)
    assert mins == s and maxs == s
    assert mins is not maxs and mins is not s   # copies, not aliases


# --- margin application -----------------------------------------------------

def _measured():
    # comfortably inside stock UR10 limits (+-360 pan/lift/wrists, +-180 elbow)
    # so no clamping happens and the round-trip is exact.
    mins = [-2.0, -2.1, -1.4, -2.5, -2.0, -3.0]
    maxs = [2.0, 0.5, 1.4, 2.5, 2.0, 3.0]
    return mins, maxs


def test_build_limits_midpoint_and_window():
    mins, maxs = _measured()
    margin = 5.0
    lim = rec.build_limits(mins, maxs, margin, 'test', 'test')
    for i, j in enumerate(JOINTS):
        mid = 0.5 * (mins[i] + maxs[i])
        half_deg = math.degrees(0.5 * (maxs[i] - mins[i]))
        assert math.isclose(lim['reference_pose_rad'][j], round(mid, 6), abs_tol=1e-6)
        assert math.isclose(lim['window_deg'][j], round(half_deg - margin, 4), abs_tol=1e-4)
    # provenance is present and honest
    m = lim['measured']
    assert m['margin_deg'] == margin
    assert math.isclose(m['extremes_rad'][JOINTS[0]]['min'], -2.0, abs_tol=1e-6)


def test_build_limits_refuses_degenerate_range():
    # range 0.1 rad (~5.7 deg) < 2*margin(10 deg) -> would invert
    mins = [0.0] * 6
    maxs = [0.1] * 6
    try:
        rec.build_limits(mins, maxs, 5.0, 'test', 'test')
    except ValueError as e:
        assert 'invert' in str(e)
        return
    raise AssertionError('expected ValueError on degenerate range')


# --- round-trip through execute_plan.py's parser ---------------------------

def _write_tmp_limits(margin=5.0):
    mins, maxs = _measured()
    lim = rec.build_limits(mins, maxs, margin, 'test', 'test')
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='measured_limits_')
    os.close(fd)
    rec.write_limits_file(path, lim)
    return path, mins, maxs, margin


def test_execute_plan_parser_windows_equal_measured_minus_margin():
    path, mins, maxs, margin = _write_tmp_limits()
    try:
        # EXACTLY what execute_plan.py does (execute_plan.py:131-134)
        lim = yaml.safe_load(open(path))
        ref, win = lim['reference_pose_rad'], lim['window_deg']
        for i, j in enumerate(JOINTS):
            lo = ref[j] - math.radians(win[j])
            hi = ref[j] + math.radians(win[j])
            assert math.isclose(lo, mins[i] + math.radians(margin), abs_tol=1e-5), \
                f'{j} lo {lo} != min+margin'
            assert math.isclose(hi, maxs[i] - math.radians(margin), abs_tol=1e-5), \
                f'{j} hi {hi} != max-margin'
    finally:
        os.unlink(path)


# --- round-trip through make_limited_urdf.py --frozen (the real consumer) ---

def test_make_limited_urdf_frozen_consumes_the_file():
    urdf_in = os.path.join(CUMOTION, 'ur10_calibrated.urdf')
    xrdf_in = os.path.join(CUMOTION, 'ur10_map.xrdf')
    script = os.path.join(CUMOTION, 'make_limited_urdf.py')
    if not (os.path.exists(urdf_in) and os.path.exists(xrdf_in)):
        print('SKIP make_limited_urdf round-trip: inputs missing')
        return
    path, mins, maxs, margin = _write_tmp_limits()
    tmpdir = tempfile.mkdtemp(prefix='limurdf_')
    out_urdf = os.path.join(tmpdir, 'out.urdf')
    out_xrdf = os.path.join(tmpdir, 'out.xrdf')
    try:
        r = subprocess.run(
            [sys.executable, script, '--frozen', '--limits', path,
             '--urdf', urdf_in, '--xrdf-in', xrdf_in,
             '--out', out_urdf, '--xrdf-out', out_xrdf],
            capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, f'make_limited_urdf failed:\n{r.stdout}\n{r.stderr}'
        s = open(out_urdf).read()
        for i, j in enumerate(JOINTS):
            m = re.search(r'<joint name="' + re.escape(j) + r'" type="revolute">.*?'
                          r'<limit[^>]*lower="([-\d.eE+]+)"[^>]*upper="([-\d.eE+]+)"',
                          s, re.S)
            assert m, f'no limit written for {j}'
            lo, hi = float(m.group(1)), float(m.group(2))
            # inputs chosen inside stock range -> no clamping -> exact
            assert math.isclose(lo, mins[i] + math.radians(margin), abs_tol=1e-4), \
                f'{j} URDF lower {lo} != min+margin'
            assert math.isclose(hi, maxs[i] - math.radians(margin), abs_tol=1e-4), \
                f'{j} URDF upper {hi} != max-margin'
        # the XRDF retract config must sit at our reference (inside every window)
        xd = yaml.safe_load(open(out_xrdf))
        lim = yaml.safe_load(open(path))
        for j in JOINTS:
            assert math.isclose(float(xd['default_joint_positions'][j]),
                                round(lim['reference_pose_rad'][j], 4), abs_tol=1e-3), \
                f'{j} retract config not at reference pose'
    finally:
        os.unlink(path)
        for f in (out_urdf, out_xrdf):
            if os.path.exists(f):
                os.unlink(f)
        os.rmdir(tmpdir)


# --- targeted re-sweep MERGE mode -----------------------------------------

def test_normalize_joint_accepts_short_and_full():
    assert rec.normalize_joint('shoulder_lift') == 'shoulder_lift_joint'
    assert rec.normalize_joint('shoulder_lift_joint') == 'shoulder_lift_joint'
    try:
        rec.normalize_joint('elbow_flange')
    except ValueError:
        return
    raise AssertionError('expected ValueError on unknown joint')


def test_merge_extends_only_named_joint_keeps_others():
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base', date='2026-09-04')
    smins, smaxs = list(mins), list(maxs)
    smins[1] = -2.5   # shoulder_lift: extend below base min (-2.1)
    smaxs[1] = 1.0    # and above base max (0.5)
    merged, changed = rec.merge_limits(base, smins, smaxs, ['shoulder_lift_joint'],
                                       'resweep evidence', date='2026-09-05')
    # re-swept joint extended to the union of old and new extremes
    er = merged['measured']['extremes_rad']['shoulder_lift_joint']
    assert er['min'] == round(-2.5, 6) and er['max'] == round(1.0, 6)
    # window recomputed = union shrunk by the FILE's margin (5 deg)
    ref = merged['reference_pose_rad']['shoulder_lift_joint']
    win = merged['window_deg']['shoulder_lift_joint']
    lo = ref - math.radians(win)
    hi = ref + math.radians(win)
    assert math.isclose(lo, -2.5 + math.radians(5), abs_tol=1e-5)
    assert math.isclose(hi, 1.0 - math.radians(5), abs_tol=1e-5)
    # every OTHER joint kept byte-for-byte
    for j in JOINTS:
        if j == 'shoulder_lift_joint':
            continue
        assert merged['reference_pose_rad'][j] == base['reference_pose_rad'][j]
        assert merged['window_deg'][j] == base['window_deg'][j]
        assert merged['measured']['extremes_rad'][j] == base['measured']['extremes_rad'][j]
    # provenance recorded
    rs = merged['measured']['resweeps'][-1]
    assert rs['joints'] == ['shoulder_lift_joint'] and rs['date'] == '2026-09-05'
    assert changed['shoulder_lift_joint']['old_deg'][0] == round(math.degrees(-2.1), 3)


def test_merge_never_narrows_a_joint():
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    smins, smaxs = list(mins), list(maxs)
    smins[1], smaxs[1] = -2.0, 0.0   # NARROWER than base [-2.1, 0.5]
    merged, _ = rec.merge_limits(base, smins, smaxs, ['shoulder_lift_joint'], 'x')
    er = merged['measured']['extremes_rad']['shoulder_lift_joint']
    assert er['min'] == round(-2.1, 6) and er['max'] == round(0.5, 6), 'union must not narrow'


def test_merged_file_regenerates_limited_model():
    """Req: the merged file must stay regenerable through make_limited_urdf --frozen."""
    urdf_in = os.path.join(CUMOTION, 'ur10_calibrated.urdf')
    xrdf_in = os.path.join(CUMOTION, 'ur10_map.xrdf')
    script = os.path.join(CUMOTION, 'make_limited_urdf.py')
    if not (os.path.exists(urdf_in) and os.path.exists(xrdf_in)):
        print('SKIP merged round-trip: inputs missing')
        return
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    smins, smaxs = list(mins), list(maxs)
    smins[1], smaxs[1] = -2.6, 1.2
    merged, _ = rec.merge_limits(base, smins, smaxs, ['shoulder_lift_joint'], 'resweep')
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='merged_limits_')
    os.close(fd)
    rec.write_limits_file(path, merged)
    tmpdir = tempfile.mkdtemp(prefix='mergeurdf_')
    out_urdf = os.path.join(tmpdir, 'out.urdf')
    out_xrdf = os.path.join(tmpdir, 'out.xrdf')
    try:
        r = subprocess.run(
            [sys.executable, script, '--frozen', '--limits', path,
             '--urdf', urdf_in, '--xrdf-in', xrdf_in,
             '--out', out_urdf, '--xrdf-out', out_xrdf],
            capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, f'make_limited_urdf on merged file failed:\n{r.stdout}\n{r.stderr}'
        s = open(out_urdf).read()
        m = re.search(r'<joint name="shoulder_lift_joint" type="revolute">.*?'
                      r'<limit[^>]*lower="([-\d.eE+]+)"[^>]*upper="([-\d.eE+]+)"', s, re.S)
        lo, hi = float(m.group(1)), float(m.group(2))
        assert math.isclose(lo, -2.6 + math.radians(5), abs_tol=1e-4)
        assert math.isclose(hi, 1.2 - math.radians(5), abs_tol=1e-4)
    finally:
        os.unlink(path)
        for f in (out_urdf, out_xrdf):
            if os.path.exists(f):
                os.unlink(f)
        os.rmdir(tmpdir)


# --- CAPTAIN-DECLARED override mode ---------------------------------------

def test_declare_widens_only_named_joint_keeps_others():
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base', date='2026-09-04')
    tmin, tmax, slack = -2.6, 1.2, 1.0        # rad, rad, deg
    out, entry = rec.declare_limits(base, 'shoulder_lift_joint', tmin, tmax, slack,
                                    'covers the validated demo travel', date='2026-09-05')
    # effective window = [tmin - slack, tmax + slack], encoded ref +- window_deg
    ref = out['reference_pose_rad']['shoulder_lift_joint']
    win = out['window_deg']['shoulder_lift_joint']
    lo = ref - math.radians(win)
    hi = ref + math.radians(win)
    assert math.isclose(lo, tmin - math.radians(slack), abs_tol=1e-5)
    assert math.isclose(hi, tmax + math.radians(slack), abs_tol=1e-5)
    # declared provenance recorded, and it is NOT presented as measured
    d = out['declared'][-1]
    assert d['joint'] == 'shoulder_lift_joint' and d['date'] == '2026-09-05'
    assert 'captain-declared' in d['source'] and 'NOT hand-swept' in d['source']
    assert d['covers_travel_deg']['min'] == round(math.degrees(tmin), 3)
    assert d['window_deg']['max'] == round(math.degrees(tmax + math.radians(slack)), 3)
    # the joint's MEASURED extremes are left as measured (documenting the sweep)
    assert out['measured']['extremes_rad']['shoulder_lift_joint'] == \
        base['measured']['extremes_rad']['shoulder_lift_joint']
    assert d['extended_from_measured_raw_deg'] == \
        base['measured']['extremes_deg']['shoulder_lift_joint']
    # every OTHER joint byte-for-byte, and there is exactly one declaration
    for j in JOINTS:
        if j == 'shoulder_lift_joint':
            continue
        assert out['reference_pose_rad'][j] == base['reference_pose_rad'][j]
        assert out['window_deg'][j] == base['window_deg'][j]
    assert len([x for x in out['declared'] if x['joint'] == 'shoulder_lift_joint']) == 1


def test_declare_one_sided_keeps_the_other_measured_bound():
    """A floor-only need must extend ONLY the floor and keep the measured
    ceiling — the union semantic, so a declaration never narrows reach."""
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    j = 'elbow_joint'
    cur_ref = base['reference_pose_rad'][j]
    cur_half = math.radians(base['window_deg'][j])
    cur_lo, cur_hi = cur_ref - cur_half, cur_ref + cur_half
    # demo travel sits BELOW the floor but well inside the ceiling
    tmin, tmax, slack = cur_lo - 0.3, cur_hi - 0.5, 1.0
    out, entry = rec.declare_limits(base, j, tmin, tmax, slack, 'floor only')
    ref = out['reference_pose_rad'][j]
    win = out['window_deg'][j]
    lo = ref - math.radians(win)
    hi = ref + math.radians(win)
    assert math.isclose(lo, tmin - math.radians(slack), abs_tol=1e-5), 'floor must extend'
    assert math.isclose(hi, cur_hi, abs_tol=1e-5), 'ceiling must stay the measured bound'
    assert entry['prior_window_deg']['max'] == round(math.degrees(cur_hi), 3)


def test_declare_only_widens_never_narrows():
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    j = 'elbow_joint'
    cur_ref = base['reference_pose_rad'][j]
    cur_half = math.radians(base['window_deg'][j])
    cur_lo, cur_hi = cur_ref - cur_half, cur_ref + cur_half
    # a travel STRICTLY inside the current window must leave the window unchanged
    out, _ = rec.declare_limits(base, j, cur_lo + 0.1, cur_hi - 0.1, 0.0, 'inside')
    assert math.isclose(out['reference_pose_rad'][j], base['reference_pose_rad'][j], abs_tol=1e-6)
    assert math.isclose(out['window_deg'][j], base['window_deg'][j], abs_tol=1e-4)


def test_declare_refuses_bad_input():
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    for bad in [('elbow_flange', -1.0, 1.0), ('elbow_joint', 1.0, 1.0),
                ('elbow_joint', 1.0, -1.0)]:
        try:
            rec.declare_limits(base, bad[0], bad[1], bad[2], 1.0, 'x')
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {bad}')
    # a non-measured file is refused
    try:
        rec.declare_limits({'window_deg': {}}, 'elbow_joint', -1.0, 1.0, 1.0, 'x')
    except ValueError:
        return
    raise AssertionError('expected ValueError on a foreign file')


def test_declare_reidempotent_replaces_prior_entry():
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    once, _ = rec.declare_limits(base, 'shoulder_lift_joint', -2.6, 1.0, 1.0, 'first')
    twice, _ = rec.declare_limits(once, 'shoulder_lift_joint', -2.7, 1.1, 1.0, 'second')
    sl = [d for d in twice['declared'] if d['joint'] == 'shoulder_lift_joint']
    assert len(sl) == 1 and sl[0]['reason'] == 'second'


def test_declared_file_round_trips_execute_plan_parser():
    """The declared window must be read identically by execute_plan.py's own
    lo/hi arithmetic — the whole point is that guard passes on it."""
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    tmin, tmax, slack = -2.6, 1.2, 1.0
    out, _ = rec.declare_limits(base, 'shoulder_lift_joint', tmin, tmax, slack, 'demo')
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='declared_limits_')
    os.close(fd)
    try:
        rec.write_limits_file(path, out)
        lim = yaml.safe_load(open(path))            # exactly what the consumers do
        ref, win = lim['reference_pose_rad'], lim['window_deg']
        lo = ref['shoulder_lift_joint'] - math.radians(win['shoulder_lift_joint'])
        hi = ref['shoulder_lift_joint'] + math.radians(win['shoulder_lift_joint'])
        assert math.isclose(lo, tmin - math.radians(slack), abs_tol=1e-5)
        assert math.isclose(hi, tmax + math.radians(slack), abs_tol=1e-5)
        # the declared block survives a write/read round-trip
        assert lim['declared'][-1]['joint'] == 'shoulder_lift_joint'
    finally:
        os.unlink(path)


def test_declared_file_regenerates_limited_model():
    """A declared file must stay make_limited_urdf --frozen -regenerable."""
    urdf_in = os.path.join(CUMOTION, 'ur10_calibrated.urdf')
    xrdf_in = os.path.join(CUMOTION, 'ur10_map.xrdf')
    script = os.path.join(CUMOTION, 'make_limited_urdf.py')
    if not (os.path.exists(urdf_in) and os.path.exists(xrdf_in)):
        print('SKIP declared round-trip: inputs missing')
        return
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    tmin, tmax, slack = -2.6, 1.2, 1.0
    out, _ = rec.declare_limits(base, 'shoulder_lift_joint', tmin, tmax, slack, 'demo')
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='declared_limits_')
    os.close(fd)
    tmpdir = tempfile.mkdtemp(prefix='declurdf_')
    out_urdf = os.path.join(tmpdir, 'out.urdf')
    out_xrdf = os.path.join(tmpdir, 'out.xrdf')
    try:
        rec.write_limits_file(path, out)
        r = subprocess.run(
            [sys.executable, script, '--frozen', '--limits', path,
             '--urdf', urdf_in, '--xrdf-in', xrdf_in,
             '--out', out_urdf, '--xrdf-out', out_xrdf],
            capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, f'make_limited_urdf on declared file failed:\n{r.stdout}\n{r.stderr}'
        s = open(out_urdf).read()
        m = re.search(r'<joint name="shoulder_lift_joint" type="revolute">.*?'
                      r'<limit[^>]*lower="([-\d.eE+]+)"[^>]*upper="([-\d.eE+]+)"', s, re.S)
        lo, hi = float(m.group(1)), float(m.group(2))
        assert math.isclose(lo, tmin - math.radians(slack), abs_tol=1e-4)
        assert math.isclose(hi, tmax + math.radians(slack), abs_tol=1e-4)
    finally:
        os.unlink(path)
        for f in (out_urdf, out_xrdf):
            if os.path.exists(f):
                os.unlink(f)
        os.rmdir(tmpdir)


def _window(lim, j):
    ref = lim['reference_pose_rad'][j]
    half = math.radians(lim['window_deg'][j])
    return ref - half, ref + half


def test_merge_after_declare_keeps_declared_bound_and_refreshes_entry():
    """A targeted re-sweep of a joint that carries a captain-declared override
    must UNION with the declared window: a sweep INSIDE it changes nothing, a
    sweep BEYOND it widens further, and the `declared:` entry (which the header
    is printed from) always states the window actually applied."""
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    j = 'shoulder_lift_joint'
    tmin, tmax, slack = -2.6, 0.0, 1.0            # floor well below measured
    declared, _ = rec.declare_limits(base, j, tmin, tmax, slack, 'demo floor')
    d_lo, d_hi = _window(declared, j)
    assert math.isclose(d_lo, tmin - math.radians(slack), abs_tol=1e-5)

    # re-sweep INSIDE both the measured and the declared window
    smins, smaxs = list(mins), list(maxs)
    smins[1], smaxs[1] = -2.0, 0.4
    inside, changed = rec.merge_limits(declared, smins, smaxs, [j], 'resweep-inside')
    lo, hi = _window(inside, j)
    assert math.isclose(lo, d_lo, abs_tol=1e-5), 'declared floor must NOT narrow'
    assert math.isclose(hi, d_hi, abs_tol=1e-5), 'measured ceiling must stay'
    assert inside['measured']['extremes_rad'][j] == base['measured']['extremes_rad'][j]
    de = [d for d in inside['declared'] if d['joint'] == j][0]
    assert de['window_deg'] == {'min': round(math.degrees(lo), 3),
                                'max': round(math.degrees(hi), 3)}
    assert changed[j]['window_deg'] == de['window_deg']

    # re-sweep BEYOND the declared floor (by more than the margin): widens
    smins[1] = -3.0
    beyond, changed = rec.merge_limits(declared, smins, smaxs, [j], 'resweep-beyond')
    lo2, hi2 = _window(beyond, j)
    assert math.isclose(lo2, -3.0 + math.radians(5.0), abs_tol=1e-5), 'must widen past declared'
    assert lo2 < d_lo
    assert math.isclose(hi2, d_hi, abs_tol=1e-5)
    er = beyond['measured']['extremes_rad'][j]
    assert er['min'] == round(-3.0, 6) and er['max'] == round(0.5, 6)
    de2 = [d for d in beyond['declared'] if d['joint'] == j][0]
    assert de2['window_deg'] == {'min': round(math.degrees(lo2), 3),
                                 'max': round(math.degrees(hi2), 3)}
    assert de2['extended_from_measured_raw_deg'] == beyond['measured']['extremes_deg'][j]
    assert de2['reason'] == 'demo floor'
    # every OTHER joint untouched
    for k in JOINTS:
        if k != j:
            assert beyond['reference_pose_rad'][k] == declared['reference_pose_rad'][k]
            assert beyond['window_deg'][k] == declared['window_deg'][k]

    # the invariant survives the write boundary: what the consumers read as the
    # applied window equals what the declared block claims
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='merged_declared_')
    os.close(fd)
    try:
        rec.write_limits_file(path, beyond)
        lim = yaml.safe_load(open(path))
        flo, fhi = _window(lim, j)
        dd = [d for d in lim['declared'] if d['joint'] == j][0]
        assert math.isclose(math.degrees(flo), dd['window_deg']['min'], abs_tol=2e-3)
        assert math.isclose(math.degrees(fhi), dd['window_deg']['max'], abs_tol=2e-3)
    finally:
        os.unlink(path)


def test_merge_on_undeclared_joint_equals_measured_minus_margin():
    """Without a declaration the union is a no-op: the applied window is
    exactly the merged measured extremes shrunk by the file's margin."""
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    smins, smaxs = list(mins), list(maxs)
    smins[2], smaxs[2] = -1.6, 1.5
    merged, _ = rec.merge_limits(base, smins, smaxs, ['elbow_joint'], 'x')
    lo, hi = _window(merged, 'elbow_joint')
    assert math.isclose(lo, -1.6 + math.radians(5.0), abs_tol=1e-5)
    assert math.isclose(hi, 1.5 - math.radians(5.0), abs_tol=1e-5)
    assert 'declared' not in merged


def test_declare_reason_with_newlines_writes_parseable_file():
    """A multi-line --reason must never leak a header line without a leading
    '#': the file must still parse and carry the same windows."""
    mins, maxs = _measured()
    base = rec.build_limits(mins, maxs, 5.0, 'base', 'base')
    j = 'shoulder_lift_joint'
    ugly = 'covers the demo\r\n   second line\n\tthird'
    out, entry = rec.declare_limits(base, j, -2.6, 1.2, 1.0, ugly)
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='reason_nl_')
    os.close(fd)
    try:
        rec.write_limits_file(path, out)
        lim = yaml.safe_load(open(path))
        assert lim['reference_pose_rad'] == out['reference_pose_rad']
        assert lim['window_deg'] == out['window_deg']
        assert lim['declared'][-1]['joint'] == j
        # and the CLI path stores the reason as one line
        path2, _, _, _ = _write_tmp_limits()
        try:
            rc = rec.main(['--declare', 'shoulder_lift=-174.2,-12.2', '--reason', ugly,
                           '--out', path2])
            assert rc == 0, f'--declare returned {rc}'
            lim2 = yaml.safe_load(open(path2))
            d = lim2['declared'][-1]
            assert d['reason'] == 'covers the demo second line third'
            lo, hi = _window(lim2, j)
            assert math.isclose(math.degrees(lo), -175.2, abs_tol=2e-3)
        finally:
            os.unlink(path2)
    finally:
        os.unlink(path)


def test_declare_and_refusal_work_without_ros_stack():
    """--declare is a pure file edit: it must run on a host where rclpy /
    sensor_msgs cannot be imported. Recording on such a host must refuse
    cleanly (exit 2, no traceback)."""
    driver = (
        'import sys\n'
        "for m in ('rclpy', 'rclpy.node', 'sensor_msgs', 'sensor_msgs.msg'):\n"
        '    sys.modules[m] = None\n'
        f'sys.path.insert(0, {HERE!r})\n'
        'import record_joint_limits as rec\n'
        'sys.exit(rec.main(sys.argv[1:]))\n')
    path, mins, maxs, margin = _write_tmp_limits()
    try:
        r = subprocess.run(
            [sys.executable, '-c', driver, '--declare', 'elbow=-100.0,10.0',
             '--reason', 'no ros needed', '--out', path],
            capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, f'--declare without ROS failed:\n{r.stdout}\n{r.stderr}'
        lim = yaml.safe_load(open(path))
        d = lim['declared'][-1]
        assert d['joint'] == 'elbow_joint' and d['reason'] == 'no ros needed'
        lo, hi = _window(lim, 'elbow_joint')
        assert math.isclose(math.degrees(lo), -101.0, abs_tol=2e-3), 'floor must extend'
        assert math.isclose(hi, maxs[2] - math.radians(margin), abs_tol=1e-5), \
            'measured ceiling must stay'

        r = subprocess.run(
            [sys.executable, '-c', driver, '--check', '--out', path],
            capture_output=True, text=True, timeout=60)
        assert r.returncode == 2, f'expected clean refusal, got {r.returncode}:\n{r.stderr}'
        assert 'Traceback' not in r.stderr
        assert 'rclpy' in r.stderr
    finally:
        os.unlink(path)


# --- make_limited_urdf.py --keep-world-collision --------------------------

def test_make_limited_urdf_keep_world_collision():
    """--keep-world-collision keeps the FULL sphere set for WORLD collision
    (nothing dropped) but still moves the retract config inside the windows."""
    urdf_in = os.path.join(CUMOTION, 'ur10_calibrated.urdf')
    xrdf_in = os.path.join(CUMOTION, 'ur10_map.xrdf')
    script = os.path.join(CUMOTION, 'make_limited_urdf.py')
    if not (os.path.exists(urdf_in) and os.path.exists(xrdf_in)):
        print('SKIP keep-world-collision: inputs missing')
        return
    path, mins, maxs, margin = _write_tmp_limits()
    tmpdir = tempfile.mkdtemp(prefix='kwc_')
    out_urdf = os.path.join(tmpdir, 'out.urdf')
    out_xrdf = os.path.join(tmpdir, 'out.xrdf')
    xin = yaml.safe_load(open(xrdf_in))
    full = xin['collision']['geometry']
    try:
        r = subprocess.run(
            [sys.executable, script, '--frozen', '--keep-world-collision',
             '--limits', path, '--urdf', urdf_in, '--xrdf-in', xrdf_in,
             '--out', out_urdf, '--xrdf-out', out_xrdf],
            capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, f'make_limited_urdf failed:\n{r.stdout}\n{r.stderr}'
        xd = yaml.safe_load(open(out_xrdf))
        # world collision keeps the full set: no *_world_only geometry created,
        # collision and self_collision point at the SAME full sphere set
        assert xd['collision']['geometry'] == full, 'world collision must keep the full set'
        assert xd['self_collision']['geometry'] == full
        assert not any(k.endswith('_world_only') for k in xd['geometry']), \
            'no reduced world-only sphere set should be created'
        # both excluded links are still present in the world sphere set
        for link in rec_make_exclude():
            assert link in xd['geometry'][full]['spheres'], f'{link} must be kept'
        # retract config still moved inside the windows
        lim = yaml.safe_load(open(path))
        for j in JOINTS:
            assert math.isclose(float(xd['default_joint_positions'][j]),
                                round(lim['reference_pose_rad'][j], 4), abs_tol=1e-3)
    finally:
        os.unlink(path)
        for f in (out_urdf, out_xrdf):
            if os.path.exists(f):
                os.unlink(f)
        os.rmdir(tmpdir)


def rec_make_exclude():
    """The links make_limited_urdf.py drops from world collision by DEFAULT —
    read from the script so this test tracks it."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'mlu', os.path.join(CUMOTION, 'make_limited_urdf.py'))
    mlu = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mlu)
    return mlu.WORLD_COLLISION_EXCLUDE


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith('test_') and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f'PASS {fn.__name__}')
        except AssertionError as e:
            failed += 1
            print(f'FAIL {fn.__name__}: {e}')
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f'ERROR {fn.__name__}: {type(e).__name__}: {e}')
    print(f'\n{len(fns) - failed}/{len(fns)} passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(_run_all())
