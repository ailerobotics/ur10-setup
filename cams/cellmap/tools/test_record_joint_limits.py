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
