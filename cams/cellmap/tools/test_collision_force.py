#!/usr/bin/env python3
"""
Offline tests for the collision-FORCE threshold — NO live rig, NO ROS.

Covers the task's acceptance for the one repository-controlled collision-force
threshold (`ros_contact_force_backstop_n` in cumotion/collision_force_limits.yaml,
consumed by loop_tags.py through collision_force.py):

  * the configured value is exactly what we think it is (200 N)
  * it is inside the sane range: above the worst measured clean-motion force
    deviation on this CB3 (151.8 N) with margin, and at or below the old 300 N
    default (this change only ever LOWERS the push before a stop, never raises it)
  * the loader returns the file's value, and the fail-safe fallback equals the
    file's value, so the guard can never silently diverge from the config
  * validate() rejects a missing / out-of-range value
  * nothing downstream silently overrides it: loop_tags.py wires --max-force-n's
    default to collision_force.contact_force_backstop_n(), reads that value in a
    comparison, and never reassigns it (proved by parsing the AST — no ROS import)
  * the pendant safety-rated Force limit is documented as PENDANT-ONLY, not
    repo-settable, with the CB3 floor and the recommended target on record

Run standalone: python3 test_collision_force.py
Or under pytest: pytest test_collision_force.py
"""
import ast
import os
import sys
import tempfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import collision_force as cf  # noqa: E402

LOOP_TAGS = os.path.join(HERE, 'loop_tags.py')

# The measured evidence this value is chosen against (loop_tags.py:197-209):
# clean motion with NOTHING touching the arm reached 151.8 N of deviation, and a
# 150 N backstop false-tripped a clean cycle. The backstop must clear that.
MEASURED_CLEAN_PEAK_N = 151.8
OLD_DEFAULT_N = 300.0            # the hard-coded loop_tags.py default before this change
EXPECTED_BACKSTOP_N = 200.0     # the new value, from cumotion/collision_force_limits.yaml


# --- the configured value is what we think it is ---------------------------

def test_config_file_backstop_is_expected_value():
    cfg = cf.load()
    assert cfg['ros_contact_force_backstop_n'] == EXPECTED_BACKSTOP_N, \
        f'config says {cfg["ros_contact_force_backstop_n"]}, expected {EXPECTED_BACKSTOP_N}'


def test_loader_returns_config_value():
    assert cf.contact_force_backstop_n() == EXPECTED_BACKSTOP_N


# --- it is inside the sane range -------------------------------------------

def test_backstop_clears_measured_clean_peak_with_margin():
    v = cf.contact_force_backstop_n()
    assert v > MEASURED_CLEAN_PEAK_N, \
        f'{v} N would nuisance-trip: not above the {MEASURED_CLEAN_PEAK_N} N clean peak'
    # at least ~5% headroom over the worst clean deviation
    assert v >= MEASURED_CLEAN_PEAK_N * 1.05, f'{v} N leaves too little margin over clean motion'


def test_backstop_never_raised_above_old_default():
    v = cf.contact_force_backstop_n()
    assert v <= OLD_DEFAULT_N, f'{v} N is ABOVE the old {OLD_DEFAULT_N} N — this must only lower it'
    assert v < OLD_DEFAULT_N, 'the whole point is to give up sooner than the old default'


def test_module_range_bounds_are_consistent():
    # the loader's own guard rails must bracket the chosen value
    assert cf.BACKSTOP_MIN_N < EXPECTED_BACKSTOP_N <= cf.BACKSTOP_MAX_N
    assert cf.BACKSTOP_MIN_N >= MEASURED_CLEAN_PEAK_N, \
        'lower bound must not admit a value that buries a push under clean-motion noise'
    assert cf.BACKSTOP_MAX_N == OLD_DEFAULT_N


# --- config and fallback can never silently diverge ------------------------

def test_fallback_equals_config_value():
    # if the file ever goes missing, the hard-coded fail-safe must match it, so a
    # missing file cannot silently change the guard.
    assert cf._FALLBACK_BACKSTOP_N == cf.load()['ros_contact_force_backstop_n']


def test_missing_file_falls_back_to_safe_constant():
    missing = os.path.join(tempfile.gettempdir(), 'no_such_collision_force_config_xyz.yaml')
    assert not os.path.exists(missing)
    assert cf.contact_force_backstop_n(missing) == cf._FALLBACK_BACKSTOP_N


# --- validate() rejects bad input ------------------------------------------

def test_validate_rejects_missing_and_out_of_range():
    for bad in ({}, {'ros_contact_force_backstop_n': 'x'},
                {'ros_contact_force_backstop_n': cf.BACKSTOP_MIN_N},        # not strictly above
                {'ros_contact_force_backstop_n': cf.BACKSTOP_MAX_N + 1}):   # above old default
        try:
            cf.validate(bad)
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {bad}')


def test_validate_accepts_the_committed_config():
    assert cf.validate(cf.load()) == EXPECTED_BACKSTOP_N


# --- nothing downstream silently overrides it (AST, no ROS import) ---------

def _loop_tags_ast():
    with open(LOOP_TAGS) as f:
        return ast.parse(f.read(), filename=LOOP_TAGS)


def _find_max_force_add_argument(tree):
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'add_argument'
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == '--max-force-n'):
            return node
    return None


def test_loop_tags_imports_the_loader():
    tree = _loop_tags_ast()
    imported = [n for n in ast.walk(tree)
                if isinstance(n, ast.ImportFrom) and n.module == 'collision_force'
                and any(a.name == 'contact_force_backstop_n' for a in n.names)]
    assert imported, 'loop_tags.py must import contact_force_backstop_n from collision_force'


def test_loop_tags_default_is_wired_to_the_loader_not_a_literal():
    node = _find_max_force_add_argument(_loop_tags_ast())
    assert node is not None, 'could not find the --max-force-n add_argument call'
    default_kw = next((k for k in node.keywords if k.arg == 'default'), None)
    assert default_kw is not None, '--max-force-n has no default= keyword'
    # the default must be a CALL to contact_force_backstop_n(...), never a magic number
    assert isinstance(default_kw.value, ast.Call), \
        'default must be sourced from the config loader, not a hard-coded number'
    fn = default_kw.value.func
    fname = fn.id if isinstance(fn, ast.Name) else getattr(fn, 'attr', None)
    assert fname == 'contact_force_backstop_n', f'default is wired to {fname!r}, not the loader'


def test_loop_tags_never_reassigns_max_force_n_and_does_read_it():
    tree = _loop_tags_ast()
    writes, reads = [], []
    for node in ast.walk(tree):
        # assignment targets: `a.max_force_n = ...` or `a.max_force_n += ...`
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AugAssign):
            targets = [node.target]
        for t in targets:
            if isinstance(t, ast.Attribute) and t.attr == 'max_force_n':
                writes.append(t)
            if isinstance(t, ast.Name) and t.id == 'max_force_n':
                writes.append(t)
        # reads: `a.max_force_n` used in Load context (the guard comparison)
        if (isinstance(node, ast.Attribute) and node.attr == 'max_force_n'
                and isinstance(node.ctx, ast.Load)):
            reads.append(node)
    assert not writes, 'a.max_force_n is reassigned/clamped downstream — it must flow through unchanged'
    assert reads, 'a.max_force_n is never read — the guard is not actually wired to the value'


# --- the pendant limit is documented as pendant-only ------------------------

def test_pendant_force_documented_as_pendant_only():
    psf = cf.load()['pendant_safety_force']
    assert psf['repo_settable'] is False, 'the safety Force limit must be marked NOT repo-settable'
    assert psf['cb3_min_n'] == 100.0, 'CB3 Force-limit floor is 100 N'
    assert psf['recommended_target_n'] >= psf['cb3_min_n']
    assert psf['recommended_target_n'] <= psf['cb3_max_normal_n']
    assert psf['current_value_n'] is None, 'the live pendant value is not knowable from this repo'
    assert psf['depends_on_payload'] is True


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
