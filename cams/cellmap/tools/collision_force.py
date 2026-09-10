#!/usr/bin/env python3
"""
Single reader for the cell's collision-FORCE thresholds.

The one repository-controlled collision-force threshold lives in
`cumotion/collision_force_limits.yaml` (`ros_contact_force_backstop_n`): the
ROS-side software backstop that cancels a running trajectory on a gross force
step. loop_tags.py sources its `--max-force-n` default from here so the value
lives in ONE documented place, never scattered as a magic number in each tool.

This module is deliberately PURE: it imports only `os` + `yaml`, never rclpy or
any ROS package, so the offline tests (and any host without a ROS stack) can
read and check the value exactly as the motion tools do.

The pendant safety-rated Force limit (the real, safety-rated collision-force
protective stop on this CB3) is NOT here and cannot be — it is a teach-pendant
setting only the captain can change. The YAML documents it for the record; no
code reads it. See cumotion/collision_force_limits.yaml and
data/collision-force-v1/decision.md.
"""
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
CONFIG_PATH = os.path.join(REPO, 'cumotion', 'collision_force_limits.yaml')

# Fail-safe fallback if the config file is ever unreadable: the LOWER (more
# protective) value, matching the committed file. test_collision_force.py
# asserts this equals the file's value so the two can never silently diverge.
_FALLBACK_BACKSTOP_N = 200.0

# Sane-range bounds for the ROS-side backstop, enforced by validate():
#  * lower: must clear the worst clean-motion force deviation ever measured on
#    this CB3 (151.8 N, 2026-08-28, nothing touching the arm) with margin, or it
#    nuisance-trips a clean cycle.
#  * upper: the old default; this task only ever LOWERS the push before a stop.
BACKSTOP_MIN_N = 160.0
BACKSTOP_MAX_N = 300.0


def load(path=CONFIG_PATH):
    """Parse the collision-force config. Raises on a malformed file."""
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f'{path}: expected a mapping, got {type(cfg).__name__}')
    return cfg


def validate(cfg):
    """Check the ROS-side backstop is present and inside the sane range.
    Returns the validated float. Raises ValueError otherwise."""
    v = cfg.get('ros_contact_force_backstop_n')
    if not isinstance(v, (int, float)):
        raise ValueError('ros_contact_force_backstop_n missing or non-numeric')
    v = float(v)
    if not (BACKSTOP_MIN_N < v <= BACKSTOP_MAX_N):
        raise ValueError(
            f'ros_contact_force_backstop_n {v} N outside the sane range '
            f'({BACKSTOP_MIN_N} < N <= {BACKSTOP_MAX_N})')
    return v


def contact_force_backstop_n(path=CONFIG_PATH):
    """N of force deviation the ROS-side backstop stops the arm at.

    This is the value loop_tags.py uses as its --max-force-n default. If the
    config file is missing or malformed, fall back to the LOWER (more
    protective) constant rather than silently running with no/too-high a guard.
    """
    try:
        return validate(load(path))
    except (OSError, ValueError, yaml.YAMLError):
        return _FALLBACK_BACKSTOP_N


if __name__ == '__main__':
    cfg = load()
    print(f'config: {CONFIG_PATH}')
    print(f'ros_contact_force_backstop_n: {contact_force_backstop_n()} N')
    psf = cfg.get('pendant_safety_force', {})
    print(f'pendant safety Force limit (PENDANT-ONLY, not repo-settable): '
          f'recommend {psf.get("recommended_target_n")} N, '
          f'CB3 floor {psf.get("cb3_min_n")} N — {psf.get("location")}')
