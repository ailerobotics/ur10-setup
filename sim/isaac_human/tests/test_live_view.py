# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the "Watch it live" WebRTC mode: the scene argument parser,
the SLOW/STOP ring radii read from the SAME safety config the detector uses, the
client-port picker (forbidden set + free scan), the zone-beacon colours, and the
preflight refusal cases the runner depends on. No Isaac / GPU needed."""
import os
import subprocess

import live_view
from human_safety.config import load_config

_HERE = os.path.dirname(os.path.abspath(__file__))
_HARNESS = os.path.dirname(_HERE)
_PREFLIGHT = os.path.join(_HARNESS, "preflight.sh")
_SIM_CFG = os.path.join(_HARNESS, "params", "human_safety.sim.yaml")


# --- port selection -------------------------------------------------------- #
def test_forbidden_ports_cover_the_brief_set():
    for p in list(range(4173, 4179)) + list(range(8443, 8450)) + [8765, 54321]:
        assert live_view.is_forbidden_port(p), p
    assert not live_view.is_forbidden_port(8080)


def test_pick_client_port_prefers_8080_when_free():
    assert live_view.pick_client_port(8080, lambda p: True) == 8080


def test_pick_client_port_skips_busy():
    busy = {8080, 8081}
    assert live_view.pick_client_port(8080, lambda p: p not in busy) == 8082


def test_pick_client_port_skips_forbidden_even_when_free():
    # 8443-8449 are forbidden; the picker must jump past them.
    assert live_view.pick_client_port(8443, lambda p: True) == 8450


def test_pick_client_port_raises_when_none_free():
    import pytest
    with pytest.raises(RuntimeError):
        live_view.pick_client_port(8080, lambda p: False, max_scan=10)


# --- ring radii read from the safety config (no drift, no hard-coding) ----- #
def test_ring_radii_match_the_detector_config():
    reach, stop_r, slow_r = live_view.read_zone_radii(_SIM_CFG)
    # cross-check against the value the REAL loader parses from the same file.
    cfg = load_config(_SIM_CFG)
    assert (reach, stop_r, slow_r) == (
        cfg.safety.reach_radius, cfg.safety.stop_radius, cfg.safety.slow_radius)


def test_zone_ring_specs_use_the_envelope_boundary_and_colours():
    reach, stop_r, slow_r = live_view.read_zone_radii(_SIM_CFG)
    specs = live_view.zone_ring_specs(reach, stop_r, slow_r)
    names = [s.name for s in specs]
    assert names == ["SLOW", "STOP"]
    by = {s.name: s for s in specs}
    # ring radius is reach + zone clearance (matches fusion_node.py)
    assert by["STOP"].radius == reach + stop_r
    assert by["SLOW"].radius == reach + slow_r
    # STOP ring is inside the SLOW ring
    assert by["STOP"].radius < by["SLOW"].radius
    # SLOW amber, STOP red (matches the RViz marker colours)
    assert by["SLOW"].rgba[:3] == (1.0, 0.8, 0.0)
    assert by["STOP"].rgba[:3] == (1.0, 0.1, 0.1)


def test_zone_ring_specs_for_the_documented_cell():
    # The committed sim cell: STOP ring 2.0 m, SLOW ring 3.3 m of base.
    specs = live_view.zone_ring_specs(*live_view.read_zone_radii(_SIM_CFG))
    by = {s.name: round(s.radius, 2) for s in specs}
    assert by == {"SLOW": 3.3, "STOP": 2.0}


# --- zone beacon colours --------------------------------------------------- #
def test_zone_colour_mapping():
    assert live_view.zone_color("CLEAR")[1] > live_view.zone_color("STOP")[1]  # greener
    assert live_view.zone_color("STOP")[0] > live_view.zone_color("SLOW")[2]
    # unknown / no state yet -> neutral grey, never green
    assert live_view.zone_color(None) == live_view.UNKNOWN_ZONE_COLOR
    assert live_view.zone_color("bogus") == live_view.UNKNOWN_ZONE_COLOR


# --- scene argument parsing ------------------------------------------------ #
def test_scene_parser_defaults_no_livestream():
    a = live_view.build_scene_arg_parser().parse_args(["--gt-log", "gt.jsonl"])
    assert a.livestream is False
    assert a.loop_walk is False
    assert a.zone_beacon is False
    assert a.public_ip is None
    assert a.livestream_port == live_view.LIVESTREAM_PORT
    assert a.cams == "wrist,shelf,table"


def test_scene_parser_live_view_flags():
    a = live_view.build_scene_arg_parser().parse_args([
        "--gt-log", "gt.jsonl", "--cams", "wrist", "--livestream",
        "--public-ip", "100.64.0.1", "--loop-walk",
        "--config", "params/human_safety.sim.yaml", "--zone-beacon"])
    assert a.livestream and a.loop_walk and a.zone_beacon
    assert a.public_ip == "100.64.0.1"
    assert a.config.endswith("human_safety.sim.yaml")


def test_scene_parser_requires_gt_log():
    import pytest
    with pytest.raises(SystemExit):
        live_view.build_scene_arg_parser().parse_args(["--livestream"])


# --- preflight refusal cases (the runner depends on these gates) ----------- #
def _run_preflight(env_extra):
    env = os.environ.copy()
    env.update(env_extra)
    return subprocess.run(
        ["bash", "-c", f'source "{_PREFLIGHT}" && preflight_check'],
        env=env, capture_output=True, text=True)


def test_preflight_refuses_when_gpu_free_below_threshold():
    r = _run_preflight({"MIN_GPU_FREE_MB": "99999999", "MIN_RAM_MB": "0"})
    assert r.returncode != 0
    assert "BLOCKED" in r.stderr


def test_preflight_refuses_when_ram_below_threshold():
    r = _run_preflight({"MIN_RAM_MB": "99999999", "MIN_GPU_FREE_MB": "0"})
    assert r.returncode != 0
    assert "BLOCKED" in r.stderr
