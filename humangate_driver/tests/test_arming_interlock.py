#!/usr/bin/env python3
"""
Offline tests for the arming interlock — proves the real robot is unreachable by
configuration alone (task brief item 3). NO ROS, NO robot.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from arming_interlock import (  # noqa: E402
    REAL_ROBOT_IP, evaluate_arming)

# A representative host layout: a physical LAN that carries the real robot, plus
# a Docker bridge subnet used by the URSim container for this task.
PHYS = ["192.168.0.0/24", "10.42.0.0/24"]
DOCK = ["172.17.0.0/16", "192.168.98.0/24"]

ARMED = dict(arm=True, ros_domain_id=79, robot_ip="127.0.0.1",
             physical_cidrs=PHYS, docker_cidrs=DOCK)


def test_fully_armed_loopback_ok():
    d = evaluate_arming(**ARMED)
    assert d.armed and d.classification == "loopback" and not d.reasons


def test_docker_internal_ok():
    d = evaluate_arming(**{**ARMED, "robot_ip": "192.168.98.2"})
    assert d.armed and d.classification == "docker-internal"


def test_disarmed_by_default():
    d = evaluate_arming(**{**ARMED, "arm": False})
    assert not d.armed and any("arm parameter" in r for r in d.reasons)


def test_forbidden_domains_refused():
    for dom in (0, 42):
        d = evaluate_arming(**{**ARMED, "ros_domain_id": dom})
        assert not d.armed and any("forbidden" in r for r in d.reasons)


def test_non_integer_domain_refused():
    d = evaluate_arming(**{**ARMED, "ros_domain_id": "not-a-number"})
    assert not d.armed


def test_real_robot_ip_refused_explicitly():
    d = evaluate_arming(**{**ARMED, "robot_ip": REAL_ROBOT_IP})
    assert not d.armed
    assert any("forbidden address" in r for r in d.reasons)


def test_any_physical_network_address_refused():
    # a different address on the physical LAN (not just .0.1) is still refused
    d = evaluate_arming(**{**ARMED, "robot_ip": "192.168.0.50"})
    assert not d.armed and any("PHYSICAL" in r for r in d.reasons)
    d2 = evaluate_arming(**{**ARMED, "robot_ip": "10.42.0.7"})
    assert not d2.armed and any("PHYSICAL" in r for r in d2.reasons)


def test_physical_takes_precedence_over_docker_overlap():
    # if an address is on BOTH a physical NIC and a docker subnet, refuse.
    d = evaluate_arming(**{**ARMED, "robot_ip": "192.168.0.5",
                           "docker_cidrs": DOCK + ["192.168.0.0/24"]})
    assert not d.armed and any("PHYSICAL" in r for r in d.reasons)


def test_unknown_address_refused_fail_closed():
    # neither loopback nor a known docker subnet nor physical -> refuse
    d = evaluate_arming(**{**ARMED, "robot_ip": "8.8.8.8"})
    assert not d.armed and any("neither loopback" in r for r in d.reasons)


def test_invalid_ip_refused():
    d = evaluate_arming(**{**ARMED, "robot_ip": "not.an.ip"})
    assert not d.armed


def test_multiple_failures_all_reported():
    d = evaluate_arming(arm=False, ros_domain_id=0, robot_ip=REAL_ROBOT_IP,
                        physical_cidrs=PHYS, docker_cidrs=DOCK)
    assert not d.armed and len(d.reasons) >= 3


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn(); print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1; print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1; print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
