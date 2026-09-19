#!/usr/bin/env python3
"""
Arming interlock — makes the REAL robot unreachable from this change by
configuration alone (task brief item 3; report §10 Phase 3 is sim-only).

The driver-backed command port (driver_port.py) refuses to start unless ALL of:

  1. an explicit `arm` parameter is set true (default is NOT armed);
  2. the ROS domain is neither 0 (the live cell domain, AGENTS.md) nor 42 (the
     shelfbot_ros container's baked domain);
  3. the robot address it is told to talk to is loopback or a Docker-internal
     address — and NOT 192.168.0.1 (the real UR10 dashboard/RTDE address,
     execute_plan.py:152) and NOT any address on the workstation's PHYSICAL
     networks.

Arming the REAL arm is a deliberately later, separate change: this interlock has
no configuration that reaches 192.168.0.1 or a physical NIC, so no combination of
launch args can point this code at the real robot. That is the whole point.

This module is PURE (stdlib `ipaddress` only): the impure part — enumerating the
host's physical vs Docker subnets — lives in the node, which hands the enumerated
lists to `evaluate_arming` so the decision is fully unit-testable offline.
"""
from __future__ import annotations

import ipaddress
from dataclasses import dataclass, field
from typing import Iterable, List, Optional

# Domains this task must never run on (AGENTS.md: live cell = 0; shelfbot_ros
# container bakes 42). The task's own domain is 79.
FORBIDDEN_DOMAINS = frozenset({0, 42})

# The real robot's address (execute_plan.py, RUNBOOK). Refused explicitly, over
# and above the physical-network check, so it is unmistakable in the logs.
REAL_ROBOT_IP = "192.168.0.1"


@dataclass(frozen=True)
class ArmingDecision:
    """Result of the interlock. `armed` is True only if every gate passed."""

    armed: bool
    reasons: List[str] = field(default_factory=list)   # why refused (empty if armed)
    resolved_ip: Optional[str] = None
    classification: str = ""   # "loopback" | "docker-internal" | "" (refused)

    def __bool__(self) -> bool:
        return self.armed


def _networks(cidrs: Optional[Iterable]) -> List[ipaddress.IPv4Network]:
    out: List[ipaddress.IPv4Network] = []
    for c in cidrs or ():
        try:
            out.append(ipaddress.ip_network(str(c), strict=False))
        except ValueError:
            continue
    return out


def evaluate_arming(
    *,
    arm: bool,
    ros_domain_id,
    robot_ip: str,
    physical_cidrs: Optional[Iterable] = None,
    docker_cidrs: Optional[Iterable] = None,
    forbidden_ips: Iterable = (REAL_ROBOT_IP,),
    forbidden_domains: Iterable = FORBIDDEN_DOMAINS,
) -> ArmingDecision:
    """Decide whether the driver-backed port may arm. Fails closed: ANY failed
    gate (or any ambiguity — an unparseable domain/IP, an address that is neither
    loopback nor provably Docker-internal) refuses.

    `physical_cidrs` / `docker_cidrs` are the host's enumerated subnets (the node
    supplies them; see enumerate_host_subnets). `robot_ip` is checked against the
    physical set FIRST so an address that appears on both a physical NIC and a
    Docker bridge is still refused.
    """
    reasons: List[str] = []

    # --- gate 1: explicit arm parameter ---------------------------------- #
    if not arm:
        reasons.append("arm parameter not set (default is DISARMED; real-robot "
                       "arming is a later, separate change)")

    # --- gate 2: ROS domain not 0 and not 42 ----------------------------- #
    try:
        domain = int(ros_domain_id)
    except (TypeError, ValueError):
        domain = None
        reasons.append(f"ROS_DOMAIN_ID {ros_domain_id!r} is not an integer")
    if domain is not None and domain in set(int(d) for d in forbidden_domains):
        reasons.append(f"ROS_DOMAIN_ID={domain} is forbidden "
                       f"(0=live cell, 42=shelfbot_ros container)")

    # --- gate 3: robot address loopback or Docker-internal --------------- #
    classification = ""
    ip_obj = None
    try:
        ip_obj = ipaddress.ip_address(str(robot_ip).strip())
    except ValueError:
        reasons.append(f"robot_ip {robot_ip!r} is not a valid IP address")

    if ip_obj is not None:
        forbidden = {str(f).strip() for f in forbidden_ips}
        phys = _networks(physical_cidrs)
        dock = _networks(docker_cidrs)

        if str(ip_obj) in forbidden:
            reasons.append(f"robot_ip {ip_obj} is an explicitly forbidden address "
                           f"(the real robot)")
        elif any(ip_obj in n for n in phys):
            hit = next(str(n) for n in phys if ip_obj in n)
            reasons.append(f"robot_ip {ip_obj} is on a PHYSICAL workstation "
                           f"network {hit} — refusing to reach real hardware")
        elif ip_obj.is_loopback:
            classification = "loopback"
        elif any(ip_obj in n for n in dock):
            classification = "docker-internal"
        else:
            reasons.append(f"robot_ip {ip_obj} is neither loopback nor a known "
                           f"Docker-internal subnet — refusing (fail closed)")

    armed = not reasons
    return ArmingDecision(
        armed=armed,
        reasons=reasons,
        resolved_ip=(str(ip_obj) if ip_obj is not None else None),
        classification=classification if armed else "",
    )
