"""Pre-contact gate: pure checks evaluated before any contact leg.

The node/CLI does the actual reads (TCP + payload via read_tcp_payload.py, the
joint feed via check_joint_feed_live.py, speed scaling + program_running from
ROS, disk free from statvfs, the operator-typed pendant Force limit, and the
external alignment veto); this pure core turns that snapshot into a pass/fail
verdict. Every missing value fails its check (fail toward stop).

The pendant safety Force limit CANNOT be read from the PC on CB3, so the
operator must TYPE the value configured on the pendant; the gate records what
was typed and flags it if it is not the scout-recommended floor (100 N with the
correct payload). The gate exposes a small JSON verdict so a separate alignment
lane can veto it (see :mod:`veto`); this module does not implement the camera
alignment itself.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .params import ContactGuardParams


@dataclass
class GateInput:
    tcp_offset: Optional[Sequence[float]] = None   # (x,y,z,rx,ry,rz) from the robot
    payload_kg: Optional[float] = None
    loaded: bool = False                           # expected tray-on-forks?
    speed_frac: Optional[float] = None             # current speed scaling 0..1
    robot_running: Optional[bool] = None           # RUNNING
    robot_normal: Optional[bool] = None            # NORMAL safety mode
    program_running: Optional[bool] = None         # External Control up
    joint_feed_live: Optional[bool] = None
    free_gb: Optional[float] = None
    pendant_force_ack_n: Optional[float] = None    # operator-typed
    align_veto_ok: Optional[bool] = None           # external alignment lane
    align_veto_reason: str = ""


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


@dataclass
class GateVerdict:
    ok: bool
    checks: List[Check] = field(default_factory=list)
    pendant_force_ack_n: Optional[float] = None

    @property
    def blocking(self) -> List[Check]:
        return [c for c in self.checks if not c.ok]

    def to_json(self) -> dict:
        return {
            "schema": "contact_guard.gate_verdict.v1",
            "ok": self.ok,
            "pendant_force_ack_n": self.pendant_force_ack_n,
            "checks": [{"name": c.name, "ok": c.ok, "detail": c.detail}
                       for c in self.checks],
        }

    def text(self) -> str:
        lines = ["pre-contact gate: " + ("PASS" if self.ok else "BLOCKED")]
        for c in self.checks:
            lines.append(f"  [{'ok ' if c.ok else 'NO '}] {c.name}: {c.detail}")
        return "\n".join(lines)


def evaluate_gate(gi: GateInput, p: ContactGuardParams) -> GateVerdict:
    checks: List[Check] = []

    # 1. fork TCP read back from the robot
    if gi.tcp_offset is None or len(gi.tcp_offset) < 3:
        checks.append(Check("fork_tcp", False, "no TCP read from the robot"))
    else:
        z, y = gi.tcp_offset[2], gi.tcp_offset[1]
        ok = (z >= p.fork_tcp_z_min and abs(y) >= p.fork_tcp_absy_min)
        checks.append(Check("fork_tcp", ok,
                            f"tcp z={z:.3f} |y|={abs(y):.3f} "
                            f"(need z>={p.fork_tcp_z_min}, |y|>={p.fork_tcp_absy_min})"))

    # 2. payload band (unloaded / loaded)
    lo, hi = ((p.payload_loaded_lo, p.payload_loaded_hi) if gi.loaded
              else (p.payload_unloaded_lo, p.payload_unloaded_hi))
    if gi.payload_kg is None:
        checks.append(Check("payload", False, "no payload read"))
    else:
        ok = lo <= gi.payload_kg <= hi
        checks.append(Check("payload", ok,
                            f"{gi.payload_kg:.2f} kg in [{lo},{hi}] "
                            f"({'loaded' if gi.loaded else 'unloaded'})"))

    # 3. speed scaling cap
    if gi.speed_frac is None:
        checks.append(Check("speed_cap", False, "no speed scaling read"))
    else:
        ok = gi.speed_frac <= p.speed_cap_frac + 1e-6
        checks.append(Check("speed_cap", ok,
                            f"{gi.speed_frac * 100:.0f}% <= {p.speed_cap_frac * 100:.0f}%"))

    # 4. RUNNING + NORMAL
    rn = bool(gi.robot_running) and bool(gi.robot_normal)
    checks.append(Check("running_normal", rn,
                        f"running={gi.robot_running} normal={gi.robot_normal}"))

    # 5. External Control running
    checks.append(Check("external_control", bool(gi.program_running),
                        f"program_running={gi.program_running}"))

    # 6. live joint feed
    checks.append(Check("joint_feed_live", bool(gi.joint_feed_live),
                        f"joint_feed_live={gi.joint_feed_live}"))

    # 7. free disk
    if gi.free_gb is None:
        checks.append(Check("free_disk", False, "free disk unknown"))
    else:
        ok = gi.free_gb >= p.min_free_gb
        checks.append(Check("free_disk", ok,
                            f"{gi.free_gb:.1f} GB free >= {p.min_free_gb} GB"))

    # 8. operator-typed pendant Force limit acknowledgement (record what was typed)
    if gi.pendant_force_ack_n is None or gi.pendant_force_ack_n <= 0:
        checks.append(Check("pendant_force_ack", False,
                            "operator did not type the pendant Force limit"))
    else:
        note = ""
        if abs(gi.pendant_force_ack_n - p.pendant_force_recommended_n) > 1.0:
            note = (f" (NOTE: scout recommends {p.pendant_force_recommended_n:.0f} N "
                    "with the correct payload)")
        checks.append(Check("pendant_force_ack", True,
                            f"operator typed {gi.pendant_force_ack_n:.0f} N{note}"))

    # 9. external alignment veto (a separate lane owns the camera alignment)
    if gi.align_veto_ok is None:
        checks.append(Check("alignment_veto", False,
                            "no alignment verdict available (fail toward stop)"))
    else:
        checks.append(Check("alignment_veto", bool(gi.align_veto_ok),
                            gi.align_veto_reason or
                            ("aligned" if gi.align_veto_ok else "vetoed")))

    return GateVerdict(ok=all(c.ok for c in checks), checks=checks,
                       pendant_force_ack_n=gi.pendant_force_ack_n)
