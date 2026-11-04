"""Post-action verification: confirm the tray is on / off the forks.

After LIFT: confirm the tray actually came up on the forks -- a wrench step
consistent with the loaded band learned from the teach-002 loaded carry, AND a
TCP height rise. On CB3 the wrench step is MARGINAL (a full tray ~24 N vs
+-9-19 N same-pose repeatability, scout 2.4), so the reliable confirmation is
the kinematic TCP rise; the wrench step corroborates. A mismatch refuses to
continue.

After LOWER: confirm the load left the forks before WITHDRAW -- the wrench must
drop back below a small residual. All pure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from .params import ContactGuardParams


@dataclass
class VerifyResult:
    ok: bool
    kind: str                  # "lift" | "release"
    detail: str
    load_step_n: float = 0.0
    dz_m: float = 0.0
    wrench_conclusive: bool = True

    def text(self) -> str:
        return (f"post-{self.kind} verify: {'OK' if self.ok else 'MISMATCH'} -- "
                f"{self.detail}")


def _load_mag(feats: Dict[str, float]) -> float:
    """Vertical-ish load proxy: the insertion-axis force magnitude (fork +z)."""
    if "f_ins" in feats:
        return abs(feats["f_ins"])
    return abs(feats.get("force_mag", 0.0))


def verify_lift(before: Dict[str, float], after: Dict[str, float],
                dz_m: float, p: ContactGuardParams) -> VerifyResult:
    """Confirm the tray is on the forks after a LIFT.

    ``before`` / ``after`` are fork-frame feature dicts (pre-lift at the holding
    pose, post-lift at the lifted pose); ``dz_m`` is the measured TCP rise.
    """
    load_step = _load_mag(after) - _load_mag(before)
    rose = dz_m >= p.lift_min_dz_m
    wrench_ok = load_step >= p.lift_min_load_n
    wrench_conclusive = abs(load_step) >= p.lift_min_load_n  # else marginal/ambiguous
    # Kinematics are the reliable signal; wrench corroborates when conclusive.
    ok = rose and (wrench_ok or not wrench_conclusive)
    if not rose:
        detail = (f"TCP rose only {dz_m * 1000:.1f} mm < {p.lift_min_dz_m * 1000:.0f} mm"
                  " -- tray likely NOT lifted; refuse to continue")
    elif wrench_ok:
        detail = (f"TCP +{dz_m * 1000:.1f} mm and wrench +{load_step:.1f} N "
                  ">= loaded step -- tray on forks")
    elif not wrench_conclusive:
        detail = (f"TCP +{dz_m * 1000:.1f} mm; wrench step {load_step:+.1f} N is "
                  "MARGINAL on CB3 (expected on this sensorless arm) -- relying on "
                  "the TCP rise + operator confirm")
    else:
        detail = (f"TCP +{dz_m * 1000:.1f} mm but wrench DROPPED {load_step:+.1f} N "
                  "-- inconsistent with a loaded lift; refuse to continue")
    return VerifyResult(ok=ok, kind="lift", detail=detail, load_step_n=load_step,
                        dz_m=dz_m, wrench_conclusive=wrench_conclusive)


def verify_release(before: Dict[str, float], after: Dict[str, float],
                   p: ContactGuardParams) -> VerifyResult:
    """Confirm the load left the forks after a LOWER, before WITHDRAW."""
    resid = _load_mag(after)
    dropped = (_load_mag(before) - _load_mag(after))
    ok = resid <= p.release_max_resid_n
    if ok:
        detail = (f"residual load {resid:.1f} N <= {p.release_max_resid_n:.0f} N "
                  f"(dropped {dropped:.1f} N) -- load released, safe to withdraw")
    else:
        detail = (f"residual load {resid:.1f} N > {p.release_max_resid_n:.0f} N -- "
                  "tray may still be on the forks; refuse to withdraw")
    return VerifyResult(ok=ok, kind="release", detail=detail, load_step_n=-dropped)
