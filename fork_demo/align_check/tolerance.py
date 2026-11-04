"""Go / no-go tolerance logic and correction proposal (pure, no I/O).

The lateral tolerance is derived from the channel clearance, never guessed:

    half_clearance = (channel_width - tine_width) / 2       # geometric slack
    lateral_tol    = half_clearance / safety_factor         # shrink -> conservative

A tine centred in the channel has ``half_clearance`` of slack to each rail;
dividing by ``safety_factor`` (>= 1) keeps a margin against calibration and
detection error. ``tune-up-never-down``: raising ``safety_factor`` or lowering
``channel_width`` only tightens the gate.

The correction proposal (dx, dy, dyaw) is the move that would zero the measured
error. It is returned for printing only -- nothing here commands the robot.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict


def lateral_tolerance_mm(channel_width_m: float, tine_width_m: float, safety_factor: float) -> float:
    half_clearance_mm = (channel_width_m - tine_width_m) / 2.0 * 1000.0
    if half_clearance_mm <= 0:
        return 0.0
    return half_clearance_mm / max(safety_factor, 1e-9)


@dataclass
class SideVerdict:
    side: str
    ok: bool
    dx_mm: float          # lateral error, mean of the two tines (shelf) / X error (table)
    dy_mm: float          # table only (0.0 on shelf)
    dyaw_deg: float
    tolerance_mm: float
    tolerance_deg: float
    margin_mm: float      # tolerance_mm - worst measured lateral error (>=0 means pass on position)
    margin_deg: float     # tolerance_deg - |dyaw|
    correction_dx_mm: float
    correction_dy_mm: float
    correction_dyaw_deg: float
    detail: dict = field(default_factory=dict)
    assumptions: list = field(default_factory=list)
    image_stamp: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


def shelf_verdict(
    lateral_per_tine_mm: dict,
    yaw_deg: float,
    *,
    channel_width_m: float,
    tine_width_m: float,
    safety_factor: float,
    yaw_tolerance_deg: float,
    image_stamp: str = "",
    assumptions=None,
    extra_detail=None,
) -> SideVerdict:
    """Build the shelf-side go/no-go.

    ``lateral_per_tine_mm``: {"left": float, "right": float}, signed lateral
    offset of each tine from its channel centre (+ = tine right of channel).
    """
    tol_mm = lateral_tolerance_mm(channel_width_m, tine_width_m, safety_factor)
    worst = max(abs(v) for v in lateral_per_tine_mm.values())
    pos_ok = all(abs(v) <= tol_mm for v in lateral_per_tine_mm.values())
    yaw_ok = abs(yaw_deg) <= yaw_tolerance_deg
    # Correction: shift laterally by the mean error (so both tines re-centre),
    # de-yaw by the measured yaw.
    mean_lat = sum(lateral_per_tine_mm.values()) / len(lateral_per_tine_mm)
    detail = {"lateral_per_tine_mm": {k: round(v, 3) for k, v in lateral_per_tine_mm.items()}}
    if extra_detail:
        detail.update(extra_detail)
    return SideVerdict(
        side="shelf",
        ok=bool(pos_ok and yaw_ok),
        dx_mm=round(mean_lat, 3),
        dy_mm=0.0,
        dyaw_deg=round(yaw_deg, 3),
        tolerance_mm=round(tol_mm, 3),
        tolerance_deg=round(yaw_tolerance_deg, 3),
        margin_mm=round(tol_mm - worst, 3),
        margin_deg=round(yaw_tolerance_deg - abs(yaw_deg), 3),
        correction_dx_mm=round(-mean_lat, 3),
        correction_dy_mm=0.0,
        correction_dyaw_deg=round(-yaw_deg, 3),
        detail=detail,
        assumptions=list(assumptions or []),
        image_stamp=image_stamp,
    )


def table_verdict(
    dx_mm: float,
    dy_mm: float,
    dyaw_deg: float,
    *,
    channel_width_m: float,
    tine_width_m: float,
    safety_factor: float,
    dyaw_tolerance_deg: float,
    image_stamp: str = "",
    assumptions=None,
    extra_detail=None,
) -> SideVerdict:
    """Build the table-side go/no-go.

    dx/dy = planar offset of the taught place pose from the tag-derived dock
    target, in base_link metres*1000; dyaw = yaw error about base Z.
    The set-down must seat in the table dock, so the same channel-clearance
    tolerance bounds both dx and dy.
    """
    tol_mm = lateral_tolerance_mm(channel_width_m, tine_width_m, safety_factor)
    pos_ok = abs(dx_mm) <= tol_mm and abs(dy_mm) <= tol_mm
    yaw_ok = abs(dyaw_deg) <= dyaw_tolerance_deg
    worst = max(abs(dx_mm), abs(dy_mm))
    detail = {}
    if extra_detail:
        detail.update(extra_detail)
    return SideVerdict(
        side="table",
        ok=bool(pos_ok and yaw_ok),
        dx_mm=round(dx_mm, 3),
        dy_mm=round(dy_mm, 3),
        dyaw_deg=round(dyaw_deg, 3),
        tolerance_mm=round(tol_mm, 3),
        tolerance_deg=round(dyaw_tolerance_deg, 3),
        margin_mm=round(tol_mm - worst, 3),
        margin_deg=round(dyaw_tolerance_deg - abs(dyaw_deg), 3),
        correction_dx_mm=round(-dx_mm, 3),
        correction_dy_mm=round(-dy_mm, 3),
        correction_dyaw_deg=round(-dyaw_deg, 3),
        detail=detail,
        assumptions=list(assumptions or []),
        image_stamp=image_stamp,
    )
