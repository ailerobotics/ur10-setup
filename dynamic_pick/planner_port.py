# SPDX-License-Identifier: Apache-2.0
"""
Planner port — the abstraction the segmented controller plans through, plus a
PURE, GPU-free analytic planner for the mock/URSim proofs.

The controller decides a Cartesian pick target and needs a JOINT trajectory to
put on `FollowJointTrajectory`. Two things sit behind this port:

  * `CartesianToJointSurrogate` — a SIM surrogate for the seeded `/compute_ik`
    step the real cell uses (AGENTS.md: "for predictable motion use seeded
    /compute_ik + a joint-space goal"). It maps a base-frame Cartesian target
    inside a workspace box to a joint-space goal inside the MEASURED joint windows.
    It is a deterministic, clamped linear map — NOT a kinematics solver — used only
    so the sim arm's joints move coherently as the scripted Cartesian target moves.
    On the real cell this is replaced by seeded IK; the controller and the joint-
    window guard are unchanged.

  * `Planner` — plan q0 -> q_goal. `AnalyticJointPlanner` (here) is pure: a
    joint-space time-scaled straight line at the cell's 5 % speed cap, with every
    waypoint validated inside the joint windows and a zero terminal velocity (the
    scaled_JTC rejects a non-zero terminal velocity — execute_plan.py). It needs no
    GPU and no ROS, so it drives the mock + URSim proofs and is unit-tested offline.
    The cuMotion-backed planner (collision-aware, through the nvblox ESDF) lives in
    the ROS layer (`cumotion_planner.py`) and implements this same interface.

Joint-window enforcement is NON-NEGOTIABLE and identical to the cell's executor
(`execute_plan.py:116,213-217`, `loop_tags.py:108-111`): every waypoint of every
leg must sit inside `measured_joint_limits.yaml`. A plan that would leave the
windows is refused, never clamped silently.

Pure: standard library + NumPy only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

# Canonical joint order (matches execute_plan.JOINTS).
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Per-joint velocity limits (rad/s) — execute_plan.py VEL.
VEL = {'shoulder_pan_joint': 2.09, 'shoulder_lift_joint': 2.09,
       'elbow_joint': 3.14, 'wrist_1_joint': 3.14,
       'wrist_2_joint': 3.14, 'wrist_3_joint': 3.14}
# The cell's hard speed cap (execute_plan.SPEED).
SPEED = 0.05


_2PI = 2.0 * math.pi


def wrap_to_nearest(target, ref):
    """Shift `target` by whole turns to the branch nearest `ref` (shortest joint
    travel). Element-wise on arrays. This is what a real planner/controller does
    for a continuous joint: -1.57 rad and 4.71 rad are the same wrist orientation,
    so command whichever is closest to where the arm already is."""
    target = np.asarray(target, float)
    ref = np.asarray(ref, float)
    return ref + ((target - ref + math.pi) % _2PI - math.pi)


def in_window_wrapped(v, lo, hi):
    """True where the PHYSICAL angle of `v` (any 2π branch) lies in [lo, hi].
    A continuous joint's safe-travel window is an arc; -1.57 rad is in-window iff
    4.71 rad is. Element-wise; returns a bool array. This is the wrapping-aware form
    of the cell executor's `lo <= v <= hi` guard — identical on the real arm (whose
    reported values already sit in the window branch), tolerant of the sim arm's
    different home branch."""
    v = np.asarray(v, float)
    lo = np.asarray(lo, float)
    hi = np.asarray(hi, float)
    vw = lo + np.mod(v - lo, _2PI)      # the branch of v that is just above lo
    return vw <= hi + 1e-9


def joint_windows(limits: dict) -> Tuple[np.ndarray, np.ndarray]:
    """Return (lo, hi) rad arrays in JOINTS order from a measured-limits yaml dict
    (reference_pose_rad + window_deg), exactly as execute_plan/loop_tags read it."""
    ref, win = limits['reference_pose_rad'], limits['window_deg']
    lo = np.array([ref[j] - math.radians(win[j]) for j in JOINTS])
    hi = np.array([ref[j] + math.radians(win[j]) for j in JOINTS])
    return lo, hi


@dataclass(frozen=True)
class PlannerResult:
    ok: bool
    joint_names: List[str]
    positions: List[List[float]]      # waypoint positions (JOINTS order)
    times_s: List[float]              # time_from_start for each waypoint
    planning_time_s: float            # solver time (analytic: ~0)
    message: str

    @property
    def duration_s(self) -> float:
        return self.times_s[-1] if self.times_s else 0.0


class Planner:
    """Abstract planner. Implementations plan q0 -> q_goal in joint space and MUST
    return only trajectories whose every waypoint is inside the joint windows."""

    def plan(self, q0, q_goal, lo, hi, seconds: float = 25.0) -> PlannerResult:
        raise NotImplementedError


class AnalyticJointPlanner(Planner):
    """Pure joint-space straight-line planner at the 5 % speed cap. No GPU, no ROS,
    no collision model — it is the mock/URSim stand-in for cuMotion so the whole
    control loop (tracker -> phase -> shim -> preempt -> execute) is exercised
    without the planner container. It is NOT collision-aware; the cuMotion planner
    is what provides that (report §1)."""

    def __init__(self, *, dt_s: float = 0.5, speed: float = SPEED) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be > 0")
        if not (0 < speed <= 1.0):
            raise ValueError("speed must be in (0, 1]")
        self.dt_s = float(dt_s)
        self.speed = float(speed)

    def plan(self, q0, q_goal, lo, hi, seconds: float = 25.0) -> PlannerResult:
        q0 = np.asarray(q0, float).reshape(6)
        qg = np.asarray(q_goal, float).reshape(6)
        lo = np.asarray(lo, float).reshape(6)
        hi = np.asarray(hi, float).reshape(6)

        # refuse a goal whose PHYSICAL angle is outside the windows (wrapping-aware).
        if not in_window_wrapped(qg, lo, hi).all():
            bad = [JOINTS[i] for i in range(6) if not in_window_wrapped(qg[i], lo[i], hi[i])]
            return PlannerResult(False, JOINTS, [], [], 0.0,
                                 f"goal outside measured joint windows: {bad}")
        # command the goal on the branch nearest the live start (shortest travel);
        # this makes the leg start exactly at q0 and move the short way, exactly as
        # a real planner does for continuous joints.
        qg = wrap_to_nearest(qg, q0)

        vmax = np.array([VEL[j] for j in JOINTS])
        delta = qg - q0
        # time so the fastest joint peaks at exactly `speed` of its limit.
        t_total = float(np.max(np.abs(delta) / (self.speed * vmax)))
        t_total = max(t_total, self.dt_s)     # at least one step
        n = max(2, int(math.ceil(t_total / self.dt_s)))
        positions, times = [], []
        for k in range(n + 1):
            frac = k / n
            q = q0 + delta * frac
            # validate every waypoint's PHYSICAL angle inside the windows (the
            # guarantee the executor relies on); fail loudly if it ever leaves.
            if not in_window_wrapped(q, lo, hi).all():
                bad = [JOINTS[i] for i in range(6) if not in_window_wrapped(q[i], lo[i], hi[i])]
                return PlannerResult(False, JOINTS, [], [], 0.0,
                                     f"waypoint {k} outside joint windows: {bad}")
            positions.append([float(v) for v in q])
            times.append(frac * t_total)
        return PlannerResult(True, JOINTS, positions, times, 0.0,
                             f"analytic straight line, {n + 1} pts, {t_total:.1f} s at "
                             f"{self.speed * 100:.0f}% cap")


@dataclass
class CartesianToJointSurrogate:
    """SIM surrogate for seeded IK (see module docstring). Maps a base-frame
    Cartesian target inside a workspace box to a joint-space goal inside the
    measured windows by a clamped linear map. Deterministic and monotonic so a
    moving Cartesian target produces a coherently moving joint goal. NOT kinematics.

    `box_min`/`box_max` define the Cartesian workspace the map spans (m, base frame);
    a target is clamped into the box first. `frac_lo`/`frac_hi` pick how much of each
    joint's window the box maps across (kept well inside the window edges so a
    clamped target never lands on a window boundary)."""

    lo: np.ndarray
    hi: np.ndarray
    box_min: np.ndarray = field(default_factory=lambda: np.array([0.3, -0.4, 0.1]))
    box_max: np.ndarray = field(default_factory=lambda: np.array([0.9, 0.4, 0.6]))
    frac_lo: float = 0.30
    frac_hi: float = 0.70

    def to_joint(self, target_xyz) -> np.ndarray:
        t = np.asarray(target_xyz, float).reshape(3)
        span = np.maximum(self.box_max - self.box_min, 1e-6)
        u = np.clip((t - self.box_min) / span, 0.0, 1.0)   # 0..1 per Cartesian axis
        # map the 3 Cartesian axes onto 3 joints (pan<-x, lift<-y, elbow<-z) and
        # hold wrists at their window centre; enough to make the arm move coherently
        # with the target in sim.
        centre = 0.5 * (self.lo + self.hi)
        q = centre.copy()
        for axis, jidx in ((0, 0), (1, 1), (2, 2)):
            f = self.frac_lo + (self.frac_hi - self.frac_lo) * u[axis]
            q[jidx] = self.lo[jidx] + f * (self.hi[jidx] - self.lo[jidx])
        return np.clip(q, self.lo, self.hi)

    def to_cartesian(self, q) -> np.ndarray:
        """Inverse of `to_joint` for the 3 mapped joints — a SIM-ONLY surrogate TCP
        position, so the phase machine's TCP-to-target distance is self-consistent
        with the joint goals the arm is driven to (when the arm reaches the joint
        goal, this returns the Cartesian target). NOT forward kinematics; on the real
        cell TCP comes from FK/TF. Returns the box-frame Cartesian estimate."""
        q = np.asarray(q, float).reshape(6)
        span_j = self.frac_hi - self.frac_lo
        box_span = self.box_max - self.box_min
        t = np.zeros(3)
        for axis, jidx in ((0, 0), (1, 1), (2, 2)):
            win = self.hi[jidx] - self.lo[jidx]
            f = (q[jidx] - self.lo[jidx]) / win if win > 1e-9 else self.frac_lo
            u = (f - self.frac_lo) / span_j if abs(span_j) > 1e-9 else 0.0
            t[axis] = self.box_min[axis] + np.clip(u, 0.0, 1.0) * box_span[axis]
        return t
