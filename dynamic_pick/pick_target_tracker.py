# SPDX-License-Identifier: Apache-2.0
"""
Pick-target tracker — a filtered estimate of the ONE object the arm is picking,
with a deadband + hysteresis so the arm chases the target, not the noise
(human-dynamic-plan-s1 report §3).

The cell has no target tracker today: `cams/approach/target_publisher.py` turns a
per-frame best YOLO box into one smoothed 3-D point and only publishes it after 5
still detections with < 3 cm spread AND an IMU-stillness gate — "a moving arm must
never yield a target". A dynamic (re-planned) pick needs a proper tracker. The
report says to reuse the shape of the people tracker in
`cams/human_safety/human_safety/tracker.py`: a constant-velocity Kalman filter
with confirmation and coasting. This is that shape, specialised to a SINGLE
target and extended with the two things a pick needs and the people tracker does
not have:

  * a **committed goal** with a **deadband + sustain (hysteresis)** so the planned
    goal only moves when the smoothed estimate has genuinely shifted (beyond the
    deadband) and stayed shifted (for `n_sustain` fresh updates). Below the
    deadband the committed goal is HELD, so the controller does not re-plan on
    estimate jitter (documented bench jitter 0.1-0.4 cm, approach/README.md);
  * explicit **target-lost** and **target-occluded** states, both fail-toward-HOLD:
    when the target stream stops, the committed goal is dropped and the estimate is
    NEVER extrapolated into a region the arm would then drive toward (report §3).

The module is PURE NumPy: standard library + NumPy only, no rclpy, no torch, no
wall-clock reads of its own (all time is passed in). Every rule is unit-tested
offline with synthetic detections, exactly like the people tracker.

States (`TrackerState`):
  ACQUIRING  a detection stream has started but not yet confirmed (< n_confirm hits)
  TRACKING   confirmed, a fresh measurement arrived within the coast window
  OCCLUDED   confirmed, but coasting through missed measurements (< max_coast_time);
             the Kalman estimate is predicted forward but the committed goal is HELD
  LOST       coast budget exceeded, or never confirmed and gone stale -> no target

Only a TRACKING state (a confirmed track with a fresh measurement) can move the
committed goal. OCCLUDED and LOST both yield `committed_target() is None`, which the
segmented controller reads as "hold, do not plan a new leg" (report §3).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

ACQUIRING = "ACQUIRING"
TRACKING = "TRACKING"
OCCLUDED = "OCCLUDED"
LOST = "LOST"
STATES = (ACQUIRING, TRACKING, OCCLUDED, LOST)


@dataclass(frozen=True)
class TargetEstimate:
    """What the tracker exposes to the controller each tick. Everything the
    segmented re-plan loop needs to decide is here; nothing else is trusted."""

    state: str                       # one of STATES
    filtered_position: Optional[np.ndarray]   # current Kalman position estimate (m, base frame)
    velocity: Optional[np.ndarray]            # Kalman velocity estimate (m/s)
    committed_target: Optional[np.ndarray]    # the goal the arm plans TO (deadband-held); None unless TRACKING
    committed_moved: bool            # did the committed goal move on THIS update (a re-plan trigger)?
    hits: int
    time_since_measurement: float    # s since the last fresh measurement
    stamp: Optional[float]           # evidence stamp of the last measurement


class PickTargetTracker:
    """Single-target constant-velocity Kalman tracker with a deadband/hysteresis
    committed goal and explicit lost/occluded states (report §3).

    Parameters mirror the people tracker's defaults where they apply; the pick
    specific ones are the deadband and its sustain count.
    """

    def __init__(
        self,
        *,
        sigma_meas: float = 0.01,        # 3-D measurement noise, m (bench jitter 0.1-0.4 cm)
        sigma_init_vel: float = 0.2,     # initial velocity uncertainty, m/s (a hand-moved target is slow)
        q_accel: float = 0.5,            # process accel noise, m/s^2 (slow, deliberate repositioning)
        n_confirm: int = 5,              # fresh hits before the target is TRACKED (matches target_publisher N_CONSEC)
        occlusion_after_s: float = 0.3,  # s without a measurement before a TRACKED target is called OCCLUDED (a few detector periods; not a normal inter-frame gap)
        max_coast_time: float = 0.8,     # s a confirmed track may coast (OCCLUDED) before it is LOST
        acquire_timeout: float = 0.6,    # s an unconfirmed (ACQUIRING) track may go without a measurement before LOST
        deadband_m: float = 0.03,        # committed goal moves only when the estimate shifts beyond this (report §3: reuse the 3 cm spread bound)
        n_sustain: int = 3,              # ...and stays shifted for this many fresh updates (hysteresis)
    ) -> None:
        if n_confirm < 1:
            raise ValueError("n_confirm must be >= 1")
        if n_sustain < 1:
            raise ValueError("n_sustain must be >= 1")
        if deadband_m < 0:
            raise ValueError("deadband_m must be >= 0")
        self.sigma_meas = float(sigma_meas)
        self.sigma_init_vel = float(sigma_init_vel)
        self.q_accel = float(q_accel)
        self.n_confirm = int(n_confirm)
        self.occlusion_after_s = float(occlusion_after_s)
        self.max_coast_time = float(max_coast_time)
        self.acquire_timeout = float(acquire_timeout)
        self.deadband_m = float(deadband_m)
        self.n_sustain = int(n_sustain)
        self.reset()

    def reset(self) -> None:
        self._x: Optional[np.ndarray] = None       # [x,y,z,vx,vy,vz]
        self._P: Optional[np.ndarray] = None
        self._state = LOST
        self._hits = 0
        self._stamp: Optional[float] = None
        self._last_meas_mono: Optional[float] = None
        self._committed: Optional[np.ndarray] = None
        # hysteresis bookkeeping: how many consecutive fresh updates the estimate
        # has stayed beyond the deadband from the committed goal.
        self._beyond_count = 0

    # ---- prediction ------------------------------------------------------- #
    def _predict(self, dt: float) -> None:
        if self._x is None or dt <= 0:
            return
        f = np.eye(6)
        f[0, 3] = f[1, 4] = f[2, 5] = dt
        self._x = f @ self._x
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        qv = self.q_accel ** 2
        q = np.zeros((6, 6))
        for i in range(3):
            q[i, i] = dt4 / 4.0 * qv
            q[i, i + 3] = q[i + 3, i] = dt3 / 2.0 * qv
            q[i + 3, i + 3] = dt2 * qv
        self._P = f @ self._P @ f.T + q

    # ---- measurement update ----------------------------------------------- #
    def update(self, position, sigma: Optional[float], stamp: float,
               now: float) -> TargetEstimate:
        """Fold in ONE fresh target measurement.

        `position` is a base-frame (x, y, z) in metres; `sigma` its 3-D std (m) or
        None to use the default; `stamp` the evidence time; `now` monotonic seconds
        (drives coast timing). Returns the post-update estimate.
        """
        pos = np.asarray(position, dtype=float).reshape(3)
        r_std = max(float(sigma), self.sigma_meas) if sigma is not None else self.sigma_meas

        if self._x is None:
            # spawn
            self._x = np.zeros(6)
            self._x[:3] = pos
            p = np.eye(6)
            p[:3, :3] *= r_std ** 2
            p[3:, 3:] *= self.sigma_init_vel ** 2
            self._P = p
            self._hits = 1
        else:
            if self._stamp is not None and stamp > self._stamp:
                self._predict(stamp - self._stamp)
            # Kalman update on position.
            h = np.zeros((3, 6))
            h[0, 0] = h[1, 1] = h[2, 2] = 1.0
            r = np.eye(3) * r_std ** 2
            y = pos - h @ self._x
            s = h @ self._P @ h.T + r
            k = self._P @ h.T @ np.linalg.inv(s)
            self._x = self._x + k @ y
            self._P = (np.eye(6) - k @ h) @ self._P
            self._hits += 1

        self._stamp = float(stamp)
        self._last_meas_mono = float(now)
        self._state = TRACKING if self._hits >= self.n_confirm else ACQUIRING

        moved = self._update_committed()
        return self._estimate(now, committed_moved=moved)

    def _update_committed(self) -> bool:
        """Advance the committed goal with deadband + sustain hysteresis. Returns
        True iff the committed goal moved on this update (a re-plan trigger).

        Only a confirmed (TRACKING) estimate may move the committed goal; while
        ACQUIRING there is no goal to plan to yet.
        """
        if self._state != TRACKING or self._x is None:
            return False
        est = self._x[:3]
        if self._committed is None:
            self._committed = est.copy()
            self._beyond_count = 0
            return True
        if float(np.linalg.norm(est - self._committed)) > self.deadband_m:
            self._beyond_count += 1
            if self._beyond_count >= self.n_sustain:
                self._committed = est.copy()
                self._beyond_count = 0
                return True
        else:
            # back inside the deadband: the shift did not sustain -> jitter, ignore.
            self._beyond_count = 0
        return False

    # ---- time advance without a measurement (coast / lifecycle) ----------- #
    def step(self, now: float) -> TargetEstimate:
        """Advance the tracker's lifecycle to `now` WITHOUT a measurement. Handles
        the coast -> OCCLUDED -> LOST progression. The Kalman estimate is predicted
        forward for situational awareness, but the committed goal is NEVER advanced
        by a coasted prediction (report §3: never chase a phantom)."""
        if self._x is None or self._last_meas_mono is None:
            self._state = LOST
            return self._estimate(now, committed_moved=False)
        dt = now - self._last_meas_mono
        if self._state == ACQUIRING:
            if dt > self.acquire_timeout:
                self._to_lost()
        elif self._state in (TRACKING, OCCLUDED):
            if dt > self.max_coast_time:
                self._to_lost()
            elif dt > self.occlusion_after_s:
                self._state = OCCLUDED
            # else: a normal inter-frame gap (the controller ticks faster than the
            # detector) — stay TRACKING so the committed goal stays available.
        return self._estimate(now, committed_moved=False)

    def _to_lost(self) -> None:
        # fail toward hold: drop the committed goal, keep no phantom to chase.
        self._state = LOST
        self._committed = None
        self._beyond_count = 0

    # ---- readout ---------------------------------------------------------- #
    def _estimate(self, now: float, *, committed_moved: bool) -> TargetEstimate:
        tsm = (0.0 if self._last_meas_mono is None
               else max(0.0, now - self._last_meas_mono))
        pos = None if self._x is None else self._x[:3].copy()
        vel = None if self._x is None else self._x[3:].copy()
        committed = None if (self._state != TRACKING or self._committed is None) \
            else self._committed.copy()
        return TargetEstimate(
            state=self._state,
            filtered_position=pos,
            velocity=vel,
            committed_target=committed,
            committed_moved=bool(committed_moved),
            hits=self._hits,
            time_since_measurement=tsm,
            stamp=self._stamp,
        )

    @property
    def state(self) -> str:
        return self._state
