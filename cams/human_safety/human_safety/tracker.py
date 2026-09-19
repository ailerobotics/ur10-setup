# SPDX-License-Identifier: Apache-2.0
"""Multi-target tracker: gated association + constant-velocity Kalman filter.

State per track is ``[x, y, z, vx, vy, vz]`` in the robot base frame. Position
measurements from any camera update the same track — that is the fusion. The
tracker is pure NumPy/SciPy and fully deterministic, so the association and
lifecycle rules are unit-tested offline with synthetic detections.

Design choices, all defensible for a safety layer that fails toward STOP:

* **Gating with a chi-square threshold** on the Mahalanobis distance between a
  track's predicted position and a detection, so a detection can only update a
  track it is statistically consistent with.
* **Hungarian assignment** (``scipy.optimize.linear_sum_assignment``) over the
  gated cost so two nearby people don't swap IDs on a single frame.
* **Confirmation** (``n_confirm`` hits) before a track is trusted, and
  **coasting** (predicted through missed frames) up to ``max_coast_time`` before
  deletion, so a person seen by only one camera, or briefly occluded, keeps a
  stable ID and keeps contributing to the safety distance.
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2

from .messages import Detection3D, Track


class _KTrack:
    """Internal constant-velocity Kalman track."""

    _next_id = 1

    def __init__(self, det: Detection3D, sigma_meas: float, sigma_init_vel: float):
        self.id = _KTrack._next_id
        _KTrack._next_id += 1
        self.x = np.zeros(6)
        self.x[:3] = det.position
        p = np.eye(6)
        p[:3, :3] *= max(det.sigma, sigma_meas) ** 2
        p[3:, 3:] *= sigma_init_vel ** 2
        self.P = p
        self.stamp = det.stamp
        self.hits = 1
        self.age = 1
        self.time_since_update = 0.0
        self.confirmed = False
        self.nearest_point = det.nearest_point
        self.cameras = {det.camera}

    def predict(self, dt, q_accel):
        if dt <= 0:
            return
        f = np.eye(6)
        f[0, 3] = f[1, 4] = f[2, 5] = dt
        self.x = f @ self.x
        # Continuous white-acceleration process noise (piecewise-constant model).
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        q1 = dt4 / 4.0
        q2 = dt3 / 2.0
        q3 = dt2
        qv = q_accel ** 2
        q = np.zeros((6, 6))
        for i in range(3):
            q[i, i] = q1 * qv
            q[i, i + 3] = q[i + 3, i] = q2 * qv
            q[i + 3, i + 3] = q3 * qv
        self.P = f @ self.P @ f.T + q
        self.time_since_update += dt

    def innovation(self, det: Detection3D, sigma_meas: float):
        h = np.zeros((3, 6))
        h[0, 0] = h[1, 1] = h[2, 2] = 1.0
        r = np.eye(3) * max(det.sigma, sigma_meas) ** 2
        y = det.position - h @ self.x
        s = h @ self.P @ h.T + r
        return y, s, h, r

    def update(self, det: Detection3D, sigma_meas: float):
        y, s, h, r = self.innovation(det, sigma_meas)
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(6) - k @ h) @ self.P
        self.stamp = det.stamp
        self.hits += 1
        self.time_since_update = 0.0
        self.nearest_point = det.nearest_point
        self.cameras.add(det.camera)

    def to_track(self) -> Track:
        return Track(
            id=self.id,
            position=self.x[:3].copy(),
            velocity=self.x[3:].copy(),
            covariance=self.P.copy(),
            stamp=self.stamp,
            hits=self.hits,
            age=self.age,
            time_since_update=self.time_since_update,
            confirmed=self.confirmed,
            nearest_point=None if self.nearest_point is None
            else np.asarray(self.nearest_point).copy(),
            cameras=tuple(sorted(self.cameras)),
        )


class MultiTargetTracker:
    """Gated, Hungarian-assigned constant-velocity multi-person tracker."""

    def __init__(self, sigma_meas=0.15, sigma_init_vel=1.0, q_accel=1.5,
                 gate_prob=0.99, n_confirm=3, max_coast_time=1.5,
                 max_confirmed_coast_time=2.5):
        self.sigma_meas = sigma_meas
        self.sigma_init_vel = sigma_init_vel
        self.q_accel = q_accel
        self.gate_thresh = float(chi2.ppf(gate_prob, df=3))
        self.n_confirm = n_confirm
        self.max_coast_time = max_coast_time
        self.max_confirmed_coast_time = max_confirmed_coast_time
        self.tracks: List[_KTrack] = []
        self._last_stamp: Optional[float] = None

    def reset(self):
        self.tracks = []
        self._last_stamp = None

    def step(self, detections: List[Detection3D], stamp: float) -> List[Track]:
        """Advance the tracker to ``stamp`` with positioned ``detections``.

        Only detections carrying a base-frame position take part; presence-only
        detections are ignored here (fusion handles them separately).
        """
        dets = [d for d in detections if d.positioned]

        # 1. Predict every track forward to this stamp.
        if self._last_stamp is None:
            dt = 0.0
        else:
            dt = max(0.0, stamp - self._last_stamp)
        for t in self.tracks:
            t.predict(dt, self.q_accel)
            t.age += 1
        self._last_stamp = stamp

        # 2. Gated cost matrix (Mahalanobis^2), then Hungarian assignment.
        matches, un_tracks, un_dets = self._associate(dets)

        # 3. Update matched, spawn new, coast/delete unmatched.
        for ti, di in matches:
            self.tracks[ti].update(dets[di], self.sigma_meas)
        for di in un_dets:
            self.tracks.append(
                _KTrack(dets[di], self.sigma_meas, self.sigma_init_vel))

        for t in self.tracks:
            if t.hits >= self.n_confirm:
                t.confirmed = True

        kept = []
        for t in self.tracks:
            budget = (self.max_confirmed_coast_time if t.confirmed
                      else self.max_coast_time)
            if t.time_since_update <= budget:
                kept.append(t)
        self.tracks = kept

        return [t.to_track() for t in self.tracks]

    def confirmed_tracks(self) -> List[Track]:
        return [t.to_track() for t in self.tracks if t.confirmed]

    def _associate(self, dets: List[Detection3D]):
        n_t, n_d = len(self.tracks), len(dets)
        if n_t == 0 or n_d == 0:
            return [], list(range(n_t)), list(range(n_d))

        big = 1e6
        cost = np.full((n_t, n_d), big)
        gated = np.zeros((n_t, n_d), dtype=bool)
        for i, t in enumerate(self.tracks):
            for j, d in enumerate(dets):
                y, s, _, _ = t.innovation(d, self.sigma_meas)
                try:
                    md2 = float(y @ np.linalg.inv(s) @ y)
                except np.linalg.LinAlgError:
                    continue
                if md2 <= self.gate_thresh:
                    cost[i, j] = md2
                    gated[i, j] = True

        rows, cols = linear_sum_assignment(cost)
        matches, un_t, un_d = [], set(range(n_t)), set(range(n_d))
        for r, c in zip(rows, cols):
            if gated[r, c]:
                matches.append((r, c))
                un_t.discard(r)
                un_d.discard(c)
        return matches, sorted(un_t), sorted(un_d)
