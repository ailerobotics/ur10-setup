# SPDX-License-Identifier: Apache-2.0
"""Fusion: merge per-camera 3D detections into base-frame person tracks.

Thin orchestrator around :class:`MultiTargetTracker`. Positioned detections from
every camera feed the same tracker (that is the multi-camera fusion); a camera
that only reports presence (no trustworthy pose, or the wrist without joint
states) contributes to ``presence_cameras`` so the safety layer still refuses
CLEAR when someone is visible but unlocalised.
"""
from __future__ import annotations

from typing import List, Tuple

from .messages import Detection3D, Track
from .tracker import MultiTargetTracker


class Fusion:
    def __init__(self, tracker: MultiTargetTracker | None = None):
        self.tracker = tracker or MultiTargetTracker()

    def step(self, detections: List[Detection3D],
             stamp: float) -> Tuple[List[Track], set]:
        """Advance fusion to ``stamp``. Returns (tracks, presence_cameras)."""
        positioned = [d for d in detections if d.positioned]
        presence_cameras = {
            d.camera for d in detections if d.present and not d.positioned}
        tracks = self.tracker.step(positioned, stamp)
        return tracks, presence_cameras

    def reset(self):
        self.tracker.reset()
