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


def gather_evidence(dets_by_role, now, frame_age_budget):
    """Collect fresh per-camera detections and derive the STOP-ring evidence.

    ``dets_by_role`` maps each camera role to ``(recv_wall, detections,
    msg_stamp)`` — the latest message from that camera. A camera whose latest
    message is itself stale (its detector stopped) is dropped so a latched last
    message can't keep a phantom person.

    Returns ``(all_dets, evidence_stamp, latency)``. The evidence stamp and the
    image->state latency come ONLY from positioned detections — the ones that
    localise a person and size the STOP ring — using the OLDEST contributing
    stamp so the latency is conservative. A camera streaming fresh empty or
    presence-only frames therefore can never make older person evidence look
    fresh; when nobody is being localised both are ``None``.
    """
    all_dets = []
    for recv, dets, _msg_stamp in dets_by_role.values():
        if now - recv <= frame_age_budget + 0.5:
            all_dets.extend(dets)
    positioned_stamps = [d.stamp for d in all_dets if d.positioned]
    evidence_stamp = min(positioned_stamps) if positioned_stamps else None
    latency = (now - evidence_stamp) if evidence_stamp is not None else None
    return all_dets, evidence_stamp, latency
