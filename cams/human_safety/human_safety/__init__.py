# SPDX-License-Identifier: Apache-2.0
"""human_safety — camera-based 3D human detection, tracking and safety-state
core for the UR10 cell.

This package is deliberately split so the *decision* logic (geometry, tracking,
zone/hysteresis/health state machine) is pure Python + NumPy with no ROS, no
torch and no GPU dependency, and is therefore testable offline and
deterministically. The ROS nodes (``cams/human_safety/nodes/``) and the torch
detector are thin shells around these modules.

Supplementary layer only: see the package README. This is NOT a certified
safety function.
"""

__all__ = [
    "messages",
    "geometry",
    "tracker",
    "safety",
    "fusion",
    "config",
    "detector",
    "imgcodec",
]
