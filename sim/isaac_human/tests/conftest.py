# SPDX-License-Identifier: Apache-2.0
"""Put the harness dir and the human_safety package on sys.path for the offline
tests (the repo has no colcon/pip install; everything runs as scripts)."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_HARNESS = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(_HARNESS))
_HUMAN_SAFETY = os.path.join(_REPO, "cams", "human_safety")

for p in (_HARNESS, _HUMAN_SAFETY):
    if p not in sys.path:
        sys.path.insert(0, p)
