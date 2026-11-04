"""Shelf side: detect the two yellow tape strips under the dock channels.

Each strip marks the lateral centre of a fork channel (captain's guide: LEFT
strip by STUDENTENFUTTER, RIGHT by BIO MARONEN). We return each strip's image
centroid and orientation, plus the rail direction (the line through the two
strip centres -- the front price-tag rail the insert must be square to).

OpenCV is used only for colour thresholding and connected components; the
returned geometry (centroids, angles) is deterministic and is exercised by
tests on a downscaled copy of the real guide photo.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import line_angle_deg


@dataclass
class StripParams:
    # HSV gate for the matte yellow tape. Broad enough for the lit guide photo;
    # narrow V floor rejects the white dock and price-tag labels.
    h_lo: int = 18
    h_hi: int = 38
    s_lo: int = 80
    v_lo: int = 80
    open_k: int = 9
    close_k: int = 15
    # Minimum blob area as a fraction of the image, to drop specks.
    min_area_frac: float = 0.0008


@dataclass
class Strip:
    cx: float
    cy: float
    angle_deg: float   # orientation of the strip's long axis (minAreaRect)
    area: int
    bbox: tuple        # (x, y, w, h)


@dataclass
class ShelfDetection:
    strips: list       # left-to-right list of Strip
    rail_angle_deg: float   # angle of the line through the two strip centres
    mask: np.ndarray

    @property
    def ok(self) -> bool:
        return len(self.strips) == 2


def _yellow_mask(bgr: np.ndarray, p: StripParams) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lo = np.array([p.h_lo, p.s_lo, p.v_lo])
    hi = np.array([p.h_hi, 255, 255])
    m = cv2.inRange(hsv, lo, hi)
    if p.open_k > 0:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((p.open_k, p.open_k), np.uint8))
    if p.close_k > 0:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((p.close_k, p.close_k), np.uint8))
    return m


def detect_strips(bgr: np.ndarray, p: StripParams | None = None) -> ShelfDetection:
    p = p or StripParams()
    mask = _yellow_mask(bgr, p)
    h, w = mask.shape[:2]
    min_area = p.min_area_frac * h * w
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    found = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        ys, xs = np.where(labels == i)
        pts = np.stack([xs, ys], axis=1).astype(np.float32)
        (_, _), (rw, rh), ang = cv2.minAreaRect(pts)
        # minAreaRect angle -> long-axis angle in image coords.
        long_angle = ang if rw >= rh else ang + 90.0
        found.append(
            Strip(
                cx=float(cents[i, 0]),
                cy=float(cents[i, 1]),
                angle_deg=float(long_angle),
                area=area,
                bbox=(
                    int(stats[i, cv2.CC_STAT_LEFT]),
                    int(stats[i, cv2.CC_STAT_TOP]),
                    int(stats[i, cv2.CC_STAT_WIDTH]),
                    int(stats[i, cv2.CC_STAT_HEIGHT]),
                ),
            )
        )
    # Keep the two largest, order left-to-right.
    found.sort(key=lambda s: -s.area)
    strips = sorted(found[:2], key=lambda s: s.cx)
    if len(strips) == 2:
        rail = line_angle_deg((strips[0].cx, strips[0].cy), (strips[1].cx, strips[1].cy))
    elif len(strips) == 1:
        rail = strips[0].angle_deg
    else:
        rail = 0.0
    return ShelfDetection(strips=strips, rail_angle_deg=rail, mask=mask)


def annotate(bgr: np.ndarray, det: ShelfDetection, projected=None) -> np.ndarray:
    """Draw strip centres, rail line, and (optionally) projected tine tips."""
    out = bgr.copy()
    for s in det.strips:
        c = (int(s.cx), int(s.cy))
        cv2.circle(out, c, 16, (0, 200, 255), 3)
        cv2.drawMarker(out, c, (0, 200, 255), cv2.MARKER_CROSS, 40, 3)
        x, y, w, h = s.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 200, 255), 2)
    if len(det.strips) == 2:
        a = (int(det.strips[0].cx), int(det.strips[0].cy))
        b = (int(det.strips[1].cx), int(det.strips[1].cy))
        cv2.line(out, a, b, (0, 160, 255), 2)
    if projected:
        for side, pr in projected.items():
            u, v = int(pr["tip_uv"][0]), int(pr["tip_uv"][1])
            cv2.circle(out, (u, v), 14, (255, 100, 0), 3)
            cv2.drawMarker(out, (u, v), (255, 100, 0), cv2.MARKER_TILTED_CROSS, 34, 3)
            cv2.putText(out, f"tine_{side}", (u + 18, v), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 100, 0), 2, cv2.LINE_AA)
    return out
