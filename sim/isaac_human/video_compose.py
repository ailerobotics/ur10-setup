# SPDX-License-Identifier: Apache-2.0
"""Pure composition helpers for ``make_video.py`` — the offline video composer for
the Isaac human-detection recording.

Everything here is dependency-light: the module top imports only the standard
library, so the unit tests import it in the plain system-python test env without
Isaac, torch, an encoder binary or even Pillow. The functions that actually paint
pixels import Pillow lazily inside the call, so a test that only exercises the
pure maths (timestamp alignment / frame hold, label formatting, even-dimension
sizing, size-cap bitrate choice, trace mapping) never needs Pillow either.

The composer's job: two panes side by side (left = the fixed OBSERVER camera that
sees the whole cell + the SLOW/STOP floor rings + the state beacon; right = the
DETECTOR camera with the detection overlay) plus a bottom strip carrying the
published zone, the TRUE nearest-body distance, the PUBLISHED distance and a thin
distance-vs-time trace with the 2.0 m protective ring and 2.25 m trigger ring.
Panes are time-aligned by wall-clock timestamp; a slower pane holds its last
frame.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


# --- even-dimension sizing (H.264 / yuv420p needs even width and height) ----- #
def even_dim(n: int) -> int:
    """Largest even integer <= n, floored at 2 (H.264 yuv420p requires even)."""
    n = int(n)
    if n < 2:
        return 2
    return n - (n % 2)


def even_size(width: int, height: int) -> Tuple[int, int]:
    return even_dim(width), even_dim(height)


# --- size-cap bitrate choice ------------------------------------------------- #
def choose_bitrate_kbps(target_bytes: int, duration_s: float,
                        headroom: float = 0.90, floor_kbps: int = 150,
                        container_overhead: float = 0.03) -> int:
    """Video bitrate (kbps) that keeps a ``duration_s`` clip near ``target_bytes``.

    ``headroom`` leaves slack under the target; ``container_overhead`` discounts
    the MP4 box/muxing bytes that are not payload. Clamped up to ``floor_kbps`` so
    a very short clip still gets a usable bitrate (quality, not size, then binds).
    """
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    usable = float(target_bytes) * headroom * (1.0 - container_overhead)
    kbps = usable * 8.0 / 1000.0 / duration_s
    return max(int(floor_kbps), int(kbps))


# --- timestamp alignment / frame hold ---------------------------------------- #
def hold_index(times: Sequence[float], t: float) -> int:
    """Index of the most recent frame at or before ``t`` (hold the last frame).

    ``times`` must be sorted ascending. If ``t`` precedes the first frame, index 0
    is returned (show the first frame rather than nothing). Empty ``times`` raises.
    """
    if not times:
        raise ValueError("times is empty")
    i = bisect.bisect_right(times, t) - 1
    if i < 0:
        return 0
    return i


def align_series(primary_times: Sequence[float],
                 secondary_times: Sequence[float]) -> List[int]:
    """For each primary timestamp, the secondary index to display (frame hold)."""
    return [hold_index(secondary_times, t) for t in primary_times]


def build_output_times(win_t0: float, win_t1: float, fps: float) -> List[float]:
    """Evenly spaced output-frame wall-clock times over [win_t0, win_t1]."""
    if fps <= 0:
        raise ValueError("fps must be positive")
    if win_t1 <= win_t0:
        return [win_t0]
    n = int(round((win_t1 - win_t0) * fps))
    n = max(1, n)
    step = 1.0 / fps
    return [win_t0 + i * step for i in range(n + 1)]


def pick_window(t_first: float, t_last: float, target_s: float,
                max_s: float) -> Tuple[float, float]:
    """A [start, end] wall-clock window of up to ``target_s`` (hard cap ``max_s``).

    Starts at ``t_first`` (the first available picture) so the clip opens on the
    person already in the scene, and takes ``min(target_s, available)`` — but
    never more than ``max_s``. Guarantees end > start when any span exists.
    """
    if t_last < t_first:
        raise ValueError("t_last precedes t_first")
    available = t_last - t_first
    want = min(float(target_s), float(max_s))
    span = min(want, available) if available > 0 else 0.0
    return t_first, t_first + span


# --- label formatting -------------------------------------------------------- #
def format_distance(d: Optional[float]) -> str:
    """A published/true distance as ``"2.34 m"``; ``"—"`` when absent."""
    if d is None:
        return "—"
    return f"{float(d):.2f} m"


def format_zone(zone: Optional[str]) -> str:
    return zone if zone else "—"


# RGB (0..255) for the zone label / beacon, matching the live-view colours
# (live_view.ZONE_COLORS, there in 0..1). Unknown/None -> neutral grey.
ZONE_RGB = {
    "CLEAR": (26, 204, 51),
    "SLOW": (255, 178, 0),
    "STOP": (230, 26, 26),
}
UNKNOWN_ZONE_RGB = (115, 115, 115)


def zone_rgb(zone: Optional[str]) -> Tuple[int, int, int]:
    return ZONE_RGB.get(zone, UNKNOWN_ZONE_RGB)


# --- distance-vs-time trace mapping ------------------------------------------ #
@dataclass(frozen=True)
class Box:
    """A pixel rectangle (inclusive-ish); x0<x1, y0<y1, image coords y-down."""
    x0: int
    y0: int
    x1: int
    y1: int

    @property
    def w(self) -> int:
        return self.x1 - self.x0

    @property
    def h(self) -> int:
        return self.y1 - self.y0


def trace_xy(t: float, d: float, win_t0: float, win_t1: float,
             d_max: float, box: Box) -> Tuple[float, float]:
    """Map a (time, distance) sample to a pixel in ``box``.

    Time runs left->right across the box; distance runs bottom(0)->top(d_max),
    clamped into the box. Image y is down, so a larger distance is a smaller y.
    """
    if win_t1 <= win_t0 or d_max <= 0:
        raise ValueError("degenerate axis range")
    fx = (t - win_t0) / (win_t1 - win_t0)
    fx = min(1.0, max(0.0, fx))
    fy = min(1.0, max(0.0, d / d_max))
    x = box.x0 + fx * box.w
    y = box.y1 - fy * box.h
    return x, y


def ring_y(d: float, d_max: float, box: Box) -> float:
    """Pixel y for a horizontal reference line at distance ``d`` (e.g. a ring)."""
    if d_max <= 0:
        raise ValueError("d_max must be positive")
    fy = min(1.0, max(0.0, d / d_max))
    return box.y1 - fy * box.h


# --- self-contained phone-first HTML page ------------------------------------ #
import html as _html  # noqa: E402


def build_video_html(video_b64: str, poster_b64: str, caption_lines: Sequence[str],
                     title: str, video_mime: str = "video/mp4",
                     poster_mime: str = "image/png") -> str:
    """A dark, phone-first, fully self-contained HTML page.

    The video is embedded as a base64 ``data:`` URI inside a
    ``<video controls playsinline muted>`` element with a base64 poster; nothing
    is loaded from the network. ``caption_lines`` are plain-words captions (what
    is shown, what the rings/colours mean, that the person and depth are
    simulated). All text is HTML-escaped.
    """
    caps = "\n".join(
        f'    <p class="cap">{_html.escape(str(c))}</p>' for c in caption_lines)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>{_html.escape(title)}</title>
<style>
  :root {{ color-scheme: dark; }}
  html, body {{ margin: 0; background: #0b0e13; color: #e7e9ee;
    font-family: -apple-system, system-ui, Segoe UI, Roboto, sans-serif; }}
  .wrap {{ max-width: 900px; margin: 0 auto; padding: 16px; }}
  h1 {{ font-size: 1.15rem; font-weight: 700; margin: 8px 0 12px; }}
  video {{ width: 100%; height: auto; border-radius: 10px; background: #000;
    display: block; }}
  .cap {{ font-size: 0.95rem; line-height: 1.4; color: #b9bec9;
    margin: 10px 0 0; }}
  .foot {{ font-size: 0.8rem; color: #6b7280; margin-top: 16px; }}
</style>
</head>
<body>
  <div class="wrap">
    <h1>{_html.escape(title)}</h1>
    <video controls playsinline muted preload="auto"
           poster="data:{poster_mime};base64,{poster_b64}">
      <source src="data:{video_mime};base64,{video_b64}" type="{video_mime}">
      Your browser cannot play this embedded video.
    </video>
{caps}
    <p class="foot">Simulation only — not a certified safety function.</p>
  </div>
</body>
</html>
"""
