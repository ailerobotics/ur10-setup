#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Compose one short phone-playable video from a ``run_session.sh --record`` run.

Offline, runs AFTER the session (no GPU, no ROS, no simulator). It reads the run
directory written under ``~/.cache/isaac_human_sim/run_*``:

  * ``observer_index.jsonl``  — the fixed OBSERVER camera JPEGs (cell + rings +
    beacon), one ``{path,t}`` per frame;
  * ``detector_index.jsonl``  — the DETECTOR camera JPEGs WITH the detection
    overlay (box, keypoints, zone banner), one per processed frame;
  * ``record.jsonl``          — the published ``/human_safety/state`` stream
    (zone + published nearest distance);
  * ``ground_truth.jsonl``    — the simulator's TRUE nearest-body distance.

and builds a ~60-90 s H.264/yuv420p MP4 (``+faststart``, even dims), 1280x720 by
default, capped near a target size: a 2 s title card, then left pane = observer,
right pane = detector-with-overlay (time-aligned by wall clock, slower pane holds
its last frame) and a bottom strip with the zone (coloured), the true + published
nearest distance and a thin distance-vs-time trace marked with the 2.0 m
protective ring and the 2.25 m trigger ring.

Encoding uses the ffmpeg binary bundled with ``imageio-ffmpeg`` (installed into a
venv OUTSIDE the repo — ``ffmpeg`` is not on the system and there is no sudo).

Run it with that venv's python:
    ~/venvs/isaac-video/bin/python sim/isaac_human/make_video.py \
        --run <RUNDIR> --out out.mp4 --poster poster.png
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import subprocess
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import video_compose as vc  # noqa: E402

CANVAS_W, CANVAS_H = 1280, 720
TRACE_D_MAX = 5.0            # metres; the start is ~4.5 m from base
PROTECTIVE_RING = 2.0       # m of base (reach+stop)
TRIGGER_RING = 2.25         # m of base (reach+stop+stop_margin)


# --------------------------------------------------------------------------- #
# Loading the run directory
# --------------------------------------------------------------------------- #
def _read_jsonl(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except Exception:  # noqa: BLE001
                continue
    return rows


def load_frame_index(path):
    """Sorted [(t, path)] from an observer/detector index JSONL."""
    rows = [(float(r["t"]), r["path"]) for r in _read_jsonl(path)
            if r.get("t") is not None and r.get("path")]
    rows.sort(key=lambda r: r[0])
    return rows


def load_states(path):
    """Sorted [(recv, zone, published_nearest_m)] from record.jsonl states."""
    out = []
    for r in _read_jsonl(path):
        if r.get("kind") != "state":
            continue
        recv = r.get("recv")
        if recv is None:
            continue
        pub = r.get("nearest_distance_base")
        if pub is None:
            pub = r.get("nearest_distance")
        out.append((float(recv), r.get("zone"),
                    None if pub is None else float(pub)))
    out.sort(key=lambda r: r[0])
    return out


def load_ground_truth(path):
    """Sorted [(t, true_nearest_m)] from ground_truth.jsonl."""
    out = []
    for r in _read_jsonl(path):
        t = r.get("t")
        d = r.get("dist_base_nearest")
        if t is None or d is None:
            continue
        out.append((float(t), float(d)))
    out.sort(key=lambda r: r[0])
    return out


# --------------------------------------------------------------------------- #
# Fonts / small drawing utilities (Pillow)
# --------------------------------------------------------------------------- #
_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
]


def _load_font(size, bold=False):
    from PIL import ImageFont
    paths = list(_FONT_CANDIDATES)
    if bold:
        paths = [p for p in paths if "Bold" in p] + paths
    for p in paths:
        try:
            return ImageFont.truetype(p, size)
        except Exception:  # noqa: BLE001
            continue
    return ImageFont.load_default()


class _ImageCache:
    """Load-once cache: consecutive output frames hold the same source path."""

    def __init__(self):
        self._path = None
        self._img = None

    def get(self, path):
        from PIL import Image
        if path != self._path:
            self._path = path
            self._img = Image.open(path).convert("RGB")
        return self._img


def _fit_letterbox(img, box_w, box_h, bg=(12, 14, 18)):
    from PIL import Image
    box_w, box_h = max(2, box_w), max(2, box_h)
    iw, ih = img.size
    scale = min(box_w / iw, box_h / ih)
    nw, nh = max(1, int(iw * scale)), max(1, int(ih * scale))
    resized = img.resize((nw, nh), Image.BILINEAR)
    canvas = Image.new("RGB", (box_w, box_h), bg)
    canvas.paste(resized, ((box_w - nw) // 2, (box_h - nh) // 2))
    return canvas


# --------------------------------------------------------------------------- #
# Frame composition
# --------------------------------------------------------------------------- #
def _draw_strip(draw, box, zone, true_d, pub_d, t_now, win_t0, win_t1,
                true_series, pub_series):
    from PIL import ImageDraw  # noqa: F401
    f_big = _load_font(30, bold=True)
    f_mid = _load_font(20, bold=True)
    f_sm = _load_font(16)

    # panel background
    draw.rectangle([box.x0, box.y0, box.x1, box.y1], fill=(20, 22, 28))

    # -- zone chip --------------------------------------------------------- #
    zc = vc.zone_rgb(zone)
    draw.rectangle([box.x0 + 10, box.y0 + 14, box.x0 + 210, box.y0 + 60],
                   fill=zc)
    draw.text((box.x0 + 22, box.y0 + 22), vc.format_zone(zone),
              fill=(15, 15, 15), font=f_big)
    draw.text((box.x0 + 12, box.y0 + 66), "safety state", fill=(150, 150, 160),
              font=f_sm)

    # -- distance readouts ------------------------------------------------- #
    x = box.x0 + 240
    draw.text((x, box.y0 + 12), "true nearest", fill=(150, 150, 160), font=f_sm)
    draw.text((x, box.y0 + 30), vc.format_distance(true_d),
              fill=(120, 220, 255), font=f_mid)
    draw.text((x, box.y0 + 66), "published", fill=(150, 150, 160), font=f_sm)
    draw.text((x, box.y0 + 84), vc.format_distance(pub_d),
              fill=(255, 210, 120), font=f_mid)

    # -- distance-vs-time trace -------------------------------------------- #
    tb = vc.Box(box.x0 + 470, box.y0 + 12, box.x1 - 16, box.y1 - 14)
    draw.rectangle([tb.x0, tb.y0, tb.x1, tb.y1], fill=(12, 14, 18),
                   outline=(60, 64, 74))
    # ring reference lines (2.0 m protective ring, 2.25 m STOP trigger). They sit
    # close together, so label the trigger ABOVE its line and the protective ring
    # BELOW its line, both right-justified, so the two never overlap.
    f_rs = _load_font(14)
    for ring_d, col, lab, dy in (
            (TRIGGER_RING, (255, 170, 60), "2.25 m STOP trigger", -16),
            (PROTECTIVE_RING, (230, 60, 60), "2.0 m protective ring", 3)):
        ry = vc.ring_y(ring_d, TRACE_D_MAX, tb)
        for xx in range(tb.x0, tb.x1, 10):
            draw.line([xx, ry, min(xx + 5, tb.x1), ry], fill=col, width=1)
        tw = draw.textlength(lab, font=f_rs)
        draw.text((tb.x1 - tw - 6, ry + dy), lab, fill=col, font=f_rs)

    def _plot(series, col):
        pts = [vc.trace_xy(t, d, win_t0, win_t1, TRACE_D_MAX, tb)
               for (t, d) in series if d is not None and win_t0 <= t <= win_t1]
        if len(pts) >= 2:
            draw.line([c for xy in pts for c in xy], fill=col, width=2)

    _plot(true_series, (120, 200, 255))
    _plot(pub_series, (255, 200, 110))
    # moving time cursor
    cx = vc.trace_xy(t_now, 0.0, win_t0, win_t1, TRACE_D_MAX, tb)[0]
    draw.line([cx, tb.y0, cx, tb.y1], fill=(210, 210, 220), width=1)
    draw.text((tb.x0 + 4, tb.y1 - 18), "distance from robot vs time",
              fill=(150, 150, 160), font=f_sm)


def compose_frame(observer_img, detector_img, ctx):
    """Build one 1280x720 composited RGB frame (PIL Image)."""
    from PIL import Image, ImageDraw
    canvas = Image.new("RGB", (CANVAS_W, CANVAS_H), (12, 14, 18))
    draw = ImageDraw.Draw(canvas)
    f_hdr = _load_font(20, bold=True)
    f_sm = _load_font(15)

    # header
    draw.text((12, 8), "Isaac Sim — human-detection test", fill=(235, 235, 240),
              font=f_hdr)
    draw.text((CANVAS_W - 220, 12), ctx["date"], fill=(150, 150, 160), font=f_sm)

    # panes
    margin = 8
    pane_top, pane_bottom = 42, CANVAS_H - 176 - margin
    pane_h = pane_bottom - pane_top
    pane_w = (CANVAS_W - 3 * margin) // 2
    lx0 = margin
    rx0 = margin + pane_w + margin
    obs = _fit_letterbox(observer_img, pane_w, pane_h)
    det = _fit_letterbox(detector_img, pane_w, pane_h)
    canvas.paste(obs, (lx0, pane_top))
    canvas.paste(det, (rx0, pane_top))
    draw.text((lx0 + 6, pane_top + 4), "observer — whole cell + rings",
              fill=(200, 220, 200), font=f_sm)
    draw.text((rx0 + 6, pane_top + 4), "detector — overlay (box · keypoints)",
              fill=(200, 220, 200), font=f_sm)

    strip = vc.Box(margin, pane_bottom + margin, CANVAS_W - margin,
                   CANVAS_H - margin)
    _draw_strip(draw, strip, ctx["zone"], ctx["true_d"], ctx["pub_d"],
                ctx["t"], ctx["win_t0"], ctx["win_t1"],
                ctx["true_series"], ctx["pub_series"])
    return canvas


def render_title_card(title, date_str):
    from PIL import Image, ImageDraw
    canvas = Image.new("RGB", (CANVAS_W, CANVAS_H), (10, 12, 16))
    draw = ImageDraw.Draw(canvas)
    f_title = _load_font(40, bold=True)
    f_sub = _load_font(24)
    lines = [title, "simulated person", date_str]
    y = CANVAS_H // 2 - 80
    for i, ln in enumerate(lines):
        font = f_title if i == 0 else f_sub
        col = (240, 240, 245) if i == 0 else (170, 175, 185)
        w = draw.textlength(ln, font=font)
        draw.text(((CANVAS_W - w) / 2, y), ln, fill=col, font=font)
        y += 60 if i == 0 else 40
    return canvas


# --------------------------------------------------------------------------- #
# Encoding + verification
# --------------------------------------------------------------------------- #
def _ffmpeg_exe(override=None):
    if override:
        return override
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def encode(frames_iter, out_path, size, fps, bitrate_kbps):
    """Encode an iterator of HxWx3 uint8 arrays to H.264/yuv420p MP4 (+faststart).

    Uses the ffmpeg binary bundled with imageio-ffmpeg (it has no ``ffmpeg_exe``
    argument; it resolves the bundled binary, or ``$IMAGEIO_FFMPEG_EXE``).
    """
    import imageio_ffmpeg
    w, h = size
    writer = imageio_ffmpeg.write_frames(
        out_path, (w, h), pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
        fps=fps, codec="libx264", macro_block_size=2,
        ffmpeg_log_level="error",
        bitrate=f"{bitrate_kbps}k",
        output_params=["-movflags", "+faststart", "-preset", "medium",
                       "-profile:v", "high", "-pix_fmt", "yuv420p"])
    writer.send(None)
    n = 0
    for frame in frames_iter:
        writer.send(np.ascontiguousarray(frame, dtype=np.uint8))
        n += 1
    writer.close()
    return n


def verify(out_path, ffmpeg_exe):
    """Return a dict describing the encoded file (codec, pix_fmt, faststart)."""
    info = {"path": out_path, "bytes": os.path.getsize(out_path)}
    # codec / pix_fmt via `ffmpeg -i` (stderr).
    p = subprocess.run([ffmpeg_exe, "-hide_banner", "-i", out_path],
                       capture_output=True, text=True)
    txt = p.stderr
    info["has_h264"] = "h264" in txt.lower()
    info["has_yuv420p"] = "yuv420p" in txt.lower()
    for ln in txt.splitlines():
        if "Duration:" in ln:
            info["ffmpeg_stream_line"] = ln.strip()
            break
    # faststart: the `moov` atom must appear before `mdat` (top-level boxes).
    info["faststart"] = _moov_before_mdat(out_path)
    return info


def _moov_before_mdat(path):
    """Scan top-level MP4 boxes; True if 'moov' precedes 'mdat' (faststart)."""
    order = []
    with open(path, "rb") as f:
        pos = 0
        f.seek(0, os.SEEK_END)
        end = f.tell()
        f.seek(0)
        while pos + 8 <= end and len(order) < 64:
            f.seek(pos)
            header = f.read(8)
            if len(header) < 8:
                break
            size = int.from_bytes(header[:4], "big")
            typ = header[4:8].decode("latin-1", "replace")
            order.append(typ)
            if size == 1:            # 64-bit largesize
                ext = f.read(8)
                size = int.from_bytes(ext, "big")
            if size < 8:
                break
            pos += size
    try:
        return order.index("moov") < order.index("mdat")
    except ValueError:
        return False


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #
def build(run_dir, out_path, poster_path, fps, target_bytes, max_bytes,
          target_s, max_s, title, date_str, ffmpeg_exe):
    obs = load_frame_index(os.path.join(run_dir, "observer_index.jsonl"))
    det = load_frame_index(os.path.join(run_dir, "detector_index.jsonl"))
    states = load_states(os.path.join(run_dir, "record.jsonl"))
    gt = load_ground_truth(os.path.join(run_dir, "ground_truth.jsonl"))
    if not obs:
        raise SystemExit(f"no observer frames in {run_dir} (did the record run "
                         f"produce observer_index.jsonl?)")
    if not det:
        raise SystemExit(f"no detector frames in {run_dir}")

    obs_t = [t for (t, _p) in obs]
    det_t = [t for (t, _p) in det]
    st_t = [t for (t, _z, _d) in states]
    gt_t = [t for (t, _d) in gt]

    win_t0, win_t1 = vc.pick_window(obs_t[0], obs_t[-1], target_s, max_s)
    out_times = vc.build_output_times(win_t0, win_t1, fps)
    duration = max(1.0 / fps, out_times[-1] - out_times[0])

    true_series = [(t, d) for (t, d) in gt if win_t0 <= t <= win_t1]
    pub_series = [(t, d) for (t, z, d) in states
                  if d is not None and win_t0 <= t <= win_t1]

    obs_cache, det_cache = _ImageCache(), _ImageCache()

    def _held(times, seq, t):
        return seq[vc.hold_index(times, t)] if times else None

    def _frames():
        # title card (2 s)
        title_img = np.asarray(render_title_card(title, date_str))
        for _ in range(int(round(2.0 * fps))):
            yield title_img
        for t in out_times:
            o_path = obs[vc.hold_index(obs_t, t)][1]
            d_path = det[vc.hold_index(det_t, t)][1]
            observer_img = obs_cache.get(o_path)
            detector_img = det_cache.get(d_path)
            zone = _held(st_t, [z for (_t, z, _d) in states], t)
            pub_d = _held(st_t, [d for (_t, _z, d) in states], t)
            true_d = _held(gt_t, [d for (_t, d) in gt], t)
            ctx = {"zone": zone, "pub_d": pub_d, "true_d": true_d, "t": t,
                   "win_t0": win_t0, "win_t1": win_t1, "date": date_str,
                   "true_series": true_series, "pub_series": pub_series}
            yield np.asarray(compose_frame(observer_img, detector_img, ctx))

    # bitrate for the size cap (title card + body)
    total_s = duration + 2.0
    bitrate = vc.choose_bitrate_kbps(target_bytes, total_s)
    size = vc.even_size(CANVAS_W, CANVAS_H)
    n = encode(_frames(), out_path, size, fps, bitrate)

    # If we overshot the hard cap, re-encode once at a lower bitrate.
    if os.path.getsize(out_path) > max_bytes:
        lower = vc.choose_bitrate_kbps(int(max_bytes * 0.85), total_s)
        encode(_frames(), out_path, size, fps, lower)

    # poster: a body frame near the middle of the window (person in the scene).
    mid_t = 0.5 * (win_t0 + win_t1)
    o_path = obs[vc.hold_index(obs_t, mid_t)][1]
    d_path = det[vc.hold_index(det_t, mid_t)][1]
    zone = _held(st_t, [z for (_t, z, _d) in states], mid_t)
    pub_d = _held(st_t, [d for (_t, _z, d) in states], mid_t)
    true_d = _held(gt_t, [d for (_t, d) in gt], mid_t)
    ctx = {"zone": zone, "pub_d": pub_d, "true_d": true_d, "t": mid_t,
           "win_t0": win_t0, "win_t1": win_t1, "date": date_str,
           "true_series": true_series, "pub_series": pub_series}
    poster = compose_frame(obs_cache.get(o_path), det_cache.get(d_path), ctx)
    poster.save(poster_path)

    info = verify(out_path, ffmpeg_exe)
    info.update({"frames": n, "fps": fps, "duration_s": round(total_s, 2),
                 "bitrate_kbps": bitrate, "size": size,
                 "observer_frames": len(obs), "detector_frames": len(det),
                 "window_s": round(win_t1 - win_t0, 2)})
    return info


DEFAULT_CAPTION = [
    "A simulated person walks in toward a simulated UR10 robot cell and back out. "
    "Left: a fixed observer camera sees the whole cell; the amber ring on the "
    "floor is SLOW (3.3 m) and the red ring is STOP (2.0 m). Right: one depth "
    "camera's view with the detection layer's box and skeleton drawn on.",
    "The coloured chip is the safety state the layer published — green CLEAR, "
    "amber SLOW, red STOP. “true nearest” is the person's real distance "
    "from the robot; “published” is what the detector reported. STOP is "
    "triggered at 2.25 m to cover the estimator's error.",
    "Everything here is simulated — the person, the cameras and the depth are all "
    "rendered in Isaac Sim. This is a test of the detection layer, not a certified "
    "safety function.",
]


def _write_html(html_path, video_path, poster_path, title):
    import base64
    with open(video_path, "rb") as f:
        video_b64 = base64.b64encode(f.read()).decode("ascii")
    with open(poster_path, "rb") as f:
        poster_b64 = base64.b64encode(f.read()).decode("ascii")
    poster_mime = "image/png" if poster_path.lower().endswith(".png") else "image/jpeg"
    page = vc.build_video_html(video_b64, poster_b64, DEFAULT_CAPTION, title,
                               poster_mime=poster_mime)
    with open(html_path, "w") as f:
        f.write(page)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", required=True, help="run directory (run_*)")
    ap.add_argument("--out", required=True, help="output MP4 path")
    ap.add_argument("--poster", required=True, help="output poster PNG path")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--target-mb", type=float, default=6.0,
                    help="target size (MB); bitrate chosen to sit near this")
    ap.add_argument("--max-mb", type=float, default=10.0,
                    help="hard cap (MB); re-encode lower once if exceeded")
    ap.add_argument("--duration", type=float, default=75.0,
                    help="target clip length in seconds (body, excl. title)")
    ap.add_argument("--max-duration", type=float, default=90.0)
    ap.add_argument("--title", default="Isaac Sim — human detection test")
    ap.add_argument("--date", default=_dt.date.today().isoformat())
    ap.add_argument("--ffmpeg", default=None, help="override ffmpeg binary path")
    ap.add_argument("--html", default=None,
                    help="also write a self-contained phone-first video.html "
                         "(video + poster embedded as base64 data: URIs)")
    args = ap.parse_args(argv)

    ffmpeg_exe = _ffmpeg_exe(args.ffmpeg)
    info = build(args.run, args.out, args.poster, args.fps,
                 int(args.target_mb * 1e6), int(args.max_mb * 1e6),
                 args.duration, args.max_duration, args.title, args.date,
                 ffmpeg_exe)
    if args.html:
        _write_html(args.html, args.out, args.poster, args.title)
        info["html"] = args.html
        info["html_bytes"] = os.path.getsize(args.html)
    print(json.dumps(info, indent=2))
    mb = info["bytes"] / 1e6
    ok = info["has_h264"] and info["has_yuv420p"] and info["faststart"]
    print(f"\n{'OK' if ok else 'CHECK'}: {mb:.2f} MB, {info['duration_s']}s, "
          f"h264={info['has_h264']} yuv420p={info['has_yuv420p']} "
          f"faststart={info['faststart']}", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
