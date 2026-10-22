#!/usr/bin/env python3
"""Regenerate the README demo media from the real teach-and-replay records.

Every artifact in this directory is derived from on-robot data, never hand-drawn:

  fork-pick-trajectory.gif / .mp4  animated 3D plot of the fork TCP path recorded
                                   during the teach-002 session (shelf -> table ->
                                   shelf fork-pick cycle), arc-length resampled so
                                   the long teach dwells compress into smooth motion.
  cell-three-cameras.png           the three real RealSense colour views of the cell
                                   (wrist D435i eye-in-hand, shelf D415, table D415).

The source recordings live OUTSIDE this repository (they are large, operator-local
and not committed): the teach session under a teach-002 run directory and the camera
stills under a cams-check run directory. Point --teach-dir / --stills-dir at them to
reproduce. The MP4 step needs an ffmpeg binary; pass --ffmpeg or install the
`imageio-ffmpeg` wheel (its bundled binary is auto-discovered).

Run:  python3 docs/media/gen_media.py --teach-dir <...> --stills-dir <...> --out docs/media
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)
from PIL import Image, ImageDraw, ImageSequence


def load_fork_path(teach_dir):
    """Return (resampled_path Nx3, per-frame time, labelled waypoints, raw path)."""
    ts, fork = [], []
    with open(os.path.join(teach_dir, "teach_log.jsonl")) as fh:
        for line in fh:
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if d.get("fork") is None:
                continue
            ts.append(d["t"])
            fork.append(d["fork"])
    ts = np.asarray(ts) - ts[0]
    fork = np.asarray(fork)
    wp = json.load(open(os.path.join(teach_dir, "labelled_waypoints.json")))["waypoints"]
    labels = [(w["label"], w["t_rel"], np.asarray(w["fork_mean"]))
              for w in wp if not w.get("segment")]
    # arc-length resample: stationary dwells accumulate no length, so they vanish
    seglen = np.linalg.norm(np.diff(fork, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seglen)])
    idx = np.clip(np.searchsorted(cum, np.linspace(0, cum[-1], 300)), 0, len(fork) - 1)
    return fork[idx], ts[idx], labels, fork


def render_frames(path, ptime, labels, raw, frame_dir):
    os.makedirs(frame_dir, exist_ok=True)
    xr = (raw[:, 0].min() - 0.05, raw[:, 0].max() + 0.05)
    yr = (raw[:, 1].min() - 0.05, raw[:, 1].max() + 0.05)
    zr = (0, raw[:, 2].max() + 0.05)

    def phase_at(t):
        name = "home"
        for lab, tt, _ in labels:
            if t >= tt:
                name = lab
        return name

    n = len(path)
    for fi in range(n):
        fig = plt.figure(figsize=(7.2, 5.4), dpi=100)
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(raw[:, 0], raw[:, 1], raw[:, 2], color="#d0d0d0", lw=0.8, alpha=0.6)
        ax.plot(path[:fi + 1, 0], path[:fi + 1, 1], path[:fi + 1, 2], color="#1f77b4", lw=2.0)
        ax.scatter(*path[fi], color="#d62728", s=55, depthshade=False)
        ax.scatter(0, 0, 0, color="black", marker="^", s=60)
        ax.text(0, 0, 0.04, "base_link", fontsize=7)
        for _, _, p in labels:
            ax.scatter(*p, color="#2ca02c", s=18, alpha=0.7)
        ax.set_xlim(*xr); ax.set_ylim(*yr); ax.set_zlim(*zr)
        ax.set_xlabel("x (m)", fontsize=8)
        ax.set_ylabel("y (m)", fontsize=8)
        ax.set_zlabel("z (m)", fontsize=8)
        ax.tick_params(labelsize=6)
        ax.view_init(elev=24, azim=-60 + 30 * np.sin(fi / n * np.pi))
        ax.set_title("UR10 fork-pick teach-and-replay  ·  real taught TCP path\n"
                     f"phase: {phase_at(ptime[fi])}", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(frame_dir, f"f{fi:04d}.png"))
        plt.close(fig)
    return n


def build_gif(frame_dir, out_gif):
    frames = sorted(glob.glob(os.path.join(frame_dir, "f*.png")))
    gif = []
    for p in frames[::2]:
        im = Image.open(p).convert("RGB")
        w, h = im.size
        im = im.resize((520, int(h * 520 / w)), Image.LANCZOS)
        gif.append(im.convert("P", palette=Image.ADAPTIVE, colors=128))
    gif[0].save(out_gif, save_all=True, append_images=gif[1:],
                duration=80, loop=0, optimize=True, disposal=2)


def build_mp4(frame_dir, out_mp4, ffmpeg):
    subprocess.run([ffmpeg, "-y", "-framerate", "25",
                    "-i", os.path.join(frame_dir, "f%04d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", "-crf", "24",
                    "-vf", "scale=720:-2", out_mp4], check=True)


def build_cameras(stills_dir, out_png):
    imgs = [
        ("wrist_d435i_018322070619_color.png",
         "Wrist  ·  RealSense D435i (eye-in-hand, mounted upside-down → /wrist/color_upright)"),
        ("shelf_d415_021422061772_color.png", "Shelf  ·  RealSense D415 (fixed)"),
        ("table_d415_025522060953_color.png", "Table  ·  RealSense D415 (fixed)"),
    ]
    W, bg = 640, (25, 30, 40)
    rows = []
    for fn, cap in imgs:
        im = Image.open(os.path.join(stills_dir, fn)).convert("RGB")
        w, h = im.size
        im = im.resize((W, int(h * W / w)), Image.LANCZOS)
        band = Image.new("RGB", (W, 24), bg)
        ImageDraw.Draw(band).text((6, 5), cap, fill=(235, 235, 235))
        row = Image.new("RGB", (W, im.size[1] + 24), bg)
        row.paste(band, (0, 0)); row.paste(im, (0, 24))
        rows.append(row)
    out = Image.new("RGB", (W, sum(r.size[1] for r in rows) + 16), bg)
    y = 0
    for r in rows:
        out.paste(r, (0, y)); y += r.size[1] + 8
    out.save(out_png)


def find_ffmpeg(explicit):
    if explicit:
        return explicit
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--teach-dir", required=True,
                    help="teach-002 run dir (teach_log.jsonl, labelled_waypoints.json)")
    ap.add_argument("--stills-dir", required=True,
                    help="cams-check run dir with the three <role>_<serial>_color.png stills")
    ap.add_argument("--out", default=os.path.dirname(__file__) or ".")
    ap.add_argument("--ffmpeg", default=None, help="ffmpeg binary (else imageio-ffmpeg's)")
    args = ap.parse_args()

    frame_dir = os.path.join(args.out, "_frames")
    path, ptime, labels, raw = load_fork_path(args.teach_dir)
    render_frames(path, ptime, labels, raw, frame_dir)
    build_gif(frame_dir, os.path.join(args.out, "fork-pick-trajectory.gif"))
    ff = find_ffmpeg(args.ffmpeg)
    if ff:
        build_mp4(frame_dir, os.path.join(args.out, "fork-pick-trajectory.mp4"), ff)
    else:
        print("WARNING: no ffmpeg found; skipped MP4 (pip install imageio-ffmpeg)", file=sys.stderr)
    build_cameras(args.stills_dir, os.path.join(args.out, "cell-three-cameras.png"))
    print("media written to", args.out)


if __name__ == "__main__":
    main()
