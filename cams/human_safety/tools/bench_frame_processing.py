#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Benchmark: every-frame BASELINE vs the EFFICIENT frame-processing path.

Two modes, printed as plain tables:

  --mode decision   (default; no GPU, no torch, deterministic)
      Replays a multi-camera sequence (default tests/data/multicam_approach.jsonl)
      through both paths with the SAME ground-truth detections, and reports:
        * detector invocations and the reduction (the GPU-time proxy),
        * safety-state agreement — the count of ticks where the efficient path is
          LESS safe than the baseline MUST be zero,
        * per-camera worst-case staleness vs the freshness budget,
        * detection recall and track continuity vs the baseline.
      This is the safety evidence and runs in CI (tests/test_frameproc_eval.py).

  --mode gpu        (needs the torch venv + a GPU; keep it short and flock'd)
      Measures the real resource win of the shared/batched detector on identical
      synthetic frames: wall ms per tick and GPU-ms per second of video for the
      per-camera-sequential path vs one batched forward pass, peak VRAM for a
      single instance (≈ per-process cost, ×N for N parallel processes) vs the
      shared instance, and the fp32-vs-fp16 detection delta (materiality check).
      Run WRAPPED so heavy GPU jobs take turns, e.g.:
        flock /tmp/aile-gpu.lock ~/venvs/human-safety/bin/python \
          cams/human_safety/tools/bench_frame_processing.py --mode gpu \
          --frames 24 --cameras 3 --device cuda

No people are rendered in the GPU frames (synthetic noise): that mode measures
TIME and MEMORY, which are apples-to-apples on identical frames regardless of
detections. Recall / safety are proven in the decision mode on ground truth.
"""
import argparse
import pathlib
import sys
import time

import numpy as np

_PKG_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

DEFAULT_SEQ = _PKG_ROOT / "tests" / "data" / "multicam_approach.jsonl"


# --------------------------------------------------------------------------- #
# Decision-level benchmark (no GPU)
# --------------------------------------------------------------------------- #
def run_decision(args):
    from human_safety.config import FrameProcessingConfig, HumanSafetyConfig
    from human_safety.frameproc_eval import run_baseline_vs_efficient
    from human_safety.replay import load_jsonl
    from human_safety.safety import SafetyParams
    from human_safety.scheduler import SchedulerParams

    records = load_jsonl(args.sequence)
    cfg = HumanSafetyConfig(
        safety=SafetyParams(),
        frame_processing=FrameProcessingConfig(
            mode="shared",
            scheduler=SchedulerParams(
                enabled=True, t_max=args.t_max,
                motion_threshold=args.motion_threshold)))
    r = run_baseline_vs_efficient(records, cfg)

    b, e = r.baseline, r.efficient
    red = 100.0 * (1 - e.detector_invocations / max(1, b.detector_invocations))
    budget = cfg.safety.frame_age_budget

    print(f"\n=== decision benchmark: {pathlib.Path(args.sequence).name} ===")
    print(f"ticks: {len(r.times)}   t_max: {args.t_max}s   "
          f"motion_threshold: {args.motion_threshold}")
    print(f"\n{'metric':<34}{'baseline':>14}{'efficient':>14}")
    print("-" * 62)
    print(f"{'detector invocations':<34}{b.detector_invocations:>14}"
          f"{e.detector_invocations:>14}")
    print(f"{'  (frames offered)':<34}{b.frames_offered:>14}{e.frames_offered:>14}")
    print(f"{'invocation reduction':<34}{'':>14}{red:>13.1f}%")
    rb, re_ = r.recall(b), r.recall(e)
    print(f"{'detection recall (vs ground truth)':<34}"
          f"{_fmt(rb):>14}{_fmt(re_):>14}")
    print(f"{'track continuity (eff vs base)':<34}{'':>14}"
          f"{_fmt(r.track_continuity):>14}")
    print("\nper-camera worst-case staleness (s), budget "
          f"{budget}s:")
    for role in sorted(e.worst_staleness):
        print(f"  {role:<12} baseline {b.worst_staleness[role]:.3f}   "
              f"efficient {e.worst_staleness[role]:.3f}   "
              f"{'OK' if e.worst_staleness[role] <= budget else 'STALE!'}")

    print("\nsafety-state agreement:")
    print(f"  disagreement ticks (either direction): {len(r.disagreement_ticks)}")
    print(f"  LESS-SAFE ticks (efficient < baseline): {len(r.less_safe_ticks)}")
    if r.less_safe_ticks:
        print("  !! efficient was LESS SAFE than baseline at ticks "
              f"{r.less_safe_ticks[:10]}{' ...' if len(r.less_safe_ticks) > 10 else ''}")
        print("  !! FAIL: the efficient path must never be less safe.")
        return 1
    print("  PASS: efficient path is never less safe than the baseline.")
    return 0


def _fmt(x):
    return "n/a" if x is None else f"{x:.3f}"


# --------------------------------------------------------------------------- #
# GPU resource benchmark (needs torch + GPU)
# --------------------------------------------------------------------------- #
def _synth_frames(n_cams, n_frames, h, w, seed=0):
    """Deterministic synthetic colour frames (gradient + noise). Identical frames
    feed both paths, so the time/memory comparison is apples-to-apples."""
    rng = np.random.default_rng(seed)
    grad = np.linspace(0, 200, w, dtype=np.float32)[None, :, None]
    base = np.broadcast_to(grad, (h, w, 3)).astype(np.float32)
    seqs = []
    for _ in range(n_cams):
        frames = []
        for _ in range(n_frames):
            noise = rng.integers(0, 55, (h, w, 3), dtype=np.uint8)
            frames.append(np.clip(base + noise, 0, 255).astype(np.uint8))
        seqs.append(frames)
    return seqs


def _peak_vram_mb(torch):
    return torch.cuda.max_memory_allocated() / (1024 ** 2)


def run_gpu(args):
    try:
        import torch
    except Exception as e:  # noqa: BLE001
        print(f"GPU mode needs torch (run under ~/venvs/human-safety): {e}")
        return 2
    from human_safety.detector import build_detector

    h, w = args.height, args.width
    n = args.cameras
    T = args.frames
    seqs = _synth_frames(n, T, h, w)
    cuda = args.device == "cuda" and torch.cuda.is_available()
    dev = "cuda" if cuda else "cpu"
    print(f"\n=== GPU resource benchmark ===")
    print(f"device: {dev}   cameras: {n}   frames/cam: {T}   "
          f"resolution: {w}x{h}   schedule_rate: {args.schedule_rate} Hz")

    det = build_detector("keypoint_rcnn", device=dev, score_thresh=0.6)
    det.warmup()
    if cuda:
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()

    # --- per-camera path: N sequential detect() calls per tick --------------- #
    t0 = time.time()
    for k in range(T):
        for c in range(n):
            det.detect(seqs[c][k])
        if cuda:
            torch.cuda.synchronize()
    seq_wall = time.time() - t0
    seq_vram1 = _peak_vram_mb(torch) if cuda else 0.0

    # --- shared path: one batched detect_batch(N) per tick ------------------- #
    if cuda:
        torch.cuda.reset_peak_memory_stats()
    t0 = time.time()
    for k in range(T):
        det.detect_batch([seqs[c][k] for c in range(n)])
        if cuda:
            torch.cuda.synchronize()
    batch_wall = time.time() - t0
    batch_vram = _peak_vram_mb(torch) if cuda else 0.0

    video_s = T / max(1e-6, args.schedule_rate)      # seconds of video per camera
    seq_ms_tick = 1000.0 * seq_wall / T
    batch_ms_tick = 1000.0 * batch_wall / T
    # GPU-ms per second of video (one camera's worth of wall-clock video).
    seq_ms_per_s = 1000.0 * seq_wall / video_s
    batch_ms_per_s = 1000.0 * batch_wall / video_s

    print(f"\n{'path':<28}{'ms/tick':>12}{'GPU-ms / s video':>20}")
    print("-" * 60)
    print(f"{'per-camera (N sequential)':<28}{seq_ms_tick:>12.1f}{seq_ms_per_s:>20.1f}")
    print(f"{'shared (1 batched pass)':<28}{batch_ms_tick:>12.1f}{batch_ms_per_s:>20.1f}")
    if batch_wall > 0:
        print(f"{'speedup (batched/seq)':<28}{seq_wall / batch_wall:>11.2f}x")

    if cuda:
        print(f"\npeak VRAM (one model instance):           {seq_vram1:8.0f} MB")
        print(f"  per-camera path = N separate processes ~= {seq_vram1 * n:8.0f} MB "
              f"({n} x one instance)")
        print(f"  shared path     = ONE instance, batch {n}  = {batch_vram:8.0f} MB")
        if seq_vram1 * n > 0:
            print(f"  VRAM saving (shared vs N processes):     "
                  f"{100 * (1 - batch_vram / (seq_vram1 * n)):7.1f}%")

    # --- fp16 materiality check --------------------------------------------- #
    if cuda and not args.skip_fp16:
        _fp16_materiality(torch, build_detector, seqs, n)
    return 0


def _fp16_materiality(torch, build_detector, seqs, n):
    print("\n--- fp16 materiality (fp32 vs fp16 on identical frames) ---")
    det16 = build_detector("keypoint_rcnn", device="cuda", score_thresh=0.6,
                           fp16=True)
    det16.warmup()
    det32 = build_detector("keypoint_rcnn", device="cuda", score_thresh=0.6)
    det32.warmup()
    count_match, max_box_shift, max_score_delta, frames = 0, 0.0, 0.0, 0
    for c in range(n):
        for img in seqs[c]:
            d32 = det32.detect(img)
            d16 = det16.detect(img)
            frames += 1
            if len(d32) == len(d16):
                count_match += 1
                for a, b in zip(sorted(d32, key=lambda d: d.bbox),
                                sorted(d16, key=lambda d: d.bbox)):
                    max_box_shift = max(
                        max_box_shift,
                        float(np.max(np.abs(np.array(a.bbox) - np.array(b.bbox)))))
                    max_score_delta = max(max_score_delta,
                                          abs(a.score - b.score))
    print(f"frames compared: {frames}   person-count agreement: "
          f"{count_match}/{frames}")
    print(f"max box corner shift: {max_box_shift:.2f} px   "
          f"max score delta: {max_score_delta:.3f}")
    print("(synthetic noise frames; re-run on real/person frames to confirm "
          "materiality before enabling fp16 in production.)")


# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["decision", "gpu"], default="decision")
    # decision mode
    ap.add_argument("--sequence", default=str(DEFAULT_SEQ))
    ap.add_argument("--t-max", type=float, default=0.4)
    ap.add_argument("--motion-threshold", type=float, default=0.02)
    # gpu mode
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--cameras", type=int, default=3)
    ap.add_argument("--frames", type=int, default=24)
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--schedule-rate", type=float, default=15.0)
    ap.add_argument("--skip-fp16", action="store_true")
    args = ap.parse_args(argv)

    if args.mode == "decision":
        return run_decision(args)
    return run_gpu(args)


if __name__ == "__main__":
    sys.exit(main())
