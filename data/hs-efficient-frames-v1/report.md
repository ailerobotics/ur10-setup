# Efficient frame processing for `cams/human_safety` — report

Goal (captain): build upon the human detection and tracking project — stop
running every frame in parallel, and instead process frames efficiently, as the
first step of a larger supermarket vision program. The layer must keep failing
toward STOP: absence of evidence is never evidence of clearance.

The design, invariants, config and how-to live in `cams/human_safety/README.md`
("Efficient frame processing"); this note records the decisions and the measured
baseline-vs-efficient evidence so they travel with the code.

## What was built (selectable by config, baseline kept as default)

- **Shared, batched detector** — `nodes/shared_detector_node.py`: one detector
  instance serves every camera and batches the frames needing detection into one
  forward pass per tick. Keeps the per-camera `/human_safety/detections|status/<role>`
  shape, so `fusion_node.py` is unchanged. Enabled by `frame_processing.mode: shared`.
- **Frame selection** — pure `human_safety/scheduler.py` (`FrameScheduler`) +
  `human_safety/frameselect.py` (cheap block-mean downscale + frame-difference
  change score, torch-free). Enabled by `frame_processing.scheduler.enabled`.
- **FP16** — opt-in half-precision via CUDA autocast in the pluggable
  `keypoint_rcnn` detector (`detect_batch` added; licences unchanged,
  torchvision/torch BSD-3-Clause, no AGPL/ultralytics).
- Default config is the **every-frame baseline** (`mode: per_camera`,
  `scheduler.enabled: false`, `fp16: false`) — unchanged behaviour until the
  evidence shows the efficient path is at least as safe.

## Safety invariants (why the efficient path is at least as safe)

Every rule only moves a camera toward *more* detection, never less:

1. **Hard T_max** — a full detection at least every `t_max` (default 0.4 s).
   Freshness is driven by the last full detection, and config load rejects
   `t_max > safety.frame_age_budget`, so a quiet camera is never reported fresh on
   skipped frames; if detection can't keep up the camera ages to stale and the
   unchanged health rules degrade toward SLOW then STOP.
2. **Motion trigger** — change score above `motion_threshold` forces immediate
   detection (a person entering view moves a large block of pixels); an unknown
   score counts as motion (fail toward looking).
3. **Near-robot priority** — any confirmed track inside/approaching the SLOW/STOP
   rings puts every camera at the full rate (read advisorily from the fusion
   node's published state).
4. **No CLEAR from skipped frames** — skipping carries forward the last full
   detection (the tracker coasts); it never fabricates an empty observation.

Guarded by `tests/test_scheduler.py` (T_max, motion trigger, near-robot priority,
staleness→unhealthy, param + config invariants) and proven end-to-end by
`tests/test_frameproc_eval.py`.

## Material

No raw multi-camera image recording exists in-repo or on disk — every frame saved
by the Isaac harness/recorder has the detector overlay baked into the pixels
(confirmed by the exploration for this task), so none is re-runnable through a
detector. Per the brief, the comparison runs on a deterministic synthetic
multi-camera sequence of a person approaching the robot
(`cams/human_safety/tests/data/multicam_approach.jsonl`, regenerate with
`make_multicam_approach.py`): 160 ticks at 10 Hz, 3 cameras (shelf+table depth,
wrist presence-only), CLEAR→SLOW→STOP→dwell→retreat→CLEAR.

## Measured: baseline vs efficient on the same material

Decision level (`bench_frame_processing.py --mode decision`, deterministic, CI):

| metric | baseline | efficient |
|---|---|---|
| detector invocations | 480 | 330 (**−31 %**) |
| less-safe ticks (efficient < baseline) | — | **0** |
| disagreement ticks (either direction) | — | 0 |
| detection recall (vs ground truth) | 1.00 | 1.00 |
| track continuity (efficient vs baseline) | — | 1.00 |
| per-camera worst-case staleness | 0.00 s | 0.40 s (budget 0.5 s) |

The −31 % reflects this sequence being busy for ~45 % of its length; the idle-time
saving approaches the 1/`t_max` floor as the cell stays quiet (a store aisle empty
for minutes benefits most).

GPU level (`bench_frame_processing.py --mode gpu`, RTX 3060 12 GB, 848×480, 3
cameras, 24 frames/cam; run under `flock /tmp/aile-gpu.lock`):

| | per-camera (N sequential / N processes) | shared (1 batched pass / 1 instance) |
|---|---|---|
| peak VRAM | ≈ 3×371 = **1114 MB** | **636 MB** (**−43 %**) |
| latency | 112 ms/tick | 98 ms/tick (**1.14×**) |
| GPU-ms per second of video | 1687 | 1475 |

FP16 vs FP32 on these frames: person-count agreement 72/72, max box-corner shift
0.00 px, max score delta 0.000 (synthetic noise; re-confirm on person frames
before enabling fp16 in production).

The VRAM saving (one model vs N) is the main resource win; the batched speedup is
modest because Keypoint R-CNN stays compute-bound on this card, and frame
selection is what cuts GPU-time further when the cell is quiet.

## Scope / not done

- Out of scope per brief: SLOW/STOP geometry, camera placement, deploying to the
  rig, anything driving the robot. No live hardware or `~/ur10` touched.
- ROI-around-predicted-boxes detection (brief item 4) is left as a documented
  extension point; the batched shared detector + frame selection + FP16 deliver
  the efficiency without it, and a per-camera ROI needs each camera's pose
  (presence-only cameras have none today).
- Bulky benchmark outputs are not committed (code, spec and the tiny fixture are).
