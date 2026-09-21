# Nearest-point safety fix — Isaac before/after

**⚠️ IMPROVED, NOT PASS — the fix cuts the missed STOP ~4.7×, but a residual
~0.25 m / ~0.9 s-late STOP remains.**

> The Isaac test in PR #22 (`data/human-detect-isaac-sim-v1/report.md`) FAILED:
> the camera safety layer saw the person on 100% of frames but published a
> distance that read them **farther** from the robot than truth, so STOP engaged
> late. This task fixed the production layer and re-ran the SAME harness.
>
> **Distance-driven** (with the latency fault disabled so the state is sized by
> distance + hysteresis + dwell only — see the confound note below), the fix
> shrinks the missed STOP from the person being **1.04 m inside the ring for
> 4.33 s** to **0.25 m inside for 0.93 s**, and moves STOP engagement from a true
> nearest of **0.95 m** to **1.74 m**. The median over-statement of the true
> nearest-body distance drops from **+0.71 m to +0.19 m**. The residual 0.93 s is
> still longer than the 0.70 s reaction budget, so this is **not** a PASS: the
> +0.19 m residual over-statement, times the person's approach speed, is a
> ~0.9 s-late STOP. Closing it is an owner decision (see the last section).

## The confound this corrects (why "0 missed STOP" was wrong at first)

On this shared 12 GB GPU the detector's img→state latency crosses the 0.70 s
reaction budget on ~5% of frames, which correctly forces a **fail-safe STOP**.
That fail-safe STOP, plus the 2 s leave-STOP dwell, was already latched **before**
the person truly crossed the 2.0 m ring — so the *live* zone stream showed STOP
"on time" and hid the distance behaviour. This is the same confound the first
harness report carried. To measure the real, distance-driven safety margin, both
runs are replayed through the safety machine with the latency fault disabled
(`compare_fix.py --run <dir>`, `reconstruct(..., ignore_latency_fault=True)`;
everything else identical). All the headline numbers below are that
**distance-only** view. (The live latency-masked numbers, for the record: BEFORE
0.73 s / 0.29 m inside; AFTER 0 — both understate the real miss.)

## Before / after — distance-only (the honest safety picture)

Both runs: single wrist RGB-D camera, identical scripted path, Isaac domain 88.
Truth = the simulator's true world bounding box; "nearest body point" = the
nearest point of that box to the base z-axis. STOP ring = 2.0 m, budget = 0.70 s.

| metric (distance-driven) | BEFORE (pre-fix) | AFTER (fix) |
|---|---|---|
| **verdict** | ❌ FAIL | ⚠️ FAIL (residual) |
| **missed STOP** (person truly inside ring, state not STOP, > 0.7 s) | **4.33 s**, 1.04 m inside | **0.93 s**, 0.25 m inside |
| STOP engages at true nearest body | **0.95 m** (4.42 s after the ring crossing) | **1.74 m** (1.06 s after) |
| signed distance error vs **true nearest body**, median | **+0.71 m** | **+0.19 m** |
| … p95 (abs) / worst (most positive) | 0.97 / +1.02 m | 0.27 / +0.34 m |
| detection rate (wrist, in-view frames) | 100% | 100% (145/145) |
| img→state latency, median / frac over 0.70 s | 0.39 s / 5.5% | 0.39 s / 4.6% |

Positive error = the published distance says the person is FARTHER than they truly
are (the unsafe direction). The BEFORE distance-only numbers reproduce PR #22's
original finding (STOP at a true nearest ~0.95–1.06 m, ~1 m inside the ring),
confirming the reconstruction.

![distance-only before/after: STOP onset vs the true ring crossing](before_after.png)

![signed error vs true nearest body point](signed_error_hist.png)

## Which fix did what (distance-only, after-run ablation)

Replaying the after-run's recorded detections with the old (centre) vs new
(nearest) zone rule — geometry fix held constant (`compare_fix.py`):

| pipeline (distance-only) | missed STOP | STOP engages at true nearest | signed err median |
|---|---|---|---|
| old geometry + centre rule (true BEFORE) | 4.33 s, 1.04 m inside | 0.95 m | +0.71 m |
| **fixed geometry** + centre rule | 1.13 s, 0.30 m inside | 1.69 m | +0.24 m |
| **fixed geometry + nearest rule** (AFTER) | 0.93 s, 0.25 m inside | 1.74 m | +0.19 m |

The foreground-centre fix does most of the work (+0.71 → +0.24 m; miss 4.33 → 1.13 s);
the nearest-point rule trims it a little further (+0.24 → +0.19 m; 1.13 → 0.93 s)
**and** adds the structural guarantees a number cannot show: the zone can never
read farther than the centre, and it reacts to a hand thrust inside the ring.

## What changed (perception-side only; fails-toward-STOP preserved)

1. **Zone sized from the nearest body point, not the centre.** `safety.py` now
   uses `min(centre, nearest)` — never larger than the centre — with the
   nearest-point *range* passed through a short robust filter (`NearestRangeFilter`)
   so a single-frame depth speckle cannot chatter STOP and a genuinely close
   reading still engages after two frames. `lost_close` uses the same distance.
2. **Body centre no longer dragged by background depth.** `geometry.lift_depth`
   takes the median of the **foreground cluster only** (points within a body-depth
   band of the near surface), not the whole box.

Schema is additive: `nearest_distance`/`_base` now carry the nearest-point
distance; the pre-fix centre distances are published as `nearest_distance_centre`
/ `_base_centre` for diagnostics. `humangate`/`humangate_driver` read only the
existing fields and their tests still pass.

## No new chatter

- In-repo replay fixture (`synthetic_approach.jsonl`, nearest == centre): zone
  changes **4 → 4**; STOP time 4.80 → 5.10 s (the filter's slow-away holds STOP
  ~0.3 s longer on retreat — conservative). Isolates that the *rule* adds no
  chatter of its own.
- After-run rule ablation (distance-only): zone changes **4 → 4**, STOP time
  31.6 → 31.8 s. No chatter increase; where the two differ the new rule only adds
  STOP.

## The residual, and how to close it (owner decision)

The residual 0.93 s / 0.25 m-late STOP is the +0.19 m over-statement of the
nearest-body distance converted to time by the person's approach speed. It was
**not** closed here, for two honest reasons:

- The residual lives in the nearest-point *estimate* (`lift_depth` reports the
  median of the closest ~5% of depth ranges, which sits ~0.2 m behind the true
  surface). Reducing it (e.g. a lower percentile, a closer order-statistic) trades
  against speckle robustness on **real** depth — and this harness's depth is clean
  sim depth, so any such tweak would look free here while risking chatter on the
  real rig. The recorded run does not carry the raw depth frames, so the tweak
  cannot be re-derived or validated offline, and a new simulator session was out
  of scope for this check.
- **No threshold was changed.** The STOP ring (2.0 m), hysteresis, dwell and the
  0.70 s budget are provisional and are the owner's decision; none were touched to
  flatter the result (enforced by `sim/isaac_human/tests/test_sim_config.py`).

**Owner options to close the residual:** (a) add a **STOP-ring margin equal to the
measured residual** (≈ +0.25 m on the STOP ring, i.e. STOP at ~2.25 m of base),
which absorbs the estimator's residual over-statement directly; or (b) tighten the
nearest-point estimator (lower `nearest_percentile` / a closer order-statistic in
`lift_depth`) **with a real-rig depth recording** to prove it does not add chatter.
Both are follow-ups; (a) is a threshold change and thus the owner's call.

## What this does NOT prove

- A rendered character is not a real person; Isaac's simulated depth is far
  cleaner than a RealSense (no holes/speckle/range dropouts). Counter-intuitively
  the clean floor makes the centre bias the fix targets **worse**, so this is a
  conservative test of the geometry fix; real depth noise would move the number
  either way.
- The character is teleported in its default **T-pose** (arms out, ~2 m-wide,
  background-heavy box), which maximises the centre bias — passing the geometry
  test here is the harder case, but it is not a natural pose. A body-filled
  standing/walking pose needs `omni.anim.people` (a skeletal animation graph),
  **not** a low-cost swap on this single-12 GB-GPU harness (the RTX renderer
  already shares the card), so it stays a follow-up. The worst-case +0.34 m is
  partly this box.
- Single wrist camera (the healthy-latency set on the shared GPU); the fixed D415s
  stay presence-only until a separate task calibrates them.
- This layer is **NOT a certified safety function** — a single-channel, unrated,
  supplementary awareness layer. The real protection is the robot's safety-rated
  protective stop (+ a recommended presence device). See
  `cams/human_safety/README.md`.
- Only perception-side distance logic changed; no robot interface, no radius /
  dwell / budget / threshold change.

## Reproduce

```bash
sim/isaac_human/run_session.sh wrist cuda          # -> RUNDIR (git-checkout the
                                                   #    pre-fix code first for BEFORE)
python3 sim/isaac_human/analyze.py --run "$RUNDIR" \
    --config sim/isaac_human/params/human_safety.sim.yaml
python3 sim/isaac_human/compare_fix.py --run "$RUNDIR"   # prints latency-ON AND
                                                         # distance-only, old vs new
```

_After-run id `run_20260921T101304Z`, before-run id `run_20260921T101648Z`
(single wrist camera, cuda). Committed here: report.md + the two plots; report.html,
run logs, bags and frames are intentionally not committed._
