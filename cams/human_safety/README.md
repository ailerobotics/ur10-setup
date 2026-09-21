# human_safety — camera-based 3-D human detection, tracking & safety state (stage 1)

Turns every camera watching the UR10 cell into one fused, tracked picture of
where people are relative to the robot, and publishes a **CLEAR / SLOW / STOP**
judgement with the distance to the nearest person. It **only publishes** — it
does not touch the robot. Connecting this state to actual motion (slow/stop/
resume) is a **separate, later change** (designed in `human-safety-s1`).

> ## ⚠️ This is NOT a certified safety function
> A camera plus a neural-network person detector is a **single-channel, unrated,
> supplementary awareness layer**. It has no diagnostic coverage and
> non-deterministic failure modes; it is **not** IEC 61496 / ISO 13849 PL d
> Cat 3 and must never be described or used as the last line of defence.
> The actual protection against injury remains the robot controller's
> **safety-rated protective stop** (CB3 pendant Force limit, 100 N floor) and —
> the recommended addition — a **safety-rated presence device** (an IEC 61496
> light curtain / safety laser scanner, PL d / Cat 3, hard-wired into the CB3
> safeguard-stop input). This layer sits *inside* that certified floor as an
> early, graceful, resumable slow/stop. See `human-safety-s1` §0/§4/§11.

Everything here fails **toward STOP**: absence of evidence is never evidence of
clearance.

---

## What it does (the 8 pieces)

1. **Camera intake** — discovers whatever RealSense units are attached by serial
   (via `cams/rig_multicam.launch.py`), each as colour + aligned depth. A plain
   colour-only camera (a RealSense colour node read with depth ignored, or any
   UVC webcam) goes through the same pipeline with `has_depth: false`. Per-camera
   rate caps (`max_rate`) mean the 6 fps wrist never stalls the others; each
   detector node runs independently.
2. **Person detection** — pluggable detector (default torchvision Keypoint
   R-CNN); returns a box **and** 17 COCO keypoints per person.
3. **3-D position** — with depth, a robust body-centre **and** the nearest
   credible body point (percentile, not the single closest pixel), handling
   depth holes and min/max range; without depth, a monocular ground-plane / body-
   height estimate with honestly larger uncertainty.
4. **One common frame** — positions in `base_link` via tf2. A camera with no
   trustworthy pose contributes **presence only** (see *Camera poses* below).
5. **Fusion & tracking** — gated association + constant-velocity Kalman tracker,
   stable IDs, confirmation/coasting, multi-camera detections update one track.
6. **Safety state** — `/human_safety/state` with zone, nearest distance,
   hysteresis, minimum-clear dwell, per-camera health, and a monotonic heartbeat.
7. **Owner view** — one window: every camera feed (boxes, keypoints, IDs,
   distances) plus a top-down map (robot, zones, tracks, state banner).
8. **No robot interface** — nothing here subscribes to drive the robot, calls a
   robot service, or publishes to a controller.

---

## Layout

```
cams/human_safety/
  human_safety/           # PURE decision logic — no ROS, no torch, no GPU (unit-tested)
    geometry.py             depth lifting, monocular lifting, frame transforms
    detector.py             pluggable detector (base + Keypoint R-CNN, torch lazy)
    tracker.py              gated association + constant-velocity Kalman tracker
    safety.py               zone / hysteresis / dwell / health state machine
    fusion.py               merge per-camera detections -> tracks
    messages.py             ROS-free dataclasses (Detection3D, Track, SafetyState…)
    json_util.py            the published JSON schema (state / detections / tracks)
    imgcodec.py             dependency-free Image <-> ndarray (no cv_bridge)
    config.py               YAML config load + validation
    replay.py               deterministic replay of a recorded log
  nodes/                  # thin ROS shells
    detector_node.py        per-camera: detect + lift + publish
    fusion_node.py          fuse + evaluate safety + publish state/tracks/markers
    viz_node.py             owner window (system python, cv2)
  tools/
    live_check.py           live plumbing check (rates, freshness, latency, GPU)
    detector_selfcheck.py   run the detector on sample images
  params/
    human_safety.yaml       cameras + zones/budgets (PROVISIONAL defaults)
    camera_poses.yaml       static extrinsics for calibrated fixed cameras (empty today)
  human_safety.launch.py  detectors + fusion + viz + static TFs
  run_live.sh             ONE command: cameras + graph + viewer + record (domain 77)
  tests/                  offline pytest suite + replay fixture
```

The package runs **as scripts**, matching the repo convention (there is no
colcon workspace anywhere in this repo). `import human_safety` works with the
package dir on `sys.path`; offline tests need only NumPy + SciPy.

---

## Detector & licensing

| | Choice | Licence |
|---|---|---|
| Detector code | torchvision `keypointrcnn_resnet50_fpn` | **BSD-3-Clause** |
| Pretrained weights | `KeypointRCNN_ResNet50_FPN_Weights.COCO_LEGACY` | **BSD-3-Clause** (torchvision) |
| Runtime | torch 2.4.1 + torchvision 0.19.1 (cu121 wheels, bundled CUDA runtime) | BSD-3-Clause |

**No AGPL / ultralytics** anywhere, per the brief. The detector is pluggable
(`human_safety/detector.py`, `build_detector(name)`); to add one, subclass
`PersonDetector` and register it. Keypoints are preferred over a bare box
because the 3-D lifting uses body points (ankles for the monocular foot point,
the shrunk box for the robust depth region).

**Measured** (this machine, RTX-class 12 GB GPU, 848×480):

- Inference ~37 ms on a blank frame, ~150 ms on a busy live frame (≈6–7 Hz per
  camera, GPU-bound — below the frame rate, so rate-capping rarely bites).
- **GPU memory: 394 MB per detector process; ~1.87 GB for all three together —
  under the 2 GB budget.** (Verify with `nvidia-smi`; keep ≤3 detectors, or move
  one to `device: cpu`, if other GPU tenants shrink the headroom.)
- Detector check on public-domain sample people (full body, **partial/upper-body
  crop**, **seated crop**, pedestrian): all detected with keypoints
  (`tools/detector_selfcheck.py`; evidence in
  `firstmate/data/human-detect-core-v1/detector-check/`).

---

## 3-D methods & expected error

- **Depth** (`geometry.lift_depth`): robust body centre = component-wise median
  of the **foreground cluster only** — the valid points within `fg_band` (default
  1.0 m) of the near surface. A plain median over the whole box is *not* robust to
  the floor/wall visible inside the person box: in clean depth the background can
  be more than half the valid pixels and drags a component-wise median metres into
  the background (the overhead camera seeing the most floor is worst; a wide-armed
  pose amplifies it), which **over-states** the person's distance. Clustering in
  range keeps only the body's own depth band. **nearest** = median of the closest
  `nearest_percentile` (default 5 %) of ranges, so one flying
  near-range speckle cannot pull the safety distance in. Invalid depth (0/NaN)
  and anything outside `[min_range, max_range]` is dropped (handles holes and the
  RealSense min/max range). Expected 1-σ ≈ **1 % of range + 2 cm** depth noise,
  **plus the camera's calibration residual** when it has a pose (wrist hand-eye
  ≈ 8 mm; a calibrated D415 target < 1 cm). ≈ 0.1–0.3 m at cell ranges — matches
  `human-safety-s1` §3 `Zd`.
- **Monocular** (`geometry.lift_monocular`, no depth): intersect the foot-pixel
  ray with the ground plane `z = ground_z`; centre lifted half a body height
  above the foot. Fallback (feet occluded / ray not descending): pinhole range
  from box height and `height_prior`. Reported 1-σ is **honestly larger**
  (≈ 0.35 m + 0.15 × range for the ground method; +0.25 m more for the height-
  prior fallback). Monocular needs a known camera pose *and* a decent floor
  estimate; it is a coarse locator, not a depth substitute.
- Both are unit-tested offline (`tests/test_geometry.py`), including holes,
  range-window clipping, and the speckle-robust nearest point.

---

## Camera poses — who is trustworthy today

Positions are lifted to `base_link` via **tf2**. A camera is *positioned* only
when a fresh `base_link ← <optical_frame>` transform exists at the image stamp;
otherwise it contributes **presence only** (`present=True`, no position), clearly
flagged. This one mechanism covers every case:

| Camera | Pose today | Why |
|---|---|---|
| **wrist** D435i (`018322070619`, upright twin) | **presence only** | hand-eye calibrated, but its pose is FK-chained through `/joint_states`; the **robot driver is down** (leave it down), so the TF chain is broken |
| **shelf** D415 (`021422061772`) | **presence only** | no `base_link` extrinsic exists — Step-2 AprilTag world-alignment is **not done** |
| **table** D415 (`025522060953`) | **presence only** | same — uncalibrated |

So **all three cameras are presence-only right now** (confirmed live). The
machinery to place them is complete: when a pose becomes available (the driver
comes up for the wrist; a fixed-camera extrinsic is added to
`params/camera_poses.yaml`), the same node immediately produces 3-D positions —
no code change. `human-safety-s1` §6 reaches the same conclusion from the repo.

### Calibrating a fixed camera into `base_link` (the tool now exists)

The fixed-camera calibrator (`human-safety-s1` §6 / Phase 2) is committed under
`cams/cellmap/` and its geometry is unit-tested; full detail and measured
accuracy are in `data/human-cam-calib-v1/report.md`. Procedure:

1. Place a printed 36h11 AprilTag (`cams/cellmap/tags/`, 120 mm) **flat, face-up**
   at a **mapped `base_link` pose** in the fixed camera's view — the known tag
   poses live in `cams/cellmap/tags/tag_world_poses.yaml` (id0 at
   `(0.116, -1.058, -0.057)`, id1 at `(-0.505, -1.058, -0.058)`). Two tags in
   view is better (enables the tag-separation fixture check).
2. Solve the extrinsic (colour PnP against the mapped tags; a colour-only camera
   works — omit `--depth`):
   ```
   ROS_DOMAIN_ID=80 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 \
     cams/cellmap/calibrate_static_camera.py --serial <SERIAL> --role <role> \
     --image /<role>/color/image_raw --info /<role>/color/camera_info \
     --depth /<role>/aligned_depth/image --depth-scale 1.0 \
     --optical-frame <role>_color_optical_frame
   ```
   It writes `cams/cellmap/calib/base_to_camera_<serial>_<date>.yaml` and reports
   reprojection, the tag-separation fixture check, and (RealSense) depth-vs-PnP
   against the < 5 px / < 1 cm target.
3. Add a **serial-keyed** entry to `params/camera_poses.yaml` pointing at that
   file; this launch then publishes the static TF and the camera reports 3-D
   positions. The loader (`human_safety/camera_poses.py`) refuses to publish an
   uncalibrated, stale, serial-mismatched, or failed-recheck calibration — such a
   camera stays presence-only.
4. Guard against a moved camera: `cams/cellmap/recheck_static_camera.py --calib
   <file> --image … --info …` re-checks that the tags still reproject where the
   stored pose predicts and stamps a `recheck` block with `status: fail` if not
   (→ presence-only).

An ordinary colour webcam additionally needs a one-time intrinsics calibration
(`cams/cellmap/calibrate_webcam_intrinsics.py`, chessboard) before PnP is
trustworthy; a RealSense uses its factory colour intrinsics from `camera_info`.

---

## Published interface (the robot-side gate consumes this)

Because this repo has no colcon build (and the offline test path must not require
one), the state is a **versioned JSON document on `std_msgs/String`**, not a
custom `.msg`. `human-safety-s1` §7 prefers a custom message; a thin adapter on
the gate side (which that report also recommends) maps this schema to one. The
schema is stable and `schema_version`-tagged so a rename is a one-line change.

| Topic | Type | Payload |
|---|---|---|
| `/human_safety/state` | `std_msgs/String` | the JSON below — the judgement |
| `/human_safety/tracks` | `std_msgs/String` | `{stamp, tracks:[…]}` (separate topic) |
| `/human_safety/markers` | `visualization_msgs/MarkerArray` | RViz: zones + tracks |
| `/human_safety/detections/<role>` | `std_msgs/String` | per-camera detections (+2-D overlay) |
| `/human_safety/status/<role>` | `std_msgs/String` | per-camera detector liveness |

**`/human_safety/state` JSON** (`json_util.state_to_json`, `schema_version: 1`):

```jsonc
{
  "schema_version": 1,
  "frame_id": "base_link",
  "stamp": 1789811234.32,          // publication wall time
  "evidence_stamp": 1789811234.13, // OLDEST positioned-detection stamp sizing the STOP ring (gate uses THIS for staleness); null when nobody is localised
  "heartbeat_seq": 412,            // monotonic; gate STOPs if it stalls
  "zone": "CLEAR|SLOW|STOP",
  "distance_status": "known|unknown|none",  // UNKNOWN is NOT 'far' (see below)
  "nearest_distance": 1.34,        // to reach envelope, metres; null unless distance_status==known — NEAREST credible body point (zone driver)
  "nearest_distance_base": 2.64,   // to base origin, metres; null unless known — NEAREST credible body point
  "nearest_distance_centre": 1.51,        // DIAGNOSTIC: body-CENTRE -> reach envelope (pre-fix zone driver); null unless known
  "nearest_distance_base_centre": 2.81,   // DIAGNOSTIC: body-CENTRE -> base origin; null unless known
  "nearest_track_id": 7,
  "any_person_visible": true,
  "reasons": ["…which rules force non-CLEAR…"],
  "cameras": [{"camera":"shelf","status":"fresh|stale|absent","fresh":true,"positioned":false,…}],
  "tracks": [{"id":7,"position":[x,y,z],"velocity":[…],"confirmed":true,…}]
}
```

**Distance honesty (the single most important line):** when only presence-only
(uncalibrated) cameras see a person, `distance_status` is `"unknown"` and
`nearest_distance` is `null` — this is **not** "far", and the state is still at
least **SLOW**. The gate must read `distance_status`, never treat a null distance
as clear. Published at a fixed **≥10 Hz** (default 15 Hz, measured 15.0 Hz) — the
6 fps wrist does **not** set this rate; per-camera freshness is separate.

---

## Zones, hysteresis, dwell, health — PROVISIONAL

Defaults from `human-safety-s1` §3 (conservative placeholders, **not** derived
separation distances — a certified analysis would tighten them):

**Zones react to the nearest credible body point.** The zone distance for each
confirmed track is `min(centre distance, nearest-point distance)` — never larger
than the body centre — so an outstretched hand inside the STOP ring engages STOP
even while the person's centre is still outside it. The track's nearest point is
copied from the latest detection **unfiltered**, so its *range* is first passed
through a short robust filter (`NearestRangeFilter`, `safety.py`): it keeps the
last `nearest_filter_window` (5) raw ranges, drops the `nearest_filter_reject` (1)
closest as a possible single-frame speckle, and takes the **min of the rest**.
This leans toward the robot (the safe direction) — a single-frame depth speckle is
dropped (no STOP chatter), while a genuinely close reading survives once it
persists two frames (~0.13 s at 15 Hz) and a single far outlier does not release
early (slow-away; the minimum-clear dwell provides the rest of the hysteretic
exit). A presence-only / monocular track whose nearest point equals its centre
therefore behaves exactly as before. `lost_close` bookkeeping uses this same
combined distance. The body-centre distances are still published as
`nearest_distance_centre` / `nearest_distance_base_centre` for diagnostics.

- **STOP** when a person is within **2.0 m of the robot base**
  (`reach_radius 1.30 + stop_radius 0.70`), **or** any presence-only camera sees
  a person, **or** any health fault.
- **SLOW** when any person is present (this small cell: SSM says any person →
  at least SLOW), outside STOP.
- **CLEAR** only when no person is anywhere in the monitored volume.
- **Hysteresis 0.30 m**: leave STOP only after clearing the ring by +0.30 m.
- **Dwell ≥ 2.0 s** of continuous healthy CLEAR before leaving STOP (anti-chatter).

**Health is part of the judgement (fail toward STOP):**

| Condition | Zone floor |
|---|---|
| Detector **stopped answering** — status gone, or no fresh frame for `detector_stall_timeout` (1.5 s), or camera blind | **STOP** (a stalled/crashed detector cannot see a person who is there) |
| Detector **slow but still answering** — frame age over `frame_age_budget` (0.5 s) but under the stall timeout | **SLOW** |
| End-to-end img→state **latency over `reaction_time_budget`** (0.7 s) — the STOP-ring sizing is no longer valid | **STOP** |
| An **unlocalised visible person** (presence-only camera) | **SLOW** |
| A confirmed track **lost while close** to the robot | **STOP** held for `lost_hold_time` |

The freshness/stall/latency thresholds distinguish a *lagging* detector (SLOW)
from one that has *stopped* (STOP). Every rule is unit-tested
(`tests/test_safety.py`). Tune radii and budgets in `params/human_safety.yaml`;
they are parameters. (Expected cameras = the ones actually launched; a
configured-but-not-launched camera is not treated as a crashed detector.)

---

## Difference from `human-safety-s1` (named per the steering note)

- **State transport.** The report §7 prefers a **custom ROS message**; this
  package ships a **versioned JSON schema on `std_msgs/String`** because the repo
  has no colcon build and the offline test path must not require one. The report
  itself endorses a thin gate-side adapter, which absorbs the difference (this is
  explicitly accepted for this stage).

Everything else follows the report: zone defaults (STOP within 2.0 m of base,
+0.30 m hysteresis, ≥2.0 s dwell), distance-unknown honesty, heartbeat, ≥10 Hz,
tracks on a separate topic, both D415s + the wrist presence-only, a
stalled/crashed detector → **STOP** (§5 fault 3) with SLOW kept for a
slow-but-answering detector and STOP once latency blows the reaction budget
(§5 fault 5), and **not** writing the fixed-camera calibrator (a separate task
owns it).

---

## Environment / install (user-scoped, survives outside the worktree)

A virtualenv **outside** the worktree so it persists: `~/venvs/human-safety`
(4.9 GB, under the 6 GB cap). Only the detector nodes use it (torch); fusion and
viz run on the system python. No sudo, no system packages, nothing installed into
any existing environment. Recreate exactly:

```bash
python3 -m venv --system-site-packages ~/venvs/human-safety   # --system-site-packages: rclpy et al.
~/venvs/human-safety/bin/pip install --upgrade pip
~/venvs/human-safety/bin/pip install "numpy==1.26.4" pillow    # match the ROS/system NumPy ABI
~/venvs/human-safety/bin/pip install torch==2.4.1 torchvision==0.19.1 \
    --index-url https://download.pytorch.org/whl/cu121         # bundled CUDA runtime, no toolkit
```

Images are decoded without `cv_bridge` (`imgcodec.py`) so the venv never imports
a ROS C-extension bound to a different NumPy ABI.

---

## Running

**One command** (brings up the cameras AND the graph on the isolated
`ROS_DOMAIN_ID=77`, opens the owner window, records a replayable log). This is
the only task allowed to open the cameras; the wrapper stops everything it starts
by **PID** and confirms the cameras are released:

```bash
cams/human_safety/run_live.sh wrist,shelf,table walkthrough.jsonl
# ... reads $DISPLAY for the window (SSH X-forwarding); Ctrl-C to stop ...
```

Graph only (cameras already up on domain 77):

```bash
ROS_DOMAIN_ID=77 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros2 launch cams/human_safety/human_safety.launch.py cams:=wrist,shelf,table viz:=true
```

Live plumbing check (cameras found, fresh, per-camera rates, img→state latency,
GPU, heartbeat; exit 0 = all live):

```bash
ROS_DOMAIN_ID=77 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  python3 cams/human_safety/tools/live_check.py --config cams/human_safety/params/human_safety.yaml
```

**Measured live** (domain 77, all three cameras, driver down): all cameras found
& FRESH (stamp age ~0.04 s); colour 13–20 Hz; state **15.0 Hz**; img→state
latency median **259 ms** / max 439 ms (dominated by detector inference on the
shared GPU); 3 detectors **~1.87 GB** GPU total; all cameras presence-only
(expected); heartbeat advancing; **cameras released cleanly** on stop.

---

## Testing

- **Offline, deterministic** (`pytest cams/human_safety/tests/`, no cameras / no
  GPU / no ROS): geometry (depth + monocular lifting, transforms), association,
  tracker lifecycle, safety zones/hysteresis/dwell and every health rule, config,
  JSON round-trip, and a **replay regression** (`tests/data/synthetic_approach.jsonl`
  — CLEAR→SLOW→STOP→dwell→CLEAR). ~68 tests, ~0.5 s.
- **Detector check** on public sample images: `tools/detector_selfcheck.py`.
- **Live plumbing check** on the real cameras: `tools/live_check.py`.
- **Replay regression from a real recording:** `run_live.sh` writes a JSONL every
  fusion cycle; `human_safety/replay.py` replays it through Fusion + Safety
  deterministically. Drop a captured walk-through under `tests/data/` and assert
  it in `tests/test_replay.py` (same shape as the synthetic fixture).

### The walk-through check needs the owner

Verifying live 3-D positions and tracking needs a **person in view** while a
camera has a trustworthy pose — which cannot be arranged here now (the robot
driver is down, so the wrist has no pose; the fixed D415s are uncalibrated). The
offline geometry/tracker/safety are fully tested; the live check confirms the
whole image→detector→fusion→state pipeline end to end on whoever is in frame. The
**owner-run walk-through** (`run_live.sh …`, then add the recording as a replay
test) is the remaining verification.

---

## Limitations (stage 1)

- **Not a certified safety function** (see the top). No robot interface yet.
- **All cameras presence-only today** — no camera has a live `base_link` pose
  (driver down; D415s uncalibrated). Presence-only forces ≥ SLOW; 3-D distance
  needs a pose.
- **Provisional zones/budgets** — conservative placeholders, to be replaced by
  measured latency/stopping data (`human-safety-s1` §3, Phase 4).
- **Latency** ≈ 0.26 s img→state on this shared GPU; the wrist at 6 fps is a slow
  path — a person seen *only* on the wrist reacts slowly (reason to add coverage).
- **Blind spots are not CLEAR** — the monitored volume is the cameras' union;
  unobserved space is presence-unknown by design, closed only by coverage + the
  certified layer.
