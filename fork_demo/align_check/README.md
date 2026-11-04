# Pre-contact alignment check (`fork_demo/align_check/`)

Before the fork tines touch anything, decide whether the pick/place will *seat*
— and refuse it if it won't, so a tine or a 3-D-printed dock never gets bent.
This is the **earliest possible failure detection**: a lateral/yaw error caught
at the pre-grip pause, before the insert.

Everything here is **offline and plan-only**. It reads an image (plus optional
depth / joint pose), prints a GO/NO-GO with a *never-applied* correction
proposal, and writes a verdict JSON for the sibling contact-guard lane's
pre-contact gate. **It never commands the robot, a controller, a driver, a
dashboard, freedrive, or a camera driver**, and the live mode only *subscribes*
to an already-running camera on a task-only `ROS_DOMAIN_ID`.

Scope: the alignment check only. Out of scope (other lanes): moving the robot,
*applying* a correction, the contact guard itself, camera calibration, new
recordings.

---

## The two checks

### Shelf (pick)
From a **wrist** colour image at the in-front-of-tray pause: detect the two
yellow tape strips (the captain's guides — LEFT by STUDENTENFUTTER, RIGHT by BIO
MARONEN), each marking a fork-channel centre. The two fork tines are rigid in
`tool0` (centre-lines at tool X = ±60 mm, Y = −38 mm, running tool Z 47→427 mm,
from the `forklift_S` kit), and the wrist camera is rigid in `tool0` too (the
trusted hand-eye `tool0 → wrist_color_upright_optical_frame`), so **the tines'
image position is a constant of hand-eye + intrinsics — no FK, no arm pose
needed.** We back-project each strip centre into `tool0` (using wrist depth, or
an assumed plane when absent) and report, per tine:

* **lateral offset (mm)** — channel centre minus tine centre-line (tool X);
* **yaw (deg)** — the channel-to-channel axis vs the tine spacing axis (about
  tool Y, the vertical): the insert must be square to the rail.

### Table (place)
Detect **AprilTag 36h11 id 1** (120 mm) in the wrist (or table) image, solve its
pose (`solvePnP`, sub-pixel corners), carry it to `base_link` with the hand-eye
and the UR10 **measured-URDF** FK at the pre-place joints, and compare the
tag-derived dock target with the taught place pose from
`teach-002/labelled_waypoints.json`. Report **dx, dy (mm)** and **dyaw (deg)**
about base Z. The `TOP` arrow is the human-readable orientation reference and,
by print convention, aligns with the tag's +Y axis (the AprilTag decode is
orientation-unambiguous, so the tag frame fixes the dock orientation).

---

## Verdict and tolerances

Tolerance is **derived from the channel clearance**, never guessed:

```
half_clearance = (channel_width - tine_width) / 2       # geometric slack
lateral_tol    = half_clearance / safety_factor         # shrink => conservative
```

`params.yaml` holds every number, under the **tune-up-never-down** convention
(raising a `safety_factor`, lowering a `channel_width`, or tightening a yaw
tolerance only makes the gate *stricter* and is always safe; widening any of
them needs a captain decision recorded here):

| side  | channel / slot | tine / tray | safety | ⇒ position tol | yaw tol |
|-------|---------------:|------------:|-------:|---------------:|--------:|
| shelf | 44 mm `*`      | 40 mm       | 1.5    | **±1.33 mm**   | 3.0°    |
| table | 210 mm `**`    | 200 mm      | 1.5    | **±3.33 mm**   | 3.0°    |

`*` shelf uses the kit **tray-pocket** width (`pocket_w` = 44 mm) as the binding
tine clearance — the 3-D-printed dock U-channel measures ~50 mm in the guide
photo (wider), so the pocket is the conservative choice.
`**` table dock-slot width is an **ASSUMPTION** (S-dock for the 200 mm tray,
~5 mm/side) — captain to confirm.

**GO** iff every position error ≤ position tol **and** |yaw| ≤ yaw tol. The
printed `margin` is `tol − worst_error` (negative ⇒ over tolerance).

### Reading the printed correction
The correction `(dx, dy, dyaw)` is the move that would *zero* the measured
error. **It is printed only and never applied** (the note is in every verdict
file). The captain applies it by hand on the pendant / jog, or the sibling lane
uses the verdict to hold before contact. Example:

```
VERDICT: NO-GO
mean lateral dx = +11.86 mm   yaw dyaw = -0.00 deg
tolerance: +/-1.33 mm, +/-3.00 deg
margin:    -10.56 mm, +3.00 deg  (negative => over tolerance)
CORRECTION PROPOSAL (PRINT ONLY -- never applied automatically):
  dx=-11.86 mm  dy=+0.00 mm  dyaw=+0.00 deg
```

---

## Verdict file schema (contact-guard contract)

The sibling contact-guard lane's pre-contact gate **reads** this file; we do not
edit their code — the schema is agreed here. Written atomically by
`verdict_io.write_verdict`. Required fields: `side, ok, dx_mm, dy_mm, dyaw_deg,
tolerance, margin, image_stamp, assumptions`. `tolerance`/`margin` are objects
with `mm` and `deg`. Additive, non-contract fields (`schema_version`,
`generated_at`, `correction`, `detail`) may be ignored by a strict reader.

```json
{
  "schema_version": "align_check/v1",
  "side": "shelf", "ok": false,
  "dx_mm": 11.863, "dy_mm": 0.0, "dyaw_deg": -0.0,
  "tolerance": {"mm": 1.333, "deg": 3.0},
  "margin":    {"mm": -10.559, "deg": 3.0},
  "correction": {"dx_mm": -11.863, "dy_mm": 0.0, "dyaw_deg": 0.0,
                 "note": "PROPOSAL ONLY -- never applied automatically; ..."},
  "image_stamp": "synthetic-shelf-nogo",
  "assumptions": ["tine geometry from forklift_S kit ...", "..."]
}
```

Full examples: `proof/shelf_verdict_{go,nogo}.json`,
`proof/table_verdict_{go,nogo}.json`.

---

## Offline proof (what's real vs synthetic)

The captain's **guide photos are phone snaps, not the calibrated wrist camera**,
and the teach-002 bag has no camera frames — so they prove the *detectors*, and
the full-geometry verdict is proven on *synthetic* wrist/look-down scenes with
ground truth (real hand-eye + measured-URDF FK + nominal K). Regenerate all with
`python3 fork_demo/align_check/tools/make_proof.py`.

**Detectors on the real guide photos** (`proof/shelf_detector_guidephoto.jpg`,
`proof/table_tag_guidephoto.jpg`):
* shelf: both yellow strips found, rail angle +2.7°;
* table: AprilTag 36h11 **id 1** found (~860 px side at full res).

**FK validated against real telemetry**: FK(pre_grip joints) @ fork TCP
reproduces the recorded teach-002 base_link fork pose to **< 0.1 mm**
(`tests/test_geometry.py::test_fk_matches_real_teach_pose`).

**End-to-end recovery on synthetic ground truth** (`proof/*_synthetic_recovery.png`):

| side  | injected            | recovered            | verdict |
|-------|---------------------|----------------------|---------|
| shelf | lat 0,  yaw 0       | dx −0.29, dyaw 0.00  | GO      |
| shelf | lat +12 mm          | dx +11.86, dyaw 0.00 | NO-GO   |
| shelf | lat −6 mm, yaw +4°  | dx −6.23, dyaw +3.99 | NO-GO   |
| table | dx dy dyaw 0        | 0.00 / 0.00 / 0.00   | GO      |
| table | +5 −3 mm, +2°       | +5.00 / −3.00 / +2.00| NO-GO   |
| table | −4 +6 mm, −1.5°     | −4.00 / +6.00 / −1.50| NO-GO   |

Shelf lateral recovers to < 0.4 mm, yaw < 0.01°; table dx/dy/dyaw recover to
< 0.1 mm / 0.1° (tag solve: ~0.5 px reproj, distance within ~1 mm).

---

## Running it

```bash
# Shelf (wrist image; depth strongly preferred for true mm scale)
python3 fork_demo/align_check/align_check.py --side shelf \
    --image wrist.png [--depth aligned_depth.npy] \
    --out ann.png --verdict shelf_verdict.json

# Table (wrist/table image; --intrinsics = CameraInfo yaml; --wrist-pose = joint yaml -> FK -> base)
python3 fork_demo/align_check/align_check.py --side table \
    --image table.png --intrinsics info.yaml --wrist-pose joints.yaml \
    --out ann.png --verdict table_verdict.json
```

Exit code 0 = GO, 1 = NO-GO / detection-only, 2 = bad input.

### Live mode (documented + dry-tested only — never run against the cell here)
`live_node.py` subscribes read-only to the calibrated topics on a task-only
`ROS_DOMAIN_ID` (`params.yaml` `live.ros_domain_id`, default **76** — never 0/42;
it refuses the cell domains), runs the identical `process_frame` core, and
writes the verdict. It publishes nothing and starts no camera driver. The
ROS-free wiring is dry-tested without rclpy:

```bash
python3 fork_demo/align_check/live_node.py --side shelf --dry-run \
    --image <frame.png> --verdict /tmp/v.json
```

### Tests
```bash
pytest fork_demo/align_check/tests/     # 22 offline tests, no ROS / no cell
```

---

## Assumptions the captain must confirm

1. **Tine geometry = `forklift_S` Isaac kit** (spacing 120 mm, lateral width
   40 mm). The kit is the *stated proxy* for the physical forks.
2. **Shelf channel clearance** uses the 44 mm kit pocket; the real dock U-channel
   should be measured.
3. **Table dock-slot width** 210 mm (≈5 mm/side slack) — assumption.
4. **`TOP` arrow == tag +Y** at print time; **tag centred on the dock slot**
   (`tag_to_place_offset` = 0).
5. **Nominal D435i colour K** offline; live mode uses the real CameraInfo.
6. Without wrist **depth**, shelf lateral mm scales with the assumed plane depth
   (0.45 m) — supply `--depth` for the true value.
7. Offline the **taught place yaw** is unknown from `fork_mean` alone (`dyaw`
   placeholder 0); live mode reads it from `tcp_pose`.

## Files
```
align_check.py   CLI + pipeline (align_shelf / align_table)
shelf_detect.py  yellow-strip detector (cv2) + annotation
tag_detect.py    AprilTag 36h11 detect + solvePnP pose + annotation
geometry.py      pure projection / pose / FK-free tine maths + place-vs-tag compare
fk_ur10.py       UR10 FK from the measured URDF (validated vs real teach pose)
tolerance.py     channel-clearance tolerance + GO/NO-GO + correction (pure)
verdict_io.py    verdict JSON contract writer
calib.py         hand-eye + intrinsics loaders
synth.py         synthetic wrist / tag scene generators (proof + tests)
live_node.py     thin ROS live mode (lazy rclpy; dry-tested; never run here)
params.yaml      all tolerances / geometry / intrinsics (tune-up-never-down)
tools/make_proof.py   regenerate the proof artifacts
tests/           22 offline tests + downscaled guide-photo fixtures
proof/           annotated detections + synthetic recovery + example verdicts
```
