# Decision — despeckle the cell map so the arm can plan from park (planner-map-fix-v1)

- **Date:** 2026-09-10
- **Author:** planner-map-fix-v1 (crewmate)
- **Authority:** scout `~/firstmate/data/planner-parkpose-s1/report.md` (verdict + recipe),
  which itself reproduced the live refusal recorded in `planner-bench-s1/report.md`.
- **Scope:** recorded data only. No arm motion, no robot commands, no driver restarts,
  no live-rig map swap. The original map was never modified or removed.

## The defect

At the park pose (`JOINTS` rad `[-0.0828, -2.8959, 0.8008, 5.2519, -0.6772, 0.0196]`,
where the arm rests), cuMotion refuses **every** plan that starts from park — a
boundary/start-state validity failure (`MoveItErrorCode 99999` at `planning_time=0`,
proven live at the bench, not a solver timeout).

The cause is a **single isolated weight-1 mid-air TSDF voxel** at base_link
`[-0.670, 0.130, 0.250]` (67 cm behind base, 25 cm high), 26-connected component
size **1**, no neighbours, invisible in the mesh — a one-shot sensor artifact the
2026-09-04 block-granularity clean could not catch. It is the **only** occupied voxel
inside the park pose's 4 cm world buffer, breaching it by ~0.12 cm on the worst
(forearm) sphere. Without the buffer the arm is **+3.9 cm** clear of it — nowhere near
touching anything real. The same speck also blocks the early park→tag1-hover retract
(the folded arm's upper-arm/forearm swing past that point). It is not the pedestal,
table, or shelf — removing it deletes a false collision, not a real obstacle.

## The fix — corrected map as a NEW file (never overwrites the original)

Produced with the repo's own tool (copies src→dst, zeroes the TSDF weight of voxels
whose centre is inside a base_link box, self-checks; refuses src==dst). The 2 cm box is
one voxel wide, centred on the speck:

```bash
python3 cams/cellmap/tools/nvblx_clean.py \
    ~/ur10/cumotion/maps/cell-2026-09-04-cleaned.nvblx \
    ~/ur10/cumotion/maps/parkpose-fix-cell-2026-09-04-despeck.nvblx \
    --remove-box=-0.68,-0.66,0.12,0.14,0.24,0.26 --self-check
```

- **Original (unchanged):** `~/ur10/cumotion/maps/cell-2026-09-04-cleaned.nvblx`
  — md5 `6b9295c06b5f855b4a3e93a73f7629ff`, 79 441 920 B.
- **Corrected (new):** `~/ur10/cumotion/maps/parkpose-fix-cell-2026-09-04-despeck.nvblx`
  — md5 `776ac34b6bd8239756e9dfe90eaeae7a`, 79 441 920 B.

Both maps are >50 MB and live in the gitignored `~/ur10/cumotion/maps/` (per RUNBOOK §10);
they are not committed. This directory (`data/planner-map-fix-v1/`) holds the exact
command, the tool output, the verification scripts and their captured results, so the
corrected map is fully **regenerable from the original** by re-running the command above.

## Verification (recorded data, reproduced independently of the scout)

Run captures are committed beside this doc.

### 1. Exactly ONE voxel removed — not a region (`voxel-diff.txt`, `voxel_diff.py`)

Value-level diff of the two TSDF layers:

| | original | corrected |
|---|---|---|
| tsdf blocks | 11247 | 11247 (0 added, 0 dropped) |
| occupied voxels (w≥0.1 ∧ d≤0) | **249337** | **249336** (Δ = 1) |
| voxels whose (distance,weight) bytes changed at all | — | **1** |
| occupied→unobserved (removed) | — | **1**, at `[-0.670, 0.130, 0.250]`, weight 1.0→0.0, distance byte unchanged |
| unobserved→occupied (added) | — | 0 |

Every other block index and every other voxel's (distance, weight) is byte-identical.
No real obstacle was quietly deleted — the change is provably the single speck.

### 2. Park clearance before/after (`clearance-before-after.txt`, `clearance.py`)

Offline FK from `cumotion/ur10_calibrated.urdf`, XRDF spheres from `cumotion/ur10_map.xrdf`
(0.04 m buffer on forearm/wrists/tool0, 0.02 m shoulder/upper_arm), ESDF surface distance
≈ nearest occupied-voxel-centre distance − ½ voxel. No ROS, no `/compute_fk`, no rig.

| | park start-state worst clearance | park→tag1-hover worst (naive straight line) | cuMotion |
|---|---|---|---|
| **Original** | **−0.12 cm** (forearm, the speck at `[-0.670,0.130,0.250]`) | **−3.92 cm** (upper_arm — same speck) → IN COLLISION | REFUSES |
| **Corrected** | **+3.23 cm** (wrist_3, real shelf at `[-0.710,0.350,1.030]`) | **+0.95 cm** (forearm, real structure) → CLEAR | PLANNABLE |

Removing the speck flips the park start state from −0.12 cm to **+3.23 cm** clear, and a
naive straight-line park→tag1-hover interpolation from −3.92 cm to **+0.95 cm** clear
end-to-end. Map-aware cuMotion, which optimises around obstacles, plans it comfortably
once the start state is valid. These numbers reproduce the scout's within ±1 cm
discretisation.

### 3. On "confirm the plan now succeeds" within the hard boundary

The bench proved the park refusal is a start-state invalidity at `planning_time=0`, whose
sole cause is the speck. The offline collision model agrees with the live refusal
(−0.12 cm ↔ 99999), which validates it; the same model shows the start state (and the
retract) become valid once the speck is gone. Re-running the live cuMotion solver needs
the GPU container and would disturb the running planner — out of scope here. The
recorded-data confirmation above is the strongest evidence available within the boundary.

## Adoption — NOT done here; a one-line change the captain makes at the rig

**The live system still uses the OLD map** (`cell-2026-09-04-cleaned.nvblx`). This task did
not swap it. Dropping the corrected file into the maps dir does **not** auto-adopt it: the
launcher's no-arg auto-pick globs `cell-*.nvblx` and the corrected file is named
`parkpose-fix-…` on purpose, so it is inert until named explicitly.

When at the rig, the captain adopts it with (RUNBOOK §10 "Use an existing map" — stop the
running planner first, then start it on the new map):

```bash
systemctl --user stop ur10-cumotion; docker stop ur10-cumotion
~/ur10/cumotion/start_cumotion_map.sh parkpose-fix-cell-2026-09-04-despeck.nvblx
```

Then re-verify per RUNBOOK §10 (must log "cell map loaded … starting cuMotion", no
"World update failed"; regenerate the mesh and check tags/table/arm). To roll back,
restart on `cell-2026-09-04-cleaned.nvblx` (or omit the arg once that is again the newest
`cell-*.nvblx`). Rollback is trivial because the original is untouched.
