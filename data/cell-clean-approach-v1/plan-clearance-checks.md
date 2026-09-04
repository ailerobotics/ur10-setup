# Arm-sweep clearance checks for `tag1-shelf-cumotion` (2026-09-04)

Both figures quoted in `captain-execute-note.md`, `cams/cellmap/README.md` §5
and the commit message come from `plan_test.py --recheck --check-box`, which
sweeps EVERY collision sphere of the arm (XRDF `ur10_map.xrdf`, + its 0.04 m
buffer) through all 313 saved waypoints via `/compute_fk` and reports the
minimum distance to an axis-aligned base_link box. Negative = the arm passes
through the box. Requires the cleaned-map planner up
(`start_cumotion_map.sh cell-2026-09-04-cleaned.nvblx`); plan-only, no motion.

`--recheck` writes its result into the plan yaml's single `box_check` slot, so
each run OVERWRITES the previous one. The committed
`cams/cellmap/plans/tag1-shelf-cumotion.yaml` therefore holds only the LAST
check run (the shelf/carton box); the table-slab result exists only here.

## 1. Table slab

```bash
cd ~/ur10
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/plan_test.py \
    --recheck tag1-shelf-cumotion --check-box=-1.0,1.0,-1.92,-0.90,-0.12,-0.03
```
```
tag1-shelf-cumotion: min arm clearance to box +4.8 cm (wrist_1_link at point 0/313) -> clear
```
Box = the table top (verify_map.py: dominant plane z −0.070..−0.060) over the
tag area, x ±1.0 / y −1.92..−0.90 / z −0.12..−0.03. Closest at the START point
(the hover above tag 1), on wrist_1 — the plan then arcs up and away.

## 2. Shelf / carton volume

```bash
cd ~/ur10
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/plan_test.py \
    --recheck tag1-shelf-cumotion --check-box=-0.6,0.4,1.10,1.94,0.45,1.55
```
```
tag1-shelf-cumotion: min arm clearance to box +13.7 cm (wrist_3_link at point 312/313) -> clear
```
Box = the gondola shelf front face through the wall behind it (map faces y
1.13..1.94, `verify-cleaned.txt`), x −0.6..0.4, z 0.45..1.55 covering the
carton shelf. Closest at the END point (the standoff), on wrist_3 — this is the
run whose result is stored in the committed yaml (`box_check.min_clearance_m
0.1369`, `link wrist_3_link`, `point 312`).

To reproduce both, run them in this order; the yaml's `box_check` will again
hold the shelf result afterwards.
