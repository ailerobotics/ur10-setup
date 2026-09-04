# cumotion/maps

Saved nvblox cell maps (`cell-YYYY-MM-DD.nvblx`, TSDF+colour in `base_link`,
2 cm; cuMotion recomputes the ESDF it plans against from the TSDF) for
`start_cumotion_map.sh`. Committed only when under ~50 MB; larger maps stay in
`~/ur10/cumotion/maps/` on the host (the script searches both, newest first).
Cleaned-in-place derivatives are named `cell-YYYY-MM-DD-cleaned.nvblx`, sort
newest and so become operational; their `.md` says what was removed. How maps
are made: `cams/cellmap/README.md`; cleaning in place and when to re-scan:
`RUNBOOK.md` §10.
