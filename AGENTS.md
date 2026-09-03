# Project agent memory

This file is the project's committed home for project-intrinsic agent knowledge: build, test, release, architecture, and sharp-edge notes that should travel with the code.

- Recurring operational commands, health checks, and recovery procedures: `RUNBOOK.md` (authoritative).
- Camera bring-up sharp edges (serial_no YAML trap, orphaned nodes, stale ros2 daemon, torchvision cu130, D415 RGB-interface USB fault): `cams/README.md` "Traps".
- The wrist D435i is mounted upside down. `/wrist/color_upright/*` (+ frame `wrist_color_upright_optical_frame`) is the stream the hand-eye calibration and the cell map consume; `/wrist/color/*` is the untouched driver output. Never pair an image from one with the CameraInfo/frame of the other — `cams/README.md` "Wrist camera is mounted upside down".
- Everything runs on ROS_DOMAIN_ID=0 with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`; the `shelfbot_ros` container bakes domain 42, so override per `docker exec -e`.
- Killing a `ros2 launch` can orphan its child nodes (they reparent to user init and keep publishing — including static TFs). `pgrep -af <node>` before relaunching anything camera- or TF-related.
- MoveIt pose-goal planning here can sample distant IK configurations (observed: +405° shoulder rewind for a 0.6 m translation). For predictable motion use seeded `/compute_ik` + a joint-space goal (Pilz PTP) and a per-joint delta guard — see `cams/approach/approach_demo.py`.
- Never execute motion on the real arm from an agent session unless the task explicitly authorizes it; plan-only (`plan_only=True` on `/move_action`) is always safe.

## Maintaining this file

Keep this file for knowledge useful to almost every future agent session in this project.
Do not repeat what the codebase already shows; point to the authoritative file or command instead.
Prefer rewriting or pruning existing entries over appending new ones.
When updating this file, preserve this bar for all agents and keep entries concise.
