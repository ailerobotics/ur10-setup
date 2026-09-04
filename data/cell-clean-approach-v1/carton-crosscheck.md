# Live carton positions (wrist cam, depth back-projected to base_link) vs cleaned map shelf

Camera: WRIST D435i upright stream /wrist/color_upright/image_raw (hand-eye
calibrated, tool0->wrist_color_upright_optical_frame from
tool0_to_camera_2026-09-03.yaml). Arm PARKED at tool0 (-0.827,-0.201,+0.888),
camera optical origin base_link (-0.777,-0.155,+0.825). No arm motion.

4 cartons are visible on the gondola middle shelf (ja! Vollmilch, Joghurt,
Zottarella, Exquisa). Depth-median back-projection of each to base_link:

| carton | base_link (x,y,z) m | wrist depth |
|---|---|---|
| ja! Vollmilch (left)   | (-0.438, +1.299, +0.861) | 1.462 m |
| Joghurt / Zottarella   | (-0.104, +1.323, +0.781) | 1.624 m |
| Exquisa (right)        | (+0.218, +1.128, +0.733) | 1.582 m |
| upper shelf board      | (-0.349, +1.504, +1.349) | 1.681 m |
| middle shelf front lip | (+0.021, +1.621, +0.580) | 1.949 m |

Cross-check vs cleaned map (verify_map.py far-field): map first shelf face at
y ~ 1.13 m, faces run y 1.28..1.94, densest 1.64..1.68. Live YOLO (tray_v5_best)
localizes the shelf front to base_link y ~ 1.14 (36 hits, conf<=0.49).

VERDICT: cartons sit at base_link y 1.13..1.32, z 0.73..0.86 -- consistent with
the map's near shelf face (y~1.13) and the live YOLO shelf front (y~1.14). Map,
depth, and detector AGREE on the shelf location to a few cm. The clean-in-place
phantom-shelf risk is NOT realized: the plan approaches a real shelf where the
cartons actually are. Per-carton YOLO confirmation defers to the captain approach
(the eye-level viewpoint boxes shelf boards as 'tray', not the cartons -- the
recorded viewpoint limitation).
