# Third-party notices

This repository is published under a proprietary **view-only** licence (see [`LICENSE`](LICENSE)).
That licence covers only the original material in this repository. The third-party software,
models, and assets below remain under their own licences; nothing in this repository changes those
terms.

## Summary

**This repository does not vendor or redistribute any third-party source code.** With one small
exception (the printable AprilTag images, see below), every dependency listed here is installed
from its own distribution (apt / pip / NVIDIA NGC / vendor container) at build or run time, not
committed here. No third-party binary, model weight, library, or source tree is shipped in the
git history.

**Conflict analysis (copyleft / non-commercial vs. this view-only publication):** no committed
file conflicts. See the dedicated section at the end for the two items worth naming — FFmpeg
(LGPL, pulled into an out-of-repo venv at run time) and the NVIDIA / Universal Robots proprietary
runtimes (referenced, never redistributed).

---

## Dependencies referenced or required (not vendored)

| Component | How used here | Licence | Permissive? |
|---|---|---|---|
| ROS 2 Jazzy (rclcpp/rclpy, std_msgs, etc.) | middleware; installed from `packages.ros.org` | Apache-2.0 | ✅ |
| `ur_robot_driver` / `Universal_Robots_ROS2_Driver` (`ros-jazzy-ur` 3.8.0) | real-arm driver bring-up | BSD-3-Clause | ✅ |
| `ur_client_library` | driver dependency | Apache-2.0 | ✅ |
| `ur_moveit_config` / MoveIt 2 | motion planning config | BSD-3-Clause | ✅ |
| Pilz industrial motion planner (MoveIt) | PTP joint-space planning | Apache-2.0 | ✅ |
| `realsense2_camera` / librealsense | RealSense ROS 2 wrapper | Apache-2.0 | ✅ |
| eProsima Fast DDS | ROS 2 RMW transport (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`) | Apache-2.0 | ✅ |
| PyTorch (`torch` 2.9.0+cu130) | tensor runtime for the detector | BSD-3-Clause (+ component notices) | ✅ |
| torchvision (0.24.0) | Keypoint R-CNN model + weights (downloaded at run time) | BSD-3-Clause | ✅ |
| nvblox (core, `nvidia-isaac/nvblox`) | ESDF mapping library | Apache-2.0 | ✅ |
| NVIDIA Isaac ROS cuMotion / `isaac_ros_nvblox` | collision-aware planning; run in NVIDIA containers | NVIDIA licence (Apache-2.0 ROS source + proprietary runtime) | ⚠️ see below |
| cuRobo **V2 (v0.8.0)** | reactive-MPC prototype (`mpc_curobo/`), pinned in its own container | Apache-2.0 | ✅ (v1/0.7.x is non-commercial — **not used**) |
| NVIDIA Isaac Sim / Omniverse Kit | sim-only human-safety test harness | NVIDIA Omniverse Licence (proprietary) | ⚠️ see below |
| Universal Robots URSim (CB3) | sim-only gate testing, run as a container | UR software EULA (proprietary) | ⚠️ see below |
| imageio-ffmpeg + bundled FFmpeg | MP4 encode in an out-of-repo venv (`~/venvs/isaac-video`) | imageio-ffmpeg BSD-2-Clause; bundled FFmpeg LGPL-2.1+ | ⚠️ see below |
| AprilTag project (tag36h11 family) | fixed-camera calibration tags | BSD-2-Clause (tag images public-domain) | ✅ |

## Committed third-party assets (the only vendored material)

- `cams/cellmap/tags/apriltag36h11-id0-120mm.{pdf,png}` and
  `apriltag36h11-id1-120mm.{pdf,png}` — printable AprilTag **tag36h11** fiducial images, from the
  AprilTag project (**BSD-2-Clause**; the generated tag bitmaps are effectively public-domain).
  Permissive and compatible with this repository's view-only publication; no attribution
  conflict.

Everything else under version control in this repository (Python/shell/YAML/URDF sources, RViz
configs, camera captures, HTML reports, teach records, the demo GIF/MP4, the UR program files
under `humangate_driver/sim/ursim_pendant/programs/`) is original to this project.

---

## Copyleft / non-commercial flags

No copyleft or non-commercial licence conflicts with this repository's proprietary view-only
publication, because **no third-party source or binary under such a licence is redistributed
here**. The items below are named for completeness:

1. **FFmpeg — LGPL-2.1+ (copyleft, weak).** The `imageio-ffmpeg` wheel bundles an LGPL build of
   FFmpeg. It is used only by `sim/isaac_human/make_video.py` and `docs/media/gen_media.py`, which
   install it into a venv **outside this repository** (`~/venvs/isaac-video`) at run time. **No
   FFmpeg code or binary is committed to this repository**, so no committed file is affected and
   there is no redistribution of LGPL material. *Affected files: none committed; the two scripts
   above merely invoke an externally-installed binary.*

2. **NVIDIA Isaac ROS cuMotion / `isaac_ros_nvblox` / Isaac Sim / Omniverse Kit, and Universal
   Robots URSim — proprietary, non-permissive (include use restrictions).** These are **referenced
   and run from their own NVIDIA / UR distributions and containers; none of their code or binaries
   is committed to or redistributed by this repository.** A reader who wants to run those layers
   must obtain them directly from NVIDIA / Universal Robots and accept those vendors' licence
   terms separately. Because nothing is redistributed here, there is no conflict with the
   view-only publication of *this* repository's own code. *Affected files: none committed;
   `mpc_curobo/Dockerfile` and the various `sim/` / container launch scripts reference these
   runtimes by name/version only.*

3. **cuRobo licence split — resolved.** cuRobo **v1 (`v0.7.x`)** is NVIDIA non-commercial, which
   *would* conflict. This repository deliberately pins **cuRobo V2 `v0.8.0` (Apache-2.0)** in
   `mpc_curobo/Dockerfile` (`CUROBO_REF=v0.8.0`) and keeps the prototype out of any shipped
   dependency path pending an owner decision (`data/plan-mpc-curobo-v1/`). No non-commercial cuRobo
   code is used or committed. *No conflict.*

---

*Licences are identified from each project's upstream distribution as of this repository's
publication date. If any identification here is inaccurate, the upstream project's own licence
file governs. Report corrections to `engineering@ailerobotics.com`.*
