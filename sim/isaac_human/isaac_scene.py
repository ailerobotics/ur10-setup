#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Isaac Sim scene for the human-detection test: a small UR10-cell stand-in, two
or three simulated RGB-D cameras publishing on the ROS 2 bridge exactly what the
detector nodes consume, and one human character walking a scripted path toward
the robot base and back. The simulator's TRUE person position is recorded every
frame in ``base_link`` with a wall-clock stamp.

SIMULATION ONLY. Run through run_session.sh (conda env + GPU lock + bundled-ROS2
DDS env on domain 88). Nothing here reaches the real robot, its network or the
RealSense cameras.

Cameras publish with ``useSystemTime=True`` so image header stamps are wall-clock
— the detector computes ``latency = wall_now - image_stamp`` and would force a
permanent STOP if we published sim-time stamps starting at 0.

The character is TELEPORTED along the path (its root Xform is moved each step);
there is no skeletal walk-cycle animation. A rendered, posed character is enough
for the COCO-trained Keypoint R-CNN to attempt a detection, and teleporting keeps
the scene light on the 12 GB card (the brief explicitly permits this).
"""
import argparse
import json
import math
import os
import sys
import time

from isaacsim import SimulationApp

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


def sim_topics(role: str) -> dict:
    """The /sim topic names the sim config (human_safety.sim.yaml) points at."""
    return {
        "rgb": f"/sim/{role}/color/image_raw",
        "info": f"/sim/{role}/color/camera_info",
        "depth": f"/sim/{role}/depth/image",
        "depth_info": f"/sim/{role}/depth/camera_info",
    }


# Candidate people-character USDs on the public NVIDIA asset server (no login).
# Tried in order; the first that resolves is used. Overridable with PERSON_USD.
PERSON_CANDIDATES = [
    "/Isaac/People/Characters/original_male_adult_construction_05/male_adult_construction_05.usd",
    "/Isaac/People/Characters/original_male_adult_construction_05/original_male_adult_construction_05.usd",
    "/Isaac/People/Characters/male_adult_construction_01/male_adult_construction_01.usd",
    "/Isaac/People/Characters/F_Business_02/F_Business_02.usd",
    "/Isaac/People/Characters/original_female_adult_business_02/female_adult_business_02.usd",
]


def _resolve_person_usd():
    """Return (usd_path_or_None, note)."""
    override = os.environ.get("PERSON_USD")
    if override:
        return override, f"PERSON_USD override: {override}"
    try:
        from isaacsim.storage.native import get_assets_root_path
    except Exception:  # noqa: BLE001
        try:
            from omni.isaac.nucleus import get_assets_root_path
        except Exception as e:  # noqa: BLE001
            return None, f"no asset-root helper available: {e!r}"
    root = get_assets_root_path()
    if not root:
        return None, "asset root path could not be resolved (offline?)"
    import omni.client

    # 1. Fixed candidates first (fast path, known-good names).
    for rel in PERSON_CANDIDATES:
        url = root + rel
        try:
            res, _ = omni.client.stat(url)
            if res == omni.client.Result.OK:
                return url, f"using people asset (candidate): {url}"
        except Exception:  # noqa: BLE001
            continue

    # 2. Discovery: list People/Characters and take the first <name>/<name>.usd.
    chars_dir = root + "/Isaac/People/Characters/"
    try:
        res, entries = omni.client.list(chars_dir)
        if res == omni.client.Result.OK:
            names = sorted(e.relative_path.strip("/") for e in entries
                           if e.relative_path and "." not in e.relative_path)
            for name in names:
                for cand in (f"{chars_dir}{name}/{name}.usd",
                             f"{chars_dir}{name}/{name}.usda"):
                    try:
                        r2, _ = omni.client.stat(cand)
                        if r2 == omni.client.Result.OK:
                            return cand, f"using people asset (discovered): {cand}"
                    except Exception:  # noqa: BLE001
                        continue
    except Exception as e:  # noqa: BLE001
        return None, f"could not list {chars_dir}: {e!r}"
    return None, f"no people asset resolved or discovered under {chars_dir}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", default="wrist,shelf,table")
    ap.add_argument("--gt-log", required=True, help="ground-truth JSONL output")
    ap.add_argument("--max-seconds", type=float, default=180.0,
                    help="hard wall-clock cap for the whole run")
    ap.add_argument("--settle-seconds", type=float, default=4.0,
                    help="render warm-up before the person starts moving")
    ap.add_argument("--sim-hz", type=float, default=5.0,
                    help="cap Isaac's update/publish rate (Hz). Isaac renders "
                         "faster than the detectors drain, so uncapped the DDS "
                         "image queue backs up and detections are stale (latency "
                         "> the 0.7 s reaction budget -> permanent STOP). Capping "
                         "to a rate the detectors keep up with keeps frames fresh "
                         "and frees GPU for inference.")
    args, _ = ap.parse_known_args()

    roles = [r.strip() for r in args.cams.split(",") if r.strip()]

    sim = SimulationApp({"headless": True, "width": 640, "height": 480})
    rc = 1
    gt_f = None
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")

        import omni.kit.app
        for _ in range(60):
            omni.kit.app.get_app().update()

        import omni.usd
        import omni.timeline
        import omni.graph.core as og
        import omni.replicator.core as rep
        from pxr import Gf, Sdf, UsdGeom, UsdLux

        import cell_layout as layout

        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        # -- ground + lights ---------------------------------------------- #
        ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/Ground"))
        ground.CreatePointsAttr([(-12, -12, 0), (12, -12, 0), (12, 12, 0), (-12, 12, 0)])
        ground.CreateFaceVertexCountsAttr([4])
        ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        ground.CreateNormalsAttr([(0, 0, 1)] * 4)
        key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
        key.CreateIntensityAttr(2500.0)
        UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
        dome.CreateIntensityAttr(600.0)

        # -- robot + furniture stand-ins (boxes) -------------------------- #
        def box(path, centre, half):
            c = UsdGeom.Cube.Define(stage, Sdf.Path(path))
            c.GetSizeAttr().Set(1.0)
            xf = UsdGeom.Xformable(c)
            xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
            xf.AddScaleOp().Set(Gf.Vec3f(2 * half[0], 2 * half[1], 2 * half[2]))
            return c
        box("/World/RobotBase", *layout.ROBOT_BASE)
        box("/World/RobotColumn", *layout.ROBOT_COLUMN)
        box("/World/Shelf", *layout.SHELF_BOX)
        box("/World/Table", *layout.TABLE_BOX)

        # -- person ------------------------------------------------------- #
        person_usd, note = _resolve_person_usd()
        print(f"[isaac_scene] person: {note}", flush=True)
        person_xform = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Person"))
        p_translate = UsdGeom.Xformable(person_xform).AddTranslateOp()
        # Face +Y (toward the robot from the -Y corridor). Character assets face
        # -Y by default in Isaac People, so yaw 180 deg about Z points them at the
        # robot; a primitive fallback is symmetric so orientation is cosmetic.
        UsdGeom.Xformable(person_xform).AddRotateZOp().Set(180.0)
        start = layout.PERSON_PATH[0]
        p_translate.Set(Gf.Vec3d(start[0], start[1], 0.0))
        person_is_asset = False
        if person_usd:
            body = stage.DefinePrim(Sdf.Path("/World/Person/Body"), "Xform")
            body.GetReferences().AddReference(person_usd)
            person_is_asset = True
        else:
            # Fallback: a crude humanoid from primitives (skin-toned). The COCO
            # detector may well NOT fire on this — that is reported honestly.
            print("[isaac_scene] WARN: no people asset — using a PRIMITIVE humanoid "
                  "stand-in; the detector may not recognise it.", flush=True)
            _primitive_person(stage, Gf, Sdf, UsdGeom)

        # -- cameras ------------------------------------------------------ #
        for role in roles:
            cam = layout.camera(role)
            _build_camera(stage, cam, og, rep, Gf, Sdf, UsdGeom)

        omni.timeline.get_timeline_interface().play()
        # Wall-clock settle (NOT a tick count): under GPU contention Isaac ticks
        # slowly, so a fixed tick count would block for minutes. This gives the
        # renderer + the detectors a fixed real-time warm-up while the person
        # stands at the far start position.
        sim_period = 1.0 / max(0.1, args.sim_hz)

        def throttled_update():
            t = time.time()
            sim.update()
            dt = time.time() - t
            if dt < sim_period:
                time.sleep(sim_period - dt)

        t_settle = time.time()
        while time.time() - t_settle < args.settle_seconds:
            throttled_update()

        # -- ground-truth path playback ---------------------------------- #
        gt_f = open(args.gt_log, "w")
        timeline = omni.timeline.get_timeline_interface()

        segments = _plan_path(layout.PERSON_PATH, layout.PERSON_SPEED,
                              layout.PERSON_DWELL_S)
        t_start = time.time()

        def record_gt(x, y):
            now = time.time()
            root = [x, y, 0.0]
            centre = [x, y, layout.PERSON_CENTRE_Z]
            dist = math.hypot(x, y)
            gt_f.write(json.dumps({
                "t": now, "sim_time": float(timeline.get_current_time()),
                "present": True, "root": root, "centre": centre,
                "dist_base": dist,
            }) + "\n")
            gt_f.flush()

        print(f"[isaac_scene] path playback: {len(segments)} legs, "
              f"asset={'people' if person_is_asset else 'primitive'}", flush=True)
        # Walk the planned (x, y) samples, teleporting the root each sim step.
        for (x, y) in _walk_samples(segments, step_dt=1.0 / 15.0):
            if time.time() - t_start > args.max_seconds:
                print("[isaac_scene] hit max-seconds cap; ending path early.", flush=True)
                break
            p_translate.Set(Gf.Vec3d(x, y, 0.0))
            throttled_update()
            record_gt(x, y)
        print("[isaac_scene] path complete; holding a moment for the last frames.",
              flush=True)
        for _ in range(int(2.0 * args.sim_hz)):
            throttled_update()
        rc = 0
    except Exception as exc:  # noqa: BLE001
        import traceback
        print(f"[isaac_scene] FAILED: {exc!r}", flush=True)
        traceback.print_exc()
        rc = 2
    finally:
        if gt_f is not None:
            gt_f.close()
        try:
            sim.close()
        except Exception:  # noqa: BLE001
            pass
    return rc


def _build_camera(stage, cam, og, rep, Gf, Sdf, UsdGeom):
    """Place the USD camera prim and wire RGB + depth + CameraInfo ROS publishers
    (wall-clock stamps) for one SimCamera. Mirrors the shelfbot reference wiring."""
    path = f"/World/cam_{cam.role}"
    ucam = UsdGeom.Camera.Define(stage, Sdf.Path(path))
    m = cam.usd_matrix()
    UsdGeom.Xformable(ucam).AddTransformOp().Set(Gf.Matrix4d(*[float(v) for v in m.flatten()]))
    ucam.GetFocalLengthAttr().Set(float(cam.focal_mm))
    ucam.GetHorizontalApertureAttr().Set(float(cam.h_aperture_mm))
    ucam.GetVerticalApertureAttr().Set(float(cam.v_aperture_mm))
    ucam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))

    topics = sim_topics(cam.role)
    rp = rep.create.render_product(path, (cam.width, cam.height))
    frame = cam.optical_frame
    og.Controller.edit(
        {"graph_path": f"/CamGraph_{cam.role}", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Rgb.inputs:renderProductPath", rp.path),
                ("Rgb.inputs:type", "rgb"),
                ("Rgb.inputs:topicName", topics["rgb"]),
                ("Rgb.inputs:frameId", frame),
                ("Rgb.inputs:useSystemTime", True),
                ("Depth.inputs:renderProductPath", rp.path),
                ("Depth.inputs:type", "depth"),
                ("Depth.inputs:topicName", topics["depth"]),
                ("Depth.inputs:frameId", frame),
                ("Depth.inputs:useSystemTime", True),
                ("Info.inputs:renderProductPath", rp.path),
                ("Info.inputs:topicName", topics["info"]),
                ("Info.inputs:frameId", frame),
                ("Info.inputs:useSystemTime", True),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "Rgb.inputs:execIn"),
                ("OnTick.outputs:tick", "Depth.inputs:execIn"),
                ("OnTick.outputs:tick", "Info.inputs:execIn"),
                ("Context.outputs:context", "Rgb.inputs:context"),
                ("Context.outputs:context", "Depth.inputs:context"),
                ("Context.outputs:context", "Info.inputs:context"),
            ],
        },
    )
    print(f"[isaac_scene] camera '{cam.role}' -> rgb {topics['rgb']}, depth "
          f"{topics['depth']}, info {topics['info']} (frame {frame})", flush=True)


def _primitive_person(stage, Gf, Sdf, UsdGeom):
    """A crude skin-toned humanoid (torso + head + limbs) as an absolute fallback."""
    def part(name, centre, half, shape="cube"):
        p = f"/World/Person/{name}"
        if shape == "sphere":
            g = UsdGeom.Sphere.Define(stage, Sdf.Path(p))
            g.GetRadiusAttr().Set(half)
            UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(*centre))
        else:
            g = UsdGeom.Cube.Define(stage, Sdf.Path(p))
            g.GetSizeAttr().Set(1.0)
            xf = UsdGeom.Xformable(g)
            xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
            xf.AddScaleOp().Set(Gf.Vec3f(2 * half[0], 2 * half[1], 2 * half[2]))
        g.GetDisplayColorAttr().Set([(0.8, 0.6, 0.5)])
    part("Torso", (0, 0, 1.05), (0.16, 0.10, 0.30))
    part("Head", (0, 0, 1.55), 0.12, shape="sphere")
    part("LegL", (-0.09, 0, 0.45), (0.07, 0.07, 0.45))
    part("LegR", (0.09, 0, 0.45), (0.07, 0.07, 0.45))
    part("ArmL", (-0.26, 0, 1.05), (0.05, 0.05, 0.28))
    part("ArmR", (0.26, 0, 1.05), (0.05, 0.05, 0.28))


def _plan_path(waypoints, speed, dwell_s):
    """Forward along the waypoints, dwell at the last, then back to the start.
    Returns a list of (p0, p1, duration_s, is_dwell) legs."""
    fwd = list(waypoints)
    back = list(reversed(waypoints))
    legs = []

    def add(seq):
        for a, b in zip(seq, seq[1:]):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            legs.append((a, b, d / max(1e-6, speed), False))
    add(fwd)
    legs.append((waypoints[-1], waypoints[-1], dwell_s, True))
    add(back)
    return legs


def _walk_samples(legs, step_dt):
    """Yield (x, y) samples along the planned legs at a fixed time step."""
    for (a, b, dur, _is_dwell) in legs:
        n = max(1, int(dur / step_dt))
        for i in range(n):
            f = (i + 1) / n
            yield (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))


if __name__ == "__main__":
    sys.exit(main())
