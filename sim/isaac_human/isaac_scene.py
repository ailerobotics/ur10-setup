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
    import live_view
    args, _ = live_view.build_scene_arg_parser().parse_known_args()

    roles = [r.strip() for r in args.cams.split(",") if r.strip()]

    # Livestreaming needs a viewport big enough to be watchable on a phone; the
    # headless analysis runs stay at the small 640x480 render the detectors use.
    if args.livestream:
        sim = SimulationApp({"headless": True, "width": 1280, "height": 720})
    else:
        sim = SimulationApp({"headless": True, "width": 640, "height": 480})
    rc = 1
    gt_f = None
    beacon = None
    observer = None
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")

        import omni.kit.app
        for _ in range(60):
            omni.kit.app.get_app().update()

        # -- WebRTC livestream (owner's shelfbot pattern) ------------------- #
        # Enable AFTER the app is up: point ICE at the tailnet IP so a phone on
        # the tailnet can reach the media, on the fixed signalling port, then
        # turn on the extension. Best-effort: a livestream failure must not sink
        # the run (the detection graph is the point), so warn and carry on.
        if args.livestream:
            try:
                import carb
                public_ip = args.public_ip or "127.0.0.1"
                settings = carb.settings.get_settings()
                settings.set("/app/livestream/publicEndpointAddress", public_ip)
                settings.set("/app/livestream/port", int(args.livestream_port))
                omni.kit.app.get_app().get_extension_manager(
                    ).set_extension_enabled_immediate(
                    "omni.kit.livestream.webrtc", True)
                print(f"[isaac_scene] WebRTC livestream ON — publicEndpointAddress "
                      f"{public_ip}, signalling port {args.livestream_port}",
                      flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"[isaac_scene] WARN: livestream enable failed ({exc!r}); "
                      "continuing headless.", flush=True)

        import omni.usd
        import omni.timeline
        import omni.graph.core as og
        import omni.replicator.core as rep
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

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

        # -- SLOW / STOP floor rings (livestream OR record) --------------- #
        # Flat coloured circles at the SAME radii the detector uses, read from
        # the safety config (never hard-coded). Makes the picture meaningful: the
        # person visibly crosses amber (SLOW) then red (STOP). Drawn for the
        # WebRTC live view and for the offline recording alike.
        if (args.livestream or args.record) and args.config:
            try:
                import live_view
                reach, stop_r, slow_r = live_view.read_zone_radii(args.config)
                for spec in live_view.zone_ring_specs(reach, stop_r, slow_r):
                    _build_floor_ring(stage, spec, Gf, Sdf, UsdGeom)
                print(f"[isaac_scene] floor rings: SLOW r={reach + slow_r:.2f} m, "
                      f"STOP r={reach + stop_r:.2f} m (from {args.config})",
                      flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"[isaac_scene] WARN: could not draw zone rings ({exc!r}).",
                      flush=True)

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

        # -- streamed viewport framing + live-zone beacon (livestream only) - #
        # -- OR the fixed observer camera + beacon capture (record) --------- #
        beacon = None
        observer = None
        if args.livestream:
            _frame_overview_viewport()
            if args.zone_beacon:
                beacon = _ZoneBeacon(stage, Gf, Sdf, UsdGeom)
        if args.record:
            observer = _ObserverCapture(stage, rep, Gf, Sdf, UsdGeom, args)
            # The recorded picture carries the same state beacon as the live view.
            if beacon is None:
                beacon = _ZoneBeacon(stage, Gf, Sdf, UsdGeom)

        omni.timeline.get_timeline_interface().play()
        # Wall-clock settle (NOT a tick count): under GPU contention Isaac ticks
        # slowly, so a fixed tick count would block for minutes. This gives the
        # renderer + the detectors a fixed real-time warm-up while the person
        # stands at the far start position.
        sim_period = 1.0 / max(0.1, args.sim_hz)

        def throttled_update():
            t = time.time()
            sim.update()
            if beacon is not None:
                beacon.pump()
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

        # World-bounding-box ground truth: the TRUE body extent in base_link each
        # frame (not just the placed root), so the analysis can compare the
        # detector against (a) the body centroid and (b) the nearest body point
        # to the base z-axis, and prove exactly where the real body is.
        person_prim = stage.GetPrimAtPath(Sdf.Path("/World/Person"))
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])

        def _true_body(px, py):
            """(centroid_xyz, nearest_xy_to_base, bbox_min, bbox_max) in base.
            Falls back to the placed root if the world bound is empty/invalid."""
            bbox_cache.Clear()
            rng = bbox_cache.ComputeWorldBound(person_prim).ComputeAlignedRange()
            if rng.IsEmpty():
                return ([px, py, layout.PERSON_CENTRE_Z], [px, py],
                        [px, py, 0.0], [px, py, layout.PERSON_BODY_HEIGHT])
            mn, mx = rng.GetMin(), rng.GetMax()
            cen = [0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1]),
                   0.5 * (mn[2] + mx[2])]
            # nearest point of the xy footprint to the base z-axis (origin): clamp
            # (0,0) into [xmin,xmax]x[ymin,ymax].
            nx = max(mn[0], min(0.0, mx[0]))
            ny = max(mn[1], min(0.0, mx[1]))
            return cen, [nx, ny], [mn[0], mn[1], mn[2]], [mx[0], mx[1], mx[2]]

        def record_gt(x, y):
            now = time.time()
            root = [x, y, 0.0]
            cen, near_xy, bmin, bmax = _true_body(x, y)
            centre = cen                      # TRUE body centroid (world bbox)
            nearest_body = [near_xy[0], near_xy[1], 0.0]
            dist = math.hypot(x, y)           # to placed root (feet), horizontal
            dist_centroid = math.hypot(cen[0], cen[1])
            dist_nearest = math.hypot(near_xy[0], near_xy[1])
            gt_f.write(json.dumps({
                "t": now, "sim_time": float(timeline.get_current_time()),
                "present": True, "root": root, "centre": centre,
                "nearest_body": nearest_body,
                "bbox_min": bmin, "bbox_max": bmax,
                "dist_base": dist,
                "dist_base_centroid": dist_centroid,
                "dist_base_nearest": dist_nearest,
            }) + "\n")
            gt_f.flush()

        print(f"[isaac_scene] path playback: {len(segments)} legs, "
              f"asset={'people' if person_is_asset else 'primitive'}, "
              f"loop={'on' if args.loop_walk else 'off'}", flush=True)
        # Walk the planned (x, y) samples, teleporting the root each sim step. In
        # loop mode (the live-view session) repeat the in-and-out walk until the
        # max-seconds cap so the owner always has motion to watch.
        capped = False
        lap = 0
        while not capped:
            lap += 1
            for (x, y) in _walk_samples(segments, step_dt=1.0 / 15.0):
                if time.time() - t_start > args.max_seconds:
                    print("[isaac_scene] hit max-seconds cap; ending path.",
                          flush=True)
                    capped = True
                    break
                p_translate.Set(Gf.Vec3d(x, y, 0.0))
                throttled_update()
                record_gt(x, y)
                if observer is not None:
                    observer.maybe_grab()
            if not args.loop_walk:
                break
            if not capped:
                print(f"[isaac_scene] lap {lap} complete; looping the walk.",
                      flush=True)
        print("[isaac_scene] path complete; holding a moment for the last frames.",
              flush=True)
        for _ in range(int(2.0 * args.sim_hz)):
            throttled_update()
            if observer is not None:
                observer.maybe_grab()
        if observer is not None:
            print(f"[isaac_scene] observer capture: {observer.n} frames saved.",
                  flush=True)
        rc = 0
    except Exception as exc:  # noqa: BLE001
        import traceback
        print(f"[isaac_scene] FAILED: {exc!r}", flush=True)
        traceback.print_exc()
        rc = 2
    finally:
        if observer is not None:
            observer.shutdown()
        if beacon is not None:
            beacon.shutdown()
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


def _build_floor_ring(stage, spec, Gf, Sdf, UsdGeom, width=0.10, n=96):
    """Draw one zone as a flat coloured annulus (a thick ring) on the floor at
    radius ``spec.radius`` about base_link. A ring band, not a filled disk, so the
    two zones read as concentric circles like the RViz markers. Slightly raised
    off z=0 (STOP just above SLOW) to avoid z-fighting with the ground."""
    import math
    r = float(spec.radius)
    inner = max(0.0, r - 0.5 * width)
    outer = r + 0.5 * width
    z = 0.02 if spec.name == "STOP" else 0.015
    pts, counts, idx = [], [], []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        ca, sa = math.cos(a), math.sin(a)
        pts.append((inner * ca, inner * sa, z))
        pts.append((outer * ca, outer * sa, z))
    for k in range(n):
        i0, i1 = 2 * k, 2 * k + 1
        i2, i3 = (2 * ((k + 1) % n)), (2 * ((k + 1) % n) + 1)
        # quad (i0 inner, i1 outer, i3 next-outer, i2 next-inner)
        idx.extend([i0, i1, i3, i2])
        counts.append(4)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(f"/World/ZoneRing_{spec.name}"))
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateDoubleSidedAttr(True)
    r0, g0, b0, a0 = spec.rgba
    m.CreateDisplayColorAttr([Gf.Vec3f(r0, g0, b0)])
    m.CreateDisplayOpacityAttr([float(a0)])


def _frame_overview_viewport(eye=(5.5, 1.0, 3.0), target=(0.2, -2.4, 0.5)):
    """Point the streamed persp viewport at the whole cell — a fixed observer
    that keeps the robot stand-in, the person's full path and the floor in view.
    Best-effort: framing must never sink the run."""
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=list(eye), target=list(target),
                        camera_prim_path="/OmniverseKit_Persp")
        # Wide FOV so the whole corridor fits.
        import omni.usd
        from pxr import UsdGeom as _UsdGeom
        stage = omni.usd.get_context().get_stage()
        persp = _UsdGeom.Camera(stage.GetPrimAtPath("/OmniverseKit_Persp"))
        if persp:
            persp.GetFocalLengthAttr().Set(12.0)
            persp.GetHorizontalApertureAttr().Set(24.0)
        print(f"[isaac_scene] viewport framed on the cell (eye={eye}, "
              f"target={target}).", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[isaac_scene] WARN: could not frame overview viewport ({exc!r}).",
              flush=True)


def _parse_even_size(spec, default=(1280, 720)):
    """Parse a ``WxH`` string into an even-dimension (w, h) tuple."""
    try:
        w_s, h_s = str(spec).lower().split("x")
        w, h = int(w_s), int(h_s)
    except Exception:  # noqa: BLE001
        w, h = default
    w = max(2, w - (w % 2))
    h = max(2, h - (h % 2))
    return w, h


class _ObserverCapture:
    """Capture the fixed OBSERVER camera (the whole cell + the floor rings + the
    state beacon — the same framing the live view streams) to timestamped JPEG
    frames for the offline video composer, WITHOUT any WebRTC stream.

    A dedicated USD camera + Replicator render product with an ``rgb`` annotator;
    the frame is read after each sim step and written straight to disk with a
    wall-clock stamp, plus a JSONL index line. Best-effort throughout: any failure
    leaves the capture inert (``ok=False``) and never sinks the run — the detector
    graph, not the picture, is the point.
    """

    # Fixed observer eye/target: matches _frame_overview_viewport so the recorded
    # picture frames the whole corridor exactly like the live stream.
    EYE = (5.5, 1.0, 3.0)
    TARGET = (0.2, -2.4, 0.5)

    def __init__(self, stage, rep, Gf, Sdf, UsdGeom, args):
        self.ok = False
        self.n = 0
        self._step = 0
        self.every = max(1, int(getattr(args, "observer_every", 1)))
        self.dir = args.observer_frames_dir
        self.index_path = args.observer_index
        self._idx_f = None
        self._annot = None
        try:
            import sim_poses
            w, h = _parse_even_size(getattr(args, "observer_size", "1280x720"))
            self.w, self.h = w, h
            path = "/World/cam_observer"
            ucam = UsdGeom.Camera.Define(stage, Sdf.Path(path))
            m = sim_poses.usd_camera_matrix(self.EYE, self.TARGET)
            UsdGeom.Xformable(ucam).AddTransformOp().Set(
                Gf.Matrix4d(*[float(v) for v in m.flatten()]))
            # Wide FOV so the whole corridor fits (mirrors _frame_overview_viewport).
            ucam.GetFocalLengthAttr().Set(12.0)
            ucam.GetHorizontalApertureAttr().Set(24.0)
            ucam.GetVerticalApertureAttr().Set(24.0 * h / float(w))
            ucam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 200.0))
            self._rp = rep.create.render_product(path, (w, h))
            self._annot = rep.AnnotatorRegistry.get_annotator("rgb")
            self._annot.attach([self._rp])
            os.makedirs(self.dir, exist_ok=True)
            self._idx_f = open(self.index_path, "w")
            self.ok = True
            print(f"[isaac_scene] observer capture -> {self.dir} ({w}x{h}), "
                  f"index {self.index_path}, every {self.every} step(s)",
                  flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[isaac_scene] WARN: observer capture setup failed ({exc!r}); "
                  "no observer frames.", flush=True)

    def maybe_grab(self):
        if not self.ok:
            return
        self._step += 1
        if self._step % self.every != 0:
            return
        try:
            import numpy as np
            from PIL import Image as PImage
            data = self._annot.get_data()
            arr = np.asarray(data)
            if arr.size == 0 or arr.ndim < 2:
                return
            if arr.ndim == 3 and arr.shape[2] == 4:
                arr = arr[:, :, :3]
            im = PImage.fromarray(np.ascontiguousarray(arr.astype(np.uint8)))
            now = time.time()
            fname = os.path.join(self.dir, f"obs_{int(now * 1000)}.jpg")
            im.save(fname, quality=80)
            self._idx_f.write(json.dumps({"path": fname, "t": now}) + "\n")
            self._idx_f.flush()
            self.n += 1
        except Exception as exc:  # noqa: BLE001
            if self.n == 0:
                print(f"[isaac_scene] WARN: observer grab failed ({exc!r}); "
                      "continuing.", flush=True)

    def shutdown(self):
        try:
            if self._idx_f is not None:
                self._idx_f.close()
        except Exception:  # noqa: BLE001
            pass


class _ZoneBeacon:
    """Best-effort live-zone indicator: a sphere above the robot column that turns
    green / amber / red from the detector's published ``/human_safety/state``.

    Read-only supplement to the picture — it subscribes on the SIM domain (88,
    set by isaac_env.sh) via Isaac's BUNDLED rclpy. Every step is wrapped so a
    ROS hiccup can never take down the render loop; if setup fails the beacon is
    simply inert and the scene still streams. The state routinely reads STOP for
    latency reasons on this shared card — that is the safety layer stopping, and
    the beacon shows it honestly.
    """

    def __init__(self, stage, Gf, Sdf, UsdGeom):
        self._ok = False
        self._node = None
        self._did_init = False
        self._last = None
        self._color_attr = None
        import live_view
        self._zone_color = live_view.zone_color
        try:
            sph = UsdGeom.Sphere.Define(stage, Sdf.Path("/World/ZoneBeacon"))
            sph.GetRadiusAttr().Set(0.16)
            UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.95))
            sph.CreateDisplayColorAttr([Gf.Vec3f(*self._zone_color(None))])
            self._color_attr = sph.GetDisplayColorAttr()
            self._Gf = Gf
        except Exception as exc:  # noqa: BLE001
            print(f"[isaac_scene] WARN: zone beacon prim failed ({exc!r}).",
                  flush=True)
            return
        try:
            import rclpy
            from std_msgs.msg import String
            if not rclpy.ok():
                rclpy.init(args=None)
                self._did_init = True
            self._rclpy = rclpy
            self._node = rclpy.create_node("isaac_zone_beacon")
            self._node.create_subscription(String, "/human_safety/state",
                                           self._on_state, 10)
            self._ok = True
            print("[isaac_scene] zone beacon subscribed to /human_safety/state.",
                  flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[isaac_scene] WARN: zone beacon ROS setup failed ({exc!r}); "
                  "beacon stays neutral.", flush=True)

    def _on_state(self, msg):
        try:
            import json
            zone = json.loads(msg.data).get("zone")
            if zone != self._last:
                self._last = zone
                self._color_attr.Set([self._Gf.Vec3f(*self._zone_color(zone))])
        except Exception:  # noqa: BLE001
            pass

    def pump(self):
        if not self._ok:
            return
        try:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:  # noqa: BLE001
            pass

    def shutdown(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
            if self._did_init and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


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
