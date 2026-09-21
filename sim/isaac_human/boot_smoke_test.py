#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Isaac Sim headless BOOT SMOKE TEST for the human-detection sim harness.

Order-of-work step 1 (per the task brief): confirm that Isaac Sim 5.1 boots
headless on this machine (earlier boots crashed in the RTX renderer at start-up),
enables the ROS 2 bridge, and publishes ONE camera whose frames actually arrive
on the ROS side.

This script is the ISAAC side only: it boots, builds a tiny scene (ground +
light + a cube + one camera), wires an RGB ROS 2 publisher via OmniGraph, plays,
and ticks for a while so frames stream on ``/sim/smoke/rgb``. A SEPARATE
system-ROS subscriber (see run_boot_smoke.sh) counts the frames and decides
pass/fail — that cross-process check is exactly the DDS path the real run uses.

Run it through run_boot_smoke.sh, never directly (it needs the conda env + the
GPU lock + the bundled-ROS2 DDS env).
"""
import argparse
import os
import sys
import time

from isaacsim import SimulationApp

_TOPIC = "/sim/smoke/rgb"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=25.0,
                    help="wall-clock seconds to keep publishing before exit")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args, _ = ap.parse_known_args()

    # headless=True renders offscreen (no window); we never open a viewer or a
    # livestream. Keep the render resolution small to stay well inside the card.
    sim = SimulationApp({
        "headless": True,
        "width": args.width,
        "height": args.height,
    })

    exit_code = 1
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

        # -- tiny scene ---------------------------------------------------- #
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        # Ground plane.
        ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/Ground"))
        ground.CreatePointsAttr([(-10, -10, 0), (10, -10, 0),
                                 (10, 10, 0), (-10, 10, 0)])
        ground.CreateFaceVertexCountsAttr([4])
        ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        ground.CreateNormalsAttr([(0, 0, 1)] * 4)

        # A distant light (dome-free, cheap for headless RTX).
        light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Light"))
        light.CreateIntensityAttr(3000.0)

        # A visible cube in front of the camera so the frame is not blank.
        cube = UsdGeom.Cube.Define(stage, Sdf.Path("/World/Cube"))
        UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))

        # One camera looking at the cube from a few metres back.
        cam_path = "/World/SmokeCam"
        cam = UsdGeom.Camera.Define(stage, Sdf.Path(cam_path))
        eye = Gf.Vec3d(3.0, 0.0, 1.5)
        target = Gf.Vec3d(0.0, 0.0, 0.5)
        fwd = (target - eye).GetNormalized()
        z = -fwd
        x = Gf.Cross(Gf.Vec3d(0, 0, 1), z).GetNormalized()
        y = Gf.Cross(z, x)
        UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d(
            x[0], x[1], x[2], 0.0,
            y[0], y[1], y[2], 0.0,
            z[0], z[1], z[2], 0.0,
            eye[0], eye[1], eye[2], 1.0))
        cam.GetFocalLengthAttr().Set(24.0)
        cam.GetHorizontalApertureAttr().Set(33.0)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))

        # -- RGB ROS 2 publisher via OmniGraph ----------------------------- #
        render_product = rep.create.render_product(cam_path, (args.width, args.height))
        og.Controller.edit(
            {"graph_path": "/SmokeCamGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Rgb.inputs:renderProductPath", render_product.path),
                    ("Rgb.inputs:type", "rgb"),
                    ("Rgb.inputs:topicName", _TOPIC),
                    ("Rgb.inputs:frameId", "smoke_cam"),
                    ("Rgb.inputs:resetSimulationTimeOnStop", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "Rgb.inputs:execIn"),
                    ("Context.outputs:context", "Rgb.inputs:context"),
                ],
            },
        )

        print(f"[boot_smoke] scene built; publishing RGB on {_TOPIC} "
              f"({args.width}x{args.height}) — domain {os.environ.get('ROS_DOMAIN_ID')}",
              flush=True)

        omni.timeline.get_timeline_interface().play()
        t0 = time.time()
        ticks = 0
        while time.time() - t0 < args.seconds:
            sim.update()
            ticks += 1
            if ticks % 60 == 0:
                print(f"[boot_smoke] ticked {ticks} frames "
                      f"({time.time() - t0:.1f}s)", flush=True)
        print(f"[boot_smoke] done publishing after {ticks} ticks; RTX renderer "
              f"survived boot + play.", flush=True)
        exit_code = 0
    except Exception as exc:  # noqa: BLE001
        import traceback
        print(f"[boot_smoke] FAILED: {exc!r}", flush=True)
        traceback.print_exc()
        exit_code = 2
    finally:
        try:
            sim.close()
        except Exception:  # noqa: BLE001
            pass
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
