#!/usr/bin/env python3
"""Compressed camera recorder for the fork REPLAY demo (task ur10-fork-pick-real-v1).

Records each camera's colour stream to an H.264 mp4 at a capped frame rate,
piping frames to ffmpeg -- NEVER raw frames into a rosbag (a 3-camera raw bag
cost 51 GB in 7 min and filled the disk). A 10-minute 3-camera run at 15 fps /
848x480 H.264 is a few hundred MB, well under the few-GB cap.

No system ffmpeg exists on this host; we use the ffmpeg binary BUNDLED in the
~/venvs/isaac-video imageio_ffmpeg wheel (AGENTS.md "Record a video"), found via
--ffmpeg or auto-detected. This node runs under system python (/opt/ros/jazzy)
for rclpy and only *shells out* to that ffmpeg binary.

Free-space guard: refuses to start below --min-free-start-gb (20) and stops
cleanly below --min-free-stop-gb (5).

The pure helpers (RateLimiter, image_to_rgb24) are unit-tested offline; the live
ROS path needs the cameras up and is exercised at replay-recording time.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time


# ---------------------------------------------------------------------------
# Pure helpers (unit-tested).
# ---------------------------------------------------------------------------
class RateLimiter:
    """Admit at most `fps` frames/second (drop the rest). Pure/clockless."""

    def __init__(self, fps: float):
        self.min_dt = 1.0 / fps if fps > 0 else 0.0
        self.last_t = None

    def admit(self, t: float) -> bool:
        if self.last_t is None or (t - self.last_t) >= self.min_dt:
            self.last_t = t
            return True
        return False


def image_to_rgb24(width, height, encoding, data):
    """sensor_msgs/Image bytes -> packed RGB24 bytes (W*H*3). Pure."""
    import numpy as np  # noqa: PLC0415

    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    enc = encoding.lower()
    if enc in ("rgb8",):
        arr = buf.reshape(height, width, 3)
    elif enc in ("bgr8",):
        arr = buf.reshape(height, width, 3)[:, :, ::-1]
    elif enc in ("mono8",):
        g = buf.reshape(height, width, 1)
        arr = np.repeat(g, 3, axis=2)
    elif enc in ("rgba8",):
        arr = buf.reshape(height, width, 4)[:, :, :3]
    elif enc in ("bgra8",):
        arr = buf.reshape(height, width, 4)[:, :, 2::-1]
    else:
        raise ValueError(f"unsupported encoding {encoding}")
    return np.ascontiguousarray(arr, dtype=np.uint8).tobytes()


def _free_gb(path: str) -> float:
    try:
        return shutil.disk_usage(path).free / 1e9
    except OSError:
        return float("inf")


def find_ffmpeg(explicit: str | None) -> str | None:
    if explicit and os.path.exists(explicit):
        return explicit
    sysf = shutil.which("ffmpeg")
    if sysf:
        return sysf
    venv_py = os.path.expanduser("~/venvs/isaac-video/bin/python")
    if os.path.exists(venv_py):
        try:
            out = subprocess.check_output(
                [venv_py, "-c", "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())"],
                text=True, timeout=15).strip()
            if out and os.path.exists(out):
                return out
        except (subprocess.SubprocessError, OSError):
            pass
    return None


def _ffmpeg_cmd(ffmpeg, width, height, fps, out_path):
    return [ffmpeg, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
            "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", out_path]


def run(args) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    ffmpeg = find_ffmpeg(args.ffmpeg)
    if not ffmpeg:
        print("no ffmpeg found (pass --ffmpeg or install ~/venvs/isaac-video)", file=sys.stderr)
        return 2
    os.makedirs(args.out_dir, exist_ok=True)
    if _free_gb(args.out_dir) < args.min_free_start_gb:
        print(f"REFUSING to start: < {args.min_free_start_gb} GB free at {args.out_dir}",
              file=sys.stderr)
        return 1

    roles = [r.strip() for r in args.cameras.split(",") if r.strip()]
    rclpy.init()
    node = Node("record_cameras_mp4")
    procs = {}       # role -> ffmpeg Popen
    limiters = {r: RateLimiter(args.fps) for r in roles}
    counts = {r: 0 for r in roles}
    stop = {"v": False}

    def make_cb(role, out_path):
        def cb(m):
            if stop["v"]:
                return
            if not limiters[role].admit(time.time()):
                return
            if role not in procs:
                cmd = _ffmpeg_cmd(ffmpeg, m.width, m.height, args.fps, out_path)
                procs[role] = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                               stdout=subprocess.DEVNULL,
                                               stderr=subprocess.DEVNULL)
                node.get_logger().info(f"{role}: encoding {m.width}x{m.height} -> {out_path}")
            try:
                procs[role].stdin.write(image_to_rgb24(m.width, m.height, m.encoding, m.data))
                counts[role] += 1
            except (BrokenPipeError, ValueError) as e:
                node.get_logger().error(f"{role}: encode error {e}")
        return cb

    for role in roles:
        out_path = os.path.join(args.out_dir, f"{role}.mp4")
        node.create_subscription(Image, args.topic_template.format(role=role),
                                 make_cb(role, out_path), 5)

    import signal
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
    node.get_logger().info(f"recording {roles} @ {args.fps} fps -> {args.out_dir}")
    last_disk = 0.0
    while rclpy.ok() and not stop["v"]:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now - last_disk > 5.0:
            last_disk = now
            if _free_gb(args.out_dir) < args.min_free_stop_gb:
                node.get_logger().error("free space low -- stopping")
                stop["v"] = True
    for role, p in procs.items():
        try:
            p.stdin.close()
            p.wait(timeout=20)
        except (OSError, subprocess.TimeoutExpired):
            p.kill()
    node.get_logger().info(f"frames written: {counts}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--cameras", default="wrist,shelf,table")
    p.add_argument("--topic-template", default="/{role}/color/image_raw")
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--ffmpeg", default=None, help="ffmpeg binary (default: auto-detect)")
    p.add_argument("--min-free-start-gb", type=float, default=20.0)
    p.add_argument("--min-free-stop-gb", type=float, default=5.0)
    return p


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
