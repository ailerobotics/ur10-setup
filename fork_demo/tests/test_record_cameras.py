#!/usr/bin/env python3
"""Offline tests for the compressed camera recorder helpers. No ROS/ffmpeg.

Run:  python3 -I fork_demo/tests/test_record_cameras.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # fork_demo/

from record_cameras_mp4 import RateLimiter, image_to_rgb24, _ffmpeg_cmd  # noqa: E402


def test_rate_limiter_caps_fps():
    rl = RateLimiter(15.0)  # min_dt ~0.0667 s
    assert rl.admit(0.0)            # first always admitted
    assert not rl.admit(0.01)       # too soon
    assert not rl.admit(0.05)       # still too soon
    assert rl.admit(0.07)           # >= 1/15 later
    assert not rl.admit(0.10)
    assert rl.admit(0.14)


def test_rate_limiter_zero_fps_admits_all():
    rl = RateLimiter(0.0)
    assert rl.admit(0.0) and rl.admit(0.0) and rl.admit(0.0)


def test_image_to_rgb24_rgb_passthrough():
    data = bytes([10, 20, 30, 40, 50, 60])  # 2 px rgb8
    out = image_to_rgb24(2, 1, "rgb8", data)
    assert out == data


def test_image_to_rgb24_bgr_swaps():
    data = bytes([10, 20, 30])  # 1 px bgr8 -> rgb (30,20,10)
    out = image_to_rgb24(1, 1, "bgr8", data)
    assert out == bytes([30, 20, 10])


def test_image_to_rgb24_mono_expands():
    out = image_to_rgb24(1, 1, "mono8", bytes([77]))
    assert out == bytes([77, 77, 77])


def test_image_to_rgb24_rejects_unknown():
    try:
        image_to_rgb24(1, 1, "yuv422", bytes([0, 0]))
        assert False, "should have raised"
    except ValueError:
        pass


def test_ffmpeg_cmd_is_h264_yuv420_faststart():
    cmd = _ffmpeg_cmd("/bin/ffmpeg", 848, 480, 15, "/tmp/x.mp4")
    assert "libx264" in cmd
    assert "yuv420p" in cmd
    assert "+faststart" in cmd
    assert "848x480" in cmd


def _run():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} tests passed")


if __name__ == "__main__":
    _run()
