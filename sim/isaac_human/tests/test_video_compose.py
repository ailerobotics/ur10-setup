# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the video composer's PURE parts (video_compose.py): the
timestamp alignment / frame-hold, the label formatting, the even-dimension
sizing, the size-cap bitrate choice, the distance-vs-time trace mapping and the
self-contained HTML page. No Isaac, no encoder binary, no Pillow needed."""
import pytest

import video_compose as vc


# --- even-dimension sizing (H.264 / yuv420p) -------------------------------- #
def test_even_dim_rounds_down_and_floors_at_two():
    assert vc.even_dim(1280) == 1280
    assert vc.even_dim(721) == 720
    assert vc.even_dim(1) == 2
    assert vc.even_dim(0) == 2
    assert vc.even_dim(3) == 2


def test_even_size():
    assert vc.even_size(1281, 721) == (1280, 720)
    assert vc.even_size(960, 540) == (960, 540)


# --- size-cap bitrate choice ------------------------------------------------ #
def test_choose_bitrate_scales_with_target_and_duration():
    # 6 MB over 77 s -> a few hundred kbps, below the raw 8*bytes/s figure.
    kbps = vc.choose_bitrate_kbps(6_000_000, 77.0)
    raw = 6_000_000 * 8 / 1000 / 77.0
    assert 0 < kbps < raw            # headroom + overhead discount applied
    # A tighter cap yields a smaller bitrate.
    assert vc.choose_bitrate_kbps(3_000_000, 77.0) < kbps
    # A longer clip at the same size yields a smaller bitrate.
    assert vc.choose_bitrate_kbps(6_000_000, 150.0) < kbps


def test_choose_bitrate_floor_for_tiny_clip():
    # A very small target over a long clip is clamped up to the floor.
    assert vc.choose_bitrate_kbps(10_000, 90.0, floor_kbps=150) == 150


def test_choose_bitrate_rejects_nonpositive_duration():
    with pytest.raises(ValueError):
        vc.choose_bitrate_kbps(6_000_000, 0.0)


# --- timestamp alignment / frame hold --------------------------------------- #
def test_hold_index_holds_last_frame():
    times = [10.0, 10.2, 10.4, 10.6]
    assert vc.hold_index(times, 10.0) == 0
    assert vc.hold_index(times, 10.3) == 1     # hold the 10.2 frame
    assert vc.hold_index(times, 10.45) == 2
    assert vc.hold_index(times, 99.0) == 3     # hold the last frame forever


def test_hold_index_before_first_shows_first():
    assert vc.hold_index([10.0, 10.2], 9.0) == 0


def test_hold_index_empty_raises():
    with pytest.raises(ValueError):
        vc.hold_index([], 1.0)


def test_align_series_slower_secondary_holds():
    # primary at 5 Hz, secondary at 2.5 Hz -> each secondary frame repeats ~twice.
    primary = [0.0, 0.2, 0.4, 0.6, 0.8]
    secondary = [0.0, 0.4, 0.8]
    assert vc.align_series(primary, secondary) == [0, 0, 1, 1, 2]


def test_build_output_times_spacing():
    ts = vc.build_output_times(100.0, 101.0, 10.0)
    assert ts[0] == 100.0
    assert ts[-1] == pytest.approx(101.0)
    assert len(ts) == 11
    # even spacing
    assert ts[1] - ts[0] == pytest.approx(0.1)


def test_build_output_times_degenerate_window():
    assert vc.build_output_times(5.0, 5.0, 10.0) == [5.0]


def test_pick_window_from_start_and_capped():
    # available 40 s, want 75 -> take the whole 40 s.
    assert vc.pick_window(1000.0, 1040.0, 75.0, 90.0) == (1000.0, 1040.0)
    # available 200 s, want 75, max 90 -> take 75 s from the start.
    assert vc.pick_window(0.0, 200.0, 75.0, 90.0) == (0.0, 75.0)
    # target above max -> clamp to max.
    assert vc.pick_window(0.0, 200.0, 120.0, 90.0) == (0.0, 90.0)


def test_pick_window_bad_order_raises():
    with pytest.raises(ValueError):
        vc.pick_window(10.0, 5.0, 60.0, 90.0)


# --- label formatting ------------------------------------------------------- #
def test_format_distance():
    assert vc.format_distance(2.34) == "2.34 m"
    assert vc.format_distance(2.006) == "2.01 m"
    assert vc.format_distance(0.0) == "0.00 m"
    assert vc.format_distance(None) == "—"


def test_format_zone():
    assert vc.format_zone("STOP") == "STOP"
    assert vc.format_zone(None) == "—"
    assert vc.format_zone("") == "—"


def test_zone_rgb_distinct_and_greyed_unknown():
    assert vc.zone_rgb("CLEAR") != vc.zone_rgb("STOP")
    assert vc.zone_rgb("SLOW") != vc.zone_rgb("CLEAR")
    assert vc.zone_rgb(None) == vc.UNKNOWN_ZONE_RGB
    assert vc.zone_rgb("BOGUS") == vc.UNKNOWN_ZONE_RGB


# --- distance-vs-time trace mapping ----------------------------------------- #
def test_trace_xy_maps_corners():
    box = vc.Box(100, 200, 300, 260)     # w=200, h=60
    # t at window start, d=0 -> bottom-left corner.
    x, y = vc.trace_xy(0.0, 0.0, 0.0, 10.0, 5.0, box)
    assert (x, y) == pytest.approx((100.0, 260.0))
    # t at window end, d=d_max -> top-right corner.
    x, y = vc.trace_xy(10.0, 5.0, 0.0, 10.0, 5.0, box)
    assert (x, y) == pytest.approx((300.0, 200.0))


def test_trace_xy_clamps_out_of_range():
    box = vc.Box(0, 0, 100, 100)
    # distance above d_max clamps to the top; time past the window clamps to the right.
    x, y = vc.trace_xy(99.0, 999.0, 0.0, 10.0, 5.0, box)
    assert x == pytest.approx(100.0)
    assert y == pytest.approx(0.0)


def test_ring_y_orders_by_distance():
    box = vc.Box(0, 0, 100, 100)
    y2 = vc.ring_y(2.0, 5.0, box)
    y225 = vc.ring_y(2.25, 5.0, box)
    # larger distance -> higher up -> smaller y (image y is down).
    assert y225 < y2
    assert vc.ring_y(0.0, 5.0, box) == pytest.approx(100.0)


def test_ring_y_bad_dmax_raises():
    with pytest.raises(ValueError):
        vc.ring_y(2.0, 0.0, vc.Box(0, 0, 10, 10))


# --- self-contained HTML page ----------------------------------------------- #
def test_build_video_html_is_self_contained_and_phone_first():
    page = vc.build_video_html("VVV", "PPP", ["line one", "line two", "line 3"],
                               "My Title")
    # embeds the video + poster as base64 data URIs, nothing from the network.
    assert "data:video/mp4;base64,VVV" in page
    assert "data:image/png;base64,PPP" in page
    assert "http://" not in page and "https://" not in page
    assert "src=\"//" not in page
    # phone-first video element attributes the brief requires.
    assert "<video" in page and "playsinline" in page and "controls" in page
    assert "muted" in page
    assert "width=device-width" in page
    # all three caption lines present.
    for cap in ("line one", "line two", "line 3"):
        assert cap in page


def test_build_video_html_escapes_captions():
    page = vc.build_video_html("V", "P", ["<script>alert(1)</script>"], "T & U")
    assert "<script>alert(1)</script>" not in page
    assert "&lt;script&gt;" in page
    assert "T &amp; U" in page
