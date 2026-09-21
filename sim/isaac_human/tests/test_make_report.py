# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the report's see-the-person / score / bottom-line logic,
which must be DERIVED from measured detection numbers (zero, low, full), never
baked in. Also covers analysis_core.detection_scores, the summary that feeds the
score phrase."""
import analysis_core as ac
import make_report as mr


# --------------------------------------------------------------------------- #
# see_person_line: derived answer for zero / low / full / no-opportunity logs.
# --------------------------------------------------------------------------- #
def test_see_person_full_detection_says_yes_with_measured_score():
    dr = {"wrist": {"opportunities": 100, "hits": 100, "rate": 1.0}}
    line = mr.see_person_line(dr, {"n": 100, "median": 0.98, "min": 0.71})
    assert line.startswith("- **Does it see the person?**")
    assert "**Yes**" in line
    assert "median detection score 0.98" in line
    # never the old baked-in phrasing
    assert "score ~1.00" not in line


def test_see_person_low_rate_says_partially_with_percentage():
    dr = {"wrist": {"opportunities": 100, "hits": 40, "rate": 0.4}}
    line = mr.see_person_line(dr, {"n": 40, "median": 0.9, "min": 0.6})
    assert "**Partially**" in line
    assert "**Yes**" not in line
    assert "40%" in line


def test_see_person_zero_hits_says_no():
    dr = {"wrist": {"opportunities": 100, "hits": 0, "rate": 0.0}}
    line = mr.see_person_line(dr, {"n": 0, "median": float("nan"), "min": float("nan")})
    assert "**No**" in line
    assert "**Yes**" not in line
    # no fabricated score when there is no scored hit
    assert "median detection score" not in line


def test_see_person_no_opportunity_says_no_data():
    line = mr.see_person_line({}, {})
    assert "**No data**" in line
    assert "**Yes**" not in line


def test_score_phrase_omitted_when_missing_but_present_when_measured():
    assert mr._score_phrase({}) == ""
    assert mr._score_phrase({"n": 0, "median": 0.9}) == ""
    assert mr._score_phrase({"n": 5, "median": float("nan")}) == ""
    assert mr._score_phrase({"n": 5, "median": 0.87}) == " (median detection score 0.87)"


# --------------------------------------------------------------------------- #
# bottom_line: honest about detection + whether a STOP crossing was exercised.
# --------------------------------------------------------------------------- #
def test_bottom_line_distance_bias_when_stop_would_fire_inside():
    dr = {"wrist": {"opportunities": 10, "hits": 10, "rate": 1.0}}
    bl = mr.bottom_line({"person_inside_ring_by_m_nearest": 0.9}, dr)
    assert "engages LATE" in bl


def test_bottom_line_no_crossing_when_detected_but_no_stop():
    dr = {"wrist": {"opportunities": 10, "hits": 10, "rate": 1.0}}
    bl = mr.bottom_line({}, dr)
    assert "no STOP-ring crossing" in bl
    assert "engages LATE" not in bl


def test_bottom_line_no_detection_draws_no_distance_conclusion():
    dr = {"wrist": {"opportunities": 10, "hits": 0, "rate": 0.0}}
    bl = mr.bottom_line({}, dr)
    assert "did not report the person" in bl
    assert "engages LATE" not in bl


# --------------------------------------------------------------------------- #
# End-to-end build_markdown: the rendered report reflects the data, not a bake.
# --------------------------------------------------------------------------- #
def _metrics(dr, score, verdict_ok=True):
    return {"verdict_ok": verdict_ok, "verdict": "x", "stop_boundary_m": 2.0,
            "detection_rates": dr, "detection_score": score,
            "missed_stops": [], "errors": {}, "latency": {}, "counts": {}}


def test_build_markdown_full_run_renders_yes_and_score():
    dr = {"wrist": {"opportunities": 50, "hits": 50, "rate": 1.0}}
    md = mr.build_markdown(_metrics(dr, {"n": 50, "median": 0.99, "min": 0.8}),
                           diag={}, per_cam_diag=None, run_id="r",
                           frame_names=[], has_signed_plot=False)
    assert "**Yes**" in md
    assert "median detection score 0.99" in md
    assert "score ~1.00" not in md


def test_build_markdown_zero_detection_does_not_claim_yes_or_fake_score():
    dr = {"wrist": {"opportunities": 50, "hits": 0, "rate": 0.0}}
    md = mr.build_markdown(_metrics(dr, {"n": 0, "median": float("nan")}, verdict_ok=False),
                           diag={}, per_cam_diag=None, run_id="r",
                           frame_names=[], has_signed_plot=False)
    assert "**No**" in md
    assert "score ~1.00" not in md
    assert "did not report the person" in md


# --------------------------------------------------------------------------- #
# analysis_core.detection_scores: the measured summary behind the score phrase.
# --------------------------------------------------------------------------- #
def _truth_present():
    recs = [{"t": i * 0.1, "present": True, "root": [0.1, -2.0, 0.0],
             "centre": [0.1, -2.0, 0.95], "dist_base": 2.0} for i in range(5)]
    return ac.Truth(recs)


def _all_in_fov(role, centre):
    return True


def test_detection_scores_summarizes_best_per_frame():
    truth = _truth_present()
    dets = [{"t": 0.0, "camera": "wrist", "n_persons": 1, "scores": [0.8, 0.95]},
            {"t": 0.1, "camera": "wrist", "n_persons": 1, "scores": [0.9]},
            {"t": 0.2, "camera": "wrist", "n_persons": 1, "scores": [0.7]}]
    s = ac.detection_scores(dets, truth, _all_in_fov)
    assert s["n"] == 3
    assert abs(s["median"] - 0.9) < 1e-9
    assert abs(s["min"] - 0.7) < 1e-9


def test_detection_scores_empty_when_no_scored_hits():
    truth = _truth_present()
    dets = [{"t": 0.0, "camera": "wrist", "n_persons": 0, "scores": []}]
    s = ac.detection_scores(dets, truth, _all_in_fov)
    assert s["n"] == 0
    assert s["median"] != s["median"]  # nan


def test_detection_scores_ignores_out_of_view_frames():
    truth = _truth_present()
    dets = [{"t": 0.0, "camera": "wrist", "n_persons": 1, "scores": [0.99]}]
    s = ac.detection_scores(dets, truth, lambda role, c: False)
    assert s["n"] == 0
