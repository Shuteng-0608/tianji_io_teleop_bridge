import math
from types import SimpleNamespace as Obj

import cv2
import numpy as np
import pytest

from force_hud_core import AXES, HudModel, RingMapping, render_hud


@pytest.mark.parametrize("mode", ["ring", "bars"])
@pytest.mark.parametrize("side,hidden", [("right", "left"), ("left", "right")])
def test_single_arm_panel_ignores_hidden_arm_and_preserves_model(mode, side, hidden):
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    image = render_hud(model, 0, display_mode=mode, display_arms=side)
    assert image.shape == (720, 1280, 3)
    before = model.arms[side].filtered.copy()
    model.arms[hidden].valid = False
    model.arms[hidden].message = "HIDDEN ARM INVALID"
    np.testing.assert_array_equal(
        image, render_hud(model, 0, display_mode=mode, display_arms=side))
    np.testing.assert_array_equal(model.arms[side].filtered, before)
    model.arms[side].valid = False
    assert not np.array_equal(image, render_hud(model, 0, display_mode=mode, display_arms=side))


def test_invalid_display_arm_rejected():
    with pytest.raises(ValueError, match="display_arms"):
        render_hud(HudModel(), 0, display_arms="none")


def report(stamp=10.0, left=1.0, right=2.0, level=1):
    statuses = []
    for side, value in (("left", left), ("right", right)):
        fields = {"schema_version": "1", "convention_confirmed": "false",
                  "scaled_condition": "10", "relative_residual": "0.1"}
        fields.update({"raw_" + key: str(value) for key in AXES})
        statuses.append(Obj(name=side, level=level, message="unverified",
                            values=[Obj(key=k, value=v) for k, v in fields.items()]))
    ns = round(stamp * 1e9)
    return Obj(header=Obj(stamp=Obj(sec=ns // 10**9, nanosec=ns % 10**9)), status=statuses)


def test_dual_arm_mapping_and_time_based_filter():
    model = HudModel(smoothing_sec=0.1)
    model.ingest(report(), 10 * 10**9, 1.0)
    np.testing.assert_allclose(model.arms["left"].filtered, 1.0)
    np.testing.assert_allclose(model.arms["right"].filtered, 2.0)
    model.ingest(report(10.1, left=3.0), 10_100_000_000, 1.1)
    np.testing.assert_allclose(model.arms["left"].filtered, 1 + 2 * (1 - math.exp(-1)))
    assert not model.arms["left"].confirmed


def test_repeated_sample_does_not_refilter_or_refresh_source():
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    model.ingest(report(left=9), 10 * 10**9, 1.1)
    np.testing.assert_allclose(model.arms["left"].filtered, 1.0)
    # Even if the ROS clock stops and reports keep arriving, monotonic timeout applies.
    model.refresh(10 * 10**9, 1.3)
    assert not model.arms["left"].valid


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_nonfinite_only_invalidates_affected_arm(value):
    model = HudModel()
    model.ingest(report(left=value), 10 * 10**9, 1.0)
    assert not model.arms["left"].valid
    assert model.arms["right"].valid


def test_stale_failure_and_recovery_reset_filter():
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    model.refresh(10_300_000_000, 1.3)
    assert not model.arms["right"].valid
    model.ingest(report(10.31, right=8), 10_310_000_000, 1.31)
    np.testing.assert_allclose(model.arms["right"].filtered, 8)
    model.ingest(report(10.32, level=2), 10_320_000_000, 1.32)
    assert not model.arms["right"].valid


@pytest.mark.parametrize("stamp", [0.0, 9.0, 11.0])
def test_rejects_bad_source_stamps(stamp):
    model = HudModel()
    model.ingest(report(stamp), 10 * 10**9, 1.0)
    assert not model.arms["left"].valid


def test_backwards_stamp_is_visible_and_recovers():
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    model.ingest(report(9.99), 10 * 10**9, 1.01)
    assert "backwards" in model.arms["left"].message
    model.ingest(report(10.02), 10_020_000_000, 1.02)
    assert model.arms["left"].valid


def test_duplicate_missing_and_malformed_statuses():
    for mutation in (lambda x: x.status.pop(),
                     lambda x: setattr(x.status[1], "name", "left"),
                     lambda x: x.status[0].values.clear()):
        model = HudModel()
        data = report()
        mutation(data)
        model.ingest(data, 10 * 10**9, 1.0)
        assert not model.arms["left"].valid


@pytest.mark.parametrize("mode", ["ring", "bars"])
def test_render_and_jpeg_decode_valid_and_stale(mode):
    model = HudModel()
    model.ingest(report(left=30, right=-2), 10 * 10**9, 1.0)
    valid = render_hud(model, 0.0, display_mode=mode)
    ok, encoded = cv2.imencode(".jpg", valid, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    assert decoded.shape == (720, 1280, 3)
    model.refresh(11 * 10**9, 2.0)
    stale = render_hud(model, 1.0, display_mode=mode)
    assert np.count_nonzero(valid != stale) > 1000


def test_invalid_parameters():
    with pytest.raises(ValueError):
        HudModel(smoothing_sec=-1)
    with pytest.raises(ValueError):
        render_hud(HudModel(), 0, force_scale=0)


def test_humble_diagnostic_byte_levels():
    model = HudModel()
    model.ingest(report(level=b"\x01"), 10 * 10**9, 1.0)
    assert model.arms["left"].valid
    model.ingest(report(10.01, level=b"\x02"), 10_010_000_000, 1.01)
    assert not model.arms["left"].valid


def test_default_is_ring_and_unconfirmed_direction_does_not_affect_picture():
    models = [HudModel(), HudModel()]
    models[0].ingest(report(left=2.0, right=3.0), 10 * 10**9, 1.0)
    models[1].ingest(report(left=-2.0, right=-3.0), 10 * 10**9, 1.0)
    np.testing.assert_array_equal(render_hud(models[0], 0), render_hud(models[1], 0))
    np.testing.assert_array_equal(render_hud(models[0], 0),
                                  render_hud(models[0], 0, display_mode="ring"))
    assert not np.array_equal(render_hud(models[0], 0),
                              render_hud(models[0], 0, display_mode="bars"))


def test_mapping_requires_both_confirmations_and_matching_frame():
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    view = model.arms["left"]
    mapping = RingMapping(confirmed=True, frame="left_base")
    assert mapping.project(view) is None
    view.confirmed = True
    view.frame = "another_frame"
    assert mapping.project(view) is None
    view.frame = "left_base"
    np.testing.assert_allclose(mapping.project(view), [1, 1, 1])
    assert RingMapping(frame="left_base").project(view) is None
    view.valid = False
    assert mapping.project(view) is None


@pytest.mark.parametrize("kwargs", [
    {"confirmed": True}, {"right": (0, 0, 0)}, {"up": (1, 0, 0)},
    {"axial": (0, 0, math.nan)}, {"right": (2, 0, 0)}, {"up": (0, 1)},
])
def test_mapping_rejects_invalid_axes_and_missing_frame(kwargs):
    with pytest.raises(ValueError):
        RingMapping(**kwargs)


def test_projected_ring_arrow_screen_sign_and_stale_removal(monkeypatch):
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    mappings = {}
    for side in model.arms:
        view = model.arms[side]
        view.confirmed, view.frame = True, side + "_base"
        view.filtered = np.array([2, 3, -4, 0, 0, 0], dtype=float)
        mappings[side] = RingMapping(confirmed=True, frame=view.frame)
    calls = []
    original = cv2.arrowedLine

    def capture(image, start, end, *args, **kwargs):
        calls.append((start, end))
        return original(image, start, end, *args, **kwargs)

    monkeypatch.setattr(cv2, "arrowedLine", capture)
    render_hud(model, 0, mappings=mappings, show_torque=True)
    assert len(calls) == 2
    for start, end in calls:
        assert end[0] > start[0] and end[1] < start[1]  # positive up draws UP
        assert np.hypot(end[0] - start[0], end[1] - start[1]) < 130
    calls.clear()
    model.refresh(11 * 10**9, 2)
    render_hud(model, 1, mappings=mappings)
    assert not calls


def test_distinct_left_right_axis_mappings():
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    left, right = model.arms.values()
    left.confirmed = right.confirmed = True
    left.frame = "left"
    right.frame = "right"
    left.filtered = right.filtered = np.array([2, 3, -4, 0, 0, 0], dtype=float)
    left_map = RingMapping(True, "left")
    right_map = RingMapping(True, "right", right=(-1, 0, 0), up=(0, 0, 1), axial=(0, 1, 0))
    np.testing.assert_allclose(left_map.project(left), [2, 3, -4])
    np.testing.assert_allclose(right_map.project(right), [-2, -4, 3])


def test_trend_warmup_repeated_frames_and_reset():
    model = HudModel(smoothing_sec=0, trend_window_sec=0.8)
    for i in range(9):
        model.ingest(report(10 + i * 0.1, left=i), 10_000_000_000 + i * 100_000_000, 1 + i * 0.1)
    delta = model.arms["left"].trend_delta
    assert delta is not None and delta > 0
    model.ingest(report(10.8, left=8), 10_800_000_000, 1.8)
    assert model.arms["left"].trend_delta == delta
    model.refresh(11_200_000_000, 2.2)
    assert model.arms["left"].trend_delta is None
    model.ingest(report(11.3, left=100), 11_300_000_000, 2.3)
    assert model.arms["left"].trend_delta is None


def test_render_mode_rejects_typo():
    with pytest.raises(ValueError):
        render_hud(HudModel(), 0, display_mode="rings")


def test_preview_projects_without_claiming_confirmation_and_keeps_frame_gate():
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    view = model.arms["left"]
    view.frame = "left_sdk_base"
    view.filtered = np.array([2, -3, 4, 0, 0, 0], dtype=float)
    mapping = RingMapping(frame="left_sdk_base", preview=True)
    np.testing.assert_allclose(mapping.project(view), [2, -3, 4])
    assert not view.confirmed and not mapping.confirmed
    assert RingMapping(frame="right_sdk_base", preview=True).project(view) is None
    assert RingMapping(frame="left_sdk_base").project(view) is None
    model.refresh(11 * 10**9, 2.0)
    assert mapping.project(model.arms["left"]) is None


def test_preview_requires_frame_and_rejects_invalid_estimates():
    with pytest.raises(ValueError):
        RingMapping(preview=True)
    model = HudModel()
    model.ingest(report(level=2), 10 * 10**9, 1.0)
    view = model.arms["left"]
    view.frame = "left_sdk_base"
    assert RingMapping(frame=view.frame, preview=True).project(view) is None


def test_preview_label_and_units_are_explicit(monkeypatch):
    model = HudModel()
    model.ingest(report(), 10 * 10**9, 1.0)
    mappings = {}
    for side, view in model.arms.items():
        view.frame = side + "_sdk_base"
        mappings[side] = RingMapping(frame=view.frame, preview=True)
    labels = []
    original = cv2.putText

    def capture(canvas, value, *args, **kwargs):
        labels.append(value)
        return original(canvas, value, *args, **kwargs)

    monkeypatch.setattr(cv2, "putText", capture)
    frame = render_hud(model, 0, mappings=mappings)
    assert frame.shape == (720, 1280, 3)
    assert any("UNCALIBRATED PREVIEW" in label for label in labels)
    assert any("Axial" in label and "N*" in label for label in labels)
