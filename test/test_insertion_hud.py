import math

import cv2
import numpy as np
import pytest

from force_hud_core import ArmView, ForceBaseline, HudModel, RingMapping, render_hud


def view(force=(1., 2., 3.), frame="base"):
    raw = np.array([*force, 0., 0., 0.])
    return ArmView(valid=True, frame=frame, raw=raw, filtered=raw.copy())


def mapping(**kwargs):
    return RingMapping(frame="base", preview=True, semantics="contact_side", **kwargs)


@pytest.mark.parametrize("force, expected", [
    ((-3, 0, 0), (3, 0)), ((3, 0, 0), (-3, 0)),
    ((0, -3, 0), (0, 3)), ((0, 3, 0), (0, -3)),
])
def test_single_wall_contact_side(force, expected):
    m = mapping()
    np.testing.assert_allclose(m.arrow(m.project(view(force))), expected)


def test_fixed_alignment_then_yaw_and_polarity_are_separate():
    # Cyclic XYZ permutation is a proper fixed rotation; yaw preserves axial force.
    m = mapping(right=(0, 1, 0), up=(0, 0, 1), axial=(1, 0, 0), yaw_deg=90)
    np.testing.assert_allclose(m.project(view((1, 2, 3))), [3, -2, 1], atol=1e-12)
    assert np.linalg.det(m.rotation()) == pytest.approx(1)
    np.testing.assert_allclose(mapping(force_sign=-1).project(view()), [-1, -2, -3])


@pytest.mark.parametrize("kwargs", [
    {"right": (-1, 0, 0)}, {"yaw_deg": math.nan}, {"force_sign": 0},
    {"lateral_deadzone": -1}, {"axial_deadzone": math.inf},
])
def test_invalid_task_rotation_or_settings_rejected(kwargs):
    with pytest.raises(ValueError):
        mapping(**kwargs)


def test_deadzone_hides_arrow_not_measured_components():
    m = mapping(lateral_deadzone=0.5)
    projected = m.project(view((0.3, 0.4, 4)))
    np.testing.assert_allclose(projected, [0.3, 0.4, 4])
    np.testing.assert_array_equal(m.arrow(projected), [0, 0])
    np.testing.assert_allclose(m.arrow([0.6, 0, 4]), [-0.6, 0])


def populate(baseline, force=(1., 2., 3.)):
    v = view(force)
    for i in range(101):
        stamp = 10_000_000_000 + i * 10_000_000
        baseline.observe(v, stamp)
    return v, stamp


def test_manual_zero_preserves_raw_filter_and_other_arm():
    left, right = ForceBaseline(), ForceBaseline()
    v, stamp = populate(left)
    left.capture(v, stamp)
    left.attach(v)
    np.testing.assert_allclose(mapping().project(v), 0)
    np.testing.assert_array_equal(v.raw[:3], [1, 2, 3])
    np.testing.assert_array_equal(v.filtered[:3], [1, 2, 3])
    assert right.bias is None
    left.clear()
    left.attach(v)
    np.testing.assert_allclose(mapping().project(v), [1, 2, 3])


def test_zero_rejects_invalid_short_duplicate_unstable_window():
    b = ForceBaseline()
    v = view()
    for _ in range(200):
        b.observe(v, 10_000_000_000)
    with pytest.raises(ValueError):
        b.capture(v, 10_000_000_000)
    for i in range(101):
        v = view((i % 2 * 5., 0., 0.))
        stamp = 11_000_000_000 + i * 10_000_000
        b.observe(v, stamp)
    with pytest.raises(ValueError):
        b.capture(v, stamp)
    assert b.bias is None
    v.valid = False
    with pytest.raises(ValueError):
        b.capture(v, stamp)


def test_zero_clears_on_frame_change_or_source_clock_rewind():
    b = ForceBaseline()
    v, stamp = populate(b)
    b.capture(v, stamp)
    b.observe(view(frame="other"), stamp + 10_000_000)
    assert b.bias is None
    v, stamp = populate(b)
    b.capture(v, stamp)
    b.observe(v, stamp - 1)
    assert b.bias is None


def test_contact_side_image_direction_and_invalid_removal(monkeypatch):
    model = HudModel()
    model.arms = {side: view((0, -4, 8)) for side in ("left", "right")}
    mappings = {side: mapping() for side in model.arms}
    arrows, labels = [], []
    original_arrow, original_text = cv2.arrowedLine, cv2.putText

    def arrow(img, start, end, *args, **kwargs):
        arrows.append((start, end))
        return original_arrow(img, start, end, *args, **kwargs)

    def text(img, value, *args, **kwargs):
        labels.append(value)
        return original_text(img, value, *args, **kwargs)

    monkeypatch.setattr(cv2, "arrowedLine", arrow)
    monkeypatch.setattr(cv2, "putText", text)
    img = render_hud(model, 0, mappings=mappings)
    assert img.shape == (720, 1280, 3)
    assert len(arrows) == 2
    assert all(end[1] < start[1] for start, end in arrows)
    assert any("SIDE ESTIMATE / UNCALIBRATED" in label for label in labels)
    assert any("Axial +8.00" in label for label in labels)
    arrows.clear()
    model.invalidate("STALE")
    render_hud(model, 1, mappings=mappings)
    assert not arrows
