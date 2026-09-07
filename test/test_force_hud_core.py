import math
from types import SimpleNamespace as Obj

import cv2
import numpy as np
import pytest

from force_hud_core import AXES, HudModel, render_hud


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


def test_render_and_jpeg_decode_valid_and_stale():
    model = HudModel()
    model.ingest(report(left=30, right=-2), 10 * 10**9, 1.0)
    valid = render_hud(model, 0.0)
    ok, encoded = cv2.imencode(".jpg", valid, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    assert decoded.shape == (720, 1280, 3)
    model.refresh(11 * 10**9, 2.0)
    stale = render_hud(model, 1.0)
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
