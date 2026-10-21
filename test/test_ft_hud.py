import math
from pathlib import Path
from types import SimpleNamespace as Obj

import cv2
import numpy as np
import pytest

from force_hud_core import FT_AXES, HudModel, RingMapping, ft_sensor_report, render_hud
from force_hud_ft import FtDirectionInput, RIGHT_JOINTS, right_joint_degrees


def message(stamp=10., values=(1., -2., 3., .01, -.02, .03)):
    ns = round(stamp * 1e9)
    return Obj(header=Obj(stamp=Obj(sec=ns // 10**9, nanosec=ns % 10**9),
                          frame_id="right_ft_measurement"),
               name=list(FT_AXES), effort=list(values), position=[], velocity=[])


def ingest(model, msg, ros=10_000_000_000, steady=1.):
    model.ingest(ft_sensor_report(msg), ros, steady)


def test_ft_names_map_without_rescaling_compensation_or_tare():
    model = HudModel(source_kind="right_ft_sensor")
    msg = message()
    msg.name.reverse()
    msg.effort.reverse()
    ingest(model, msg)
    right = model.arms["right"]
    assert right.valid and right.confirmed
    assert right.reference_point == "sensor"
    assert right.frame == "right_ft_measurement"
    assert math.isnan(right.condition) and math.isnan(right.residual)
    np.testing.assert_array_equal(right.raw, [1, -2, 3, .01, -.02, .03])
    np.testing.assert_array_equal(right.filtered, right.raw)
    assert not model.arms["left"].valid
    assert model.arms["left"].raw is None


@pytest.mark.parametrize("fault", ["missing", "duplicate", "extra", "length", "nan", "inf", "frame", "empty_frame"])
def test_rejects_malformed_ft_input(fault):
    msg = message()
    if fault == "missing":
        msg.name[-1] = "Joint7_R"
    elif fault == "duplicate":
        msg.name[-1] = msg.name[0]
    elif fault == "extra":
        msg.name.append("extra")
        msg.effort.append(0)
    elif fault == "length":
        msg.effort.pop()
    elif fault == "nan":
        msg.effort[0] = math.nan
    elif fault == "inf":
        msg.effort[-1] = math.inf
    else:
        msg.header.frame_id = "" if fault == "empty_frame" else "right_sdk_base"
    model = HudModel(source_kind="right_ft_sensor")
    ingest(model, msg)
    assert not model.arms["right"].valid
    assert model.arms["right"].filtered is None


@pytest.mark.parametrize("stamp", [0., 9., 11.])
def test_ft_rejects_invalid_source_time(stamp):
    model = HudModel(source_kind="right_ft_sensor")
    ingest(model, message(stamp))
    assert not model.arms["right"].valid


def test_duplicate_ft_stamp_cannot_keep_stale_display_alive_and_recovers_cleanly():
    model = HudModel(source_kind="right_ft_sensor")
    ingest(model, message())
    ingest(model, message(values=[9] * 6), steady=1.1)
    np.testing.assert_array_equal(model.arms["right"].filtered, [1, -2, 3, .01, -.02, .03])
    model.refresh(10_000_000_000, 1.3)
    assert not model.arms["right"].valid
    ingest(model, message(10.31, [4] * 6), 10_310_000_000, 1.31)
    np.testing.assert_array_equal(model.arms["right"].filtered, [4] * 6)


def test_old_base_frame_mapping_is_not_applied_to_sensor_measurement():
    model = HudModel(source_kind="right_ft_sensor")
    ingest(model, message())
    assert RingMapping(frame="right_sdk_base", preview=True).project(model.arms["right"]) is None


@pytest.mark.parametrize("mode", ["ring", "bars"])
def test_sensor_labels_and_no_left_sensor_claim(monkeypatch, mode):
    model = HudModel(source_kind="right_ft_sensor")
    ingest(model, message())
    labels = []
    original = cv2.putText
    def text(img, label, *args, **kwargs):
        labels.append(label)
        return original(img, label, *args, **kwargs)
    monkeypatch.setattr(cv2, "putText", text)
    image = render_hud(model, 0, display_mode=mode, display_arms="right")
    assert image.shape == (720, 1280, 3)
    assert any("FT SENSOR" in s for s in labels)
    assert any("Point: sensor" in s for s in labels)
    assert not any("Point: flange" in s or "ESTIMATED" in s or "not a force sensor" in s for s in labels)
    if mode == "bars":
        assert {"Tx", "Ty", "Tz"}.issubset(labels)
        assert not any(s.startswith("cond ") for s in labels)


def joints(stamp=10., degrees=(0.,) * 7):
    return Obj(header=message(stamp).header, name=list(RIGHT_JOINTS),
               position=np.deg2rad(degrees).tolist())


def insertion_mapping():
    import yaml
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                            "config/force_hud_insertion_preview.yaml").read_text())
    p = config["tianji_force_hud"]["ros__parameters"]
    return RingMapping(frame=p["right_direction_frame"], preview=p["right_direction_preview"],
                       semantics=p["ring_semantics"], right=p["right_screen_right_axis"],
                       up=p["right_screen_up_axis"], axial=p["right_axial_axis"],
                       lateral_deadzone=p["lateral_display_deadzone"],
                       axial_deadzone=p["axial_display_deadzone"])


@pytest.mark.parametrize("order", [("ft", "joints"), ("joints", "ft")])
def test_same_stamp_rotation_and_sensor_origin_preserved(order):
    r = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    seen = []
    def fk(q):
        seen.append(q)
        return r
    adapter = FtDirectionInput(fk, np.eye(3), "right_ft_measurement")
    q = joints(degrees=[1, 2, 3, 4, 5, 6, 7])
    q.name.reverse()
    q.position.reverse()
    ft = message()
    inputs = {"ft": ft, "joints": q}
    assert adapter.feed(order[0], inputs[order[0]]) is None
    report = adapter.feed(order[1], inputs[order[1]])
    model = HudModel(smoothing_sec=0, source_kind="right_ft_sensor")
    model.ingest(report, 10_000_000_000, 1.)
    v = model.arms["right"]
    assert v.valid and v.reference_point == "sensor" and v.frame == "right_sdk_base"
    assert v.measurement_frame == "right_ft_measurement" and v.joint_stamp_ns == model.stamp_ns
    np.testing.assert_allclose(seen[0], [1, 2, 3, 4, 5, 6, 7])
    np.testing.assert_allclose(v.raw, [2, 1, 3, .02, .01, .03])
    np.testing.assert_allclose(v.measurement_raw, ft.effort)
    assert ft.header.frame_id == "right_ft_measurement"  # Source record never mutated.


def test_nearby_or_missing_pose_never_reuses_last_pose_and_cache_is_bounded():
    adapter = FtDirectionInput(lambda q: np.eye(3), np.eye(3), "right_ft_measurement", capacity=4)
    assert adapter.feed("joints", joints(10.)) is None
    assert adapter.feed("ft", message(10.001)) is None
    for n in range(20):
        assert adapter.feed("ft", message(10.01 + n * .001)) is None
    assert len(adapter.pending["ft"]) == 4
    assert adapter.feed("joints", joints(10.001)) is None  # Evicted, no stale interpolation.
    assert adapter.feed("joints", joints(10.029)) is not None


@pytest.mark.parametrize("fault", ["missing", "duplicate", "length", "nan", "fk", "reflection"])
def test_pose_errors_remove_force_and_arrow(fault):
    q = joints()
    def fk(_):
        if fault == "fk":
            raise RuntimeError("FK failed")
        return np.diag([-1., 1., 1.]) if fault == "reflection" else np.eye(3)
    if fault == "missing":
        q.name[-1] = "unknown"
    elif fault == "duplicate":
        q.name[-1] = q.name[0]
    elif fault == "length":
        q.position.pop()
    elif fault == "nan":
        q.position[-1] = math.nan
    adapter = FtDirectionInput(fk, np.eye(3), "right_ft_measurement")
    adapter.feed("ft", message())
    model = HudModel(source_kind="right_ft_sensor")
    model.ingest(adapter.feed("joints", q), 10_000_000_000, 1.)
    assert not model.arms["right"].valid
    assert insertion_mapping().project(model.arms["right"]) is None


def test_real_local_fk_keeps_fixed_hole_force_direction_when_wrist_moves(monkeypatch):
    import ctypes
    import socket
    from ft_calibration_core import Model
    from ft_calibration_sdk import Kinematics
    original = ctypes.CDLL
    def only_math(path, *args, **kwargs):
        assert Path(path).name == "libKine.so"
        return original(path, *args, **kwargs)
    def no_network(*args, **kwargs):
        raise AssertionError("no robot/network access")
    monkeypatch.setattr(ctypes, "CDLL", only_math)
    monkeypatch.setattr(socket, "socket", no_network)
    sdk_root = Path("/home/stw/TJ_FX_ROBOT_CONTRL_SDK")
    kine = Kinematics(None, Model.load(sdk_root / "DEMO_C++/ccs_m6_40.MvKDCfg"),
                      library_path=sdk_root / "DEMO_C++/libKine.so")
    mapping = insertion_mapping()
    sensor_mount = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    adapter = FtDirectionInput(kine.rotation, sensor_mount, "right_ft_measurement")
    model = HudModel(smoothing_sec=0.08, source_kind="right_ft_sensor")
    target_task_force = np.array([-4., 2., 8.])
    base_force = mapping.rotation().T @ target_task_force
    for index, q in enumerate(([0] * 7, [0, 0, 0, 0, 0, -40, -50],
                               [-110, -90, 90, -110, 0, 20, 30])):
        stamp = 10. + index * .01
        r = kine.rotation(q) @ sensor_mount
        ft = message(stamp, [*(r.T @ base_force), .01, -.02, .03])
        adapter.feed("ft", ft)
        report = adapter.feed("joints", joints(stamp, q))
        model.ingest(report, round(stamp * 1e9), 1. + index * .01)
        np.testing.assert_allclose(mapping.project(model.arms["right"]), target_task_force, atol=1e-9)
        np.testing.assert_allclose(mapping.arrow(mapping.project(model.arms["right"])), [4, -2], atol=1e-9)
    model.refresh(10_300_000_000, 1.3)
    assert not model.arms["right"].valid
