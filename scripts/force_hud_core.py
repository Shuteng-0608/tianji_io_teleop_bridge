"""Pure, hardware-independent dual-arm HUD model and OpenCV renderer.

Uses the simulation HUD's operator-only panel concept, but deliberately does not
copy its task directions, contact thresholds or SAFE/JAM labels onto real data.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np


AXES = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")


@dataclass
class ArmView:
    valid: bool = False
    message: str = "WAITING for estimate"
    confirmed: bool = False
    frame: str = ""
    raw: object = None
    filtered: object = None
    condition: float = math.nan
    residual: float = math.nan


class HudModel:
    def __init__(self, timeout=0.2, smoothing_sec=0.08, future_tolerance=0.05):
        if (not all(math.isfinite(x) for x in (timeout, smoothing_sec, future_tolerance))
                or timeout <= 0 or smoothing_sec < 0 or future_tolerance < 0):
            raise ValueError("invalid HUD timing parameters")
        self.timeout = timeout
        self.smoothing_sec = smoothing_sec
        self.future_tolerance = future_tolerance
        self.arms = {side: ArmView() for side in ("left", "right")}
        self.stamp_ns = 0
        self.received_at = None
        self.sample_seen_at = None

    def invalidate(self, message):
        self.arms = {side: ArmView(message=message) for side in self.arms}

    def ingest(self, report, now_ros_ns, now_steady):
        stamp = report.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (now_ros_ns - stamp_ns) / 1e9
        self.received_at = now_steady
        if stamp_ns <= 0 or age < -self.future_tolerance or age > self.timeout:
            self.invalidate("STALE / invalid source timestamp")
            return
        if stamp_ns < self.stamp_ns:
            # Allow a later clean sample to recover after a ROS clock reset.
            self.stamp_ns = 0
            self.sample_seen_at = None
            self.invalidate("source clock moved backwards")
            return
        changed = stamp_ns != self.stamp_ns
        dt = (stamp_ns - self.stamp_ns) / 1e9 if self.stamp_ns else 0.0
        if changed:
            self.sample_seen_at = now_steady
        alpha = 1.0 if self.smoothing_sec == 0 else -math.expm1(-dt / self.smoothing_sec)
        statuses = {item.name: item for item in report.status}
        if len(report.status) != 2 or set(statuses) != {"left", "right"}:
            self.invalidate("invalid dual-arm estimate schema")
            return
        for side, status in statuses.items():
            old = self.arms[side]
            values = {item.key: item.value for item in status.values}
            view = ArmView(message=status.message)
            self.arms[side] = view
            try:
                if values.get("schema_version") != "1":
                    raise ValueError("unsupported estimate schema")
                view.confirmed = values.get("convention_confirmed") == "true"
                view.frame = values.get("wrench_frame", "")
                # diagnostic_msgs uses ROS 'byte', represented as bytes in Humble Python.
                level = (int.from_bytes(status.level, "little")
                         if isinstance(status.level, bytes) else int(status.level))
                if level >= 2:
                    continue
                view.raw = np.array([float(values["raw_" + key]) for key in AXES])
                view.condition = float(values["scaled_condition"])
                view.residual = float(values["relative_residual"])
                if not np.isfinite(view.raw).all() or not all(
                        math.isfinite(v) for v in (view.condition, view.residual)):
                    raise ValueError("non-finite estimate")
                if view.confirmed and not view.frame:
                    raise ValueError("confirmed convention needs a frame")
                view.valid = True
                same_basis = old.confirmed == view.confirmed and old.frame == view.frame
                if old.valid and same_basis and 0 <= dt <= self.timeout:
                    view.filtered = (old.filtered + alpha * (view.raw - old.filtered)
                                     if changed else old.filtered.copy())
                else:
                    view.filtered = view.raw.copy()
            except (KeyError, TypeError, ValueError, OverflowError) as error:
                self.arms[side] = ArmView(message="INVALID: " + str(error))
        self.stamp_ns = stamp_ns

    def refresh(self, now_ros_ns, now_steady):
        if self.received_at is None:
            return
        age = (now_ros_ns - self.stamp_ns) / 1e9
        if (self.sample_seen_at is None
                or now_steady - self.received_at > self.timeout
                or now_steady - self.sample_seen_at > self.timeout
                or age > self.timeout or age < -self.future_tolerance):
            self.invalidate("STALE - no fresh source sample")


def render_hud(model, source_age, width=1280, height=720, force_scale=20.0, torque_scale=2.0):
    """Display ranges control bar lengths, NOT risk or safety thresholds."""
    if (width < 640 or height < 360 or width > 1920 or height > 1080
            or not all(math.isfinite(v) and v > 0 for v in (force_scale, torque_scale))):
        raise ValueError("unsupported HUD size or display range")
    canvas = np.full((720, 1280, 3), (24, 19, 15), np.uint8)
    muted, bright = (176, 168, 155), (241, 237, 228)
    cyan, amber = (222, 201, 83), (75, 189, 244)

    def text(value, x, y, scale=0.6, color=bright, thickness=1):
        cv2.putText(canvas, str(value), (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, thickness, cv2.LINE_AA)

    text("DUAL ARM / FORCE FEEDBACK", 32, 43, 0.85, bright, 2)
    text("ESTIMATED  |  Operator aid only - not a force sensor or safety system", 32, 76,
         0.56, amber)
    age_text = f"{source_age * 1000:.0f} ms" if math.isfinite(source_age) else "--"
    text("SOURCE AGE " + age_text, 965, 43, 0.55, muted)
    for index, side in enumerate(("left", "right")):
        view = model.arms[side]
        x = 24 + index * 632
        color = cyan if view.valid and view.confirmed else amber
        cv2.rectangle(canvas, (x, 104), (x + 608, 636), (44, 37, 29), -1)
        cv2.rectangle(canvas, (x, 104), (x + 608, 110), color, -1)
        text(side.upper() + " ARM", x + 20, 147, 0.8, bright, 2)
        text("ESTIMATED" if view.confirmed else "UNVERIFIED", x + 388, 146, 0.57, color, 2)
        text("Point: flange | Frame: " + (view.frame or "UNCONFIRMED")[:35],
             x + 20, 179, 0.48, muted)
        if not view.valid:
            text("NO VALID FORCE DATA", x + 40, 300, 0.78, amber, 2)
            # Wrap reason text so a long SDK/error message cannot overlap the other arm.
            message = view.message[:150]
            for line in range(3):
                text(message[line * 45:(line + 1) * 45], x + 30, 346 + line * 26, 0.5, amber)
            text("Values hidden; never substituted with zero", x + 30, 520, 0.51, muted)
            continue
        data = view.filtered
        force, torque = np.linalg.norm(data[:3]), np.linalg.norm(data[3:])
        text(f"|F| {force:.2f}", x + 20, 230, 0.9, cyan, 2)
        text(f"|M| {torque:.3f}", x + 310, 230, 0.9, cyan, 2)
        text("N / Nm" if view.confirmed else "N* / Nm*  (SDK unit convention unverified)",
             x + 20, 260, 0.48, muted)
        for i, key in enumerate(AXES):
            y = 303 + i * 43
            limit = force_scale if i < 3 else torque_scale
            text(key, x + 22, y + 6, 0.62, bright)
            cv2.line(canvas, (x + 95, y), (x + 355, y), (78, 67, 53), 8)
            cv2.line(canvas, (x + 225, y - 11), (x + 225, y + 11), muted, 1)
            end = x + 225 + int(130 * np.clip(data[i] / limit, -1, 1))
            cv2.line(canvas, (x + 225, y), (end, y), cyan, 7)
            text(f"{data[i]:+.3f}", x + 384, y + 6, 0.64, bright)
            if abs(data[i]) > limit:
                text("RANGE", x + 527, y + 5, 0.38, amber)
        text(f"cond {view.condition:.1f} | residual {view.residual:.3f}",
             x + 20, 585, 0.49, muted)
        text("Filtered display | no direction guidance", x + 20, 616, 0.49, muted)
    text("Bars: +/- %.1f force, +/- %.1f torque | display scale only, NOT safe limits" %
         (force_scale, torque_scale), 32, 670, 0.54, muted)
    text("* Verify units, frame, sign and tool load before interpreting contact forces.",
         32, 701, 0.52, amber)
    return cv2.resize(canvas, (width, height), interpolation=cv2.INTER_AREA)
