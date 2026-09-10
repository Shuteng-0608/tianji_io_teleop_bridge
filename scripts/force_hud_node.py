#!/usr/bin/python3
"""Read-only 30 Hz JPEG HUD; no command publisher or robot control dependency."""

import math
import signal
import time

import cv2
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

from force_hud_core import AXES, ForceBaseline, HudModel, RingMapping, render_hud


class ForceHudNode(Node):
    def __init__(self):
        super().__init__("tianji_force_hud")

        def param(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        estimate_topic = param("estimate_topic", "/tianji_force_hud/estimates")
        image_topic = param("image_topic", "/tianji_force_hud/image/compressed")
        self.rate = param("image_rate_hz", 30.0)
        self.width = param("image_width", 1280)
        self.height = param("image_height", 720)
        self.quality = param("jpeg_quality", 80)
        self.force_scale = param("force_display_range", 20.0)
        self.torque_scale = param("torque_display_range", 2.0)
        self.display_mode = param("display_mode", "ring")
        self.show_torque = param("ring_show_torque", False)
        semantics = param("ring_semantics", "force")
        lateral_deadzone = param("lateral_display_deadzone", 0.0)
        axial_deadzone = param("axial_display_deadzone", 0.0)
        self.mappings = {}
        for side in ("left", "right"):
            self.mappings[side] = RingMapping(
                confirmed=param(side + "_direction_mapping_confirmed", False),
                frame=param(side + "_direction_frame", ""),
                preview=param(side + "_direction_preview", False),
                semantics=semantics,
                yaw_deg=param(side + "_task_yaw_deg", 0.0),
                force_sign=param(side + "_external_force_sign", 1.0),
                lateral_deadzone=lateral_deadzone,
                axial_deadzone=axial_deadzone,
                right=tuple(param(side + "_screen_right_axis", [1.0, 0.0, 0.0])),
                up=tuple(param(side + "_screen_up_axis", [0.0, 1.0, 0.0])),
                axial=tuple(param(side + "_axial_axis", [0.0, 0.0, 1.0])))
        self.model = HudModel(param("input_timeout_sec", 0.2),
                              param("smoothing_time_constant_sec", 0.08),
                              param("future_tolerance_sec", 0.05),
                              param("trend_window_sec", 0.8))
        zero_window = param("zero_window_sec", 1.0)
        zero_max_std = param("zero_max_std", 0.3)
        self.baselines = {side: ForceBaseline(zero_window, zero_max_std, self.model.timeout)
                          for side in self.model.arms}
        self.zero_services = []
        for side in self.model.arms:
            self.zero_services.append(self.create_service(
                Trigger, "~/" + side + "/zero_force",
                lambda request, response, side=side: self.zero_force(side, response)))
            self.zero_services.append(self.create_service(
                Trigger, "~/" + side + "/clear_force_zero",
                lambda request, response, side=side: self.zero_force(side, response, clear=True)))
        reliability = param("image_qos_reliability", "best_effort")
        if reliability not in ("best_effort", "reliable"):
            raise ValueError("image_qos_reliability must be best_effort or reliable")
        if (not math.isfinite(self.rate) or not 1 <= self.rate <= 60
                or not 1 <= self.quality <= 100):
            raise ValueError("image rate must be 1..60 Hz and JPEG quality 1..100")
        # Validate geometry and scales before starting any ROS callbacks.
        render_hud(self.model, math.nan, self.width, self.height,
                   self.force_scale, self.torque_scale, self.display_mode,
                   self.mappings, self.show_torque)
        cv2.setNumThreads(1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                         reliability=(ReliabilityPolicy.RELIABLE if reliability == "reliable"
                                      else ReliabilityPolicy.BEST_EFFORT))
        self.image_pub = self.create_publisher(CompressedImage, image_topic, qos)
        self.values_pub = self.create_publisher(DiagnosticArray, "~/display_values", qos)
        self.subscription = self.create_subscription(
            DiagnosticArray, estimate_topic, self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.render_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.rate ** -1, self.tick, clock=self.render_clock)
        self.last_error_log = -math.inf
        self.get_logger().info(
            f"Read-only JPEG HUD ({self.display_mode}): {self.width}x{self.height} "
            f"@ {self.rate} Hz -> {image_topic}")
        if any(mapping.preview for mapping in self.mappings.values()):
            self.get_logger().warning(
                "UNCALIBRATED direction preview enabled: axes/sign are under test; "
                "not a motion instruction or a calibration confirmation")

    def receive(self, report):
        self.model.ingest(report, self.get_clock().now().nanoseconds, time.monotonic())
        for side, view in self.model.arms.items():
            self.baselines[side].observe(view, self.model.stamp_ns)

    def zero_force(self, side, response, clear=False):
        if clear:
            self.baselines[side].clear()
            response.success = True
            response.message = "display force zero cleared for " + side
            return response
        self.model.refresh(self.get_clock().now().nanoseconds, time.monotonic())
        try:
            if self.display_mode != "ring":
                raise ValueError("force zero is only supported in ring display mode")
            self.baselines[side].capture(self.model.arms[side], self.model.stamp_ns)
            response.success = True
            response.message = (
                "display-only force zero captured for " + side +
                "; assumes operator verified NO CONTACT; raw estimates and robot unchanged")
            self.get_logger().warning(response.message)
        except ValueError as error:
            response.success = False
            response.message = str(error)
        return response

    def tick(self):
        now = self.get_clock().now()
        steady = time.monotonic()
        self.model.refresh(now.nanoseconds, steady)
        for side, view in self.model.arms.items():
            self.baselines[side].attach(view)
        age = ((now.nanoseconds - self.model.stamp_ns) / 1e9
               if self.model.stamp_ns else math.nan)
        frame = render_hud(self.model, age, self.width, self.height,
                           self.force_scale, self.torque_scale, self.display_mode,
                           self.mappings, self.show_torque)
        try:
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                raise RuntimeError("JPEG encoder returned failure")
        except (cv2.error, RuntimeError) as error:
            if steady - self.last_error_log > 1:
                self.get_logger().error(str(error))
                self.last_error_log = steady
            return
        image = CompressedImage()
        # Synthetic canvas generation time; NOT the force acquisition timestamp.
        # frame_id stays empty: this canvas is not a physical camera optical frame.
        image.header.stamp = now.to_msg()
        image.format = "jpeg"
        image.data = encoded.tobytes()
        self.image_pub.publish(image)
        values = DiagnosticArray()
        values.header = image.header  # Allows exact pairing with the displayed image.
        for side, view in self.model.arms.items():
            mapping = self.mappings[side]
            item = DiagnosticStatus(name=side, hardware_id="Marvin_M6_estimate_only")
            item.level = (DiagnosticStatus.OK if view.confirmed else DiagnosticStatus.WARN
                          ) if view.valid else DiagnosticStatus.ERROR
            item.message = view.message
            if self.display_mode == "ring" and self.mappings[side].preview and view.valid:
                item.level = DiagnosticStatus.WARN
                item.message = "UNCALIBRATED direction preview: " + view.message
            fields = {"source_stamp_ns": str(self.model.stamp_ns),
                      "source_age_sec": str(age), "wrench_frame": view.frame,
                      "convention_confirmed": str(view.confirmed).lower(),
                      "smoothing_time_constant_sec": str(self.model.smoothing_sec),
                      "display_mode": self.display_mode,
                      "trend_window_sec": str(self.model.trend_window_sec),
                      "force_display_range": str(self.force_scale)}
            fields.update({"ring_semantics": mapping.semantics,
                           "task_yaw_deg": str(mapping.yaw_deg),
                           "external_force_sign": str(mapping.force_sign),
                           "direction_mapping_confirmed": str(mapping.confirmed).lower(),
                           "lateral_display_deadzone": str(mapping.lateral_deadzone),
                           "axial_display_deadzone": str(mapping.axial_deadzone),
                           "display_zero_stamp_ns": str(view.zero_stamp_ns)})
            # Record the exact fixed rotation for later reconstruction; no TF is implied.
            fields["source_to_task_rotation_row_major"] = ",".join(
                str(x) for x in mapping.rotation().flatten())
            if view.force_bias is not None:
                fields.update({"display_bias_" + k: str(v) for k, v in
                               zip(("Fx", "Fy", "Fz"), view.force_bias)})
            projected = mapping.project(view) if self.display_mode == "ring" else None
            fields["direction_active"] = str(projected is not None).lower()
            fields["direction_preview"] = str(self.mappings[side].preview).lower()
            if projected is not None:
                fields.update({"projected_" + k: str(v) for k, v in
                               zip(("screen_right", "screen_up", "axial"), projected)})
                arrow = mapping.arrow(projected)
                fields.update({"arrow_" + k: str(v) for k, v in
                               zip(("screen_right", "screen_up"), arrow)})
                fields["arrow_visible"] = str(bool(any(arrow))).lower()
                fields["axial_magnitude"] = str(abs(projected[2]))
            if view.valid:
                fields.update({"raw_" + k: str(v) for k, v in zip(AXES, view.raw)})
                fields.update({"filtered_" + k: str(v) for k, v in zip(AXES, view.filtered)})
                if view.trend_delta is not None:
                    fields["force_trend_delta"] = str(view.trend_delta)
            item.values = [KeyValue(key=k, value=v) for k, v in fields.items()]
            values.status.append(item)
        self.values_pub.publish(values)


def main():
    rclpy.init()
    node = None
    try:
        node = ForceHudNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT can reach both the launch group and then each child.
        # Do not let the second signal interrupt destruction halfway through.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
