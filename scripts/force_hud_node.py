#!/usr/bin/python3
"""Read-only JPEG HUD; no command publisher or robot control dependency."""

import math
from pathlib import Path
import signal
import time

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import Trigger

from force_hud_core import (
    AXES, FT_AXES, ForceBaseline, HudModel, RingMapping, capture_force_baselines,
    ft_sensor_report, render_hud)


class ForceHudNode(Node):
    def __init__(self):
        super().__init__("tianji_force_hud")

        def param(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        self.input_source = param("input_source", "right_ft_sensor")
        if self.input_source not in ("right_ft_sensor", "joint_estimate"):
            raise ValueError("input_source must be right_ft_sensor or joint_estimate")
        self.ft_frame = param("right_ft_frame", "right_ft_measurement")
        self.ft_display_frame = param("right_ft_display_frame", self.ft_frame)
        self.joint_topic = param("joint_state_topic", "/io_teleop/joint_states")
        sensor_to_flange = param("right_ft_sensor_to_flange_rotation",
                                 [1., 0., 0., 0., 1., 0., 0., 0., 1.])
        ft_topic = param("right_ft_topic", "/io_teleop/right/ft_sensor")
        estimate_topic = param("estimate_topic", "/tianji_force_hud/estimates")
        self.source_topic = ft_topic if self.input_source == "right_ft_sensor" else estimate_topic
        image_topic = param("image_topic", "/tianji_force_hud/image/compressed")
        self.rate = param("image_rate_hz", 100.0)
        self.width = param("image_width", 1280)
        self.height = param("image_height", 720)
        self.quality = param("jpeg_quality", 80)
        self.force_scale = param("force_display_range", 20.0)
        self.torque_scale = param("torque_display_range", 2.0)
        self.display_mode = param("display_mode", "ring")
        self.display_arms = param("display_arms", "right" if self.input_source == "right_ft_sensor" else "both")
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
                              param("trend_window_sec", 0.8), source_kind=self.input_source)
        self.ft_direction_input = None
        if self.input_source == "right_ft_sensor":
            if self.ft_display_frame not in (self.ft_frame, "right_sdk_base"):
                raise ValueError("right_ft_display_frame must be measurement frame or right_sdk_base")
            if self.ft_display_frame == "right_sdk_base":
                from ament_index_python.packages import get_package_prefix, get_package_share_directory
                from force_hud_ft import FtDirectionInput
                from ft_calibration_core import Model
                from ft_calibration_sdk import Kinematics
                package = "tianji_io_teleop_bridge"
                model_file = Path(get_package_share_directory(package)) / "marvin_kinematics/ccs_m6_40.MvKDCfg"
                library = Path(get_package_prefix(package)) / "lib/libKine.so"
                # Only local libKine math; no Feedback/PositionFeedback or control SDK is opened.
                kine = Kinematics(None, Model.load(model_file), library_path=library)
                self.ft_direction_input = FtDirectionInput(
                    kine.rotation, np.asarray(sensor_to_flange).reshape(3, 3), self.ft_frame)
        zero_window = param("zero_window_sec", 1.0)
        zero_max_std = param("zero_max_std", 0.3)
        self.enable_display_force_zero = param("enable_display_force_zero", False)
        self.baselines = {side: ForceBaseline(zero_window, zero_max_std, self.model.timeout)
                          for side in self.model.arms}
        self.zero_services = []
        for side in ("left", "right", "both"):
            self.zero_services.append(self.create_service(
                Trigger, "~/" + side + "/zero_force",
                lambda request, response, side=side: self.zero_force(side, response)))
            self.zero_services.append(self.create_service(
                Trigger, "~/" + side + "/clear_force_zero",
                lambda request, response, side=side: self.zero_force(side, response, clear=True)))
        reliability = param("image_qos_reliability", "best_effort")
        if reliability not in ("best_effort", "reliable"):
            raise ValueError("image_qos_reliability must be best_effort or reliable")
        if (not math.isfinite(self.rate) or not 1 <= self.rate <= 100
                or not 1 <= self.quality <= 100):
            raise ValueError("image rate must be 1..100 Hz and JPEG quality 1..100")
        # Validate geometry and scales before starting any ROS callbacks.
        render_hud(self.model, math.nan, self.width, self.height,
                   self.force_scale, self.torque_scale, self.display_mode,
                   self.mappings, self.show_torque, self.display_arms)
        cv2.setNumThreads(1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                         reliability=(ReliabilityPolicy.RELIABLE if reliability == "reliable"
                                      else ReliabilityPolicy.BEST_EFFORT))
        self.image_pub = self.create_publisher(CompressedImage, image_topic, qos)
        self.values_pub = self.create_publisher(DiagnosticArray, "~/display_values", qos)
        self.subscription = self.create_subscription(
            JointState if self.input_source == "right_ft_sensor" else DiagnosticArray,
            self.source_topic, self.receive_ft if self.input_source == "right_ft_sensor" else self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.joint_subscription = None
        if self.ft_direction_input is not None:
            self.joint_subscription = self.create_subscription(
                JointState, self.joint_topic, self.receive_joints,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(
                f"FT direction: exact-stamp pairing with {self.joint_topic}, local flange FK -> right_sdk_base")
        self.render_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.rate ** -1, self.tick, clock=self.render_clock)
        self.last_error_log = -math.inf
        self.get_logger().info(
            f"Read-only JPEG HUD ({self.display_mode}, {self.display_arms}): {self.width}x{self.height} "
            f"@ {self.rate} Hz -> {image_topic}; input={self.input_source}: {self.source_topic}")
        if any(mapping.preview for mapping in self.mappings.values()):
            self.get_logger().warning(
                "UNCALIBRATED direction preview enabled: axes/sign are under test; "
                "not a motion instruction or a calibration confirmation")

    def receive(self, report):
        self.model.ingest(report, self.get_clock().now().nanoseconds, time.monotonic())
        for side, view in self.model.arms.items():
            self.baselines[side].observe(view, self.model.stamp_ns)

    def receive_ft(self, message):
        report = (self.ft_direction_input.feed("ft", message) if self.ft_direction_input is not None
                  else ft_sensor_report(message, self.ft_frame))
        if report is not None:
            self.model.ingest(report, self.get_clock().now().nanoseconds, time.monotonic())

    def receive_joints(self, message):
        report = self.ft_direction_input.feed("joints", message)
        if report is not None:
            self.model.ingest(report, self.get_clock().now().nanoseconds, time.monotonic())

    def zero_force(self, side, response, clear=False):
        if self.input_source == "right_ft_sensor":
            response.success = False
            response.message = (
                "HUD uses compensated+tared right FT data without another display zero. "
                "Use /tianji_marvin_teleop_bridge/right/zero_ft_sensor before recording; "
                "left FT sensor is unavailable.")
            return response
        sides = ("left", "right") if side == "both" else (side,)
        if clear:
            for arm in sides:
                self.baselines[arm].clear()
            response.success = True
            response.message = "display force zero cleared for " + side
            return response
        if not self.enable_display_force_zero:
            response.success = False
            response.message = (
                "Display-only tare is disabled. Use /tianji_marvin_teleop_bridge/" +
                side + "/zero_joint_torque to correct HUD input AND recorded joint effort.")
            return response
        self.model.refresh(self.get_clock().now().nanoseconds, time.monotonic())
        try:
            if self.display_mode != "ring":
                raise ValueError("force zero is only supported in ring display mode")
            capture_force_baselines(self.baselines, self.model.arms, self.model.stamp_ns, sides)
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
        if self.input_source == "joint_estimate":
            for side, view in self.model.arms.items():
                self.baselines[side].attach(view)
        age = ((now.nanoseconds - self.model.stamp_ns) / 1e9
               if self.model.stamp_ns else math.nan)
        frame = render_hud(self.model, age, self.width, self.height,
                           self.force_scale, self.torque_scale, self.display_mode,
                           self.mappings, self.show_torque, self.display_arms)
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
            item = DiagnosticStatus(name=side, hardware_id=("Marvin_right_FT_sensor"
                                    if self.input_source == "right_ft_sensor" else "Marvin_M6_estimate_only"))
            item.level = (DiagnosticStatus.OK if view.confirmed else DiagnosticStatus.WARN
                          ) if view.valid else DiagnosticStatus.ERROR
            item.message = view.message
            if self.display_mode == "ring" and self.mappings[side].preview and view.valid:
                item.level = DiagnosticStatus.WARN
                item.message = "UNCALIBRATED direction preview: " + view.message
            fields = {"source_stamp_ns": str(self.model.stamp_ns),
                      "source_topic": self.source_topic, "source_kind": self.input_source,
                      "reference_point": view.reference_point,
                      "input_processing": ("gravity_compensated_and_software_tared"
                                           if self.input_source == "right_ft_sensor" else "joint_disturbance_estimate"),
                      "source_age_sec": str(age), "wrench_frame": view.frame,
                      "convention_confirmed": str(view.confirmed).lower(),
                      "smoothing_time_constant_sec": str(self.model.smoothing_sec),
                      "display_mode": self.display_mode,
                      "display_arms": self.display_arms,
                      "panel_visible": str(self.display_arms in ("both", side)).lower(),
                      "trend_window_sec": str(self.model.trend_window_sec),
                      "force_display_range": str(self.force_scale)}
            fields.update({"ring_semantics": mapping.semantics,
                           "task_yaw_deg": str(mapping.yaw_deg),
                           "external_force_sign": str(mapping.force_sign),
                           "direction_mapping_confirmed": str(mapping.confirmed).lower(),
                           "lateral_display_deadzone": str(mapping.lateral_deadzone),
                           "axial_display_deadzone": str(mapping.axial_deadzone),
                           "display_zero_stamp_ns": str(view.zero_stamp_ns)})
            # Static base/display -> task mapping, plus same-frame sensor -> base when enabled.
            display_to_task = mapping.rotation()
            fields["display_to_task_rotation_row_major"] = ",".join(str(x) for x in display_to_task.flat)
            fields["source_to_task_rotation_row_major"] = ",".join(
                str(x) for x in (display_to_task if view.sensor_to_display_rotation is None
                                else display_to_task @ view.sensor_to_display_rotation).flat)
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
                if self.input_source == "right_ft_sensor":
                    fields["measurement_frame"] = view.measurement_frame
                    fields.update({"sensor_" + k: str(v) for k, v in zip(FT_AXES, view.measurement_raw)})
                    if view.sensor_to_display_rotation is not None:
                        fields["joint_source_stamp_ns"] = str(view.joint_stamp_ns)
                        fields["joint_state_topic"] = self.joint_topic
                        fields["sensor_to_display_rotation_row_major"] = ",".join(
                            str(x) for x in view.sensor_to_display_rotation.flat)
                    fields.update({"raw_" + k: str(v) for k, v in zip(FT_AXES, view.raw)})
                    fields.update({"filtered_" + k: str(v) for k, v in zip(FT_AXES, view.filtered)})
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
