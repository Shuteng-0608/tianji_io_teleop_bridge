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

from force_hud_core import AXES, HudModel, render_hud


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
        self.model = HudModel(param("input_timeout_sec", 0.2),
                              param("smoothing_time_constant_sec", 0.08),
                              param("future_tolerance_sec", 0.05))
        reliability = param("image_qos_reliability", "best_effort")
        if reliability not in ("best_effort", "reliable"):
            raise ValueError("image_qos_reliability must be best_effort or reliable")
        if (not math.isfinite(self.rate) or not 1 <= self.rate <= 60
                or not 1 <= self.quality <= 100):
            raise ValueError("image rate must be 1..60 Hz and JPEG quality 1..100")
        # Validate geometry and scales before starting any ROS callbacks.
        render_hud(self.model, math.nan, self.width, self.height,
                   self.force_scale, self.torque_scale)
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
            f"Read-only JPEG HUD: {self.width}x{self.height} @ {self.rate} Hz -> {image_topic}")

    def receive(self, report):
        self.model.ingest(report, self.get_clock().now().nanoseconds, time.monotonic())

    def tick(self):
        now = self.get_clock().now()
        steady = time.monotonic()
        self.model.refresh(now.nanoseconds, steady)
        age = ((now.nanoseconds - self.model.stamp_ns) / 1e9
               if self.model.stamp_ns else math.nan)
        frame = render_hud(self.model, age, self.width, self.height,
                           self.force_scale, self.torque_scale)
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
            item = DiagnosticStatus(name=side, hardware_id="Marvin_M6_estimate_only")
            item.level = (DiagnosticStatus.OK if view.confirmed else DiagnosticStatus.WARN
                          ) if view.valid else DiagnosticStatus.ERROR
            item.message = view.message
            fields = {"source_stamp_ns": str(self.model.stamp_ns),
                      "source_age_sec": str(age), "wrench_frame": view.frame,
                      "convention_confirmed": str(view.confirmed).lower(),
                      "smoothing_time_constant_sec": str(self.model.smoothing_sec)}
            if view.valid:
                fields.update({"raw_" + k: str(v) for k, v in zip(AXES, view.raw)})
                fields.update({"filtered_" + k: str(v) for k, v in zip(AXES, view.filtered)})
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
