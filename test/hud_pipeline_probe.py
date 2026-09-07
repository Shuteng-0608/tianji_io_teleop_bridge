#!/usr/bin/python3
"""Offline ROS pipeline probe. Publishes ONLY to unique /hud_probe_* topics.

Starts only the kinematics estimator and renderer, never the robot bridge.
Artifacts go to a temporary directory (or --output-dir). No hardware required.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid


def run(duration=10.0, output_dir=None):
    # Set before importing/initializing ROS; localhost + unique topics isolate the probe.
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    import cv2
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, JointState

    output = Path(output_dir or tempfile.mkdtemp(prefix="tianji_hud_probe_"))
    output.mkdir(parents=True, exist_ok=True)
    prefix = "/hud_probe_" + uuid.uuid4().hex[:12]
    executable_dir = Path(get_package_prefix("tianji_io_teleop_bridge")) / "lib/tianji_io_teleop_bridge"
    rclpy.init()
    node = rclpy.create_node("hud_probe_" + uuid.uuid4().hex[:8])
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(JointState, prefix + "/input", 1)
    processes, logs = [], []
    latest = {"estimate": None, "display": None}
    arrivals = {key: [] for key in ("source", "estimate", "image")}
    transport_ms, image_bytes = [], []
    mode, collecting = "valid", False
    image_count = 0
    image_snapshot = None

    def publish():
        if mode == "stopped":
            return
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = [f"Joint{i}_{side}" for side in ("L", "R") for i in range(1, 8)]
        # Nonsingular synthetic poses; zero disturbance should give zero equivalent wrench.
        message.position = [math.radians(x) for x in
                            [30, -55, 25, -80, 35, 30, 15, -30, -55, -25, -80, -35, 30, -15]]
        message.effort = [0.0] * 14 if mode == "valid" else []
        if mode in ("singular", "future", "old", "duplicate"):
            message.effort = [0.0] * 14
        if mode == "singular":
            message.position = [0.0] * 14
        elif mode == "future":
            message.header.stamp.sec += 2
        elif mode == "old":
            message.header.stamp.sec -= 2
        elif mode == "duplicate":
            message.name[7] = message.name[0]
        # Reverse fields together to exercise the runtime name mapping.
        message.name.reverse()
        message.position.reverse()
        message.effort.reverse()
        pub.publish(message)
        if collecting:
            arrivals["source"].append(time.monotonic())

    def estimate(message):
        latest["estimate"] = message
        if collecting:
            arrivals["estimate"].append(time.monotonic())

    def display(message):
        latest["display"] = message

    def image(message):
        nonlocal image_count, image_snapshot
        image_count += 1
        if collecting:
            arrivals["image"].append(time.monotonic())
            stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            transport_ms.append((node.get_clock().now().nanoseconds - stamp_ns) / 1e6)
            image_bytes.append(len(message.data))
        if image_count % 10 == 0:
            assert message.format == "jpeg"
            decoded = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            assert decoded is not None and decoded.shape == (720, 1280, 3)
            image_snapshot = decoded

    timer = node.create_timer(1 / 500, publish)
    subscriptions = [
        node.create_subscription(DiagnosticArray, prefix + "/estimates", estimate, qos),
        node.create_subscription(CompressedImage, prefix + "/image", image, qos),
        node.create_subscription(DiagnosticArray, prefix + "/display_values", display, qos),
    ]

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.003)

    def assert_valid():
        for key in latest:
            msg = latest[key]
            assert msg is not None and len(msg.status) == 2, (key, msg)
            assert all(item.level == DiagnosticStatus.WARN for item in msg.status), [x.message for x in msg.status]
            for item in msg.status:
                fields = {x.key: x.value for x in item.values}
                assert fields["convention_confirmed"] == "false"
                for axis in ("Fx", "Fy", "Fz", "Mx", "My", "Mz"):
                    assert abs(float(fields["raw_" + axis])) < 1e-8

    def assert_invalid():
        assert latest["display"] is not None
        assert all(x.level == DiagnosticStatus.ERROR for x in latest["display"].status)
        assert all(not any(v.key.startswith("filtered_") for v in x.values)
                   for x in latest["display"].status)

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

    try:
        estimator_args = [str(executable_dir / "force_hud_estimator_node"), "--ros-args",
                          "-r", "__node:=hud_probe_estimator", "-p", "state_topic:=" + prefix + "/input",
                          "-p", "estimate_topic:=" + prefix + "/estimates"]
        renderer_args = [str(executable_dir / "force_hud_node"), "--ros-args",
                         "-r", "__node:=hud_probe_renderer", "-p", "estimate_topic:=" + prefix + "/estimates",
                         "-p", "image_topic:=" + prefix + "/image",
                         "-r", "~/display_values:=" + prefix + "/display_values"]
        for name, command in (("estimator", estimator_args), ("renderer", renderer_args)):
            log = (output / (name + ".log")).open("w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        spin(3)
        assert all(p.poll() is None for p in processes), "child exited; inspect probe logs"
        assert_valid()
        collecting = True
        spin(duration)
        collecting = False
        assert_valid()
        assert image_snapshot is not None
        cv2.imwrite(str(output / "valid.jpg"), image_snapshot)
        mode = "invalid"
        spin(0.8)
        assert_invalid()
        mode = "valid"
        spin(0.8)
        assert_valid()
        mode = "stopped"
        spin(0.8)
        assert_invalid()
        cv2.imwrite(str(output / "stale.jpg"), image_snapshot)
        mode = "valid"
        spin(0.8)
        assert_valid()
        for invalid_mode in ("singular", "future", "old", "duplicate"):
            mode = invalid_mode
            spin(0.5)
            assert_invalid()
            mode = "valid"
            spin(0.5)
            assert_valid()
        stop(processes[0])
        spin(0.8)
        assert_invalid()

        def stats(times):
            delta = np.diff(times)
            assert len(delta) > 10
            return {"samples": len(times), "hz": float(1 / delta.mean()),
                    "interval_ms_p99": float(np.percentile(delta, 99) * 1000),
                    "interval_ms_max": float(delta.max() * 1000),
                    "interval_ms_std": float(delta.std() * 1000)}

        results = {key: stats(value) for key, value in arrivals.items()}
        results["image_generation_to_local_receive_ms_p99"] = float(np.percentile(transport_ms, 99))
        results["mean_jpeg_bytes"] = float(np.mean(image_bytes))
        results["duration_sec"] = duration
        results["checks"] = ["reversed joint mapping", "zero-wrench conversion", "JPEG decode",
                             "missing effort invalidation", "input timeout", "recovery",
                             "singular pose", "future/old timestamps", "duplicate joint names",
                             "estimator death timeout", "unverified convention preserved"]
        (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
        print(json.dumps({"artifacts": str(output), **results}, indent=2), flush=True)
        assert 28.5 <= results["image"]["hz"] <= 31.5, results
        assert 95 <= results["estimate"]["hz"] <= 105, results
        return results
    finally:
        for process in processes:
            stop(process)
        for log in logs:
            log.close()
        node.destroy_timer(timer)
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output-dir")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 2:
        parser.error("duration must be >= 2 seconds")
    run(args.duration, args.output_dir)
