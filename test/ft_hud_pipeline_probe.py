#!/usr/bin/python3
"""Offline ROS FT-to-HUD integration probe; never starts a robot bridge or SDK."""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid


def run(insertion=False):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    import cv2
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, JointState
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node("ft_hud_probe")
    prefix = "/ft_hud_probe_" + uuid.uuid4().hex[:10]
    output = Path(tempfile.mkdtemp(prefix="tianji_ft_hud_probe_"))
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    ft_pub = node.create_publisher(JointState, prefix + "/ft", 1)
    old_pub = node.create_publisher(DiagnosticArray, prefix + "/estimates", 1)
    joint_pub = node.create_publisher(JointState, prefix + "/disturbance", 1)
    pose_pub = node.create_publisher(JointState, prefix + "/joints", 1)
    values = [1.2, -2.3, 3.4, .01, -.02, .03]
    expected = list(values)
    pose_index, pose_cases, input_parameters = 0, [], {}
    if insertion:
        import yaml
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
        from ft_calibration_core import Model
        from ft_calibration_sdk import Kinematics
        package = "tianji_io_teleop_bridge"
        share = Path(get_package_share_directory(package))
        input_parameters = yaml.safe_load((share / "config/force_hud_insertion_preview.yaml").read_text())[
            "tianji_force_hud"]["ros__parameters"]
        axes = np.array([input_parameters["right_" + key] for key in
                         ("screen_right_axis", "screen_up_axis", "axial_axis")])
        base_force = axes.T @ [-4., 2., 8.]
        base_torque = np.array([.01, -.02, .03])
        expected = [*base_force, *base_torque]
        kine = Kinematics(None, Model.load(share / "marvin_kinematics/ccs_m6_40.MvKDCfg"),
                          library_path=Path(get_package_prefix(package)) / "lib/libKine.so")
        for q in ([0.] * 7, [0., 0., 0., 0., 0., -40., -50.],
                  [-110., -90., 90., -110., 0., 20., 30.]):
            r = kine.rotation(q)
            pose_cases.append((np.deg2rad(q).tolist(), [*(r.T @ base_force), *(r.T @ base_torque)]))
        values = pose_cases[0][1]
    latest, images, source_stamps = {}, [], set()
    mode, frozen_stamp = "valid", None
    decoded = None

    def publish():
        nonlocal frozen_stamp
        # Legacy streams continue even while FT is stopped; there must be no fallback.
        old_pub.publish(DiagnosticArray())
        joint_pub.publish(JointState(name=["Joint1_R"], effort=[99.]))
        if mode == "stopped":
            return
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "right_ft_measurement"
        msg.name = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
        msg.effort = values
        if mode == "missing":
            msg.effort = []
        elif mode == "duplicate_name":
            msg.name[-1] = msg.name[0]
        elif mode == "wrong_frame":
            msg.header.frame_id = "right_sdk_base"
        elif mode == "nan":
            msg.effort[0] = float("nan")
        elif mode == "future":
            msg.header.stamp.sec += 2
        elif mode == "old":
            msg.header.stamp.sec -= 2
        elif mode == "repeated":
            if frozen_stamp is None:
                frozen_stamp = msg.header.stamp
            msg.header.stamp = frozen_stamp
        msg.name.reverse()
        msg.effort.reverse()
        source_stamps.add(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
        ft_pub.publish(msg)
        if insertion and mode != "pose_stopped":
            pose = JointState()
            pose.header.stamp.sec = msg.header.stamp.sec
            pose.header.stamp.nanosec = msg.header.stamp.nanosec
            pose.name = [f"Joint{i}_R" for i in range(1, 8)]
            pose.position = pose_cases[pose_index][0]
            if mode == "pose_skew":
                pose.header.stamp.sec += 1
            elif mode == "pose_nan":
                pose.position[0] = float("nan")
            elif mode == "pose_names":
                pose.name[0] = "Joint2_R"
            pose_pub.publish(pose)

    def receive_display(msg):
        latest["display"] = msg

    def receive_image(msg):
        nonlocal decoded
        images.append(time.monotonic())
        if len(images) % 10 == 0:
            decoded = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            assert msg.format == "jpeg" and decoded.shape == (720, 1280, 3)

    subscriptions = [node.create_subscription(DiagnosticArray, prefix + "/display", receive_display, qos),
                     node.create_subscription(CompressedImage, prefix + "/image", receive_image, qos)]
    timer = node.create_timer(1 / 300, publish)
    executable = Path(get_package_prefix("tianji_io_teleop_bridge")) / "lib/tianji_io_teleop_bridge/force_hud_node"
    command = [str(executable), "--ros-args", "-r", "__node:=ft_hud_probe_renderer",
               "-p", "input_source:=right_ft_sensor", "-p", "right_ft_topic:=" + prefix + "/ft",
               "-p", "estimate_topic:=" + prefix + "/estimates", "-p", "image_topic:=" + prefix + "/image",
               "-p", "smoothing_time_constant_sec:=0.0", "-p", "enable_display_force_zero:=true",
               "-r", "~/display_values:=" + prefix + "/display",
               "-r", "~/right/zero_force:=" + prefix + "/zero"]
    if insertion:
        params_file = output / "insertion.yaml"
        params_file.write_text(yaml.safe_dump({"ft_hud_probe_renderer": {"ros__parameters": input_parameters}}))
        command[2:2] = ["--params-file", str(params_file)]
        command += ["-p", "joint_state_topic:=" + prefix + "/joints"]
    log = (output / "renderer.log").open("w")
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.003)

    def valid():
        items = {s.name: s for s in latest["display"].status}
        assert items["right"].level == (DiagnosticStatus.WARN if insertion else DiagnosticStatus.OK)
        assert items["left"].level == DiagnosticStatus.ERROR
        right = {v.key: v.value for v in items["right"].values}
        assert right["source_kind"] == "right_ft_sensor"
        assert right["source_topic"] == prefix + "/ft"
        assert right["reference_point"] == "sensor"
        assert right["input_processing"] == "gravity_compensated_and_software_tared"
        assert int(right["source_stamp_ns"]) in source_stamps
        assert right["display_zero_stamp_ns"] == "0"
        assert right["display_arms"] == "right" and right["direction_active"] == str(insertion).lower()
        for axis, component in zip(("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"), expected):
            assert abs(float(right["raw_" + axis]) - component) < 1e-9
            assert abs(float(right["filtered_" + axis]) - component) < 1e-9
        if insertion:
            assert right["wrench_frame"] == "right_sdk_base"
            assert right["measurement_frame"] == "right_ft_measurement"
            assert right["joint_source_stamp_ns"] == right["source_stamp_ns"]
            for key, component in (("arrow_screen_right", 4.), ("arrow_screen_up", -2.),
                                   ("projected_axial", 8.)):
                assert abs(float(right[key]) - component) < 1e-9
            for axis, component in zip(("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"), values):
                assert abs(float(right["sensor_" + axis]) - component) < 1e-9
        assert not any(v.key.startswith("raw_") for v in items["left"].values)

    try:
        for _ in range(100):
            spin(.1)
            if "display" in latest and decoded is not None and ft_pub.get_subscription_count():
                break
        assert process.poll() is None, output
        valid()
        assert old_pub.get_subscription_count() == joint_pub.get_subscription_count() == 0
        images.clear()
        spin(3.)
        valid()
        rate = (len(images) - 1) / (images[-1] - images[0])
        assert 95 < rate < 105, rate
        cv2.imwrite(str(output / "valid.jpg"), decoded)
        if insertion:
            assert pose_pub.get_subscription_count() == 1
            for index in (1, 2, 0):
                pose_index, values = index, pose_cases[index][1]
                spin(.3)
                valid()
        client = node.create_client(Trigger, prefix + "/zero")
        try:
            assert client.wait_for_service(timeout_sec=2)
            future = client.call_async(Trigger.Request())
            for _ in range(30):
                spin(.1)
                if future.done():
                    break
            response = future.result()
            assert response is not None and not response.success
            assert "/right/zero_ft_sensor" in response.message
        finally:
            node.destroy_client(client)
        faults = ["missing", "duplicate_name", "wrong_frame", "nan", "future", "old", "repeated", "stopped"]
        if insertion:
            faults += ["pose_stopped", "pose_skew", "pose_nan", "pose_names"]
        for bad in faults:
            mode = bad
            spin(.5)
            right = next(s for s in latest["display"].status if s.name == "right")
            assert right.level == DiagnosticStatus.ERROR, (bad, right)
            assert not any(v.key.startswith("filtered_") for v in right.values)
            if bad == "stopped":
                cv2.imwrite(str(output / "stale.jpg"), decoded)
            mode = "valid"
            spin(.5)
            valid()
        result = {"artifacts": str(output), "image_rate_hz": rate, "insertion": insertion,
                  "checks": ["same compensated/tared six-axis values and source stamp", "right sensor only",
                             "legacy streams ignored", "no second tare", "invalid data hidden",
                             "stale and repeated stamps expire", "recovery", "JPEG 1280x720 at 100Hz"]}
        if insertion:
            result["checks"] += ["same-stamp FK to original hole mapping", "fixed force direction across wrist poses",
                                 "missing/mismatched/invalid pose clears arrows", "unmodified sensor values retained"]
        (output / "results.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        log.close()
        node.destroy_timer(timer)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--insertion", action="store_true")
    run(parser.parse_args().insertion)
