#!/usr/bin/python3
"""Isolated synthetic feedback + computation-only services; never starts a control node."""
import math
import os
import subprocess
import time

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = str(110 + os.getpid() % 80)

import rclpy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node('retreat_offline_probe')
    topic = '/retreat_probe_' + str(os.getpid())
    publisher = node.create_publisher(JointState, topic, 1)
    executable = get_package_prefix('tianji_io_teleop_bridge') + '/lib/tianji_io_teleop_bridge/retreat_check_node'
    process = subprocess.Popen([executable, '--ros-args', '-p', 'state_topic:=' + topic])
    names = [f'Joint{i}_{side}' for side in ('L', 'R') for i in range(1, 8)]
    q = [90, -75, -90, -90, 0, 0, 0, -90, -75, 90, -90, 0, 0, 0]

    def publish(valid=True, age=0):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.stamp.sec -= age
        msg.name = list(reversed(names))  # Exercise name-based mapping.
        msg.position = list(reversed([math.radians(x) for x in q]))
        if not valid:
            msg.name[1] = msg.name[0]
        publisher.publish(msg)

    def call(client, feed=False):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            if feed:
                publish()
            rclpy.spin_once(node, timeout_sec=0.01)
        assert future.done(), 'service response timeout'
        return future.result()

    try:
        clients = [node.create_client(Trigger, '/tianji_retreat_check/retreat_' + arm)
                   for arm in ('left', 'right')]
        assert all(client.wait_for_service(timeout_sec=5) for client in clients)
        assert not call(clients[0]).success  # No feedback.
        for client in clients:
            for _ in range(25):
                publish()
                rclpy.spin_once(node, timeout_sec=0.01)
            result = call(client, feed=True)
            print(result.message, flush=True)
            assert result.success
            assert 'NO_MOTION' in result.message
            assert 'flange_offset_mm=[-30,0,0]' in result.message
        time.sleep(0.3)
        assert not call(clients[0]).success  # Stale feedback.
        for _ in range(20):
            publish(valid=False)
            rclpy.spin_once(node, timeout_sec=0.01)
        assert not call(clients[0]).success
        for _ in range(20):
            publish(age=2)
            rclpy.spin_once(node, timeout_sec=0.01)
        assert not call(clients[0]).success
        print('PASS: both services; reversed names; missing, stale, duplicate and old feedback rejected')
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
