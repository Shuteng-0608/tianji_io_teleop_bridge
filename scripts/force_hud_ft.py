"""Read-only FT/pose pairing and display-axis rotation. No control SDK or ROS imports."""
from collections import OrderedDict

import numpy as np

from force_hud_core import ft_sensor_report
from ft_calibration_core import finite_array, rotation


RIGHT_JOINTS = tuple(f"Joint{i}_R" for i in range(1, 8))


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def right_joint_degrees(message):
    if (len(message.name) != len(message.position)
            or len(set(message.name)) != len(message.name)
            or not set(RIGHT_JOINTS).issubset(message.name)):
        raise ValueError("expected uniquely named right-arm joint positions")
    positions = dict(zip(message.name, message.position))
    return np.rad2deg(finite_array([positions[key] for key in RIGHT_JOINTS], (7,),
                                  "right joint radians"))


class FtDirectionInput:
    """Pair exact bridge timestamps, independent of callback order, with bounded queues.

    The bridge publishes FT and joint states from the same DCSS snapshot. Never use
    nearest/latest pose for a force sample. Missing pairs expire through HudModel's
    existing source-time and monotonic timeout checks.
    """
    def __init__(self, flange_rotation, sensor_to_flange, sensor_frame, capacity=128):
        self.flange_rotation = flange_rotation
        self.sensor_to_flange = rotation(sensor_to_flange, "sensor to flange rotation")
        self.sensor_frame = sensor_frame
        self.capacity = capacity
        self.pending = {"ft": OrderedDict(), "joints": OrderedDict()}

    def feed(self, kind, message):
        stamp = stamp_ns(message)
        own = self.pending[kind]
        own[stamp] = message
        own.move_to_end(stamp)
        while len(own) > self.capacity:
            own.popitem(last=False)
        if stamp not in self.pending["ft"] or stamp not in self.pending["joints"]:
            return None
        ft = self.pending["ft"].pop(stamp)
        joints = self.pending["joints"].pop(stamp)
        try:
            r = rotation(self.flange_rotation(right_joint_degrees(joints)), "flange FK rotation")
            return ft_sensor_report(ft, self.sensor_frame,
                                    display_rotation=r @ self.sensor_to_flange,
                                    display_frame="right_sdk_base", joint_stamp_ns=stamp)
        except (ValueError, TypeError, RuntimeError, OverflowError) as error:
            return ft_sensor_report(ft, self.sensor_frame, error="POSE/FK: " + str(error))
