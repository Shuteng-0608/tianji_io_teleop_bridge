"""Right-arm effort display model; no ROS, GUI, or robot-control dependencies."""

import math


RIGHT_JOINTS = tuple(f'Joint{i}_R' for i in range(1, 8))


def format_torque(value):
    """Three decimals, with no misleading negative zero."""
    if not math.isfinite(value):
        return '—'
    text = f'{value:.3f}'
    return '0.000' if text == '-0.000' else text


class TorqueView:
    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self.values = None
        self.stamp_ns = None
        self.received = None
        self.reason = '等待右臂数据'

    def accept(self, names, effort, stamp_ns, now_ros_ns, now_steady):
        if stamp_ns <= 0 or not -0.05 <= (now_ros_ns - stamp_ns) / 1e9 <= self.timeout:
            self.values = None
            self.reason = '源时间戳无效或已过期'
            self.stamp_ns = None  # Permit recovery after a ROS clock reset.
            return
        if len(names) != len(set(names)) or len(effort) != len(names):
            self.values = None
            self.reason = '关节名称重复或力矩数组缺失'
            return
        try:
            mapped = dict(zip(names, effort))
            values = tuple(float(mapped[name]) for name in RIGHT_JOINTS)
            if not all(math.isfinite(v) for v in values):
                raise ValueError()
        except (KeyError, ValueError, TypeError):
            self.values = None
            self.reason = '右臂力矩缺失或包含非有限值'
            return
        if self.stamp_ns is not None and (now_ros_ns - self.stamp_ns) / 1e9 < -0.05:
            self.stamp_ns = None  # Local ROS clock moved behind the previous sample.
        if self.stamp_ns is not None and stamp_ns <= self.stamp_ns:
            # Repeated/reordered messages must not extend the lifetime of an old value.
            return
        self.values, self.stamp_ns, self.received = values, stamp_ns, now_steady
        self.reason = ''

    def current(self, now_ros_ns, now_steady):
        if self.values is None:
            return None, self.reason
        if (now_steady - self.received > self.timeout or
                not -0.05 <= (now_ros_ns - self.stamp_ns) / 1e9 <= self.timeout):
            return None, '数据过期 / 连接中断'
        return self.values, '实时'
