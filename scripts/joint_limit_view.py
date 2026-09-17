"""Read-only joint-limit display model. No ROS, SDK, or control interfaces."""

from dataclasses import dataclass
import math


JOINT_NAMES = [f'Joint{i}_{side}' for side in ('L', 'R') for i in range(1, 8)]


def number(values, key):
    try:
        value = float(values[key])
        return value if math.isfinite(value) else math.nan
    except (KeyError, ValueError, TypeError):
        return math.nan


def degree(values, key):
    return math.degrees(number(values, key))


@dataclass
class JointView:
    name: str
    enabled: bool
    actual: float
    target: float
    hard: tuple
    soft: tuple
    release: tuple
    allowed: tuple
    tolerance: float

    @classmethod
    def from_values(cls, name, values):
        def bounds(prefix):
            return (degree(values, prefix + '_lower_rad'),
                    degree(values, prefix + '_upper_rad'))
        return cls(name, values.get('enabled') == 'true',
                   degree(values, 'feedback_rad'), degree(values, 'command_rad'),
                   bounds('hard'), bounds('soft'), bounds('release'), bounds('safe'),
                   degree(values, 'tolerance_rad'))

    def ranges_valid(self):
        ranges = (self.hard, self.soft, self.release, self.allowed)
        if not all(math.isfinite(v) for pair in ranges for v in pair):
            return False
        return (self.hard[0] <= self.soft[0] <= self.release[0] <
                self.release[1] <= self.soft[1] <= self.hard[1] and
                self.hard[0] <= self.allowed[0] < self.allowed[1] <= self.hard[1])

    def advice(self, feedback_fresh, command_fresh, state):
        if not self.ranges_valid():
            return '限位数据缺失/无效，请重启新版桥接', 'stale'
        if not feedback_fresh or not math.isfinite(self.actual):
            return '反馈不可用：暂停操作，等待反馈恢复', 'stale'
        if not self.enabled:
            return '此臂未启用控制，仅显示位置', 'stale'
        if not command_fresh or not math.isfinite(self.target):
            return '没有新鲜有效目标：检查艾欧指令', 'stale'
        if not math.isfinite(self.tolerance) or self.tolerance <= 0:
            return '对齐容差无效，不能给出调整建议', 'stale'
        # Both the limit-release and alignment conditions must be satisfied.
        lo = max(self.allowed[0], self.actual - self.tolerance)
        hi = min(self.allowed[1], self.actual + self.tolerance)
        if lo > hi:
            return '限位与对齐范围无交集：暂停并检查姿态', 'danger'
        if self.target < lo:
            return f'目标角增大 ↑ 至 ≥ {lo:+.3f}°（至少 +{lo-self.target:.3f}°）', 'adjust'
        if self.target > hi:
            return f'目标角减小 ↓ 至 ≤ {hi:+.3f}°（至少 −{self.target-hi:.3f}°）', 'adjust'
        if state in ('SOFT_LIMIT_HOLD', 'COMMAND_WAIT', 'WAITING_FOR_ALIGNMENT'):
            return '本关节条件满足：保持，等待其余关节及稳定计数', 'ok'
        return '目标在范围内且已对齐', 'ok'


def decode(statuses):
    """Map by explicit names, never message order; duplicates are not displayable."""
    summary, values_by_joint = {}, {}
    duplicates = set()
    for status in statuses:
        values = {entry.key: entry.value for entry in status.values}
        if status.name.endswith('alignment_summary'):
            summary = values
        name = values.get('joint_name')
        if name in JOINT_NAMES:
            if name in values_by_joint:
                duplicates.add(name)
            values_by_joint[name] = values
    rows = [JointView.from_values(name, values_by_joint.get(name, {}))
            if name not in duplicates else JointView.from_values(name, {})
            for name in JOINT_NAMES]
    return summary, rows


def demo():
    """Synthetic example only. Intentionally not the robot's actual configuration."""
    summary = dict(control_state='SOFT_LIMIT_HOLD', feedback_fresh='true',
                   command_fresh='true', aligned_frames='0', required_frames='20')
    rows = []
    for name in JOINT_NAMES:
        rows.append(JointView(name, True, 0.0, 1.0, (-100, 100), (-99, 99),
                              (-98, 98), (-98, 98), 5.73))
    rows[8] = JointView('Joint2_R', True, -95.5, -95.747, (-96.03, 0),
                        (-95.741, -0.286), (-95.169, -0.859),
                        (-95.169, -0.859), 5.73)
    return summary, rows
