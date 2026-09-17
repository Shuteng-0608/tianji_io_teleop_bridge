#!/usr/bin/python3

import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


JOINT_NAMES = [
    *(f'Joint{i}_L' for i in range(1, 8)),
    *(f'Joint{i}_R' for i in range(1, 8)),
]


class AlignmentDashboard(Node):
    def __init__(self):
        super().__init__('tianji_alignment_dashboard')
        topic = self.declare_parameter(
            'diagnostics_topic',
            '/tianji_marvin_teleop_bridge/alignment_status',
        ).value
        self._use_color = bool(
            self.declare_parameter('use_color', True).value and sys.stdout.isatty()
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._latest = None
        self._received_at = 0.0
        self._last_plain_signature = None
        self.create_subscription(DiagnosticArray, topic, self._on_diagnostics, qos)
        self.create_timer(0.1, self._render)
        self.get_logger().info(f'Alignment dashboard listening on {topic}')

    def _on_diagnostics(self, message):
        self._latest = message
        self._received_at = time.monotonic()

    @staticmethod
    def _values(status):
        return {entry.key: entry.value for entry in status.values}

    def _paint(self, text, code):
        if not self._use_color:
            return text
        return f'\033[{code}m{text}\033[0m'

    def _status_label(self, message):
        labels = {
            'ALIGNED': ('✓ ALIGNED', '32'),
            'NOT_ALIGNED': ('✗ ADJUST', '31'),
            'OUTSIDE_SAFE_LIMIT': ('! LIMIT', '33;1'),
            'NO_FRESH_COMMAND': ('? NO CMD', '35'),
            'NO_FRESH_FEEDBACK': ('? NO FB', '35'),
            'DISABLED': ('— OFF', '90'),
        }
        label, color = labels.get(message, (message[:10], '33'))
        return self._paint(f'{label:<10}', color)

    def _error_bar(self, error, tolerance, message, width=12):
        if not math.isfinite(error) or not math.isfinite(tolerance) or tolerance <= 0.0:
            return ' ' * width
        ratio = min(error / tolerance, 1.0)
        filled = int(round(ratio * width))
        bar = '█' * filled + '·' * (width - filled)
        if message == 'ALIGNED':
            color = '32'
        elif message == 'DISABLED':
            color = '90'
        else:
            color = '31'
        return self._paint(bar, color)

    def _render(self):
        if self._latest is None:
            if self._use_color:
                sys.stdout.write('\033[2J\033[H')
                sys.stdout.write('等待 /tianji_marvin_teleop_bridge/alignment_status ...\n')
                sys.stdout.flush()
            return

        summary_status = next(
            (item for item in self._latest.status if item.name.endswith('alignment_summary')),
            None,
        )
        joint_statuses = {}
        for item in self._latest.status:
            values = self._values(item)
            joint_name = values.get('joint_name')
            if joint_name:
                joint_statuses[joint_name] = (item, values)

        if summary_status is None:
            return
        summary = self._values(summary_status)
        age = time.monotonic() - self._received_at
        signature = (
            summary_status.message,
            summary.get('aligned_joints'),
            summary.get('aligned_frames'),
            tuple(
                joint_statuses[name][0].message if name in joint_statuses else 'MISSING'
                for name in JOINT_NAMES
            ),
        )
        if not self._use_color and signature == self._last_plain_signature:
            return
        self._last_plain_signature = signature

        if self._use_color:
            sys.stdout.write('\033[2J\033[H')
        state = summary.get('control_state', 'UNKNOWN')
        aligned = summary.get('aligned_joints', '0')
        enabled = summary.get('enabled_joints', '0')
        frames = summary.get('aligned_frames', '0')
        required_frames = summary.get('required_frames', '?')
        worst_joint = summary.get('maximum_error_joint', 'none')
        worst_deg = summary.get('maximum_error_deg', '-')
        reason = summary_status.message
        reason_color = '32' if reason == 'ALIGNED' else '33;1'

        lines = [
            self._paint('天机双臂遥操作 — 启动/恢复对齐面板', '36;1'),
            (
                f'状态: {state:<22}  结果: {self._paint(reason, reason_color)}  '
                f'诊断延迟: {age:0.2f}s'
            ),
            (
                f'已对齐关节: {aligned}/{enabled}  稳定帧: {frames}/{required_frames}  '
                f'最大误差: {worst_joint} ({worst_deg}°)'
            ),
            '',
            '关节       状态          目标(°)    实机(°)    误差(°)    误差/容差',
            '─' * 78,
        ]

        for index, name in enumerate(JOINT_NAMES):
            if index == 7:
                lines.extend(['', '─' * 78])
            if name not in joint_statuses:
                lines.append(f'{name:<10} {self._paint("? MISSING", "35")}')
                continue
            status, values = joint_statuses[name]
            try:
                command_deg = float(values.get('command_deg', 'nan'))
                feedback_deg = float(values.get('feedback_deg', 'nan'))
                error_deg = float(values.get('absolute_error_deg', 'nan'))
                error_rad = float(values.get('absolute_error_rad', 'nan'))
                tolerance_rad = float(values.get('tolerance_rad', 'nan'))
            except ValueError:
                command_deg = feedback_deg = error_deg = error_rad = tolerance_rad = math.nan
            lines.append(
                f'{name:<10} {self._status_label(status.message)}  '
                f'{command_deg:>9.2f}  {feedback_deg:>9.2f}  {error_deg:>9.2f}    '
                f'{self._error_bar(error_rad, tolerance_rad, status.message)}'
            )

        lines.extend(
            [
                '',
                '绿色=已对齐  红色=需要调整  黄色=限位  灰色=未启用',
                '调整操作端姿态，使所有启用关节变绿并保持到稳定帧计满。Ctrl-C 退出面板。',
            ]
        )
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
