#!/usr/bin/python3
"""Right-arm joint disturbance estimate window. Subscriptions only; no control or tare calls."""

import argparse
import math
import sys
import time
import tkinter as tk

from right_joint_torque_view import RIGHT_JOINTS, TorqueView, format_torque


BG = '#101923'
PANEL = '#192634'
TEXT = '#edf3fa'
MUTED = '#9dafc3'
POSITIVE = '#64d5c4'
NEGATIVE = '#7bb6ff'


class TorqueWindow:
    def __init__(self, root, source, raw=False, demo=False, bar_range=2.0):
        self.root, self.raw, self.demo = root, raw, demo
        self.bar_range = bar_range
        self.model = TorqueView()
        self.zero_active, self.zero_received, self.zero_stamp_ns = None, None, None
        root.title('右臂关节扰动力矩估计 · 只读监测')
        root.geometry('820x730')
        root.minsize(680, 670)
        root.configure(bg=BG)
        tk.Label(root, text='右臂 · 关节扰动力矩估计', bg=BG, fg=TEXT,
                 font=('sans-serif', -25, 'bold'), anchor='w').pack(fill='x', padx=26, pady=(24, 5))
        tk.Label(root, text='控制器原始估计' if raw else '逐关节去偏置后的估计',
                 bg=BG, fg=MUTED, font=('sans-serif', -14), anchor='w').pack(fill='x', padx=27)
        self.status = tk.Label(root, text='等待数据', bg=BG, fg=MUTED,
                               font=('sans-serif', -14), anchor='w')
        self.status.pack(fill='x', padx=27, pady=(12, 8))
        self.canvas = tk.Canvas(root, bg=BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, padx=18)
        tk.Label(root, text=f'条形范围 ±{bar_range:.3f} N·m，仅用于显示；超范围仍显示完整数值。',
                 bg=BG, fg=MUTED, font=('sans-serif', -12), anchor='w').pack(fill='x', padx=27)
        tk.Label(root, text='保留三位小数 · 无额外滤波 · 不改变控制、去皮或记录数据',
                 bg=BG, fg=MUTED, font=('sans-serif', -12), anchor='w').pack(fill='x', padx=27, pady=5)
        self.source = tk.Label(root, text=source, bg=BG, fg=MUTED,
                               font=('monospace', -11), anchor='w')
        self.source.pack(fill='x', padx=27, pady=(0, 18))

    def accept_zero(self, message, now_ros_ns):
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if not -0.05 <= (now_ros_ns - stamp_ns) / 1e9 <= 2.5:
            self.zero_active = None
            return
        right = [s for s in message.status if s.name == 'right']
        fields = {v.key: v.value for v in right[0].values} if len(right) == 1 else {}
        active = fields.get('active')
        self.zero_active = {'true': True, 'false': False}.get(active)
        self.zero_received, self.zero_stamp_ns = time.monotonic(), stamp_ns

    def draw(self, now_ros_ns):
        now = time.monotonic()
        values, state = self.model.current(now_ros_ns, now)
        if self.demo:
            values = tuple((i + 1) * 0.18 * math.sin(now * 0.8 + i) for i in range(7))
            state = '演示数据 · 非实机'
        elif not self.raw:
            if (self.zero_active is None or self.zero_received is None or
                    now - self.zero_received > 2.5 or
                    not -0.05 <= (now_ros_ns - self.zero_stamp_ns) / 1e9 <= 2.5):
                state += '  |  去皮状态未知'
            else:
                state += '  |  右臂已去皮' if self.zero_active else '  |  尚未去皮：当前等于原始估计'
        self.status.config(text=state, fg=POSITIVE if values is not None else '#efbd74')
        c = self.canvas
        c.delete('all')
        width, height = max(640, c.winfo_width()), max(450, c.winfo_height())
        row_h = height / 7
        left, right = 166, width - 210
        center, extent = (left + right) / 2, (right - left) / 2
        for i, name in enumerate(RIGHT_JOINTS):
            top, cy = i * row_h + 4, (i + 0.5) * row_h
            c.create_rectangle(4, top, width - 4, (i + 1) * row_h - 4, fill=PANEL, outline='')
            c.create_text(22, cy, text=name, anchor='w', fill=TEXT, font=('monospace', -17))
            c.create_line(left, cy, right, cy, fill='#304254', width=8)
            c.create_line(center, cy - 13, center, cy + 13, fill=MUTED)
            value = values[i] if values is not None else math.nan
            color = POSITIVE if value >= 0 else NEGATIVE
            if math.isfinite(value):
                end = center + max(-1, min(1, value / self.bar_range)) * extent
                c.create_line(center, cy, end, cy, fill=color, width=8)
                if abs(value) > self.bar_range:
                    c.create_text(right, cy + 19, text='超显示范围', anchor='e',
                                  fill=MUTED, font=('sans-serif', -10))
            c.create_text(width - 73, cy, text=format_torque(value), anchor='e',
                          fill=color if values is not None else MUTED, font=('monospace', -25, 'bold'))
            c.create_text(width - 23, cy, text='N·m', anchor='e', fill=MUTED, font=('sans-serif', -13))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', action='store_true',
                        help='show raw joint disturbance estimate instead of bias-corrected estimate')
    parser.add_argument('--demo', action='store_true', help='offline synthetic preview; no ROS')
    parser.add_argument('--range', type=float, default=2.0, dest='bar_range', help='bar half-range in Nm; numbers are never clipped')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.bar_range) or args.bar_range <= 0:
        parser.error('--range must be finite and positive')
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f'无法打开窗口：{error}\n请在图形桌面的终端运行。', file=sys.stderr)
        return 1
    topic = ('/io_teleop/joint_states_raw' if args.raw else
             '/tianji_marvin_teleop_bridge/estimated_joint_disturbance')
    window = TorqueWindow(root, topic, args.raw, args.demo, args.bar_range)
    node = None
    if not args.demo:
        import rclpy
        from sensor_msgs.msg import JointState
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        rclpy.init(args=ros_args)
        node = rclpy.create_node('tianji_right_joint_torque_dashboard')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        def receive(message):
            stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            window.model.accept(message.name, message.effort, stamp_ns,
                                node.get_clock().now().nanoseconds, time.monotonic())

        subscription = node.create_subscription(JointState, topic, receive, qos)
        window.source.config(text=subscription.topic_name)
        node.get_logger().info('Right-arm torque window (read-only): ' + subscription.topic_name)
        if not args.raw:
            node.create_subscription(DiagnosticArray,
                '/tianji_marvin_teleop_bridge/joint_torque_zero_status',
                lambda m: window.accept_zero(m, node.get_clock().now().nanoseconds), qos)

    def tick():
        if node is not None:
            if not rclpy.ok():
                root.quit()
                return
            # Bounded work per refresh; consume latest state and calibration metadata.
            for _ in range(2):
                rclpy.spin_once(node, timeout_sec=0)
        window.draw(node.get_clock().now().nanoseconds if node else time.time_ns())
        root.after(50, tick)

    root.after(0, tick)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        try:
            root.destroy()
        except tk.TclError:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
