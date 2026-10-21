#!/usr/bin/python3
"""Live joint-limit window. Subscribes to diagnostics only; never sends control."""

import argparse
import math
import sys
import time
import tkinter as tk
from tkinter import ttk

from joint_limit_view import decode, demo


COLORS = {'ok': '#19704a', 'adjust': '#a34b00', 'danger': '#b82f39', 'stale': '#68717d'}


class LimitWindow:
    def __init__(self, root, demo_mode=False):
        self.root = root
        self.demo_mode = demo_mode
        self.summary, self.rows = {}, []
        self.received = None
        self.stamp_valid = False
        # Pixel fonts keep chart geometry legible/aligned on high-DPI Linux desktops.
        ttk.Style(root).configure('TLabel', font=('sans-serif', -14))
        root.title('天机关节位置与限位 — 只读监视')
        root.geometry(f'{min(1200, root.winfo_screenwidth()-60)}x'
                      f'{min(570, root.winfo_screenheight()-100)}')
        root.minsize(520, 320)
        self.legend = ttk.Label(root, padding=10, text=(
            '● 实机    ▼ 有效目标    红线：桥接保护边界    绿区：允许/恢复范围    单位 °'
        ))
        self.legend.pack(fill='x')
        body = ttk.Frame(root)
        body.pack(fill='both', expand=True)
        scroll = ttk.Scrollbar(body, orient='vertical')
        scroll.pack(side='right', fill='y')
        self.canvas = tk.Canvas(body, background='#f7f9fc', highlightthickness=0,
                                yscrollcommand=scroll.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        scroll.config(command=self.canvas.yview)
        self.canvas.bind('<Button-4>', lambda _: self.canvas.yview_scroll(-2, 'units'))
        self.canvas.bind('<Button-5>', lambda _: self.canvas.yview_scroll(2, 'units'))

    def accept(self, message, stamp_age):
        self.summary, self.rows = decode(message.status)
        self.received = time.monotonic()
        self.stamp_valid = math.isfinite(stamp_age) and -0.1 <= stamp_age <= 0.5

    def draw(self):
        c = self.canvas
        c.delete('all')
        width = max(c.winfo_width(), 490)
        self.legend.config(wraplength=width-20)
        age = math.inf if self.received is None else time.monotonic() - self.received
        live = self.demo_mode or (age <= 0.5 and self.stamp_valid)
        state = self.summary.get('control_state', 'UNKNOWN')
        fb = live and self.summary.get('feedback_fresh') == 'true'
        cmd = live and self.summary.get('command_fresh') == 'true'
        banner = '演示数据（非实机）' if self.demo_mode else (
            '数据过期 / 未连接' if not live else ('反馈不可用' if not fb else '实时'))
        if live and self.rows and any(not row.ranges_valid() for row in self.rows):
            banner += ' | 限位数据缺失'
        self.root.title(f'关节限位 · 只读 | {banner} | {state}')
        columns = 2 if width >= 1000 else 1
        col_width = width / columns
        row_height = 70
        for index, row in enumerate(self.rows):
            col = index // 7 if columns == 2 else 0
            row_index = index % 7 if columns == 2 else index
            x = col * col_width + 16
            y = row_index * row_height + 12
            w = col_width - 32
            self.draw_row(row, x, y, w, fb, cmd, live)
        c.config(scrollregion=(0, 0, width, (7 if columns == 2 else 14) * row_height + 20))

    def draw_row(self, row, x, y, width, fb, cmd, live):
        c = self.canvas
        def text(dx, dy, value, color='#263448', bold=False):
            c.create_text(x+dx, y+dy, text=value, fill=color, anchor='nw',
                          font=('sans-serif', -13, 'bold' if bold else 'normal'))
        def angle(value, fresh):
            return f'{value:+.2f}°' if fresh and math.isfinite(value) else '—'
        text(0, 8, row.name, '#263448' if row.enabled else COLORS['stale'], bold=True)
        text(0, 29, angle(row.actual, fb), '#1665b5' if fb else COLORS['stale'])
        left, right, cy = x+112, x+width-14, y+22
        c.create_rectangle(left, cy-5, right, cy+5, fill='#e4e8ee', outline='')
        if row.ranges_valid():
            bounds = row.display_bounds()
            text(left-x, 38, f'{bounds[0]:+.2f}°', COLORS['stale'])
            c.create_text(right, y+38, text=f'{bounds[1]:+.2f}°',
                          fill=COLORS['stale'], anchor='ne', font=('sans-serif', -13))
            def px(value):
                ratio = (value-bounds[0]) / (bounds[1]-bounds[0])
                return left + max(0, min(1, ratio)) * (right-left)
            c.create_rectangle(px(row.release[0]), cy-4, px(row.release[1]), cy+4,
                               fill='#b8dfc7' if fb else '#d5d9df', outline='')
            for limit in row.soft:
                c.create_line(px(limit), cy-8, px(limit), cy+8,
                              fill='#b82f39' if live else '#68717d', width=2)
            if fb and math.isfinite(row.actual):
                a = px(row.actual)
                c.create_oval(a-4, cy-2, a+4, cy+6, fill='#1665b5', outline='')
            if cmd and math.isfinite(row.target):
                t = px(row.target)
                c.create_polygon(t-5, cy-13, t+5, cy-13, t, cy-5,
                                 fill='#b85b00', outline='')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='synthetic offline preview; no ROS')
    args, ros_args = parser.parse_known_args()
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f'无法打开窗口：{error}\n请在图形桌面的终端运行，或使用 alignment_dashboard。',
              file=sys.stderr)
        return 1
    window = LimitWindow(root, args.demo)
    node = None
    if args.demo:
        window.summary, window.rows = demo()
    else:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        rclpy.init(args=ros_args)
        node = rclpy.create_node('tianji_joint_limit_dashboard')
        topic = node.declare_parameter('diagnostics_topic',
                                      '/tianji_marvin_teleop_bridge/alignment_status').value
        def on_diagnostics(message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            window.accept(message, node.get_clock().now().nanoseconds / 1e9 - stamp)
        node.create_subscription(DiagnosticArray, topic, on_diagnostics,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def tick():
        if node is not None:
            if not rclpy.ok():
                root.quit()
                return
            rclpy.spin_once(node, timeout_sec=0)
        window.draw()
        root.after(100, tick)

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
