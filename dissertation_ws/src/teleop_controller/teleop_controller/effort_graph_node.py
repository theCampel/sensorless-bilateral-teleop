#!/usr/bin/env python3
"""
Effort Graph Node - Real-time scrolling graph of follower motor effort (mA).

Highlights when effort exceeds the force feedback threshold and logs to CSV.
"""

import csv
import os
import threading
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle']


class EffortGraphNode(Node):
    def __init__(self):
        super().__init__('effort_graph_node')

        self.declare_parameter('follower_name', 'follower_left')
        self.declare_parameter('effort_threshold', 300.0)
        self.declare_parameter('window_seconds', 10.0)
        self.declare_parameter('save_csv', True)
        self.declare_parameter('show_per_joint', True)

        self.follower_name = self.get_parameter('follower_name').value
        self.threshold = self.get_parameter('effort_threshold').value
        self.window_seconds = self.get_parameter('window_seconds').value
        self.save_csv = self.get_parameter('save_csv').value
        self.show_per_joint = self.get_parameter('show_per_joint').value

        max_samples = int(self.window_seconds * 100)
        self.times = deque(maxlen=max_samples)
        self.max_efforts = deque(maxlen=max_samples)
        self.per_joint = {j: deque(maxlen=max_samples) for j in ARM_JOINTS}
        self.start_time = None
        self.lock = threading.Lock()

        self.csv_file = None
        self.csv_writer = None
        if self.save_csv:
            log_dir = os.path.expanduser('~/dissertation26/effort_logs')
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_path = os.path.join(log_dir, f'effort_{self.follower_name}_{ts}.csv')
            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['time_s', 'max_effort'] + ARM_JOINTS)
            self.get_logger().info(f'CSV: {self.csv_path}')

        self.create_subscription(
            JointState, f'/{self.follower_name}/joint_states',
            self._joint_state_cb, 10
        )

        self.get_logger().info(
            f'Effort graph: {self.follower_name} | '
            f'threshold={self.threshold:.0f} mA | window={self.window_seconds:.0f}s'
        )

    def _joint_state_cb(self, msg):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        t = (now - self.start_time).nanoseconds / 1e9

        efforts = {}
        for i, name in enumerate(msg.name):
            if name in ARM_JOINTS and i < len(msg.effort):
                efforts[name] = abs(msg.effort[i])

        if not efforts:
            return

        max_e = max(efforts.values())

        with self.lock:
            self.times.append(t)
            self.max_efforts.append(max_e)
            for j in ARM_JOINTS:
                self.per_joint[j].append(efforts.get(j, 0.0))

        if self.csv_writer:
            self.csv_writer.writerow(
                [f'{t:.3f}', f'{max_e:.1f}'] +
                [f'{efforts.get(j, 0.0):.1f}' for j in ARM_JOINTS]
            )

    def shutdown(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'Data saved: {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = EffortGraphNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    BG = '#0d1117'
    GRID = '#21262d'
    TEXT = '#c9d1d9'
    BLUE = '#58a6ff'
    RED = '#f85149'
    THRESH_CLR = '#8b949e'
    JOINT_CLR = {
        'waist': '#7ee787',
        'shoulder': '#d2a8ff',
        'elbow': '#ffa657',
        'wrist_angle': '#79c0ff',
    }

    plt.rcParams.update({
        'font.family': 'monospace',
        'font.size': 11,
        'text.color': TEXT,
        'axes.labelcolor': TEXT,
        'xtick.color': TEXT,
        'ytick.color': TEXT,
    })

    fig, ax = plt.subplots(figsize=(14, 5))
    fig.canvas.manager.set_window_title(f'Effort Monitor — {node.follower_name}')
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    fig.subplots_adjust(left=0.07, right=0.95, top=0.88, bottom=0.15)

    ax.axhline(y=node.threshold, color=THRESH_CLR, linestyle='--',
               linewidth=1, alpha=0.6, zorder=2)
    ax.text(0.002, node.threshold + 12,
            f'feedback threshold ({node.threshold:.0f} mA)',
            transform=ax.get_yaxis_transform(),
            color=THRESH_CLR, fontsize=9, alpha=0.6)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Effort (mA)')
    ax.set_ylim(0, 700)
    ax.set_xlim(-node.window_seconds, 0)
    ax.grid(True, color=GRID, linewidth=0.5, alpha=0.4)
    for spine in ['top', 'right']:
        ax.spines[spine].set_visible(False)
    for spine in ['left', 'bottom']:
        ax.spines[spine].set_color(GRID)

    ax.set_title(f'{node.follower_name} — motor effort',
                 fontsize=13, color=TEXT, loc='left', pad=10)

    value_text = ax.text(0.98, 0.93, '— mA', transform=ax.transAxes,
                         fontsize=28, fontweight='bold', ha='right', va='top',
                         color=BLUE, zorder=10)
    status_text = ax.text(0.98, 0.76, '', transform=ax.transAxes,
                          fontsize=11, ha='right', va='top',
                          color=RED, alpha=0.9, zorder=10)

    main_line, = ax.plot([], [], color=BLUE, linewidth=2, zorder=5)

    joint_lines = {}
    if node.show_per_joint:
        for j in ARM_JOINTS:
            jl, = ax.plot([], [], color=JOINT_CLR[j], linewidth=0.8,
                          alpha=0.35, zorder=3, label=j)
            joint_lines[j] = jl
        ax.legend(loc='upper left', fontsize=8, framealpha=0.3,
                  edgecolor=GRID, facecolor=BG)

    def update(_frame):
        with node.lock:
            if len(node.times) < 2:
                return (main_line, value_text, status_text)
            times = np.array(node.times)
            efforts = np.array(node.max_efforts)
            jdata = {}
            if node.show_per_joint:
                for j in ARM_JOINTS:
                    jdata[j] = np.array(node.per_joint[j])

        rel = times - times[-1]

        main_line.set_data(rel, efforts)
        for j, jl in joint_lines.items():
            jl.set_data(rel, jdata[j])

        while ax.collections:
            ax.collections[0].remove()

        ax.fill_between(rel, 0, efforts,
                        where=efforts <= node.threshold,
                        color=BLUE, alpha=0.12, interpolate=True, zorder=1)

        ax.fill_between(rel, 0, efforts,
                        where=efforts > node.threshold,
                        color=RED, alpha=0.2, interpolate=True, zorder=1)
        ax.fill_between(rel, node.threshold, efforts,
                        where=efforts > node.threshold,
                        color=RED, alpha=0.25, interpolate=True, zorder=1)

        curr = efforts[-1]
        value_text.set_text(f'{curr:.0f} mA')
        if curr > node.threshold:
            value_text.set_color(RED)
            status_text.set_text('FEEDBACK ACTIVE')
        else:
            value_text.set_color(BLUE)
            status_text.set_text('')

        peak = np.max(efforts)
        ax.set_ylim(0, max(700, peak * 1.25))
        ax.set_xlim(-node.window_seconds, 0)

        return (main_line, value_text, status_text)

    _anim = FuncAnimation(fig, update, interval=33, blit=False,
                          cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
