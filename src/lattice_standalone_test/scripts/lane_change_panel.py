#!/usr/bin/env python3.10
"""
Interactive Lane Change Control Panel

Two-button GUI for triggering left/right lane changes in scenario 5
(S-curve highway). Publishes to /lattice_test/lane_change_cmd.

Run alongside the main launch:
  ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=5
  python3.10 scripts/lane_change_panel.py
"""

import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from threading import Thread


class LaneChangePanel(Node):
    def __init__(self):
        super().__init__('lane_change_panel')
        self.pub = self.create_publisher(Int8, '/lattice_test/lane_change_cmd', 5)
        self.get_logger().info('Lane change panel ready — use Left/Right buttons')

    def send_command(self, value):
        msg = Int8()
        msg.data = value
        self.pub.publish(msg)
        direction = 'LEFT  (+3.75m)' if value == 1 else 'RIGHT (-3.75m)'
        self.get_logger().info(f'Lane change command: {direction}')


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = LaneChangePanel()

    # Run ROS spinning in background thread
    thread = Thread(target=ros_spin, args=(node,), daemon=True)
    thread.start()

    # Build tkinter GUI
    root = tk.Tk()
    root.title('Lane Change Control')
    root.geometry('280x150')
    root.resizable(False, False)

    # Make window stay on top
    root.attributes('-topmost', True)

    frame = tk.Frame(root, padx=20, pady=10)
    frame.pack(expand=True, fill='both')

    tk.Label(frame, text='Lattice Lane Change',
             font=('Arial', 14, 'bold')).pack(pady=(0, 10))

    btn_frame = tk.Frame(frame)
    btn_frame.pack()

    btn_left = tk.Button(
        btn_frame, text='←  Left Lane\n(+3.75m)',
        font=('Arial', 11, 'bold'),
        bg='#4CAF50', fg='white',
        width=12, height=2,
        command=lambda: node.send_command(1))
    btn_left.pack(side='left', padx=5)

    btn_right = tk.Button(
        btn_frame, text='Right Lane  →\n(-3.75m)',
        font=('Arial', 11, 'bold'),
        bg='#2196F3', fg='white',
        width=12, height=2,
        command=lambda: node.send_command(-1))
    btn_right.pack(side='left', padx=5)

    tk.Label(frame, text='Scenario 5: S-curve highway',
             font=('Arial', 8), fg='gray').pack(pady=(10, 0))

    root.mainloop()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
