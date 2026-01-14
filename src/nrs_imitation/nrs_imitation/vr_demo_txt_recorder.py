#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench

import numpy as np
import os
import threading


class VRDemoTXTRecorder(Node):
    def __init__(self):
        super().__init__('vr_demo_txt_recorder')

        # =====================================================
        # Parameters (YAML managed)
        # =====================================================
        self.declare_parameter(
            'save_dir',
            '/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd'
        )
        self.declare_parameter('file_name', 'cmd_continue9D.txt')
        self.declare_parameter('start_force_th', 10.0)
        self.declare_parameter('end_force_th', 10.0)

        self.declare_parameter('ma_window', 5)
        self.declare_parameter('upsample_factor', 10)

        self.save_dir = self.get_parameter('save_dir').value
        self.file_name = self.get_parameter('file_name').value
        self.start_force_th = float(self.get_parameter('start_force_th').value)
        self.end_force_th = float(self.get_parameter('end_force_th').value)
        self.ma_window = int(self.get_parameter('ma_window').value)
        self.upsample_factor = int(self.get_parameter('upsample_factor').value)

        self.file_path = os.path.join(self.save_dir, self.file_name)
        os.makedirs(self.save_dir, exist_ok=True)

        # =====================================================
        # Node start → TXT file initialization
        # =====================================================
        open(self.file_path, 'w').close()
        self.get_logger().info(f"TXT file initialized: {self.file_path}")

        self.get_logger().info(
            f"Params | start_th={self.start_force_th}, "
            f"end_th={self.end_force_th}, "
            f"ma_window={self.ma_window}, "
            f"upsample={self.upsample_factor}x"
        )

        # =====================================================
        # State
        # =====================================================
        self.lock = threading.Lock()

        self.recording = False
        self.episode_done = False

        self.latest_pose = None   # [x(mm) y(mm) z(mm) r p yw]
        self.latest_ft = None     # [fx fy fz]

        self.pose_received = False
        self.ft_received = False

        self.buffer = []          # Nx9 raw buffer

        # =====================================================
        # Subscribers
        # =====================================================
        self.create_subscription(
            Float64MultiArray,
            '/calibrated_pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            Wrench,
            '/ftsensor/measured_Cvalue',
            self.ft_callback,
            10
        )

        # =====================================================
        # Main loop (20 Hz)
        # =====================================================
        self.timer = self.create_timer(0.05, self.main_loop)

    # =====================================================
    # Callbacks
    # =====================================================
    def pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return

        with self.lock:
            x_m, y_m, z_m, r, p, yw = msg.data[:6]

            # 🔴 meter → millimeter 변환
            x = x_m * 1000.0
            y = y_m * 1000.0
            z = z_m * 1000.0

            self.latest_pose = np.array(
                [x, y, z, r, p, yw],
                dtype=np.float64
            )
            self.pose_received = True

    def ft_callback(self, msg: Wrench):
        fx = msg.force.x
        fy = msg.force.y
        fz = msg.force.z

        with self.lock:
            self.latest_ft = np.array([fx, fy, fz], dtype=np.float64)
            self.ft_received = True

        if self.episode_done:
            return

        # ===============================
        # Episode control
        # ===============================
        if abs(fx) >= self.start_force_th and not self.recording:
            self.start_episode()

        if abs(fy) >= self.end_force_th and self.recording:
            self.end_episode()

    # =====================================================
    # Main loop
    # =====================================================
    def main_loop(self):
        if not self.recording or self.episode_done:
            return

        with self.lock:
            if not (self.pose_received and self.ft_received):
                return

            row = np.hstack([self.latest_pose, self.latest_ft])
            self.buffer.append(row)

    # =====================================================
    # Episode control
    # =====================================================
    def start_episode(self):
        with self.lock:
            self.recording = True
            self.buffer.clear()

        self.get_logger().info("=== EPISODE STARTED (|fx| >= threshold) ===")

    def end_episode(self):
        with self.lock:
            self.recording = False
            self.episode_done = True

        self.get_logger().info("=== EPISODE ENDED (|fy| >= threshold) ===")
        self.save_txt()
        self.get_logger().info("Recorder finished. Shutting down.")
        rclpy.shutdown()

    # =====================================================
    # Processing utilities
    # =====================================================
    def moving_average(self, data, window):
        if window <= 1:
            return data

        filt = np.zeros_like(data)
        kernel = np.ones(window) / window

        for i in range(data.shape[1]):
            filt[:, i] = np.convolve(data[:, i], kernel, mode='same')

        return filt

    def upsample(self, data, factor):
        if factor <= 1:
            return data

        N, D = data.shape
        t = np.arange(N)
        t_new = np.linspace(0, N - 1, N * factor)

        up = np.zeros((len(t_new), D))
        for i in range(D):
            up[:, i] = np.interp(t_new, t, data[:, i])

        return up

    # =====================================================
    # Save
    # =====================================================
    def save_txt(self):
        if len(self.buffer) == 0:
            self.get_logger().warn("No data to save.")
            return

        raw = np.vstack(self.buffer)

        # 1) Moving average
        filtered = self.moving_average(raw, self.ma_window)

        # 2) Upsampling
        upsampled = self.upsample(filtered, self.upsample_factor)

        # 3) Save
        np.savetxt(
            self.file_path,
            upsampled,
            fmt="%.10f"
        )

        self.get_logger().info(
            f"Saved TXT episode | raw={raw.shape[0]} → "
            f"saved={upsampled.shape[0]} steps"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoTXTRecorder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
