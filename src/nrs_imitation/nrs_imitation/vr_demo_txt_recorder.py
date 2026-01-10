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

        # =========================
        # Parameters (YAML)
        # =========================
        self.declare_parameter('save_dir', '/tmp')
        self.declare_parameter('file_name', 'demo_episode.txt')
        self.declare_parameter('start_force_th', 10.0)
        self.declare_parameter('end_force_th', 10.0)

        self.save_dir = self.get_parameter('save_dir').value
        self.file_name = self.get_parameter('file_name').value
        self.start_force_th = float(self.get_parameter('start_force_th').value)
        self.end_force_th = float(self.get_parameter('end_force_th').value)

        os.makedirs(self.save_dir, exist_ok=True)
        self.file_path = os.path.join(self.save_dir, self.file_name)

        self.get_logger().info(f"TXT save path: {self.file_path}")

        # =========================
        # State
        # =========================
        self.lock = threading.Lock()

        self.recording = False
        self.episode_done = False

        self.latest_pose = None   # (6,) -> x y z r p yw
        self.latest_ft = None     # (3,) -> fx fy fz

        self.pose_received = False
        self.ft_received = False

        self.buffer = []          # list of (9,)

        # =========================
        # Subscribers
        # =========================
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

        # =========================
        # Main loop (20 Hz)
        # =========================
        self.timer = self.create_timer(0.05, self.main_loop)

    # ======================================================
    # Callbacks
    # ======================================================
    def pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn("calibrated_pose length < 6, ignored")
            return

        with self.lock:
            # 이미 mm / rad 단위라고 가정
            x, y, z, r, p, yw = msg.data[:6]

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

        # =========================
        # Episode logic
        # =========================
        # Start condition
        if abs(fx) >= self.start_force_th and not self.recording:
            self.start_episode()

        # End condition
        if abs(fy) >= self.end_force_th and self.recording:
            self.end_episode()

    # ======================================================
    # Main loop
    # ======================================================
    def main_loop(self):
        if not self.recording or self.episode_done:
            return

        with self.lock:
            if not (self.pose_received and self.ft_received):
                return

            row = np.hstack([self.latest_pose, self.latest_ft])
            self.buffer.append(row)

    # ======================================================
    # Episode control
    # ======================================================
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

    # ======================================================
    # Save
    # ======================================================
    def save_txt(self):
        if len(self.buffer) == 0:
            self.get_logger().warn("No data to save.")
            return

        data = np.vstack(self.buffer)

        np.savetxt(
            self.file_path,
            data,
            fmt="%.10f"
        )

        self.get_logger().info(
            f"Saved TXT episode: {data.shape[0]} steps"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoTXTRecorder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
