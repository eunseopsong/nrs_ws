#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Wrench

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
        self.declare_parameter('file_name', 'demo.txt')
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

        self.latest_pose = None   # (6,)
        self.latest_ft = None     # (3,)

        self.pose_received = False
        self.ft_received = False

        self.buffer = []  # list of (9,)

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(
            PoseStamped,
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
    def pose_callback(self, msg: PoseStamped):
        with self.lock:
            p = msg.pose.position
            q = msg.pose.orientation

            roll, pitch, yaw = self.quaternion_to_rpy(
                q.x, q.y, q.z, q.w
            )

            # meter → mm
            self.latest_pose = np.array([
                p.x * 1000.0,
                p.y * 1000.0,
                p.z * 1000.0,
                roll,
                pitch,
                yaw
            ], dtype=np.float64)

            self.pose_received = True

    def ft_callback(self, msg: Wrench):
        fx = msg.force.x
        fy = msg.force.y
        fz = msg.force.z

        with self.lock:
            self.latest_ft = np.array([fx, fy, fz], dtype=np.float64)
            self.ft_received = True

        # =========================
        # Episode logic
        # =========================
        if self.episode_done:
            return

        # Start
        if abs(fx) >= self.start_force_th and not self.recording:
            self.start_episode()

        # End
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
        self.get_logger().info("Node will shutdown after 1 episode.")
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

    # ======================================================
    # Utils
    # ======================================================
    @staticmethod
    def quaternion_to_rpy(x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2.0
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoTXTRecorder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
