#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Wrench

import numpy as np
import h5py
import os
import time
import threading


class VRDemoRecorder(Node):
    def __init__(self):
        super().__init__('vr_demo_recorder')

        # =====================
        # Parameters
        # =====================
        self.start_force_th = 10.0
        self.end_force_th   = 10.0
        self.shutdown_force_th = 20.0

        # =====================
        # State
        # =====================
        self.recording = False
        self.lock = threading.Lock()

        self.latest_pose = None     # (6,)
        self.latest_ft   = None     # (3,)

        self.pose_received = False
        self.ft_received   = False

        self.buffer_pose = []
        self.buffer_ft   = []

        # =====================
        # HDF5 path
        # =====================
        timestamp = time.strftime('%m%d_%H%M')
        self.base_dir = f'/home/eunseop/nrs_lab2/datasets/VR_DEMO/{timestamp}'
        os.makedirs(self.base_dir, exist_ok=True)

        self.hdf5_path = os.path.join(self.base_dir, 'vr_demo.hdf5')
        self.get_logger().info(f'Saving to: {self.hdf5_path}')

        # =====================
        # Subscribers
        # =====================
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

        # =====================
        # Main loop (20 Hz)
        # =====================
        self.timer = self.create_timer(0.05, self.main_loop)

    # ==========================================================
    # Callbacks
    # ==========================================================
    def pose_callback(self, msg: PoseStamped):
        with self.lock:
            p = msg.pose.position
            q = msg.pose.orientation

            # RPY 변환
            r, p_, y = self.quaternion_to_rpy(q.x, q.y, q.z, q.w)

            self.latest_pose = np.array(
                [p.x, p.y, p.z, r, p_, y],
                dtype=np.float32
            )
            self.pose_received = True

    def ft_callback(self, msg: Wrench):
        fx = msg.force.x
        fy = msg.force.y
        fz = msg.force.z

        with self.lock:
            self.latest_ft = np.array([fx, fy, fz], dtype=np.float32)
            self.ft_received = True

        # =====================
        # Episode logic
        # =====================
        abs_fx = abs(fx)
        abs_fy = abs(fy)

        # Shutdown
        if abs_fx >= self.shutdown_force_th and abs_fy >= self.shutdown_force_th:
            self.get_logger().warn(
                f'Shutdown condition met: fx={fx:.2f}, fy={fy:.2f}'
            )
            if self.recording:
                self.stop_and_save()
            rclpy.shutdown()
            return

        # Start
        if abs_fx >= self.start_force_th and not self.recording:
            self.start_recording()

        # End
        if abs_fy >= self.end_force_th and self.recording:
            self.stop_and_save()

    # ==========================================================
    # Main loop
    # ==========================================================
    def main_loop(self):
        if not self.recording:
            return

        with self.lock:
            if not (self.pose_received and self.ft_received):
                return

            self.buffer_pose.append(self.latest_pose.copy())
            self.buffer_ft.append(self.latest_ft.copy())

    # ==========================================================
    # Episode control
    # ==========================================================
    def start_recording(self):
        with self.lock:
            self.recording = True
            self.buffer_pose.clear()
            self.buffer_ft.clear()

        self.get_logger().info('=== EPISODE STARTED (|fx| >= 10) ===')

    def stop_and_save(self):
        with self.lock:
            self.recording = False

        self.get_logger().info('=== EPISODE ENDED (|fy| >= 10) ===')
        self.save_to_hdf5()

        with self.lock:
            self.buffer_pose.clear()
            self.buffer_ft.clear()

    # ==========================================================
    # HDF5 Save
    # ==========================================================
    def save_to_hdf5(self):
        if len(self.buffer_pose) < 2:
            self.get_logger().warn('Not enough data to save.')
            return

        pose_arr = np.stack(self.buffer_pose, axis=0)
        ft_arr   = np.stack(self.buffer_ft, axis=0)

        with h5py.File(self.hdf5_path, 'a') as f:
            data_grp = f.require_group('data')
            demo_id = f'demo_{len(data_grp)}'
            demo_grp = data_grp.create_group(demo_id)

            demo_grp.create_dataset('ee_pose', data=pose_arr)
            demo_grp.create_dataset('ft', data=ft_arr)

        self.get_logger().info(
            f'Saved {demo_id} (len={pose_arr.shape[0]})'
        )

    # ==========================================================
    # Utils
    # ==========================================================
    @staticmethod
    def quaternion_to_rpy(x, y, z, w):
        # roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        # yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoRecorder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
