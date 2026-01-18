#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VRDemoTXTRecorder (ROS 2)

[기능]
- /calibrated_pose (Float64MultiArray): [x y z wx wy wz]  (단위: m, rad)
- /ftsensor/measured_Cvalue (geometry_msgs/Wrench): force.x/y/z (단위: N)
를 받아서, episode 단위로 TXT(cmd_continue9D.txt)를 저장.

[Episode rule]
- start: |fx| >= start_force_th
- end  : |fy| >= end_force_th

[저장 전 자동 필터 적용]
- (A) 행 개수는 변하지 않음 (N -> N)
- (B) 첫/끝 행의 pose(x y z wx wy wz)만 원본과 동일하게 고정
- (C) moving average (reflect padding) 적용 (전체 9컬럼)
- (D) fx, fy = 0
- (E) fz는 0~10 saturation (fz<0 ->0, fz>10 ->10)

[사용법]
1) 파라미터(yaml/cli)로 저장 경로/파일명/threshold/window 설정
2) 실행 예:
   ros2 run <your_pkg> vr_demo_txt_recorder --ros-args \
     -p save_dir:=/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd \
     -p file_name:=cmd_continue9D.txt \
     -p start_force_th:=10.0 \
     -p end_force_th:=10.0 \
     -p ma_window:=5

※ upsample 기능은 더 이상 사용하지 않도록 기본값 factor=1로 유지(행 개수 불변 조건 때문).
"""

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

        # moving average window (reflect padded MA)
        self.declare_parameter('ma_window', 5)

        # upsample_factor: 행 개수 불변 요구 때문에 기본 1 유지 권장
        # (1이 아니면 행 개수가 바뀌므로 저장 전에 다시 N으로 되돌려야 해서 의미가 없음)
        self.declare_parameter('upsample_factor', 1)

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
            f"upsample_factor={self.upsample_factor} (recommend 1)"
        )

        # =====================================================
        # State
        # =====================================================
        self.lock = threading.Lock()

        self.recording = False
        self.episode_done = False

        self.latest_pose = None   # [x(mm) y(mm) z(mm) wx wy wz]
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
        # Main loop
        # - 기존 20 Hz에서, 네가 맞춘 125 Hz에 맞추려면 0.008s 권장
        # =====================================================
        self.timer = self.create_timer(0.008, self.main_loop)  # 125 Hz

    # =====================================================
    # Callbacks
    # =====================================================
    def pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return

        with self.lock:
            x_m, y_m, z_m, wx, wy, wz = msg.data[:6]

            # meter -> millimeter
            x = x_m * 1000.0
            y = y_m * 1000.0
            z = z_m * 1000.0

            self.latest_pose = np.array([x, y, z, wx, wy, wz], dtype=np.float64)
            self.pose_received = True

    def ft_callback(self, msg: Wrench):
        fx = float(msg.force.x)
        fy = float(msg.force.y)
        fz = float(msg.force.z)

        with self.lock:
            self.latest_ft = np.array([fx, fy, fz], dtype=np.float64)
            self.ft_received = True

        if self.episode_done:
            return

        # Episode control
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

            row = np.hstack([self.latest_pose, self.latest_ft])  # 9D
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
    # Filtering (same as our latest spec)
    # =====================================================
    @staticmethod
    def moving_average_reflect(data: np.ndarray, w: int) -> np.ndarray:
        """
        Non-causal moving average with reflect padding.
        - data: (N, D)
        - w: window size (>=1)
        Returns same shape (N, D).
        """
        if w <= 1:
            return data.copy()

        N, D = data.shape
        half = w // 2
        if half == 0:
            return data.copy()

        # Need at least 3 samples to reflect meaningfully
        if N < 3:
            return data.copy()

        left = data[1:half + 1][::-1]
        right = data[-2:-half - 2:-1]
        padded = np.vstack([left, data, right])  # (N+2*half, D)

        csum = np.cumsum(padded, axis=0)
        out = np.empty_like(data)

        for i in range(N):
            start = i
            end = i + w - 1
            if start == 0:
                win_sum = csum[end]
            else:
                win_sum = csum[end] - csum[start - 1]
            out[i] = win_sum / float(w)

        return out

    def apply_cmd_filter(self, raw: np.ndarray, window: int) -> np.ndarray:
        """
        raw: (N,9) with columns
          0..8 = x y z wx wy wz fx fy fz

        Constraints:
          - N preserved
          - pose endpoints fixed (cols 0..5) at first/last row
          - fx fy forced 0 for ALL rows
          - fz clamped to [0,10] for ALL rows
        """
        if raw.ndim != 2 or raw.shape[1] != 9 or raw.shape[0] == 0:
            raise RuntimeError(f"Invalid raw shape: {raw.shape}")

        N = raw.shape[0]
        if N < 2:
            # Nothing meaningful; still apply fx/fy/fz rules
            out = raw.copy()
            out[:, 6] = 0.0
            out[:, 7] = 0.0
            out[:, 8] = np.clip(out[:, 8], 0.0, 10.0)
            return out

        # pose endpoints (0..5) to keep fixed
        first_pose = raw[0, 0:6].copy()
        last_pose = raw[-1, 0:6].copy()

        # MA on all columns
        filt = self.moving_average_reflect(raw, window)

        # fx/fy = 0, fz clamp [0,10] (ALL rows)
        filt[:, 6] = 0.0
        filt[:, 7] = 0.0
        filt[:, 8] = np.clip(filt[:, 8], 0.0, 10.0)

        # restore pose endpoints (only pose cols)
        filt[0, 0:6] = first_pose
        filt[-1, 0:6] = last_pose

        return filt

    # =====================================================
    # Optional upsample (NOT recommended for N-invariant requirement)
    # =====================================================
    @staticmethod
    def upsample(data: np.ndarray, factor: int) -> np.ndarray:
        """
        Linear upsample. NOTE: This changes row count (N -> N*factor).
        Kept for compatibility, but NOT recommended under your current spec.
        """
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

        raw = np.vstack(self.buffer)  # (N,9)

        # 1) Apply our command filter (N stays N)
        filtered = self.apply_cmd_filter(raw, self.ma_window)

        # 2) (Optional) Upsampling - generally keep factor=1
        if self.upsample_factor != 1:
            self.get_logger().warn(
                f"upsample_factor={self.upsample_factor} will change row count "
                f"(N->{raw.shape[0]*self.upsample_factor}). Recommended=1."
            )
            filtered_to_save = self.upsample(filtered, self.upsample_factor)
        else:
            filtered_to_save = filtered

        # 3) Save
        np.savetxt(self.file_path, filtered_to_save, fmt="%.10f")

        self.get_logger().info(
            f"Saved TXT episode | raw={raw.shape[0]} → saved={filtered_to_save.shape[0]} steps"
        )
        # quick sanity for pose endpoints if no upsample
        if self.upsample_factor == 1 and raw.shape[0] >= 2:
            same_start_pose = np.allclose(filtered[0, 0:6], raw[0, 0:6], atol=0.0, rtol=0.0)
            same_end_pose = np.allclose(filtered[-1, 0:6], raw[-1, 0:6], atol=0.0, rtol=0.0)
            self.get_logger().info(f"Pose endpoint fixed? start={same_start_pose}, end={same_end_pose}")


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoTXTRecorder()
    rclpy.spin(node)
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
