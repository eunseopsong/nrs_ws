#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from scipy.ndimage import gaussian_filter1d
import json

# ================================ #
#      Kalman EM 알고리즘 함수     #
# ================================ #
def kalman_em_diff_estimate(observed_data, max_iters=100, tol=1e-4):
    T, dim = observed_data.shape
    Q = np.eye(dim) * 1e-5
    R = np.eye(dim) * 1e-2
    x_init = observed_data[0]
    P_init = np.eye(dim)
    A = np.eye(dim)
    C = np.eye(dim)

    log_likelihood_old = -np.inf
    for _ in range(max_iters):
        x_pred = np.zeros((T, dim))
        P_pred = np.zeros((T, dim, dim))
        x_filt = np.zeros((T, dim))
        P_filt = np.zeros((T, dim, dim))
        K_gain = np.zeros((T, dim, dim))

        x_pred[0] = x_init
        P_pred[0] = P_init

        for t in range(T):
            if t > 0:
                x_pred[t] = A @ x_filt[t - 1]
                P_pred[t] = A @ P_filt[t - 1] @ A.T + Q
            S = C @ P_pred[t] @ C.T + R
            K_gain[t] = P_pred[t] @ C.T @ np.linalg.inv(S)
            x_filt[t] = x_pred[t] + K_gain[t] @ (observed_data[t] - C @ x_pred[t])
            P_filt[t] = (np.eye(dim) - K_gain[t] @ C) @ P_pred[t]

        x_smooth = np.copy(x_filt)
        P_smooth = np.copy(P_filt)

        for t in range(T - 2, -1, -1):
            J = P_filt[t] @ A.T @ np.linalg.inv(P_pred[t + 1])
            x_smooth[t] += J @ (x_smooth[t + 1] - x_pred[t + 1])
            P_smooth[t] += J @ (P_smooth[t + 1] - P_pred[t + 1]) @ J.T

        Exx = np.zeros((dim, dim))
        Eyy = np.zeros((dim, dim))
        Exy = np.zeros((dim, dim))

        for t in range(T):
            xt = x_smooth[t][:, np.newaxis]
            Exx += xt @ xt.T + P_smooth[t]
            yt = observed_data[t][:, np.newaxis]
            Eyy += yt @ yt.T
            Exy += xt @ yt.T

        Q_new = np.zeros_like(Q)
        for t in range(1, T):
            xt = x_smooth[t][:, np.newaxis]
            xtm1 = x_smooth[t - 1][:, np.newaxis]
            Q_new += (xt - xtm1) @ (xt - xtm1).T + P_smooth[t] + P_smooth[t - 1]
        Q = Q_new / (T - 1)

        R = (Eyy - Exy.T @ np.linalg.inv(Exx) @ Exy) / T

        log_likelihood = -0.5 * T * (dim * np.log(2 * np.pi) + np.log(np.linalg.det(R)))
        if np.abs(log_likelihood - log_likelihood_old) < tol:
            break
        log_likelihood_old = log_likelihood

    return x_smooth


# ================================ #
#       ROS2 노드 클래스 정의       #
# ================================ #
class TicEMKMNode(Node):
    def __init__(self):
        super().__init__('tic_emkm_node')
        self.get_logger().info("✅ TIC KalmanEM Node 시작")

        # 파일 경로 설정
        current_dir = os.path.dirname(os.path.realpath(__file__))
        data_path = os.path.join(current_dir, '..', '..', 'data', 'TIC', '2')
        file_rot = os.path.join(data_path, 'TIC_input_rot.txt')
        file_ft = os.path.join(data_path, 'TIC_output_ft.txt')

        try:
            raw_rot = np.loadtxt(file_rot)[28:, :]
            raw_ft = np.loadtxt(file_ft)[28:, :]
        except Exception as e:
            raise RuntimeError(f"❌ 파일 로딩 실패: {e}")

        input_min = raw_rot.min(axis=0)
        input_max = raw_rot.max(axis=0)
        target_min = raw_ft.min(axis=0)
        target_max = raw_ft.max(axis=0)

        def normalize(value, min_val, max_val):
            return 2 * ((value - min_val) / (max_val - min_val)) - 1

        def denormalize(value, min_val, max_val):
            return ((value + 1) / 2) * (max_val - min_val) + min_val

        input_norm = normalize(raw_rot, input_min, input_max)
        target_norm = normalize(raw_ft, target_min, target_max)

        filtered_norm = kalman_em_diff_estimate(target_norm)
        filtered_force = denormalize(filtered_norm[:, 0:3], target_min[:3], target_max[:3])
        filtered_moment = denormalize(filtered_norm[:, 3:6], target_min[3:], target_max[3:])

        # Validation: Derivative + Smoothing
        dt = 0.01
        F_pre = np.zeros(3)
        M_pre = np.zeros(3)
        F_deri = []
        M_deri = []

        for f, m in zip(filtered_force, filtered_moment):
            F_deri.append((f - F_pre) / dt)
            M_deri.append((m - M_pre) / dt)
            F_pre = f
            M_pre = m

        F_deri = np.array(F_deri)
        M_deri = np.array(M_deri)

        F_smooth = gaussian_filter1d(F_deri, sigma=5, axis=0)
        M_smooth = gaussian_filter1d(M_deri, sigma=5, axis=0)

        # 저장
        save_dir = os.path.join(current_dir, 'TIC')
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, 'kalman_filtered_result.json')

        result_dict = {
            'filtered_force': filtered_force.tolist(),
            'filtered_moment': filtered_moment.tolist(),
            'F_derivative': F_deri.tolist(),
            'M_derivative': M_deri.tolist(),
            'F_smooth': F_smooth.tolist(),
            'M_smooth': M_smooth.tolist()
        }

        with open(save_path, 'w') as f:
            json.dump(result_dict, f, indent=2)

        self.get_logger().info(f"✅ 결과 저장 완료: {save_path}")


# ================================ #
#            main 함수             #
# ================================ #
def main(args=None):
    rclpy.init(args=args)
    node = TicEMKMNode()
    rclpy.shutdown()
