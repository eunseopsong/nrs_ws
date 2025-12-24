#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory

from pathlib import Path
import numpy as np

import matplotlib
matplotlib.use('Agg')  # ROS2 / headless 환경
import matplotlib.pyplot as plt


class GCompMatrixNode(Node):
    """
    ft_gcomp_log.txt 전체를 이용해서

    1) r = [0,0,r_z] 가정 하에 모멘트 식
         tau = r × F_meas
       으로 r_z 추정

    2) 힘은 다음 모델로 보상 성능 확인:
         센서 init 시 자중 제거했다고 가정하면,
         F_meas(k) ≈ m * (g_s(k) - g_s(0)) = m * Δg_s(k)

       여기서:
         - wx, wy, wz : sensor → base 회전벡터(axis-angle, [rad])
         - g_base = [0, 0, -9.81]
         - g_s(k) = R_s_to_b(k)^T * g_base
         - Δg_s(k) = g_s(k) - g_s(0)

    3) 따라서
         F_model = m * Δg_s
         F_res   = F_meas - F_model
       로 힘 보상 결과를 평가하고,
       모멘트는
         tau_model = r × F_meas
         tau_res   = tau_meas - tau_model
       로 평가한다.

    로그 한 줄 형식:
      t Fx Fy Fz Mx My Mz wx wy wz
    """

    def __init__(self) -> None:
        super().__init__('g_comp_matrix_node')

        # 질량은 force 모델에 사용, r_z 추정엔 직접 사용하지 않음
        self.declare_parameters(
            '',
            [
                ('tool_mass', 1.6),             # [kg]
                ('tool_cog', [0.0, 0.0, 0.0]),  # [m] (r_z 결과 저장용)
            ],
        )

        # 로그 파일 경로 설정
        share_dir = Path(get_package_share_directory('Y2FT_AQ'))
        self.log_dir = share_dir.parent / 'log'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file_path = self.log_dir / 'ft_gcomp_log.txt'

        self.get_logger().info(f'Log file path: {self.log_file_path}')

        # 메인 처리
        self.estimate_and_evaluate()

    # ----------------------------
    # skew-symmetric matrix
    # ----------------------------
    @staticmethod
    def skew(v: np.ndarray) -> np.ndarray:
        x, y, z = v
        return np.array(
            [
                [0.0, -z,   y],
                [z,   0.0, -x],
                [-y,  x,   0.0],
            ],
            dtype=float,
        )

    # ----------------------------
    # Force/Moment 그래프 저장
    # ----------------------------
    def plot_results(self,
                     F_raw: np.ndarray,
                     F_model: np.ndarray,
                     F_res: np.ndarray,
                     tau_raw: np.ndarray,
                     tau_model: np.ndarray,
                     tau_res: np.ndarray) -> None:
        n = F_raw.shape[0]
        t_idx = np.arange(n)
        comp = ['x', 'y', 'z']

        # ----- Force Plot -----
        fig_f, axs_f = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        for i in range(3):
            axs_f[i].plot(t_idx, F_raw[:, i],   label='F_raw',   linestyle='-')
            axs_f[i].plot(t_idx, F_model[:, i], label='F_model', linestyle='--')
            axs_f[i].plot(t_idx, F_res[:, i],   label='F_res',   linestyle=':')
            axs_f[i].set_ylabel(f'F_{comp[i]} [N]')
            axs_f[i].grid(True)
            if i == 0:
                axs_f[i].legend(loc='best')

        axs_f[-1].set_xlabel('Sample index')
        fig_f.suptitle('Force: raw vs model (m·Δg_s) vs residual')
        fig_f.tight_layout()
        force_plot_path = self.log_dir / 'gcomp_force_results.png'
        fig_f.savefig(force_plot_path)
        plt.close(fig_f)
        self.get_logger().info(f'Saved force plot to: {force_plot_path}')

        # ----- Moment Plot -----
        fig_m, axs_m = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        for i in range(3):
            axs_m[i].plot(t_idx, tau_raw[:, i],   label='tau_raw',   linestyle='-')
            axs_m[i].plot(t_idx, tau_model[:, i], label='tau_model', linestyle='--')
            axs_m[i].plot(t_idx, tau_res[:, i],   label='tau_res',   linestyle=':')
            axs_m[i].set_ylabel(f'M_{comp[i]} [Nm]')
            axs_m[i].grid(True)
            if i == 0:
                axs_m[i].legend(loc='best')

        axs_m[-1].set_xlabel('Sample index')
        fig_m.suptitle('Moment: raw vs model (r×F_meas) vs residual')
        fig_m.tight_layout()
        moment_plot_path = self.log_dir / 'gcomp_moment_results.png'
        fig_m.savefig(moment_plot_path)
        plt.close(fig_m)
        self.get_logger().info(f'Saved moment plot to: {moment_plot_path}')

    # ----------------------------
    # 메인 처리 함수
    # ----------------------------
    def estimate_and_evaluate(self) -> None:
        # 0) 질량 파라미터
        m_param = self.get_parameter('tool_mass')
        m_est = float(m_param.value)

        if m_est <= 0.0:
            self.get_logger().error(
                "tool_mass 파라미터가 0 또는 음수입니다.\n"
                "저울로 측정한 툴 질량 [kg]을 tool_mass에 설정한 뒤 다시 실행해 주세요.\n"
                "예: ros2 run Y2FT_AQ G_Comp_Matrix.py --ros-args -p tool_mass:=1.63"
            )
            return

        self.get_logger().info(f"Using tool mass [kg]: {m_est:.6f}")

        # 1) 로그 파일 검사
        if not self.log_file_path.exists():
            self.get_logger().error(f'Log file not found: {self.log_file_path}')
            return

        text = self.log_file_path.read_text().strip()
        if not text:
            self.get_logger().error('Log file is empty.')
            return

        lines = text.splitlines()
        if len(lines) < 3:
            self.get_logger().error('Not enough samples in log (need multiple poses).')
            return

        # 회귀 및 평가용 데이터
        H_list = []        # (2N,)   r_z 계수 (tau_x, tau_y 식)
        tau_xy_list = []   # (2N,)   tau_x, tau_y
        tau_raw = []       # (N,3)   모멘트 원데이터
        F_raw = []         # (N,3)   힘 원데이터

        g_base = np.array([0.0, 0.0, -9.81], dtype=float)
        g_s0 = None        # 초기 sensor 기준 중력
        dg_all = []        # (N,3)   Δg_s(k) = g_s(k) - g_s(0)

        valid_count = 0

        for idx, line in enumerate(lines):
            parts = line.split()
            if len(parts) < 10:
                self.get_logger().warn(
                    f'Skip line {idx}: not enough columns -> "{line}"'
                )
                continue

            try:
                # t = float(parts[0])  # 필요시 사용 가능
                Fx, Fy, Fz = map(float, parts[1:4])
                Mx, My, Mz = map(float, parts[4:7])
                wx, wy, wz = map(float, parts[7:10])  # sensor → base 회전벡터
            except ValueError:
                self.get_logger().warn(f'Skip line {idx}: parse failed -> "{line}"')
                continue

            F_vec = np.array([Fx, Fy, Fz], dtype=float)
            tau_vec = np.array([Mx, My, Mz], dtype=float)

            # ----- sensor→base 회전벡터 → 회전행렬 R_s_to_b -----
            w_vec = np.array([wx, wy, wz], dtype=float)
            theta = np.linalg.norm(w_vec)

            if theta < 1e-8:
                R_s_to_b = np.eye(3, dtype=float)
            else:
                k = w_vec / theta  # 단위 회전축
                K = self.skew(k)
                # Rodrigues formula
                R_s_to_b = (
                    np.eye(3, dtype=float)
                    + np.sin(theta) * K
                    + (1.0 - np.cos(theta)) * (K @ K)
                )

            # Base 기준 중력 → Sensor 기준 중력
            # v_sensor = R_s_to_b^T * v_base
            g_s = R_s_to_b.T @ g_base

            if g_s0 is None:
                g_s0 = g_s.copy()

            dg_s = g_s - g_s0  # Δg_s(k)

            # ----- r = [0,0,r_z] 모멘트 모델용 데이터 쌓기 -----
            # tau_x = -r_z * Fy
            # tau_y =  r_z * Fx
            H_list.extend([-Fy, Fx])
            tau_xy_list.extend([Mx, My])

            # 평가용 원데이터
            F_raw.append(F_vec)
            tau_raw.append(tau_vec)
            dg_all.append(dg_s)

            valid_count += 1

        if valid_count < 2:
            self.get_logger().error('Not enough valid samples after parsing.')
            return

        # numpy 배열로 변환
        H = np.array(H_list, dtype=float).reshape(-1, 1)            # (2N, 1)
        tau_arr = np.array(tau_xy_list, dtype=float).reshape(-1, 1) # (2N, 1)

        F_raw = np.vstack(F_raw)        # (N,3)
        tau_raw = np.vstack(tau_raw)    # (N,3)
        dg_all = np.vstack(dg_all)      # (N,3)

        # ----------------------------
        # 2) r_z 추정 (모멘트 기준)
        #    tau_arr ≈ H * r_z
        # ----------------------------
        rz_vec, *_ = np.linalg.lstsq(H, tau_arr, rcond=None)
        r_z = float(rz_vec[0])
        r_est = np.array([0.0, 0.0, r_z])

        # 파라미터에 저장
        self.set_parameters(
            [
                Parameter('tool_cog',
                          value=r_est.tolist(),
                          type_=Parameter.Type.DOUBLE_ARRAY),
            ]
        )

        self.get_logger().info(
            "\n================ GRAVITY PARAM ESTIMATION (r aligned with sensor z-axis) ================\n"
            f"  Used tool mass [kg]: {m_est:.6f}\n"
            f"  Estimated CoM in sensor frame [m] (r_x = r_y = 0 가정):\n"
            f"    r_x = 0.000000\n"
            f"    r_y = 0.000000\n"
            f"    r_z = {r_z:.6f}\n"
            "======================================================================================="
        )

        # ----------------------------
        # 3) 힘 보상 품질 평가 (Δg_s 모델)
        #    F_model = m * Δg_s
        #    F_res   = F_meas - F_model
        # ----------------------------
        F_model = m_est * dg_all
        F_res = F_raw - F_model

        F_res_norm = np.linalg.norm(F_res, axis=1)
        F_res_mean = float(np.mean(F_res_norm))
        F_res_max = float(np.max(F_res_norm))
        F_res_mean_abs = np.mean(np.abs(F_res), axis=0)

        self.get_logger().info(
            "\n================ FORCE COMPENSATION CHECK (m·Δg_s model) ==========================\n"
            f"  Samples used           : {valid_count}\n"
            "\n"
            f"  |F_res| mean  [N]      : {F_res_mean:.6f}\n"
            f"  |F_res| max   [N]      : {F_res_max:.6f}\n"
            "\n"
            f"  Mean abs(F_res) [N]:\n"
            f"    Fx: {F_res_mean_abs[0]:.6f}\n"
            f"    Fy: {F_res_mean_abs[1]:.6f}\n"
            f"    Fz: {F_res_mean_abs[2]:.6f}\n"
            "  (주의: 여기서는 init 이후 자중 변화(m·Δg_s)만 모델링해서 빼본 값입니다.)\n"
            "================================================================================"
        )

        # ----------------------------
        # 4) 모멘트 보상 품질 평가
        #    tau_model = r × F_meas
        #    tau_res   = tau_meas - tau_model
        # ----------------------------
        N = valid_count
        tau_model = np.cross(np.tile(r_est, (N, 1)), F_raw)
        tau_res = tau_raw - tau_model

        tau_res_norm = np.linalg.norm(tau_res, axis=1)
        tau_res_mean = float(np.mean(tau_res_norm))
        tau_res_max = float(np.max(tau_res_norm))
        tau_res_mean_abs = np.mean(np.abs(tau_res), axis=0)

        self.get_logger().info(
            "\n================ MOMENT COMPENSATION CHECK (using r × F_meas) ==========================\n"
            f"  Samples used           : {valid_count}\n"
            "\n"
            f"  |tau_res| mean  [Nm]   : {tau_res_mean:.6f}\n"
            f"  |tau_res| max   [Nm]   : {tau_res_max:.6f}\n"
            "\n"
            f"  Mean abs(tau_res) [Nm]:\n"
            f"    Mx: {tau_res_mean_abs[0]:.6f}\n"
            f"    My: {tau_res_mean_abs[1]:.6f}\n"
            f"    Mz: {tau_res_mean_abs[2]:.6f}\n"
            "================================================================================"
        )

        # ----------------------------
        # 5) Force & Moment 그래프 저장
        # ----------------------------
        self.plot_results(F_raw, F_model, F_res,
                          tau_raw, tau_model, tau_res)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GCompMatrixNode()
    # 한 번 계산하고 바로 종료
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
