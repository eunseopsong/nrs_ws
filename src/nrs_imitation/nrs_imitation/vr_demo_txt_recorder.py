#!/usr/bin/env python3
# ============================================================
# vr_demo_txt_recorder.py
#
# - A PC에서 실행 (데이터 수집, 필터링, 로컬 저장)
# - 저장 완료 후 B PC(제어용)로 파일 자동 전송 (SCP)
# ============================================================

import os
import threading
import numpy as np
import subprocess  # <--- [추가됨] 파일 전송을 위해 필요

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


# ---------------------------
# Smooth utilities (fast, no scipy)
# ---------------------------
def _apply_DTD(x: np.ndarray) -> np.ndarray:
    n = x.shape[0]
    if n < 3:
        return np.zeros_like(x)
    z = x[:-2] - 2.0 * x[1:-1] + x[2:]
    res = np.zeros_like(x)
    res[0] = z[0]
    if n >= 4:
        res[1] = -2.0 * z[0] + z[1]
        res[-2] = z[-2] - 2.0 * z[-1]
    else:
        res[1] = -2.0 * z[0]
        res[-2] = res[1]
    res[-1] = z[-1]
    if n >= 5:
        res[2:-2] = z[:-2] - 2.0 * z[1:-1] + z[2:]
    return res


def _cg_solve(apply_A, b, x0=None, tol=1e-10, max_iter=200):
    b = b.astype(np.float64, copy=False)
    nrm_b = float(np.linalg.norm(b)) + 1e-30
    x = (b.copy() if x0 is None else x0.astype(np.float64, copy=True))
    r = b - apply_A(x)
    p = r.copy()
    rs_old = float(r @ r)
    if np.sqrt(rs_old) / nrm_b < tol:
        return x
    for _ in range(max_iter):
        Ap = apply_A(p)
        denom = float(p @ Ap) + 1e-30
        alpha = rs_old / denom
        x += alpha * p
        r -= alpha * Ap
        rs_new = float(r @ r)
        if np.sqrt(rs_new) / nrm_b < tol:
            break
        beta = rs_new / (rs_old + 1e-30)
        p = r + beta * p
        rs_old = rs_new
    return x


def whittaker_smooth_1d(y: np.ndarray, lam: float, cg_tol=1e-10, cg_max_iter=200) -> np.ndarray:
    y = y.astype(np.float64, copy=False)
    n = y.shape[0]
    if n < 3 or lam <= 0.0:
        return y.copy()
    def apply_A(x):
        return x + lam * _apply_DTD(x)
    x0 = y.copy()
    x = _cg_solve(apply_A, y, x0=x0, tol=cg_tol, max_iter=cg_max_iter)
    return x


def ema_filtfilt_1d(y: np.ndarray, alpha: float) -> np.ndarray:
    y = y.astype(np.float64, copy=False)
    n = y.shape[0]
    if n == 0 or alpha >= 1.0:
        return y.copy()
    alpha = float(np.clip(alpha, 1e-6, 1.0))
    out = np.empty_like(y)
    out[0] = y[0]
    for i in range(1, n):
        out[i] = alpha * y[i] + (1.0 - alpha) * out[i - 1]
    out2 = np.empty_like(y)
    out2[-1] = out[-1]
    for i in range(n - 2, -1, -1):
        out2[i] = alpha * out[i] + (1.0 - alpha) * out2[i + 1]
    return out2


def smoothstep01(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def unwrap_angle_series(a: np.ndarray, discont=np.pi) -> np.ndarray:
    return np.unwrap(a.astype(np.float64, copy=False), discont=discont)


def upsample_linear(data: np.ndarray, factor: int) -> np.ndarray:
    if factor <= 1:
        return data.copy()
    n, d = data.shape
    if n < 2:
        return data.copy()
    n_new = (n - 1) * factor + 1
    t = np.arange(n, dtype=np.float64)
    t_new = np.linspace(0.0, float(n - 1), n_new, dtype=np.float64)
    out = np.empty((n_new, d), dtype=np.float64)
    for j in range(d):
        out[:, j] = np.interp(t_new, t, data[:, j])
    return out


def end_ramp_match(x: np.ndarray, target_end: float) -> np.ndarray:
    n = x.shape[0]
    if n < 2:
        return x
    delta = float(target_end - x[-1])
    if abs(delta) < 1e-15:
        return x
    s = smoothstep01(np.linspace(0.0, 1.0, n, dtype=np.float64))
    return x + delta * s


def apply_edge_force_window(force: np.ndarray, playback_hz: float, edge_zero_sec: float, fade_sec: float) -> np.ndarray:
    n = force.shape[0]
    if n == 0:
        return force
    hz = float(playback_hz)
    k0 = int(max(0.0, edge_zero_sec) * hz)
    k0 = min(k0, n // 2)
    if k0 <= 0:
        return force
    kfade = int(max(0.0, fade_sec) * hz)
    kfade = min(kfade, max(0, n - 2 * k0))
    w = np.ones(n, dtype=np.float64)
    w[:k0] = 0.0
    if kfade > 1:
        ramp = smoothstep01(np.linspace(0.0, 1.0, kfade, dtype=np.float64))
        w[k0:k0 + kfade] = ramp
    w[-k0:] = 0.0
    if kfade > 1:
        ramp = smoothstep01(np.linspace(1.0, 0.0, kfade, dtype=np.float64))
        w[-k0 - kfade:-k0] = ramp
    out = force.copy()
    out[:, 0] *= w
    out[:, 1] *= w
    out[:, 2] *= w
    return out


# ---------------------------
# ROS2 Node
# ---------------------------
class VRDemoTXTRecorder(Node):
    def __init__(self):
        super().__init__('vr_demo_txt_recorder')

        # ===== parameters =====
        # A PC의 로컬 저장 경로 (여기서 먼저 저장됨)
        self.declare_parameter('save_dir', '/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd')
        self.declare_parameter('file_name', 'cmd_continue9D.txt')

        self.declare_parameter('pose_topic', '/calibrated_pose')
        self.declare_parameter('ft_topic', '/ftsensor/measured_Cvalue')

        self.declare_parameter('record_hz', 125.0)
        self.declare_parameter('playback_hz', 125.0)

        self.declare_parameter('start_force_th', 10.0)
        self.declare_parameter('end_force_th', 10.0)

        self.declare_parameter('upsample_factor', 8)
        self.declare_parameter('omega_unwrap', True)

        self.declare_parameter('lam_pose', 3000.0)
        self.declare_parameter('lam_omega', 8000.0)
        self.declare_parameter('lam_force', 3000.0)

        self.declare_parameter('cg_tol', 1e-10)
        self.declare_parameter('cg_max_iter', 250)

        self.declare_parameter('ema_alpha_pose', 0.12)
        self.declare_parameter('ema_alpha_omega', 0.10)
        self.declare_parameter('ema_alpha_force', 0.18)

        self.declare_parameter('force_xy_zero', True)
        self.declare_parameter('fz_min', 0.0)
        self.declare_parameter('fz_max', 10.0)

        self.declare_parameter('edge_force_zero_sec', 5.0)
        self.declare_parameter('edge_force_fade_sec', 0.6)

        self.declare_parameter('plot_enable', True)
        self.declare_parameter('plot_show', False)
        self.declare_parameter('plot_prefix', '')

        # ===== load params =====
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.file_name = str(self.get_parameter('file_name').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.ft_topic = str(self.get_parameter('ft_topic').value)

        self.record_hz = float(self.get_parameter('record_hz').value)
        self.playback_hz = float(self.get_parameter('playback_hz').value)
        self.start_force_th = float(self.get_parameter('start_force_th').value)
        self.end_force_th = float(self.get_parameter('end_force_th').value)
        self.upsample_factor = int(self.get_parameter('upsample_factor').value)
        self.omega_unwrap = bool(self.get_parameter('omega_unwrap').value)
        self.lam_pose = float(self.get_parameter('lam_pose').value)
        self.lam_omega = float(self.get_parameter('lam_omega').value)
        self.lam_force = float(self.get_parameter('lam_force').value)
        self.cg_tol = float(self.get_parameter('cg_tol').value)
        self.cg_max_iter = int(self.get_parameter('cg_max_iter').value)
        self.ema_alpha_pose = float(self.get_parameter('ema_alpha_pose').value)
        self.ema_alpha_omega = float(self.get_parameter('ema_alpha_omega').value)
        self.ema_alpha_force = float(self.get_parameter('ema_alpha_force').value)
        self.force_xy_zero = bool(self.get_parameter('force_xy_zero').value)
        self.fz_min = float(self.get_parameter('fz_min').value)
        self.fz_max = float(self.get_parameter('fz_max').value)
        self.edge_force_zero_sec = float(self.get_parameter('edge_force_zero_sec').value)
        self.edge_force_fade_sec = float(self.get_parameter('edge_force_fade_sec').value)
        self.plot_enable = bool(self.get_parameter('plot_enable').value)
        self.plot_show = bool(self.get_parameter('plot_show').value)
        self.plot_prefix = str(self.get_parameter('plot_prefix').value)

        os.makedirs(self.save_dir, exist_ok=True)
        self.file_path = os.path.join(self.save_dir, self.file_name)
        open(self.file_path, 'w').close()

        if not self.plot_prefix:
            base = os.path.splitext(self.file_name)[0]
            self.plot_prefix = base

        self.lock = threading.Lock()
        self.recording = False
        self.episode_done = False
        self.latest_pose = None
        self.latest_ft = None
        self.pose_received = False
        self.ft_received = False
        self.buffer = []

        self.create_subscription(Float64MultiArray, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(Wrench, self.ft_topic, self.ft_callback, 10)

        if self.record_hz <= 0.0:
            raise ValueError("record_hz must be > 0")
        self.timer = self.create_timer(1.0 / self.record_hz, self.main_loop)

        self.get_logger().info(f"Initialized. Local save path: {self.file_path}")

    # ---------------------------
    # callbacks
    # ---------------------------
    def pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        with self.lock:
            x_m, y_m, z_m, wx, wy, wz = msg.data[:6]
            x = float(x_m) * 1000.0
            y = float(y_m) * 1000.0
            z = float(z_m) * 1000.0
            self.latest_pose = np.array([x, y, z, float(wx), float(wy), float(wz)], dtype=np.float64)
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

        if (not self.recording) and (abs(fx) >= self.start_force_th):
            self.start_episode()
        if self.recording and (abs(fy) >= self.end_force_th):
            self.end_episode()

    # ---------------------------
    # main loop
    # ---------------------------
    def main_loop(self):
        if (not self.recording) or self.episode_done:
            return
        with self.lock:
            if not (self.pose_received and self.ft_received):
                return
            row = np.hstack([self.latest_pose, self.latest_ft])
            self.buffer.append(row)

    # ---------------------------
    # episode control
    # ---------------------------
    def start_episode(self):
        with self.lock:
            self.recording = True
            self.buffer.clear()
        self.get_logger().info("=== EPISODE STARTED ===")

    def end_episode(self):
        with self.lock:
            self.recording = False
            self.episode_done = True
        self.get_logger().info("=== EPISODE ENDED ===")
        try:
            self.save_txt_filtered_super_smooth()
        except Exception as e:
            self.get_logger().error(f"save failed: {e}")
        
        self.get_logger().info("Shutting down.")
        rclpy.shutdown()

    # ---------------------------
    # filtering / saving / SENDING
    # ---------------------------
    def save_txt_filtered_super_smooth(self):
        if len(self.buffer) == 0:
            self.get_logger().warn("No data to save.")
            return

        raw = np.vstack(self.buffer).astype(np.float64)
        n_raw = raw.shape[0]

        pose_raw = raw[:, 0:6].copy()
        force_raw = raw[:, 6:9].copy()

        if self.omega_unwrap:
            for k in range(3, 6):
                pose_raw[:, k] = unwrap_angle_series(pose_raw[:, k], discont=np.pi)

        factor = max(1, int(self.upsample_factor))
        pose_up = upsample_linear(pose_raw, factor)
        force_up = upsample_linear(force_raw, factor)
        n_out = pose_up.shape[0]

        pose_f = pose_up.copy()
        force_f = force_up.copy()

        for j in range(3):
            pose_f[:, j] = whittaker_smooth_1d(pose_f[:, j], self.lam_pose, self.cg_tol, self.cg_max_iter)
        for j in range(3, 6):
            pose_f[:, j] = whittaker_smooth_1d(pose_f[:, j], self.lam_omega, self.cg_tol, self.cg_max_iter)
        for j in range(3):
            force_f[:, j] = whittaker_smooth_1d(force_f[:, j], self.lam_force, self.cg_tol, self.cg_max_iter)

        for j in range(3):
            pose_f[:, j] = ema_filtfilt_1d(pose_f[:, j], self.ema_alpha_pose)
        for j in range(3, 6):
            pose_f[:, j] = ema_filtfilt_1d(pose_f[:, j], self.ema_alpha_omega)
        for j in range(3):
            force_f[:, j] = ema_filtfilt_1d(force_f[:, j], self.ema_alpha_force)

        for j in range(6):
            pose_f[:, j] = end_ramp_match(pose_f[:, j], pose_raw[-1, j])
            pose_f[0, j] = pose_raw[0, j]

        if self.force_xy_zero:
            force_f[:, 0] = 0.0
            force_f[:, 1] = 0.0

        force_f[:, 2] = np.clip(force_f[:, 2], self.fz_min, self.fz_max)

        force_f = apply_edge_force_window(
            force_f, playback_hz=self.playback_hz,
            edge_zero_sec=self.edge_force_zero_sec,
            fade_sec=self.edge_force_fade_sec
        )
        force_f[:, 2] = np.clip(force_f[:, 2], self.fz_min, self.fz_max)

        out = np.hstack([pose_f, force_f]).astype(np.float64)

        # 1. 로컬 저장 (A PC)
        np.savetxt(self.file_path, out, fmt="%.10f")
        self.get_logger().info(f"Saved local file: {self.file_path}")

        # 2. 그래프 저장
        if self.plot_enable:
            self.save_plots_filtered_only(out)

        # =========================================================
        # 3. B PC (제어용)로 파일 자동 전송 (SCP)
        # =========================================================
        try:
            target_user = "nrs_forcecon"
            target_ip = "192.168.0.151"
            # B PC의 저장 경로 (확실하게 존재하는지 확인 필요)
            target_dir = "/home/nrs_forcecon/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/"

            self.get_logger().info(f"Sending file to Control PC ({target_ip})...")
            
            # scp 명령어 실행
            cmd = ["scp", self.file_path, f"{target_user}@{target_ip}:{target_dir}"]
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                self.get_logger().info(f"SUCCESS: File transferred to B PC ({target_dir})")
            else:
                self.get_logger().error(f"SCP FAILED:\n{result.stderr}")
        except Exception as e:
            self.get_logger().error(f"File transfer exception: {e}")
        # =========================================================

    def save_plots_filtered_only(self, out: np.ndarray):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        hz = float(self.playback_hz) if self.playback_hz > 0 else 125.0
        t = np.arange(out.shape[0], dtype=np.float64) / hz

        x, y, z = out[:, 0], out[:, 1], out[:, 2]
        wx, wy, wz = out[:, 3], out[:, 4], out[:, 5]
        fx, fy, fz = out[:, 6], out[:, 7], out[:, 8]

        prefix = os.path.join(self.save_dir, self.plot_prefix)

        plt.figure()
        plt.title("XYZ (filtered)")
        plt.plot(t, x, label="x")
        plt.plot(t, y, label="y")
        plt.plot(t, z, label="z")
        plt.xlabel("time [s]")
        plt.ylabel("pos [mm]")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_xyz.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("Omega (filtered)")
        plt.plot(t, wx, label="wx")
        plt.plot(t, wy, label="wy")
        plt.plot(t, wz, label="wz")
        plt.xlabel("time [s]")
        plt.ylabel("rad")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_omega.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("Force (filtered)")
        plt.plot(t, fx, label="fx")
        plt.plot(t, fy, label="fy")
        plt.plot(t, fz, label="fz")
        plt.xlabel("time [s]")
        plt.ylabel("N")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_force.png", dpi=150, bbox_inches="tight")
        plt.close()
        
        if self.plot_show:
            import matplotlib.pyplot as plt_show
            plt_show.show()


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoTXTRecorder()
    rclpy.spin(node)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()