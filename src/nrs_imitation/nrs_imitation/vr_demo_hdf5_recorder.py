#!/usr/bin/env python3
# ============================================================
# vr_demo_hdf5_recorder.py  (Stage-1: Human Demonstration)
#
# - VR tracker 기반 /calibrated_pose (x y z wx wy wz)
# - FT sensor   /ftsensor/measured_Cvalue (fx fy fz)
#
# Episode rule:
#   start: |fx| >= start_force_th
#   end  : |fy| >= end_force_th
#
# Record stop:
#   - Keyboard 'q' (no Enter needed)
#
# Save:
#   - One HDF5 file containing multiple episodes
#
# Notes:
#   - pose x,y,z are converted to [mm] (input assumed [m])
#   - wx,wy,wz assumed [rad]
# ============================================================

import os
import sys
import time
import atexit
import threading
import select
import termios
import tty
import numpy as np
import h5py

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
# Keyboard watcher (press 'q' to quit)
# ---------------------------
class _KeyboardQuitter:
    def __init__(self, quit_key: str = 'q'):
        self.quit_key = (quit_key or 'q').lower()
        self._stop_evt = threading.Event()
        self._hit_quit = threading.Event()
        self._thread = None
        self._enabled = False
        self._fd = None
        self._old_term = None

    def start(self):
        # interactive TTY only
        if not sys.stdin.isatty():
            self._enabled = False
            return False

        self._enabled = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        self._restore_term()

    def hit(self) -> bool:
        return self._hit_quit.is_set()

    def _restore_term(self):
        try:
            if self._enabled and self._fd is not None and self._old_term is not None:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass
        self._fd = None
        self._old_term = None

    def _loop(self):
        try:
            self._fd = sys.stdin.fileno()
            self._old_term = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)  # no Enter needed

            while not self._stop_evt.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                if ch.lower() == self.quit_key:
                    self._hit_quit.set()
                    break
        except Exception:
            # if anything goes wrong, just disable quit
            pass
        finally:
            self._restore_term()


# ---------------------------
# ROS2 Node (Stage-1 HDF5 recorder)
# ---------------------------
class VRDemoHDF5Recorder(Node):
    def __init__(self):
        super().__init__('vr_demo_hdf5_recorder')

        # ===== parameters =====
        self.declare_parameter('save_dir', '/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/datasets')
        self.declare_parameter('hdf5_name', 'vr_demo_stage1.hdf5')
        self.declare_parameter('overwrite', True)

        self.declare_parameter('pose_topic', '/calibrated_pose')
        self.declare_parameter('ft_topic', '/ftsensor/measured_Cvalue')

        self.declare_parameter('record_hz', 125.0)
        self.declare_parameter('playback_hz', 125.0)

        self.declare_parameter('start_force_th', 10.0)  # |fx| >=
        self.declare_parameter('end_force_th', 10.0)    # |fy| >=

        self.declare_parameter('num_episodes', 50)      # 목표 에피소드 수 (도달하면 자동 종료)
        self.declare_parameter('min_raw_samples', 10)   # 너무 짧은 에피소드 저장 방지
        self.declare_parameter('quit_key', 'q')         # 전체 종료 키

        # filtering params (same as txt)
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

        # ===== load params =====
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.hdf5_name = str(self.get_parameter('hdf5_name').value)
        self.overwrite = bool(self.get_parameter('overwrite').value)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.ft_topic = str(self.get_parameter('ft_topic').value)

        self.record_hz = float(self.get_parameter('record_hz').value)
        self.playback_hz = float(self.get_parameter('playback_hz').value)

        self.start_force_th = float(self.get_parameter('start_force_th').value)
        self.end_force_th = float(self.get_parameter('end_force_th').value)

        self.num_episodes = int(self.get_parameter('num_episodes').value)
        self.min_raw_samples = int(self.get_parameter('min_raw_samples').value)
        self.quit_key = str(self.get_parameter('quit_key').value)

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

        if self.record_hz <= 0.0:
            raise ValueError("record_hz must be > 0")

        os.makedirs(self.save_dir, exist_ok=True)
        self.hdf5_path = os.path.join(self.save_dir, self.hdf5_name)

        if self.overwrite and os.path.exists(self.hdf5_path):
            os.remove(self.hdf5_path)

        # ===== state =====
        self.lock = threading.Lock()
        self.recording = False
        self.latest_pose = None
        self.latest_ft = None
        self.pose_received = False
        self.ft_received = False
        self.buffer = []  # list of (9,) rows (raw)
        self.episode_count = 0

        self.stop_requested = False
        self.stop_reason = ""

        self._finalized = False

        # ===== HDF5 open =====
        self.h5 = h5py.File(self.hdf5_path, 'a')
        self.grp_eps = self.h5.require_group("episodes")

        # episode index: append mode 지원
        self.episode_count = self._detect_existing_episode_count()

        # meta
        self.h5.attrs["created_unix"] = float(time.time())
        self.h5.attrs["record_hz"] = float(self.record_hz)
        self.h5.attrs["playback_hz"] = float(self.playback_hz)
        self.h5.attrs["columns"] = np.string_("x_mm,y_mm,z_mm,wx,wy,wz,fx,fy,fz")
        self.h5.attrs["note_pose"] = np.string_("pose xyz assumed meters -> stored millimeters; omega assumed radians")

        # ===== subscribers / timers =====
        self.create_subscription(Float64MultiArray, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(Wrench, self.ft_topic, self.ft_callback, 10)

        self.timer = self.create_timer(1.0 / self.record_hz, self.main_loop)
        self.stop_timer = self.create_timer(0.05, self._check_stop)  # 20 Hz

        # ===== keyboard quitter =====
        self.kb = _KeyboardQuitter(quit_key=self.quit_key)
        enabled = self.kb.start()
        atexit.register(self.kb.stop)

        # ===== logs =====
        self.get_logger().info("============================================================")
        self.get_logger().info("VRDemoHDF5Recorder initialized (Stage-1 Human Demo)")
        self.get_logger().info(f"  Save HDF5: {self.hdf5_path}")
        self.get_logger().info(f"  episode rule: start=|fx|>={self.start_force_th}, end=|fy|>={self.end_force_th}")
        self.get_logger().info(f"  target episodes: {self.num_episodes} (current existing: {self.episode_count})")
        if enabled:
            self.get_logger().info(f"  press '{self.quit_key}' to stop recording (no Enter)")
        else:
            self.get_logger().warn("  stdin is not a TTY -> 'q' quit disabled. Use Ctrl+C instead.")
        self.get_logger().info("============================================================")

    def _detect_existing_episode_count(self) -> int:
        # ep_0000, ep_0001 ... 형태로 저장한다고 가정하고 마지막 인덱스+1 반환
        max_idx = -1
        for k in self.grp_eps.keys():
            if k.startswith("ep_"):
                try:
                    idx = int(k.split("_")[1])
                    max_idx = max(max_idx, idx)
                except Exception:
                    pass
        return max_idx + 1

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

        # stop 요청이면 episode 상태 변화는 timer에서 정리
        if self.stop_requested or self._finalized:
            return

        # episode start/end rules
        if (not self.recording) and (abs(fx) >= self.start_force_th):
            self.start_episode()

        if self.recording and (abs(fy) >= self.end_force_th):
            self.end_episode(reason="fy_threshold")

    # ---------------------------
    # main loop
    # ---------------------------
    def main_loop(self):
        if self._finalized:
            return

        # keyboard quit check
        if self.kb.hit() and (not self.stop_requested):
            self.request_stop(reason=f"keyboard_{self.quit_key}")

        if (not self.recording) or self.stop_requested:
            return

        with self.lock:
            if not (self.pose_received and self.ft_received):
                return
            row = np.hstack([self.latest_pose, self.latest_ft])
            self.buffer.append(row)

    # ---------------------------
    # stop checker (handles graceful exit)
    # ---------------------------
    def _check_stop(self):
        if self._finalized:
            return
        if not self.stop_requested:
            return

        # stop requested: if currently recording, finalize this episode first
        if self.recording:
            self.end_episode(reason=self.stop_reason or "stop_requested")

        self.finalize_and_shutdown()

    def request_stop(self, reason: str = "user_request"):
        self.stop_requested = True
        self.stop_reason = str(reason)
        self.get_logger().warn(f"[STOP REQUEST] reason={self.stop_reason} (will save and shutdown)")

    # ---------------------------
    # episode control
    # ---------------------------
    def start_episode(self):
        with self.lock:
            self.recording = True
            self.buffer.clear()
        self.get_logger().info(f"=== EPISODE STARTED (idx={self.episode_count:04d}) ===")

    def end_episode(self, reason: str = "end"):
        with self.lock:
            self.recording = False
            buf = self.buffer.copy()
            self.buffer.clear()

        if len(buf) < max(1, self.min_raw_samples):
            self.get_logger().warn(
                f"=== EPISODE DROPPED (too short) raw_len={len(buf)} < {self.min_raw_samples}, reason={reason} ==="
            )
            return

        try:
            out = self.filter_episode(buf)
            self.save_episode_to_hdf5(out, reason=reason)
            self.episode_count += 1
            self.get_logger().info(
                f"=== EPISODE SAVED (idx={self.episode_count-1:04d}) len={out.shape[0]} reason={reason} ==="
            )
        except Exception as e:
            self.get_logger().error(f"Episode save failed: {e}")

        # auto stop if reached target
        if self.episode_count >= self.num_episodes:
            self.request_stop(reason="reached_num_episodes")

    # ---------------------------
    # filtering (same idea as txt)
    # ---------------------------
    def filter_episode(self, buf_rows):
        raw = np.vstack(buf_rows).astype(np.float64)
        pose_raw = raw[:, 0:6].copy()
        force_raw = raw[:, 6:9].copy()

        if self.omega_unwrap:
            for k in range(3, 6):
                pose_raw[:, k] = unwrap_angle_series(pose_raw[:, k], discont=np.pi)

        factor = max(1, int(self.upsample_factor))
        pose_up = upsample_linear(pose_raw, factor)
        force_up = upsample_linear(force_raw, factor)

        pose_f = pose_up.copy()
        force_f = force_up.copy()

        # whittaker
        for j in range(3):
            pose_f[:, j] = whittaker_smooth_1d(pose_f[:, j], self.lam_pose, self.cg_tol, self.cg_max_iter)
        for j in range(3, 6):
            pose_f[:, j] = whittaker_smooth_1d(pose_f[:, j], self.lam_omega, self.cg_tol, self.cg_max_iter)
        for j in range(3):
            force_f[:, j] = whittaker_smooth_1d(force_f[:, j], self.lam_force, self.cg_tol, self.cg_max_iter)

        # ema filtfilt
        for j in range(3):
            pose_f[:, j] = ema_filtfilt_1d(pose_f[:, j], self.ema_alpha_pose)
        for j in range(3, 6):
            pose_f[:, j] = ema_filtfilt_1d(pose_f[:, j], self.ema_alpha_omega)
        for j in range(3):
            force_f[:, j] = ema_filtfilt_1d(force_f[:, j], self.ema_alpha_force)

        # endpoint match (pose)
        for j in range(6):
            pose_f[:, j] = end_ramp_match(pose_f[:, j], pose_raw[-1, j])
            pose_f[0, j] = pose_raw[0, j]

        # force shaping
        if self.force_xy_zero:
            force_f[:, 0] = 0.0
            force_f[:, 1] = 0.0
        force_f[:, 2] = np.clip(force_f[:, 2], self.fz_min, self.fz_max)

        force_f = apply_edge_force_window(
            force_f,
            playback_hz=self.playback_hz,
            edge_zero_sec=self.edge_force_zero_sec,
            fade_sec=self.edge_force_fade_sec,
        )
        force_f[:, 2] = np.clip(force_f[:, 2], self.fz_min, self.fz_max)

        out = np.hstack([pose_f, force_f]).astype(np.float32)
        return out

    # ---------------------------
    # HDF5 save
    # ---------------------------
    def save_episode_to_hdf5(self, out: np.ndarray, reason: str = ""):
        ep_name = f"ep_{self.episode_count:04d}"
        if ep_name in self.grp_eps:
            # overwrite single episode group if exists
            del self.grp_eps[ep_name]
        g = self.grp_eps.create_group(ep_name)
        g.attrs["saved_unix"] = float(time.time())
        g.attrs["reason"] = np.string_(reason)
        g.attrs["len"] = int(out.shape[0])
        g.attrs["dtype"] = np.string_(str(out.dtype))

        # dataset
        g.create_dataset("traj", data=out, compression="gzip", compression_opts=4, shuffle=True)

        # flush per-episode for safety
        self.h5.flush()

    # ---------------------------
    # finalize / shutdown
    # ---------------------------
    def finalize_and_shutdown(self):
        if self._finalized:
            return
        self._finalized = True
        try:
            self.get_logger().warn("Finalizing HDF5 and shutting down...")
            try:
                self.h5.flush()
            except Exception:
                pass
            try:
                self.h5.close()
            except Exception:
                pass
        finally:
            try:
                self.kb.stop()
            except Exception:
                pass
            try:
                self.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoHDF5Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C도 안전 종료
        try:
            node.request_stop(reason="KeyboardInterrupt")
            node.finalize_and_shutdown()
        except Exception:
            pass
    finally:
        # 혹시 남아있으면 정리
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
