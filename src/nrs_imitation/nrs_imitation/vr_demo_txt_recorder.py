#!/usr/bin/env python3
# ============================================================
# vr_demo_txt_recorder.py
#
# Goal (this revision):
# - Keep raw row count N (125 Hz) as much as possible
# - Reduce IK-QP blow-ups by making pose command "QP-friendly"
#   WITHOUT retiming / upsampling.
#
# Key change:
# - Remove spike-patch (it can create discontinuities when most samples are "bad")
# - Use continuous velocity+acceleration limiter on pose (XYZ + omega)
# ============================================================

import os
import threading
import numpy as np
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


# ---------------------------
# Smooth utilities (fast, no scipy)  [FOR FORCE: keep as-is]
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


def unwrap_angle_series(a: np.ndarray, discont=np.pi) -> np.ndarray:
    return np.unwrap(a.astype(np.float64, copy=False), discont=discont)


def smoothstep01(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


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
# Contact gating (force)  [same as your last]
# ---------------------------
def find_contact_index_fz(
    fz: np.ndarray,
    th_on: float,
    th_off: float,
    consecutive_on: int,
    consecutive_off: int,
) -> int:
    n = int(fz.shape[0])
    if n <= 0:
        return -1

    consec_on = max(1, int(consecutive_on))
    consec_off = max(1, int(consecutive_off))
    th_on = float(th_on)
    th_off = float(th_off)

    state_on = False
    on_count = 0
    off_count = 0
    first_on_start = -1

    for i in range(n):
        v = float(fz[i])

        if not state_on:
            if v >= th_on:
                if on_count == 0:
                    first_on_start = i
                on_count += 1
                if on_count >= consec_on:
                    return first_on_start
            else:
                on_count = 0
                first_on_start = -1
        else:
            if v <= th_off:
                off_count += 1
                if off_count >= consec_off:
                    state_on = False
                    off_count = 0
                    on_count = 0
                    first_on_start = -1
            else:
                off_count = 0

    return -1


def zero_force_before_contact(force: np.ndarray, contact_idx: int) -> np.ndarray:
    out = force.copy()
    k = int(contact_idx)
    if k > 0:
        out[:k, :] = 0.0
    return out


# ---------------------------
# QP-proxy evaluation (pose)
# ---------------------------
def _diff1(x: np.ndarray, dt: float) -> np.ndarray:
    # centered-ish via forward diff (simple + stable)
    return np.diff(x, axis=0) / dt


def qp_eval_pose(pose: np.ndarray, dt: float, pos_vmax: float, pos_amax: float, ang_vmax: float, ang_amax: float):
    """
    pose: (N,6) [mm,mm,mm, rad,rad,rad]
    dt: seconds
    """
    N = pose.shape[0]
    T = (N - 1) * dt if N >= 2 else 0.0
    pos = pose[:, 0:3]
    ang = pose[:, 3:6]

    if N < 3:
        return {
            "N": N, "dt": dt, "T": T,
            "vpos_max": 0.0, "apos_max": 0.0, "vang_max": 0.0, "aang_max": 0.0,
        }

    vpos = _diff1(pos, dt)           # (N-1,3)
    vang = _diff1(ang, dt)           # (N-1,3)
    apos = _diff1(vpos, dt)          # (N-2,3)
    aang = _diff1(vang, dt)          # (N-2,3)

    vpos_n = np.linalg.norm(vpos, axis=1)
    vang_n = np.linalg.norm(vang, axis=1)
    apos_n = np.linalg.norm(apos, axis=1)
    aang_n = np.linalg.norm(aang, axis=1)

    def p95(x): return float(np.percentile(x, 95.0)) if x.size else 0.0
    def mean(x): return float(np.mean(x)) if x.size else 0.0

    out = {
        "N": N, "dt": float(dt), "T": float(T),

        "vpos_max": float(np.max(vpos_n)), "vpos_p95": p95(vpos_n), "vpos_mean": mean(vpos_n),
        "apos_max": float(np.max(apos_n)), "apos_p95": p95(apos_n), "apos_mean": mean(apos_n),

        "vang_max": float(np.max(vang_n)), "vang_p95": p95(vang_n), "vang_mean": mean(vang_n),
        "aang_max": float(np.max(aang_n)), "aang_p95": p95(aang_n), "aang_mean": mean(aang_n),
    }

    # violation rates (proxy)
    eps = 1e-12
    out["vpos_viol_rate"] = float(np.mean(vpos_n > (pos_vmax + eps)) * 100.0) if pos_vmax > 0 else 0.0
    out["apos_viol_rate"] = float(np.mean(apos_n > (pos_amax + eps)) * 100.0) if pos_amax > 0 else 0.0
    out["vang_viol_rate"] = float(np.mean(vang_n > (ang_vmax + eps)) * 100.0) if ang_vmax > 0 else 0.0
    out["aang_viol_rate"] = float(np.mean(aang_n > (ang_amax + eps)) * 100.0) if ang_amax > 0 else 0.0

    # jerk reference (3rd diff magnitude proxy)
    jpos = _diff1(apos, dt)  # (N-3,3)
    jang = _diff1(aang, dt)
    out["jpos_max"] = float(np.max(np.linalg.norm(jpos, axis=1))) if jpos.size else 0.0
    out["jang_max"] = float(np.max(np.linalg.norm(jang, axis=1))) if jang.size else 0.0

    return out


def format_qp_eval(e, pos_vmax, pos_amax, ang_vmax, ang_amax) -> str:
    return (
        f"\n  N={e['N']}  dt={e['dt']:.6f}s  T={e['T']:.3f}s\n"
        f"  pos |v|: max={e['vpos_max']:.3f} (lim {pos_vmax:.3f}, {e['vpos_max']/max(pos_vmax,1e-9):.3f}x), "
        f"p95={e['vpos_p95']:.3f}, mean={e['vpos_mean']:.3f}  [mm/s]\n"
        f"  pos |a|: max={e['apos_max']:.3f} (lim {pos_amax:.3f}, {e['apos_max']/max(pos_amax,1e-9):.3f}x), "
        f"p95={e['apos_p95']:.3f}, mean={e['apos_mean']:.3f}  [mm/s^2]\n"
        f"  ang |w|: max={e['vang_max']:.3f} (lim {ang_vmax:.3f}, {e['vang_max']/max(ang_vmax,1e-9):.3f}x), "
        f"p95={e['vang_p95']:.3f}, mean={e['vang_mean']:.3f}  [rad/s]\n"
        f"  ang |alpha|: max={e['aang_max']:.3f} (lim {ang_amax:.3f}, {e['aang_max']/max(ang_amax,1e-9):.3f}x), "
        f"p95={e['aang_p95']:.3f}, mean={e['aang_mean']:.3f}  [rad/s^2]\n"
        f"  jerk(ref): pos max={e['jpos_max']:.3f} [mm/s^3], ang max={e['jang_max']:.3f} [rad/s^3]\n"
        f"  violation_rate: vpos={e['vpos_viol_rate']:.3f}%, apos={e['apos_viol_rate']:.3f}%, "
        f"vang={e['vang_viol_rate']:.3f}%, aang={e['aang_viol_rate']:.3f}%"
    )


# ---------------------------
# QP-proxy limiter (pose)  [NEW]
# ---------------------------
def _clamp_norm(v: np.ndarray, vmax: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if vmax <= 0.0 or n <= vmax or n < 1e-12:
        return v
    return v * (vmax / n)


def _limit_forward_to_target(target: np.ndarray, dt: float, vmax: float, amax: float, anchor0: np.ndarray) -> np.ndarray:
    """
    target: (N,D)
    output y tracks target but respects |v|<=vmax and |dv/dt|<=amax in L2 norm.
    """
    N, D = target.shape
    y = np.empty_like(target, dtype=np.float64)
    y[0] = anchor0.astype(np.float64, copy=False)
    v_prev = np.zeros(D, dtype=np.float64)

    for i in range(1, N):
        v_des = (target[i] - y[i - 1]) / dt
        v = _clamp_norm(v_des, vmax)
        if amax > 0.0:
            dv = v - v_prev
            dv = _clamp_norm(dv, amax * dt)
            v = v_prev + dv
        y[i] = y[i - 1] + v * dt
        v_prev = v
    return y


def qp_proxy_limit_pose(
    pose_target: np.ndarray,
    dt: float,
    pos_vmax: float,
    pos_amax: float,
    ang_vmax: float,
    ang_amax: float,
    safety: float = 1.05,
    iters: int = 3
) -> np.ndarray:
    """
    Keeps N the same. No retiming. Continuous limiter (no discontinuities).
    safety>1 makes the effective limits tighter.
    """
    pose_target = pose_target.astype(np.float64, copy=False)
    N = pose_target.shape[0]
    if N < 2:
        return pose_target.copy()

    eff = max(1e-9, float(safety))
    pv = float(pos_vmax) / eff
    pa = float(pos_amax) / eff
    av = float(ang_vmax) / eff
    aa = float(ang_amax) / eff

    out = pose_target.copy()

    # Split
    pos_t = pose_target[:, 0:3]
    ang_t = pose_target[:, 3:6]

    pos = out[:, 0:3]
    ang = out[:, 3:6]

    iters = max(1, int(iters))

    for _ in range(iters):
        # Forward (anchor start)
        pos_f = _limit_forward_to_target(pos_t, dt, pv, pa, anchor0=pos_t[0])
        ang_f = _limit_forward_to_target(ang_t, dt, av, aa, anchor0=ang_t[0])

        # Backward (anchor end) via reversing target
        pos_tr = pos_t[::-1].copy()
        ang_tr = ang_t[::-1].copy()
        pos_b_r = _limit_forward_to_target(pos_tr, dt, pv, pa, anchor0=pos_tr[0])  # anchored at original end
        ang_b_r = _limit_forward_to_target(ang_tr, dt, av, aa, anchor0=ang_tr[0])
        pos_b = pos_b_r[::-1]
        ang_b = ang_b_r[::-1]

        # Blend (still continuous)
        pos = 0.5 * (pos_f + pos_b)
        ang = 0.5 * (ang_f + ang_b)

        # Re-impose exact endpoints to original targets
        pos[0] = pos_t[0]
        pos[-1] = pos_t[-1]
        ang[0] = ang_t[0]
        ang[-1] = ang_t[-1]

    out[:, 0:3] = pos
    out[:, 3:6] = ang
    return out


# ---------------------------
# ROS2 Node
# ---------------------------
class VRDemoTXTRecorder(Node):
    def __init__(self):
        super().__init__('vr_demo_txt_recorder')

        # ===== parameters =====
        self.declare_parameter('save_dir', '/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd')
        self.declare_parameter('file_name', 'cmd_continue9D.txt')

        self.declare_parameter('pose_topic', '/calibrated_pose')
        self.declare_parameter('ft_topic', '/ftsensor/measured_Cvalue')

        self.declare_parameter('record_hz', 125.0)
        self.declare_parameter('playback_hz', 125.0)

        self.declare_parameter('start_force_th', 10.0)
        self.declare_parameter('end_force_th', 10.0)

        # ===== pose pipeline =====
        self.declare_parameter('omega_unwrap', True)

        # QP-proxy limits (same knobs you already use for evaluation)
        self.declare_parameter('pos_vmax', 30.0)     # [mm/s]   (proxy)
        self.declare_parameter('pos_amax', 120.0)    # [mm/s^2] (proxy)
        self.declare_parameter('ang_vmax', 0.6)      # [rad/s]
        self.declare_parameter('ang_amax', 3.0)      # [rad/s^2]

        # NEW: continuous limiter (keeps N)
        self.declare_parameter('qp_limiter_enable', True)
        self.declare_parameter('qp_limiter_safety', 1.05)  # >1 tighter than limits
        self.declare_parameter('qp_limiter_iters', 3)

        # Optional sanity hold (prevents a single corrupted sample from exploding diffs)
        self.declare_parameter('pose_sanity_enable', True)
        self.declare_parameter('pos_abs_max_mm', 5000.0)
        self.declare_parameter('ang_abs_max_rad', 10.0)

        # ===== force pipeline (keep as-is) =====
        self.declare_parameter('lam_force', 3000.0)
        self.declare_parameter('cg_tol', 1e-10)
        self.declare_parameter('cg_max_iter', 250)
        self.declare_parameter('ema_alpha_force', 0.18)

        self.declare_parameter('force_xy_zero', True)
        self.declare_parameter('fz_min', 0.0)
        self.declare_parameter('fz_max', 20.0)

        self.declare_parameter('edge_force_zero_sec', 5.0)
        self.declare_parameter('edge_force_fade_sec', 0.6)

        # ===== contact gating =====
        self.declare_parameter('contact_fz_on', 5.0)
        self.declare_parameter('contact_fz_off', 3.0)
        self.declare_parameter('contact_consec_on', 10)
        self.declare_parameter('contact_consec_off', 10)
        self.declare_parameter('precontact_force_zero', True)

        # ===== plots =====
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

        self.omega_unwrap = bool(self.get_parameter('omega_unwrap').value)

        self.pos_vmax = float(self.get_parameter('pos_vmax').value)
        self.pos_amax = float(self.get_parameter('pos_amax').value)
        self.ang_vmax = float(self.get_parameter('ang_vmax').value)
        self.ang_amax = float(self.get_parameter('ang_amax').value)

        self.qp_limiter_enable = bool(self.get_parameter('qp_limiter_enable').value)
        self.qp_limiter_safety = float(self.get_parameter('qp_limiter_safety').value)
        self.qp_limiter_iters = int(self.get_parameter('qp_limiter_iters').value)

        self.pose_sanity_enable = bool(self.get_parameter('pose_sanity_enable').value)
        self.pos_abs_max_mm = float(self.get_parameter('pos_abs_max_mm').value)
        self.ang_abs_max_rad = float(self.get_parameter('ang_abs_max_rad').value)

        self.lam_force = float(self.get_parameter('lam_force').value)
        self.cg_tol = float(self.get_parameter('cg_tol').value)
        self.cg_max_iter = int(self.get_parameter('cg_max_iter').value)
        self.ema_alpha_force = float(self.get_parameter('ema_alpha_force').value)

        self.force_xy_zero = bool(self.get_parameter('force_xy_zero').value)
        self.fz_min = float(self.get_parameter('fz_min').value)
        self.fz_max = float(self.get_parameter('fz_max').value)
        self.edge_force_zero_sec = float(self.get_parameter('edge_force_zero_sec').value)
        self.edge_force_fade_sec = float(self.get_parameter('edge_force_fade_sec').value)

        self.contact_fz_on = float(self.get_parameter('contact_fz_on').value)
        self.contact_fz_off = float(self.get_parameter('contact_fz_off').value)
        self.contact_consec_on = int(self.get_parameter('contact_consec_on').value)
        self.contact_consec_off = int(self.get_parameter('contact_consec_off').value)
        self.precontact_force_zero = bool(self.get_parameter('precontact_force_zero').value)

        self.plot_enable = bool(self.get_parameter('plot_enable').value)
        self.plot_show = bool(self.get_parameter('plot_show').value)
        self.plot_prefix = str(self.get_parameter('plot_prefix').value)

        os.makedirs(self.save_dir, exist_ok=True)
        self.file_path = os.path.join(self.save_dir, self.file_name)
        open(self.file_path, 'w').close()

        if not self.plot_prefix:
            self.plot_prefix = os.path.splitext(self.file_name)[0]

        # buffers
        self.lock = threading.Lock()
        self.recording = False
        self.episode_done = False
        self.latest_pose = None
        self.latest_ft = None
        self.pose_received = False
        self.ft_received = False
        self.buffer = []

        # sanity hold
        self.last_good_pose = None
        self.last_good_ft = None

        # subs
        self.create_subscription(Float64MultiArray, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(Wrench, self.ft_topic, self.ft_callback, 10)

        if self.record_hz <= 0.0:
            raise ValueError("record_hz must be > 0")
        self.timer = self.create_timer(1.0 / self.record_hz, self.main_loop)

        self.get_logger().info(f"Initialized. Local save path: {self.file_path}")
        self.get_logger().info(
            f"Pose: keep N raw. QP-limiter enable={self.qp_limiter_enable} "
            f"(pos_vmax={self.pos_vmax} mm/s, pos_amax={self.pos_amax} mm/s^2, "
            f"ang_vmax={self.ang_vmax} rad/s, ang_amax={self.ang_amax} rad/s^2, "
            f"safety={self.qp_limiter_safety}, iters={self.qp_limiter_iters})"
        )
        self.get_logger().info(
            f"Pre-contact force gating: {self.precontact_force_zero}  "
            f"(fz_on={self.contact_fz_on}, fz_off={self.contact_fz_off}, "
            f"consec_on={self.contact_consec_on}, consec_off={self.contact_consec_off})"
        )

    # ---------------------------
    # callbacks
    # ---------------------------
    def pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        with self.lock:
            x_m, y_m, z_m, wx, wy, wz = msg.data[:6]
            # meters -> mm
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

        # keep your existing start/end triggers
        if (not self.recording) and (abs(fx) >= self.start_force_th):
            self.start_episode()
        if self.recording and (abs(fy) >= self.end_force_th):
            self.end_episode()

    # ---------------------------
    # main loop
    # ---------------------------
    def _sanity_pose(self, p: np.ndarray) -> bool:
        if p is None or p.shape[0] != 6:
            return False
        if not np.isfinite(p).all():
            return False
        if np.any(np.abs(p[0:3]) > self.pos_abs_max_mm):
            return False
        if np.any(np.abs(p[3:6]) > self.ang_abs_max_rad):
            return False
        return True

    def _sanity_ft(self, f: np.ndarray) -> bool:
        if f is None or f.shape[0] != 3:
            return False
        if not np.isfinite(f).all():
            return False
        # no hard bounds; just finite
        return True

    def main_loop(self):
        if (not self.recording) or self.episode_done:
            return
        with self.lock:
            if not (self.pose_received and self.ft_received):
                return

            pose = self.latest_pose.copy()
            ft = self.latest_ft.copy()

            if self.pose_sanity_enable:
                if self._sanity_pose(pose):
                    self.last_good_pose = pose
                else:
                    if self.last_good_pose is None:
                        return
                    pose = self.last_good_pose.copy()

                if self._sanity_ft(ft):
                    self.last_good_ft = ft
                else:
                    if self.last_good_ft is None:
                        return
                    ft = self.last_good_ft.copy()

            row = np.hstack([pose, ft])
            self.buffer.append(row)

    # ---------------------------
    # episode control
    # ---------------------------
    def start_episode(self):
        with self.lock:
            self.recording = True
            self.buffer.clear()
            self.last_good_pose = None
            self.last_good_ft = None
        self.get_logger().info("=== EPISODE STARTED ===")

    def end_episode(self):
        with self.lock:
            self.recording = False
            self.episode_done = True
        self.get_logger().info("=== EPISODE ENDED ===")
        try:
            self.save_txt_filtered()
        except Exception as e:
            self.get_logger().error(f"save failed: {e}")

        self.get_logger().info("Shutting down.")
        rclpy.shutdown()

    # ---------------------------
    # filtering / saving / sending
    # ---------------------------
    def save_txt_filtered(self):
        if len(self.buffer) == 0:
            self.get_logger().warn("No data to save.")
            return

        raw = np.vstack(self.buffer).astype(np.float64)
        pose_raw = raw[:, 0:6].copy()
        force_raw = raw[:, 6:9].copy()

        dt = 1.0 / float(self.playback_hz if self.playback_hz > 0 else self.record_hz)

        # unwrap omega before any pose limiting (prevents wrap-induced jumps)
        if self.omega_unwrap:
            for k in range(3, 6):
                pose_raw[:, k] = unwrap_angle_series(pose_raw[:, k], discont=np.pi)

        # ---- QP-EVAL BEFORE (pose only) ----
        e0 = qp_eval_pose(pose_raw, dt, self.pos_vmax, self.pos_amax, self.ang_vmax, self.ang_amax)
        self.get_logger().info("[QP-EVAL] ===== BEFORE pose limiter =====")
        self.get_logger().info(format_qp_eval(e0, self.pos_vmax, self.pos_amax, self.ang_vmax, self.ang_amax))

        # ---- Pose limiter (keeps N, continuous) ----
        pose_f = pose_raw.copy()
        if self.qp_limiter_enable and pose_raw.shape[0] >= 3:
            pose_f = qp_proxy_limit_pose(
                pose_target=pose_raw,
                dt=dt,
                pos_vmax=self.pos_vmax,
                pos_amax=self.pos_amax,
                ang_vmax=self.ang_vmax,
                ang_amax=self.ang_amax,
                safety=self.qp_limiter_safety,
                iters=self.qp_limiter_iters
            )

        # ---- QP-EVAL AFTER ----
        e1 = qp_eval_pose(pose_f, dt, self.pos_vmax, self.pos_amax, self.ang_vmax, self.ang_amax)
        self.get_logger().info("[QP-EVAL] ===== AFTER pose limiter =====")
        self.get_logger().info(format_qp_eval(e1, self.pos_vmax, self.pos_amax, self.ang_vmax, self.ang_amax))

        # deviation stats (how much we changed the path)
        dpos = np.linalg.norm(pose_f[:, 0:3] - pose_raw[:, 0:3], axis=1)
        dang = np.linalg.norm(pose_f[:, 3:6] - pose_raw[:, 3:6], axis=1)
        self.get_logger().info(
            f"[POSE-DELTA] pos: rms={float(np.sqrt(np.mean(dpos*dpos))):.3f} mm, max={float(np.max(dpos)):.3f} mm | "
            f"ang: rms={float(np.sqrt(np.mean(dang*dang))):.5f} rad, max={float(np.max(dang)):.5f} rad"
        )

        # ---- Force: keep your pipeline (no retiming) ----
        force_f = force_raw.copy()

        for j in range(3):
            force_f[:, j] = whittaker_smooth_1d(force_f[:, j], self.lam_force, self.cg_tol, self.cg_max_iter)
        for j in range(3):
            force_f[:, j] = ema_filtfilt_1d(force_f[:, j], self.ema_alpha_force)

        if self.force_xy_zero:
            force_f[:, 0] = 0.0
            force_f[:, 1] = 0.0

        force_f[:, 2] = np.clip(force_f[:, 2], self.fz_min, self.fz_max)
        force_f = apply_edge_force_window(
            force_f,
            playback_hz=self.playback_hz,
            edge_zero_sec=self.edge_force_zero_sec,
            fade_sec=self.edge_force_fade_sec
        )
        force_f[:, 2] = np.clip(force_f[:, 2], self.fz_min, self.fz_max)

        # ---- Pre-contact force gating (on filtered fz) ----
        contact_idx = -1
        if self.precontact_force_zero and force_f.shape[0] > 0:
            contact_idx = find_contact_index_fz(
                fz=force_f[:, 2],
                th_on=self.contact_fz_on,
                th_off=self.contact_fz_off,
                consecutive_on=self.contact_consec_on,
                consecutive_off=self.contact_consec_off,
            )
            if contact_idx < 0:
                self.get_logger().warn(
                    f"[CONTACT] Not detected (fz never >= {self.contact_fz_on} stably). Zeroing ALL desired forces."
                )
                force_f[:, :] = 0.0
            else:
                self.get_logger().info(
                    f"[CONTACT] Detected at idx={contact_idx}/{force_f.shape[0]} (t={contact_idx*dt:.3f}s) "
                    f"-> Zeroing forces for [0:{contact_idx})"
                )
                force_f = zero_force_before_contact(force_f, contact_idx)

        # final
        out = np.hstack([pose_f, force_f]).astype(np.float64)

        # 1) local save
        np.savetxt(self.file_path, out, fmt="%.10f")
        self.get_logger().info(f"Saved local file: {self.file_path}  (raw rows={raw.shape[0]} -> out rows={out.shape[0]})")

        # 2) plots (optional)
        if self.plot_enable:
            self.save_plots(out, dt=dt, contact_idx=contact_idx)

        # 3) send to B PC
        try:
            target_user = "nrs_forcecon"
            target_ip = "192.168.0.151"
            target_dir = "/home/nrs_forcecon/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/"
            self.get_logger().info(f"Sending file to Control PC ({target_ip})...")
            cmd = ["scp", self.file_path, f"{target_user}@{target_ip}:{target_dir}"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"SUCCESS: File transferred to B PC ({target_dir})")
            else:
                self.get_logger().error(f"SCP FAILED:\n{result.stderr}")
        except Exception as e:
            self.get_logger().error(f"File transfer exception: {e}")

    def save_plots(self, out: np.ndarray, dt: float, contact_idx: int = -1):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = np.arange(out.shape[0], dtype=np.float64) * dt
        x, y, z = out[:, 0], out[:, 1], out[:, 2]
        wx, wy, wz = out[:, 3], out[:, 4], out[:, 5]
        fx, fy, fz = out[:, 6], out[:, 7], out[:, 8]

        prefix = os.path.join(self.save_dir, self.plot_prefix)

        def vnorm(sig3):
            v = np.diff(sig3, axis=0) / dt
            return np.linalg.norm(v, axis=1)

        pos = out[:, 0:3]
        ang = out[:, 3:6]
        vpos_n = vnorm(pos)
        vang_n = vnorm(ang)
        apos_n = vnorm(np.diff(pos, axis=0) / dt)  # diff(v)/dt
        aang_n = vnorm(np.diff(ang, axis=0) / dt)

        plt.figure()
        plt.title("XYZ (final)")
        plt.plot(t, x, label="x")
        plt.plot(t, y, label="y")
        plt.plot(t, z, label="z")
        if contact_idx >= 0:
            plt.axvline(t[contact_idx], linestyle="--", label="contact")
        plt.xlabel("time [s]")
        plt.ylabel("pos [mm]")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_xyz.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("Omega (final)")
        plt.plot(t, wx, label="wx")
        plt.plot(t, wy, label="wy")
        plt.plot(t, wz, label="wz")
        if contact_idx >= 0:
            plt.axvline(t[contact_idx], linestyle="--", label="contact")
        plt.xlabel("time [s]")
        plt.ylabel("rad")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_omega.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("Force (final)")
        plt.plot(t, fx, label="fx")
        plt.plot(t, fy, label="fy")
        plt.plot(t, fz, label="fz")
        if contact_idx >= 0:
            plt.axvline(t[contact_idx], linestyle="--", label="contact")
            plt.axhline(self.contact_fz_on, linestyle=":", label="fz_on")
        plt.xlabel("time [s]")
        plt.ylabel("N")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_force.png", dpi=150, bbox_inches="tight")
        plt.close()

        # speed/acc proxy plots
        tt1 = t[1:]
        tt2 = t[2:]

        plt.figure()
        plt.title("pos |v| (proxy)")
        plt.plot(tt1, vpos_n, label="|v|")
        plt.axhline(self.pos_vmax, linestyle="--", label="vmax")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_pos_speed.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("pos |a| (proxy)")
        plt.plot(tt2, apos_n, label="|a|")
        plt.axhline(self.pos_amax, linestyle="--", label="amax")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_pos_acc.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("ang |w| (proxy)")
        plt.plot(tt1, vang_n, label="|w|")
        plt.axhline(self.ang_vmax, linestyle="--", label="wmax")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_ang_speed.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("ang |alpha| (proxy)")
        plt.plot(tt2, aang_n, label="|alpha|")
        plt.axhline(self.ang_amax, linestyle="--", label="amax")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_plot_ang_acc.png", dpi=150, bbox_inches="tight")
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
