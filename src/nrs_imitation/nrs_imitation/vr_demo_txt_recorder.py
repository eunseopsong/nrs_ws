#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import math
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple, Dict

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# ----------------------------
# Utility: percentile safely
# ----------------------------
def pctl(x: np.ndarray, q: float) -> float:
    if x.size == 0:
        return 0.0
    return float(np.percentile(x, q))


def norm_rows(x: np.ndarray) -> np.ndarray:
    return np.linalg.norm(x, axis=1)


# ----------------------------
# Hampel filter (per-dim)
# ----------------------------
def hampel_1d(x: np.ndarray, win: int, n_sigmas: float) -> np.ndarray:
    """
    Robust outlier filter. Replaces outliers with local median.
    win: half-window size (samples)
    """
    if win <= 0:
        return x.copy()
    n = x.size
    y = x.copy()
    k = 1.4826  # scale factor for MAD -> std approx
    for i in range(n):
        i0 = max(0, i - win)
        i1 = min(n, i + win + 1)
        w = x[i0:i1]
        med = np.median(w)
        mad = np.median(np.abs(w - med))
        sigma = k * mad + 1e-12
        if abs(x[i] - med) > n_sigmas * sigma:
            y[i] = med
    return y


def hampel_nd(X: np.ndarray, win: int, n_sigmas: float) -> np.ndarray:
    Y = X.copy()
    for d in range(X.shape[1]):
        Y[:, d] = hampel_1d(X[:, d], win=win, n_sigmas=n_sigmas)
    return Y


# ----------------------------
# Whittaker smoother via CG
#   minimize ||y-z||^2 + lam*||D2 z||^2
# ----------------------------
def _apply_D2(x: np.ndarray) -> np.ndarray:
    # length N-2: x[i] - 2x[i+1] + x[i+2]
    return x[:-2] - 2.0 * x[1:-1] + x[2:]


def _apply_D2t(u: np.ndarray, n: int) -> np.ndarray:
    # u length n-2 -> out length n
    out = np.zeros(n, dtype=np.float64)
    out[:-2] += u
    out[1:-1] += -2.0 * u
    out[2:] += u
    return out


def whittaker_cg_1d(y: np.ndarray, lam: float, cg_iters: int = 200, tol: float = 1e-8) -> np.ndarray:
    """
    Solve (I + lam*D2^T D2) z = y with conjugate gradient.
    """
    n = y.size
    if n < 5 or lam <= 0.0:
        return y.copy()

    def A(x: np.ndarray) -> np.ndarray:
        d2 = _apply_D2(x)
        return x + lam * _apply_D2t(d2, n)

    x = y.copy()  # good initial
    r = y - A(x)
    p = r.copy()
    rr = float(r @ r)

    if rr < tol:
        return x

    for _ in range(cg_iters):
        Ap = A(p)
        denom = float(p @ Ap) + 1e-12
        alpha = rr / denom
        x = x + alpha * p
        r = r - alpha * Ap
        rr_new = float(r @ r)
        if rr_new < (tol * tol) * float(y @ y + 1e-12):
            break
        beta = rr_new / (rr + 1e-12)
        p = r + beta * p
        rr = rr_new
    return x


def whittaker_cg_nd(Y: np.ndarray, lam: float, cg_iters: int = 200, tol: float = 1e-8) -> np.ndarray:
    Z = np.empty_like(Y)
    for d in range(Y.shape[1]):
        Z[:, d] = whittaker_cg_1d(Y[:, d], lam=lam, cg_iters=cg_iters, tol=tol)
    return Z


def ema_nd(Y: np.ndarray, alpha: float) -> np.ndarray:
    if alpha <= 0.0 or alpha >= 1.0:
        return Y.copy()
    Z = Y.copy()
    for i in range(1, Y.shape[0]):
        Z[i] = alpha * Y[i] + (1.0 - alpha) * Z[i - 1]
    return Z


# ----------------------------
# QP-proxy kinematics eval
# ----------------------------
@dataclass
class Limits:
    pos_vmax: float   # mm/s
    pos_amax: float   # mm/s^2
    ang_vmax: float   # rad/s
    ang_amax: float   # rad/s^2
    pos_jmax: float   # mm/s^3 (proxy)
    ang_jmax: float   # rad/s^3 (proxy)


@dataclass
class EvalStats:
    N: int
    dt: float
    T: float
    vpos_max: float
    apos_max: float
    vang_max: float
    aang_max: float
    jpos_max: float
    jang_max: float
    vpos_p95: float
    apos_p95: float
    vang_p95: float
    aang_p95: float
    jpos_p95: float
    jang_p95: float
    vpos_mean: float
    apos_mean: float
    vang_mean: float
    aang_mean: float
    viol_v: float
    viol_a: float
    viol_w: float
    viol_alpha: float


def eval_qp_proxy(pose6: np.ndarray, dt: float, lim: Limits, safety: float = 1.0) -> Tuple[EvalStats, Dict[str, np.ndarray]]:
    """
    pose6: [x y z wx wy wz] with x,y,z in mm; w* in rad
    """
    N = int(pose6.shape[0])
    T = dt * max(0, (N - 1))

    dp = pose6[1:, :3] - pose6[:-1, :3]
    dw = pose6[1:, 3:] - pose6[:-1, 3:]
    vpos = norm_rows(dp) / dt
    vang = norm_rows(dw) / dt

    v = (pose6[1:, :] - pose6[:-1, :]) / dt  # (N-1,6)
    a = (v[1:, :] - v[:-1, :]) / dt          # (N-2,6)
    apos = norm_rows(a[:, :3])
    aang = norm_rows(a[:, 3:])

    j = (a[1:, :] - a[:-1, :]) / dt          # (N-3,6)
    jpos = norm_rows(j[:, :3])
    jang = norm_rows(j[:, 3:])

    vpos_max = float(vpos.max()) if vpos.size else 0.0
    vang_max = float(vang.max()) if vang.size else 0.0
    apos_max = float(apos.max()) if apos.size else 0.0
    aang_max = float(aang.max()) if aang.size else 0.0
    jpos_max = float(jpos.max()) if jpos.size else 0.0
    jang_max = float(jang.max()) if jang.size else 0.0

    vpos_lim = lim.pos_vmax * safety
    apos_lim = lim.pos_amax * safety
    vang_lim = lim.ang_vmax * safety
    aang_lim = lim.ang_amax * safety

    viol_v = float(np.mean(vpos > vpos_lim)) if vpos.size else 0.0
    viol_w = float(np.mean(vang > vang_lim)) if vang.size else 0.0
    viol_a = float(np.mean(apos > apos_lim)) if apos.size else 0.0
    viol_alpha = float(np.mean(aang > aang_lim)) if aang.size else 0.0

    st = EvalStats(
        N=N, dt=dt, T=T,
        vpos_max=vpos_max, apos_max=apos_max, vang_max=vang_max, aang_max=aang_max,
        jpos_max=jpos_max, jang_max=jang_max,
        vpos_p95=pctl(vpos, 95), apos_p95=pctl(apos, 95), vang_p95=pctl(vang, 95), aang_p95=pctl(aang, 95),
        jpos_p95=pctl(jpos, 95), jang_p95=pctl(jang, 95),
        vpos_mean=float(vpos.mean()) if vpos.size else 0.0,
        apos_mean=float(apos.mean()) if apos.size else 0.0,
        vang_mean=float(vang.mean()) if vang.size else 0.0,
        aang_mean=float(aang.mean()) if aang.size else 0.0,
        viol_v=viol_v, viol_a=viol_a, viol_w=viol_w, viol_alpha=viol_alpha
    )

    debug = {"vpos": vpos, "vang": vang, "apos": apos, "aang": aang, "jpos": jpos, "jang": jang}
    return st, debug


def print_eval(logger, title: str, st: EvalStats, lim: Limits, safety: float):
    logger.info(f"[QP-EVAL] ===== {title} =====")
    logger.info(
        f"\n  N={st.N}  dt={st.dt:.6f}s  T={st.T:.3f}s"
        f"\n  pos |v|: max={st.vpos_max:.3f} (lim {lim.pos_vmax:.3f}, {(st.vpos_max/(lim.pos_vmax+1e-9)):.3f}x), "
        f"p95={st.vpos_p95:.3f}, mean={st.vpos_mean:.3f}  [mm/s]"
        f"\n  pos |a|: max={st.apos_max:.3f} (lim {lim.pos_amax:.3f}, {(st.apos_max/(lim.pos_amax+1e-9)):.3f}x), "
        f"p95={st.apos_p95:.3f}, mean={st.apos_mean:.3f}  [mm/s^2]"
        f"\n  ang |w|: max={st.vang_max:.3f} (lim {lim.ang_vmax:.3f}, {(st.vang_max/(lim.ang_vmax+1e-9)):.3f}x), "
        f"p95={st.vang_p95:.3f}, mean={st.vang_mean:.3f}  [rad/s]"
        f"\n  ang |alpha|: max={st.aang_max:.3f} (lim {lim.ang_amax:.3f}, {(st.aang_max/(lim.ang_amax+1e-9)):.3f}x), "
        f"p95={st.aang_p95:.3f}, mean={st.aang_mean:.3f}  [rad/s^2]"
        f"\n  jerk(ref): pos max={st.jpos_max:.3f} [mm/s^3], ang max={st.jang_max:.3f} [rad/s^3]"
        f"\n  violation_rate(safety={safety:.3f}): vpos={100*st.viol_v:.3f}%, apos={100*st.viol_a:.3f}%, "
        f"vang={100*st.viol_w:.3f}%, aang={100*st.viol_alpha:.3f}%"
    )


def topk_violations(arr: np.ndarray, limit: float, k: int = 6) -> np.ndarray:
    if arr.size == 0:
        return np.zeros((0,), dtype=np.int64)
    idx = np.where(arr > limit)[0]
    if idx.size == 0:
        return np.zeros((0,), dtype=np.int64)
    vals = arr[idx]
    order = np.argsort(-vals)
    return idx[order[:k]]


# ----------------------------
# Uniform upsample (time dilation)
# ----------------------------
def upsample_linear(X: np.ndarray, factor: int) -> np.ndarray:
    """
    factor=k: (N-1)*k + 1 rows, linear interpolation between samples.
    """
    if factor <= 1:
        return X.copy()
    N, D = X.shape
    outN = (N - 1) * factor + 1
    out = np.empty((outN, D), dtype=np.float64)

    frac = (np.arange(factor, dtype=np.float64) / float(factor)).reshape(-1, 1)  # (k,1)
    for i in range(N - 1):
        base = i * factor
        delta = (X[i + 1] - X[i]).reshape(1, -1)
        out[base:base + factor, :] = X[i].reshape(1, -1) + frac * delta
    out[-1, :] = X[-1, :]
    return out


# ----------------------------
# Contact detection (fz hysteresis + consecutive)
# ----------------------------
def detect_contact_idx(fz: np.ndarray, fz_on: float, fz_off: float, consec_on: int, consec_off: int) -> Optional[int]:
    """
    Returns first index where contact is considered ON.
    """
    on = False
    cnt_on = 0
    cnt_off = 0
    first_on_idx = None

    for i in range(fz.size):
        if not on:
            if fz[i] >= fz_on:
                cnt_on += 1
                if cnt_on >= consec_on:
                    on = True
                    first_on_idx = i - consec_on + 1
                    break
            else:
                cnt_on = 0
        else:
            # not used (we only need first ON)
            if fz[i] <= fz_off:
                cnt_off += 1
                if cnt_off >= consec_off:
                    on = False
                    cnt_off = 0
            else:
                cnt_off = 0

    return first_on_idx


# ----------------------------
# Main Node
# ----------------------------
class VrDemoTxtRecorder(Node):
    def __init__(self):
        super().__init__("vr_demo_txt_recorder")

        # --- topics
        self.declare_parameter("pose_topic", "/calibrated_pose")   # Float64MultiArray [x y z wx wy wz] (m,rad)
        self.declare_parameter("force_topic", "/vive_force")       # Float64MultiArray [fx fy fz] (N)

        # --- recorder timing
        self.declare_parameter("record_hz", 125.0)       # sampling while recording
        self.declare_parameter("require_fresh_sec", 0.2)

        # --- episode rule (same as your logs)
        self.declare_parameter("start_abs_fx", 10.0)
        self.declare_parameter("stop_abs_fy", 10.0)

        # --- save path
        self.declare_parameter("save_path", "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/cmd_continue9D.txt")

        # --- SCP transfer
        self.declare_parameter("transfer_enable", True)
        self.declare_parameter("remote_user", "nrs_forcecon")
        self.declare_parameter("remote_ip", "192.168.0.151")
        self.declare_parameter("remote_dir", "/home/nrs_forcecon/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/")

        # --- force shaping
        self.declare_parameter("zero_xy_forces", True)
        self.declare_parameter("force_clamp_abs", 200.0)           # N
        self.declare_parameter("force_ema_alpha", 0.2)             # 0 disables
        self.declare_parameter("edge_force_zero_sec", 0.5)         # smaller than 5s 권장
        self.declare_parameter("edge_force_fade_sec", 0.3)         # linear fade

        # --- pre-contact gating
        self.declare_parameter("precontact_gating", True)
        self.declare_parameter("fz_on", 5.0)
        self.declare_parameter("fz_off", 3.0)
        self.declare_parameter("consec_on", 10)
        self.declare_parameter("consec_off", 10)

        # --- pose smoothing
        self.declare_parameter("hampel_enable", True)
        self.declare_parameter("hampel_win", 6)
        self.declare_parameter("hampel_sig", 3.0)

        self.declare_parameter("whittaker_auto", True)
        self.declare_parameter("lam_pos_init", 20000.0)
        self.declare_parameter("lam_ang_init", 200.0)
        self.declare_parameter("lam_growth", 3.0)
        self.declare_parameter("lam_iters", 6)
        self.declare_parameter("cg_iters", 200)
        self.declare_parameter("cg_tol", 1e-8)
        self.declare_parameter("pose_ema_enable", False)
        self.declare_parameter("pose_ema_alpha", 0.2)

        # --- QP-proxy limits (your logs)
        self.declare_parameter("pos_vmax", 30.0)
        self.declare_parameter("pos_amax", 120.0)
        self.declare_parameter("ang_vmax", 0.6)
        self.declare_parameter("ang_amax", 3.0)

        # jerk limits (proxy) - 너무 낮게 잡으면 N이 과도하게 커짐
        self.declare_parameter("pos_jmax", 5000.0)
        self.declare_parameter("ang_jmax", 80.0)

        self.declare_parameter("safety", 1.05)

        # --- retiming (the 핵심)
        self.declare_parameter("retime_enable", True)
        self.declare_parameter("retime_use_jerk", True)
        self.declare_parameter("retime_max_k", 20)      # 여기서 폭증 방지
        self.declare_parameter("retime_passes", 3)      # 부족하면 여러 번 나눠서 적용

        # -------------------------
        # internal state
        # -------------------------
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)

        self.record_hz = float(self.get_parameter("record_hz").value)
        self.dt = 1.0 / max(1e-9, self.record_hz)
        self.require_fresh_sec = float(self.get_parameter("require_fresh_sec").value)

        self.save_path = str(self.get_parameter("save_path").value)

        self.start_abs_fx = float(self.get_parameter("start_abs_fx").value)
        self.stop_abs_fy = float(self.get_parameter("stop_abs_fy").value)

        self.transfer_enable = bool(self.get_parameter("transfer_enable").value)
        self.remote_user = str(self.get_parameter("remote_user").value)
        self.remote_ip = str(self.get_parameter("remote_ip").value)
        self.remote_dir = str(self.get_parameter("remote_dir").value)

        self.zero_xy_forces = bool(self.get_parameter("zero_xy_forces").value)
        self.force_clamp_abs = float(self.get_parameter("force_clamp_abs").value)
        self.force_ema_alpha = float(self.get_parameter("force_ema_alpha").value)
        self.edge_force_zero_sec = float(self.get_parameter("edge_force_zero_sec").value)
        self.edge_force_fade_sec = float(self.get_parameter("edge_force_fade_sec").value)

        self.precontact_gating = bool(self.get_parameter("precontact_gating").value)
        self.fz_on = float(self.get_parameter("fz_on").value)
        self.fz_off = float(self.get_parameter("fz_off").value)
        self.consec_on = int(self.get_parameter("consec_on").value)
        self.consec_off = int(self.get_parameter("consec_off").value)

        self.hampel_enable = bool(self.get_parameter("hampel_enable").value)
        self.hampel_win = int(self.get_parameter("hampel_win").value)
        self.hampel_sig = float(self.get_parameter("hampel_sig").value)

        self.whittaker_auto = bool(self.get_parameter("whittaker_auto").value)
        self.lam_pos_init = float(self.get_parameter("lam_pos_init").value)
        self.lam_ang_init = float(self.get_parameter("lam_ang_init").value)
        self.lam_growth = float(self.get_parameter("lam_growth").value)
        self.lam_iters = int(self.get_parameter("lam_iters").value)
        self.cg_iters = int(self.get_parameter("cg_iters").value)
        self.cg_tol = float(self.get_parameter("cg_tol").value)
        self.pose_ema_enable = bool(self.get_parameter("pose_ema_enable").value)
        self.pose_ema_alpha = float(self.get_parameter("pose_ema_alpha").value)

        self.safety = float(self.get_parameter("safety").value)

        self.lim = Limits(
            pos_vmax=float(self.get_parameter("pos_vmax").value),
            pos_amax=float(self.get_parameter("pos_amax").value),
            ang_vmax=float(self.get_parameter("ang_vmax").value),
            ang_amax=float(self.get_parameter("ang_amax").value),
            pos_jmax=float(self.get_parameter("pos_jmax").value),
            ang_jmax=float(self.get_parameter("ang_jmax").value),
        )

        self.retime_enable = bool(self.get_parameter("retime_enable").value)
        self.retime_use_jerk = bool(self.get_parameter("retime_use_jerk").value)
        self.retime_max_k = int(self.get_parameter("retime_max_k").value)
        self.retime_passes = int(self.get_parameter("retime_passes").value)

        # latest samples
        self.latest_pose6_mm_rad: Optional[np.ndarray] = None
        self.latest_force3_N: Optional[np.ndarray] = None
        self.latest_pose_t: float = 0.0
        self.latest_force_t: float = 0.0

        # episode buffer
        self.episode_active = False
        self.buf_pose = []
        self.buf_force = []
        self.buf_t = []

        # subs + timer
        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self.cb_pose, 50)
        self.sub_force = self.create_subscription(Float64MultiArray, self.force_topic, self.cb_force, 50)
        self.timer = self.create_timer(self.dt, self.cb_timer)

        self.get_logger().info(f"Initialized. Local save path: {self.save_path}")
        self.get_logger().info(
            f"Pose smoothing: Hampel={self.hampel_enable}(win={self.hampel_win}, sig={self.hampel_sig}) + "
            f"WhittakerAuto={self.whittaker_auto}(lam_pos_init={self.lam_pos_init}, lam_ang_init={self.lam_ang_init}, "
            f"growth={self.lam_growth}, iters={self.lam_iters}) + PoseEMA={self.pose_ema_enable}(alpha={self.pose_ema_alpha})"
        )
        self.get_logger().info(
            f"Pre-contact force gating: {self.precontact_gating}  (fz_on={self.fz_on}, fz_off={self.fz_off}, "
            f"consec_on={self.consec_on}, consec_off={self.consec_off})"
        )
        self.get_logger().info(
            f"QP-proxy limits: pos_vmax={self.lim.pos_vmax} mm/s, pos_amax={self.lim.pos_amax} mm/s^2, "
            f"ang_vmax={self.lim.ang_vmax} rad/s, ang_amax={self.lim.ang_amax} rad/s^2, safety={self.safety}"
        )
        self.get_logger().info(
            f"Retime: enable={self.retime_enable}, use_jerk={self.retime_use_jerk}, max_k={self.retime_max_k}, passes={self.retime_passes}"
        )

    # -------------------------
    # callbacks
    # -------------------------
    def cb_pose(self, msg: Float64MultiArray):
        # [x y z wx wy wz] (m, rad) -> (mm, rad)
        if len(msg.data) < 6:
            return
        x, y, z, wx, wy, wz = msg.data[:6]
        self.latest_pose6_mm_rad = np.array([1000.0 * x, 1000.0 * y, 1000.0 * z, wx, wy, wz], dtype=np.float64)
        self.latest_pose_t = time.time()

    def cb_force(self, msg: Float64MultiArray):
        # [fx fy fz] (N)
        if len(msg.data) < 3:
            return
        fx, fy, fz = msg.data[:3]
        self.latest_force3_N = np.array([fx, fy, fz], dtype=np.float64)
        self.latest_force_t = time.time()

        # episode rule
        if (not self.episode_active) and (abs(fx) >= self.start_abs_fx):
            self.episode_active = True
            self.buf_pose.clear()
            self.buf_force.clear()
            self.buf_t.clear()
            self.get_logger().info("=== EPISODE STARTED ===")
            if self.latest_pose6_mm_rad is None:
                self.get_logger().info(f"[START] fx={fx:.3f} fy={fy:.3f} fz={fz:.3f} | (pose not received yet)")
        elif self.episode_active and (abs(fy) >= self.stop_abs_fy):
            self.get_logger().info("=== EPISODE ENDED ===")
            self.finish_episode()

    def cb_timer(self):
        if not self.episode_active:
            return

        now = time.time()

        # freshness check
        if self.latest_pose6_mm_rad is None or (now - self.latest_pose_t) > self.require_fresh_sec:
            return
        if self.latest_force3_N is None or (now - self.latest_force_t) > self.require_fresh_sec:
            return

        self.buf_pose.append(self.latest_pose6_mm_rad.copy())
        self.buf_force.append(self.latest_force3_N.copy())
        self.buf_t.append(now)

    # -------------------------
    # processing pipeline
    # -------------------------
    def _force_process(self, F: np.ndarray) -> np.ndarray:
        # clamp
        Fp = F.copy()
        Fp = np.clip(Fp, -self.force_clamp_abs, self.force_clamp_abs)
        if self.zero_xy_forces:
            Fp[:, 0] = 0.0
            Fp[:, 1] = 0.0

        # EMA (optional)
        if 0.0 < self.force_ema_alpha < 1.0:
            Fp = ema_nd(Fp, alpha=self.force_ema_alpha)

        return Fp

    def _apply_edge_force_window(self, F: np.ndarray, hz: float) -> np.ndarray:
        """
        Zero first/last edge_force_zero_sec, with linear fade edge_force_fade_sec.
        """
        out = F.copy()
        n = out.shape[0]
        zN = int(round(self.edge_force_zero_sec * hz))
        fN = int(round(self.edge_force_fade_sec * hz))
        zN = max(0, min(n, zN))
        fN = max(0, min(n, fN))

        # start
        if zN > 0:
            out[:zN, :] = 0.0
        if fN > 0 and (zN + fN) < n:
            w = np.linspace(0.0, 1.0, fN, dtype=np.float64).reshape(-1, 1)
            out[zN:zN + fN, :] = w * out[zN:zN + fN, :]

        # end
        if zN > 0:
            out[n - zN:, :] = 0.0
        if fN > 0 and (n - zN - fN) > 0:
            w = np.linspace(1.0, 0.0, fN, dtype=np.float64).reshape(-1, 1)
            out[n - zN - fN:n - zN, :] = w * out[n - zN - fN:n - zN, :]

        return out

    def _pose_smooth(self, P: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
        """
        Hampel + Whittaker(auto) + optional EMA
        """
        P0 = P.copy()

        if self.hampel_enable:
            P0 = hampel_nd(P0, win=self.hampel_win, n_sigmas=self.hampel_sig)

        if not self.whittaker_auto:
            # single-pass
            Pp = P0.copy()
            Pp[:, :3] = whittaker_cg_nd(Pp[:, :3], lam=self.lam_pos_init, cg_iters=self.cg_iters, tol=self.cg_tol)
            Pp[:, 3:] = whittaker_cg_nd(Pp[:, 3:], lam=self.lam_ang_init, cg_iters=self.cg_iters, tol=self.cg_tol)
            if self.pose_ema_enable:
                Pp = ema_nd(Pp, alpha=self.pose_ema_alpha)
            info = {"lam_pos": self.lam_pos_init, "lam_ang": self.lam_ang_init}
            return Pp, info

        # auto: increase lambda while keeping delta bounded
        lam_pos = self.lam_pos_init
        lam_ang = self.lam_ang_init

        best = None
        best_score = 1e18
        best_info = {"lam_pos": lam_pos, "lam_ang": lam_ang}

        # allowed deformation (너무 왜곡되면 stop)
        max_pos_delta_allow = 5.0      # mm
        max_ang_delta_allow = 0.03     # rad

        for _ in range(max(1, self.lam_iters)):
            Pp = P0.copy()
            Pp[:, :3] = whittaker_cg_nd(Pp[:, :3], lam=lam_pos, cg_iters=self.cg_iters, tol=self.cg_tol)
            Pp[:, 3:] = whittaker_cg_nd(Pp[:, 3:], lam=lam_ang, cg_iters=self.cg_iters, tol=self.cg_tol)

            if self.pose_ema_enable:
                Pp = ema_nd(Pp, alpha=self.pose_ema_alpha)

            # deformation check
            dpos = norm_rows(Pp[:, :3] - P[:, :3])
            dang = norm_rows(Pp[:, 3:] - P[:, 3:])
            if float(dpos.max()) > max_pos_delta_allow or float(dang.max()) > max_ang_delta_allow:
                break

            st, _ = eval_qp_proxy(Pp, self.dt, self.lim, safety=self.safety)
            # score: reduce a/jerk mainly (v는 retime이 해결)
            score = max(st.apos_p95 / (self.lim.pos_amax + 1e-9),
                        st.aang_p95 / (self.lim.ang_amax + 1e-9),
                        st.jpos_p95 / (self.lim.pos_jmax + 1e-9),
                        st.jang_p95 / (self.lim.ang_jmax + 1e-9)) \
                    + 0.05 * (float(dpos.mean()) / 1.0)

            if score < best_score:
                best_score = score
                best = Pp
                best_info = {"lam_pos": lam_pos, "lam_ang": lam_ang}

            lam_pos *= self.lam_growth
            lam_ang *= self.lam_growth

        if best is None:
            best = P0
        return best, best_info

    def _retime_uniform(self, P: np.ndarray, F: np.ndarray) -> Tuple[np.ndarray, np.ndarray, int]:
        """
        Uniform time dilation by integer factor(s), using correct scaling:
          v -> v/k, a -> a/k^2, jerk -> jerk/k^3
        """
        if not self.retime_enable:
            return P, F, 1

        Pcur = P.copy()
        Fcur = F.copy()
        k_total = 1

        for _ in range(max(1, self.retime_passes)):
            st, dbg = eval_qp_proxy(Pcur, self.dt, self.lim, safety=self.safety)

            # required factor from maxima
            rv = max(
                st.vpos_max / (self.lim.pos_vmax * self.safety + 1e-9),
                st.vang_max / (self.lim.ang_vmax * self.safety + 1e-9),
            )
            ra = max(
                math.sqrt(st.apos_max / (self.lim.pos_amax * self.safety + 1e-9)),
                math.sqrt(st.aang_max / (self.lim.ang_amax * self.safety + 1e-9)),
            )

            rj = 1.0
            if self.retime_use_jerk:
                rj = max(
                    (st.jpos_max / (self.lim.pos_jmax * self.safety + 1e-9)) ** (1.0 / 3.0),
                    (st.jang_max / (self.lim.ang_jmax * self.safety + 1e-9)) ** (1.0 / 3.0),
                )

            r_need = max(1.0, rv, ra, rj)
            k_need = int(math.ceil(r_need))

            # cap (avoid explosion)
            remaining = max(1, self.retime_max_k // max(1, k_total))
            k_need = min(k_need, remaining)

            # already ok or can't increase more
            if k_need <= 1:
                break

            Pcur = upsample_linear(Pcur, k_need)
            Fcur = upsample_linear(Fcur, k_need)
            k_total *= k_need

        return Pcur, Fcur, k_total

    def finish_episode(self):
        if len(self.buf_pose) < 10:
            self.get_logger().warn("Episode too short. Discarding.")
            self.episode_active = False
            return

        P = np.asarray(self.buf_pose, dtype=np.float64)   # (N,6) [mm, rad]
        F = np.asarray(self.buf_force, dtype=np.float64)  # (N,3) [N]
        rawN = int(P.shape[0])

        # --- eval raw
        st_raw, dbg_raw = eval_qp_proxy(P, self.dt, self.lim, safety=self.safety)
        print_eval(self.get_logger(), "BEFORE pose smoothing (RAW)", st_raw, self.lim, self.safety)

        # top violators (optional hint)
        vpos_lim = self.lim.pos_vmax * self.safety
        apos_lim = self.lim.pos_amax * self.safety
        vang_lim = self.lim.ang_vmax * self.safety
        aang_lim = self.lim.ang_amax * self.safety

        idx_v = topk_violations(dbg_raw["vpos"], vpos_lim)
        if idx_v.size:
            self.get_logger().info("[QP-EVAL] pos|v| top6 violating indices:")
            for i in idx_v:
                self.get_logger().info(f"    idx={int(i):6d}, t={float(i)*self.dt:8.3f}s, value={float(dbg_raw['vpos'][i]):.6f}")

        idx_a = topk_violations(dbg_raw["apos"], apos_lim)
        if idx_a.size:
            self.get_logger().info("[QP-EVAL] pos|a| top6 violating indices:")
            for i in idx_a:
                self.get_logger().info(f"    idx={int(i):6d}, t={float(i+1)*self.dt:8.3f}s, value={float(dbg_raw['apos'][i]):.6f}")

        idx_w = topk_violations(dbg_raw["vang"], vang_lim)
        if idx_w.size:
            self.get_logger().info("[QP-EVAL] ang|w| top6 violating indices:")
            for i in idx_w:
                self.get_logger().info(f"    idx={int(i):6d}, t={float(i)*self.dt:8.3f}s, value={float(dbg_raw['vang'][i]):.6f}")

        idx_alpha = topk_violations(dbg_raw["aang"], aang_lim)
        if idx_alpha.size:
            self.get_logger().info("[QP-EVAL] ang|alpha| top6 violating indices:")
            for i in idx_alpha:
                self.get_logger().info(f"    idx={int(i):6d}, t={float(i+1)*self.dt:8.3f}s, value={float(dbg_raw['aang'][i]):.6f}")

        # --- process force base (clamp/ema/xy zero)
        Fp = self._force_process(F)

        # --- pose smoothing
        Ps, info = self._pose_smooth(P)
        st_sm, _ = eval_qp_proxy(Ps, self.dt, self.lim, safety=self.safety)
        print_eval(self.get_logger(), "AFTER pose smoothing", st_sm, self.lim, self.safety)
        self.get_logger().info(f"[POSE-SMOOTH] used_lams={info}")

        # delta report
        dpos = norm_rows(Ps[:, :3] - P[:, :3])
        dang = norm_rows(Ps[:, 3:] - P[:, 3:])
        self.get_logger().info(
            f"[POSE-DELTA] pos: rms={float(np.sqrt(np.mean(dpos**2))):.3f} mm, max={float(dpos.max()):.3f} mm | "
            f"ang: rms={float(np.sqrt(np.mean(dang**2))):.6f} rad, max={float(dang.max()):.6f} rad"
        )

        # --- retime (uniform, correct scaling)
        Pr, Fr, k_total = self._retime_uniform(Ps, Fp)

        st_rt, _ = eval_qp_proxy(Pr, self.dt, self.lim, safety=self.safety)
        print_eval(self.get_logger(), "AFTER retiming (pose)", st_rt, self.lim, self.safety)
        if k_total > 1:
            self.get_logger().info(f"[QP-EVAL] Applied time-scale k_total={k_total}  (rows: {Ps.shape[0]} -> {Pr.shape[0]})")

        # --- contact gating on retimed force (before edge window)
        if self.precontact_gating:
            cidx = detect_contact_idx(Fr[:, 2], self.fz_on, self.fz_off, self.consec_on, self.consec_off)
            if cidx is not None and cidx > 0:
                self.get_logger().info(f"[CONTACT] Detected at idx={cidx}/{Pr.shape[0]} (t={cidx*self.dt:.3f}s) -> Zeroing forces for [0:{cidx})")
                Fr[:cidx, :] = 0.0

        # --- edge force window (to avoid start/end force shocks)
        Fr = self._apply_edge_force_window(Fr, hz=self.record_hz)

        # --- save txt
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        out = np.hstack([Pr, Fr])  # (N,9)

        with open(self.save_path, "w") as f:
            for row in out:
                f.write("\t".join([f"{v:.6f}" for v in row.tolist()]) + "\n")

        self.get_logger().info(
            f"Saved local file: {self.save_path}  (raw rows={rawN} -> out rows={out.shape[0]})"
        )

        # --- transfer
        if self.transfer_enable:
            self._transfer_file()

        # reset
        self.episode_active = False
        self.buf_pose.clear()
        self.buf_force.clear()
        self.buf_t.clear()

        self.get_logger().info("Shutting down.")
        rclpy.shutdown()

    def _transfer_file(self):
        # scp local -> remote_dir
        try:
            self.get_logger().info(f"Sending file to Control PC ({self.remote_ip})...")
            dst = f"{self.remote_user}@{self.remote_ip}:{self.remote_dir}"
            subprocess.run(["scp", self.save_path, dst], check=True)
            self.get_logger().info(f"SUCCESS: File transferred to B PC ({self.remote_dir})")
        except Exception as e:
            self.get_logger().error(f"FAILED: scp transfer error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VrDemoTxtRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
