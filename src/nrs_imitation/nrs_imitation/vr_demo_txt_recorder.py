#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
vr_demo_txt_recorder.py  (QP-safe version)

핵심
- retime은 무조건 2배(k=2)만 수행
- "플롯용 미분"이 아니라, txt에 저장되는 pose 자체(Pr)가 v/a/j 제약을 만족하도록 만든다.
- retime 이후에 D3(3차 차분) jerk-penalty smoothing을 추가하고,
  QP-guard 루프에서 위반 시 lam을 자동 증가시켜 제약 만족을 유도한다.
- 플롯은 before/after 동시, AFTER(retimed) 시간축 기준 정렬 유지.
"""

import os
import time
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple, Dict

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ----------------------------
# Utility
# ----------------------------
def pctl(x: np.ndarray, q: float) -> float:
    if x.size == 0:
        return 0.0
    return float(np.percentile(x, q))


def norm_rows(x: np.ndarray) -> np.ndarray:
    return np.linalg.norm(x, axis=1)


def finite_diff_pad(y: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    단순 후진차분 (QP가 보는 것과 유사한 형태로 디버깅용)
    """
    y = y.astype(np.float64).reshape(-1)
    N = y.size
    v = np.full(N, np.nan, dtype=np.float64)
    a = np.full(N, np.nan, dtype=np.float64)
    j = np.full(N, np.nan, dtype=np.float64)
    if N >= 2:
        v[1:] = (y[1:] - y[:-1]) / dt
    if N >= 3:
        a[2:] = (v[2:] - v[1:-1]) / dt
    if N >= 4:
        j[3:] = (a[3:] - a[2:-1]) / dt
    return v, a, j


# ----------------------------
# Hampel filter (per-dim)
# ----------------------------
def hampel_1d(x: np.ndarray, win: int, n_sigmas: float) -> np.ndarray:
    if win <= 0:
        return x.copy()
    n = x.size
    y = x.copy()
    k = 1.4826  # MAD->std
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
# Whittaker smoother via CG (D2 penalty)
#   minimize ||y-z||^2 + lam*||D2 z||^2
# ----------------------------
def _apply_D2(x: np.ndarray) -> np.ndarray:
    return x[:-2] - 2.0 * x[1:-1] + x[2:]


def _apply_D2t(u: np.ndarray, n: int) -> np.ndarray:
    out = np.zeros(n, dtype=np.float64)
    out[:-2] += u
    out[1:-1] += -2.0 * u
    out[2:] += u
    return out


def whittaker_cg_1d(y: np.ndarray, lam: float, cg_iters: int = 200, tol: float = 1e-8) -> np.ndarray:
    n = y.size
    if n < 5 or lam <= 0.0:
        return y.copy()

    def A(x: np.ndarray) -> np.ndarray:
        d2 = _apply_D2(x)
        return x + lam * _apply_D2t(d2, n)

    x = y.copy()
    r = y - A(x)
    p = r.copy()
    rr = float(r @ r)
    if rr < tol:
        return x

    yy = float(y @ y) + 1e-12
    for _ in range(cg_iters):
        Ap = A(p)
        denom = float(p @ Ap) + 1e-12
        alpha = rr / denom
        x = x + alpha * p
        r = r - alpha * Ap
        rr_new = float(r @ r)
        if rr_new < (tol * tol) * yy:
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
# Jerk-penalty smoother via CG (D3 penalty)
#   minimize ||y-z||^2 + lam*||D3 z||^2
# D3 forward: x[i] - 3x[i+1] + 3x[i+2] - x[i+3]   (i=0..n-4)
# ----------------------------
def _apply_D3(x: np.ndarray) -> np.ndarray:
    return x[:-3] - 3.0 * x[1:-2] + 3.0 * x[2:-1] - x[3:]


def _apply_D3t(u: np.ndarray, n: int) -> np.ndarray:
    out = np.zeros(n, dtype=np.float64)
    # u length = n-3
    out[:-3] += u
    out[1:-2] += -3.0 * u
    out[2:-1] += 3.0 * u
    out[3:] += -1.0 * u
    return out


def whittaker_jerk_cg_1d(y: np.ndarray, lam: float, cg_iters: int = 200, tol: float = 1e-8) -> np.ndarray:
    n = y.size
    if n < 6 or lam <= 0.0:
        return y.copy()

    def A(x: np.ndarray) -> np.ndarray:
        d3 = _apply_D3(x)
        return x + lam * _apply_D3t(d3, n)

    x = y.copy()
    r = y - A(x)
    p = r.copy()
    rr = float(r @ r)
    if rr < tol:
        return x

    yy = float(y @ y) + 1e-12
    for _ in range(cg_iters):
        Ap = A(p)
        denom = float(p @ Ap) + 1e-12
        alpha = rr / denom
        x = x + alpha * p
        r = r - alpha * Ap
        rr_new = float(r @ r)
        if rr_new < (tol * tol) * yy:
            break
        beta = rr_new / (rr + 1e-12)
        p = r + beta * p
        rr = rr_new
    return x


def whittaker_jerk_cg_nd(Y: np.ndarray, lam: float, cg_iters: int = 200, tol: float = 1e-8) -> np.ndarray:
    Z = np.empty_like(Y)
    for d in range(Y.shape[1]):
        Z[:, d] = whittaker_jerk_cg_1d(Y[:, d], lam=lam, cg_iters=cg_iters, tol=tol)
    return Z


# ----------------------------
# QP-proxy evaluation
# ----------------------------
@dataclass
class Limits:
    pos_vmax: float   # mm/s
    pos_amax: float   # mm/s^2
    ang_vmax: float   # rad/s  (rotation-vector rate magnitude)
    ang_amax: float   # rad/s^2
    pos_jmax: float   # mm/s^3
    ang_jmax: float   # rad/s^3


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
    viol_v: float
    viol_a: float
    viol_w: float
    viol_alpha: float
    viol_jpos: float
    viol_jang: float


def eval_qp_proxy(pose6: np.ndarray, dt: float, lim: Limits, safety: float = 1.0) -> Tuple[EvalStats, Dict[str, np.ndarray]]:
    N = int(pose6.shape[0])
    T = dt * max(0, (N - 1))

    dp = pose6[1:, :3] - pose6[:-1, :3]
    dr = pose6[1:, 3:] - pose6[:-1, 3:]

    vpos = norm_rows(dp) / dt
    vang = norm_rows(dr) / dt

    v = (pose6[1:, :] - pose6[:-1, :]) / dt
    a = (v[1:, :] - v[:-1, :]) / dt
    apos = norm_rows(a[:, :3])
    aang = norm_rows(a[:, 3:])

    j = (a[1:, :] - a[:-1, :]) / dt
    jpos = norm_rows(j[:, :3])
    jang = norm_rows(j[:, 3:])

    vpos_lim = lim.pos_vmax * safety
    apos_lim = lim.pos_amax * safety
    vang_lim = lim.ang_vmax * safety
    aang_lim = lim.ang_amax * safety
    jpos_lim = lim.pos_jmax * safety
    jang_lim = lim.ang_jmax * safety

    st = EvalStats(
        N=N, dt=dt, T=T,
        vpos_max=float(vpos.max()) if vpos.size else 0.0,
        apos_max=float(apos.max()) if apos.size else 0.0,
        vang_max=float(vang.max()) if vang.size else 0.0,
        aang_max=float(aang.max()) if aang.size else 0.0,
        jpos_max=float(jpos.max()) if jpos.size else 0.0,
        jang_max=float(jang.max()) if jang.size else 0.0,
        vpos_p95=pctl(vpos, 95),
        apos_p95=pctl(apos, 95),
        vang_p95=pctl(vang, 95),
        aang_p95=pctl(aang, 95),
        jpos_p95=pctl(jpos, 95),
        jang_p95=pctl(jang, 95),
        viol_v=float(np.mean(vpos > vpos_lim)) if vpos.size else 0.0,
        viol_a=float(np.mean(apos > apos_lim)) if apos.size else 0.0,
        viol_w=float(np.mean(vang > vang_lim)) if vang.size else 0.0,
        viol_alpha=float(np.mean(aang > aang_lim)) if aang.size else 0.0,
        viol_jpos=float(np.mean(jpos > jpos_lim)) if jpos.size else 0.0,
        viol_jang=float(np.mean(jang > jang_lim)) if jang.size else 0.0,
    )

    debug = {
        "vpos": vpos, "vang": vang,
        "apos": apos, "aang": aang,
        "jpos": jpos, "jang": jang,
        "vpos_lim": vpos_lim, "vang_lim": vang_lim,
        "apos_lim": apos_lim, "aang_lim": aang_lim,
        "jpos_lim": jpos_lim, "jang_lim": jang_lim,
    }
    return st, debug


def _topk_idx(arr: np.ndarray, k: int = 6) -> np.ndarray:
    if arr.size == 0:
        return np.zeros((0,), dtype=np.int64)
    idx = np.argsort(-arr)[:k]
    return idx.astype(np.int64)


def log_topk_violations(logger, dbg: Dict[str, np.ndarray], dt: float, k: int = 6):
    # 각 배열은 길이가 서로 다름: vpos/vang: N-1, apos/aang: N-2, jpos/jang: N-3
    items = [
        ("vpos", "mm/s", dbg["vpos"], dbg["vpos_lim"]),
        ("vang", "rad/s", dbg["vang"], dbg["vang_lim"]),
        ("apos", "mm/s^2", dbg["apos"], dbg["apos_lim"]),
        ("aang", "rad/s^2", dbg["aang"], dbg["aang_lim"]),
        ("jpos", "mm/s^3", dbg["jpos"], dbg["jpos_lim"]),
        ("jang", "rad/s^3", dbg["jang"], dbg["jang_lim"]),
    ]
    for name, unit, arr, lim in items:
        if arr.size == 0:
            continue
        idx = _topk_idx(arr, k=k)
        # time mapping:
        # v: i corresponds between pose[i] and pose[i+1] -> use t=(i+0.5)dt
        # a: between v[i] and v[i+1] -> roughly t=(i+1)dt
        # j: -> roughly t=(i+1.5)dt
        if name in ("vpos", "vang"):
            t = (idx + 0.5) * dt
        elif name in ("apos", "aang"):
            t = (idx + 1.0) * dt
        else:
            t = (idx + 1.5) * dt

        over = arr[idx] - lim
        logger.info(f"[TOP-{k}] {name} (limit={lim:.3f} {unit})")
        for ii, tt, vv, oo in zip(idx.tolist(), t.tolist(), arr[idx].tolist(), over.tolist()):
            flag = "VIOL" if vv > lim else " ok "
            logger.info(f"   idx={ii:5d}  t={tt:7.3f}s  val={vv:10.3f} {unit}  (val-limit={oo:10.3f})  {flag}")


def print_eval(logger, title: str, st: EvalStats, lim: Limits, safety: float):
    logger.info(f"[QP-EVAL] ===== {title} =====")
    logger.info(
        f"\n  N={st.N}  dt={st.dt:.6f}s  T={st.T:.3f}s"
        f"\n  pos |v|: max={st.vpos_max:.3f} (lim {lim.pos_vmax*safety:.3f}), p95={st.vpos_p95:.3f}  [mm/s]"
        f"\n  pos |a|: max={st.apos_max:.3f} (lim {lim.pos_amax*safety:.3f}), p95={st.apos_p95:.3f}  [mm/s^2]"
        f"\n  rotvec |r_dot|: max={st.vang_max:.3f} (lim {lim.ang_vmax*safety:.3f}), p95={st.vang_p95:.3f}  [rad/s]"
        f"\n  rotvec |r_ddot|: max={st.aang_max:.3f} (lim {lim.ang_amax*safety:.3f}), p95={st.aang_p95:.3f}  [rad/s^2]"
        f"\n  jerk: pos max={st.jpos_max:.3f} (lim {lim.pos_jmax*safety:.3f}), p95={st.jpos_p95:.3f}  [mm/s^3]"
        f"\n        ang max={st.jang_max:.3f} (lim {lim.ang_jmax*safety:.3f}), p95={st.jang_p95:.3f}  [rad/s^3]"
        f"\n  violation_rate: vpos={100*st.viol_v:.3f}%, apos={100*st.viol_a:.3f}%, rdot={100*st.viol_w:.3f}%, rddot={100*st.viol_alpha:.3f}%, "
        f"jpos={100*st.viol_jpos:.3f}%, jang={100*st.viol_jang:.3f}%"
    )


def constraints_ok(st: EvalStats) -> bool:
    # "rate"는 비율이라서 0이 아니면 위반 존재
    return (st.viol_v == 0.0 and st.viol_a == 0.0 and st.viol_w == 0.0 and st.viol_alpha == 0.0 and
            st.viol_jpos == 0.0 and st.viol_jang == 0.0)


# ----------------------------
# Uniform upsample (time dilation)
# ----------------------------
def upsample_linear(X: np.ndarray, factor: int) -> np.ndarray:
    if factor <= 1:
        return X.copy()
    N, D = X.shape
    outN = (N - 1) * factor + 1
    out = np.empty((outN, D), dtype=np.float64)

    frac = (np.arange(factor, dtype=np.float64) / float(factor)).reshape(-1, 1)
    for i in range(N - 1):
        base = i * factor
        delta = (X[i + 1] - X[i]).reshape(1, -1)
        out[base:base + factor, :] = X[i].reshape(1, -1) + frac * delta
    out[-1, :] = X[-1, :]
    return out


# ----------------------------
# Contact detection
# ----------------------------
def detect_contact_idx(fz: np.ndarray, fz_on: float, consec_on: int) -> Optional[int]:
    cnt_on = 0
    for i in range(fz.size):
        if fz[i] >= fz_on:
            cnt_on += 1
            if cnt_on >= consec_on:
                return i - consec_on + 1
        else:
            cnt_on = 0
    return None


# ----------------------------
# Plot helpers (AFTER-time-aligned)
# ----------------------------
def _time_axes_time_aligned(dt: float, rawN: int, filN: int):
    if filN <= 0:
        t_after = np.zeros((0,), dtype=np.float64)
        T_after = 0.0
    elif filN == 1:
        t_after = np.array([0.0], dtype=np.float64)
        T_after = 0.0
    else:
        t_after = np.arange(filN, dtype=np.float64) * dt
        T_after = float(t_after[-1])

    if rawN <= 0:
        t_before = np.zeros((0,), dtype=np.float64)
    elif rawN == 1:
        t_before = np.array([0.0], dtype=np.float64)
    else:
        t_before = np.linspace(0.0, T_after, rawN, dtype=np.float64)

    return t_before, t_after


def plot_before_after(ax, t0, y0, t1, y1, title, ylabel):
    ax.plot(t0, y0, label="before")
    ax.plot(t1, y1, label="after")
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True)


def save_plot_1_lin_kinematics(viz_dir: str, dt: float, rawP: np.ndarray, filtP: np.ndarray):
    rawN = rawP.shape[0]
    filN = filtP.shape[0]
    t_raw, t_fil = _time_axes_time_aligned(dt, rawN, filN)

    fig = plt.figure(figsize=(16, 12))
    names = ["x", "y", "z"]

    for c in range(3):
        y_raw = rawP[:, c]
        y_fil = filtP[:, c]
        v_raw, a_raw, j_raw = finite_diff_pad(y_raw, dt)
        v_fil, a_fil, j_fil = finite_diff_pad(y_fil, dt)

        ax = plt.subplot(4, 3, 1 + c)
        plot_before_after(ax, t_raw, y_raw, t_fil, y_fil, f"{names[c]}", "mm")
        if c == 0:
            ax.legend()

        ax = plt.subplot(4, 3, 4 + c)
        plot_before_after(ax, t_raw, v_raw, t_fil, v_fil, f"v{names[c]}", "mm/s")

        ax = plt.subplot(4, 3, 7 + c)
        plot_before_after(ax, t_raw, a_raw, t_fil, a_fil, f"a{names[c]}", "mm/s^2")

        ax = plt.subplot(4, 3, 10 + c)
        plot_before_after(ax, t_raw, j_raw, t_fil, j_fil, f"j{names[c]}", "mm/s^3")
        ax.set_xlabel("time [s]")

    fig.suptitle("Linear kinematics (TRUE finite-diff): pos/vel/acc/jerk (before vs after, AFTER-time-aligned)", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    outpath = os.path.join(viz_dir, "plot_1_lin_kinematics_true.png")
    plt.savefig(outpath, dpi=200)
    plt.close(fig)


def save_plot_2_rotvec_kinematics(viz_dir: str, dt: float, rawP: np.ndarray, filtP: np.ndarray):
    rawN = rawP.shape[0]
    filN = filtP.shape[0]
    t_raw, t_fil = _time_axes_time_aligned(dt, rawN, filN)

    fig = plt.figure(figsize=(16, 12))
    names = ["rx", "ry", "rz"]

    for c in range(3):
        y_raw = rawP[:, 3 + c]
        y_fil = filtP[:, 3 + c]
        v_raw, a_raw, j_raw = finite_diff_pad(y_raw, dt)
        v_fil, a_fil, j_fil = finite_diff_pad(y_fil, dt)

        ax = plt.subplot(4, 3, 1 + c)
        plot_before_after(ax, t_raw, y_raw, t_fil, y_fil, f"{names[c]}", "rad")
        if c == 0:
            ax.legend()

        ax = plt.subplot(4, 3, 4 + c)
        plot_before_after(ax, t_raw, v_raw, t_fil, v_fil, f"{names[c]}_rate", "rad/s")

        ax = plt.subplot(4, 3, 7 + c)
        plot_before_after(ax, t_raw, a_raw, t_fil, a_fil, f"{names[c]}_acc", "rad/s^2")

        ax = plt.subplot(4, 3, 10 + c)
        plot_before_after(ax, t_raw, j_raw, t_fil, j_fil, f"{names[c]}_jerk", "rad/s^3")
        ax.set_xlabel("time [s]")

    fig.suptitle("Rotvec kinematics (TRUE finite-diff): r/rate/acc/jerk (before vs after, AFTER-time-aligned)", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    outpath = os.path.join(viz_dir, "plot_2_rotvec_kinematics_true.png")
    plt.savefig(outpath, dpi=200)
    plt.close(fig)


def save_plot_3_forces(viz_dir: str, dt: float, rawF: np.ndarray, filtF: np.ndarray):
    rawN = rawF.shape[0]
    filN = filtF.shape[0]
    t_raw, t_fil = _time_axes_time_aligned(dt, rawN, filN)

    fig = plt.figure(figsize=(16, 4))
    names = ["fx", "fy", "fz"]
    for c in range(3):
        ax = plt.subplot(1, 3, 1 + c)
        plot_before_after(ax, t_raw, rawF[:, c], t_fil, filtF[:, c], f"{names[c]}", "N")
        ax.set_xlabel("time [s]")
        if c == 0:
            ax.legend()

    fig.suptitle("Forces: fx/fy/fz (before vs after, AFTER-time-aligned)", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.90])
    outpath = os.path.join(viz_dir, "plot_3_forces.png")
    plt.savefig(outpath, dpi=200)
    plt.close(fig)


# ----------------------------
# Main Node
# ----------------------------
class VrDemoTxtRecorder(Node):
    def __init__(self):
        super().__init__("vr_demo_txt_recorder")

        # topics
        self.declare_parameter("pose_topic", "/calibrated_pose")
        self.declare_parameter("force_topic", "/ftsensor/measured_Cvalue")

        # timing
        self.declare_parameter("record_hz", 125.0)
        self.declare_parameter("require_fresh_sec", 0.2)

        # episode rule
        self.declare_parameter("start_abs_fx", 10.0)
        self.declare_parameter("stop_abs_fy", 10.0)

        # save path
        self.declare_parameter(
            "save_path",
            "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/cmd_continue9D.txt"
        )

        # viz
        self.declare_parameter("viz_root", "/home/eunseop/nrs_ws/src/nrs_imitation/log")

        # SCP transfer
        self.declare_parameter("transfer_enable", True)
        self.declare_parameter("remote_user", "nrs_forcecon")
        self.declare_parameter("remote_ip", "192.168.0.151")
        self.declare_parameter("remote_dir", "/home/nrs_forcecon/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/")

        # force shaping
        self.declare_parameter("zero_xy_forces", True)
        self.declare_parameter("force_clamp_abs", 200.0)
        self.declare_parameter("force_ema_alpha", 0.2)
        self.declare_parameter("edge_force_zero_sec", 0.5)
        self.declare_parameter("edge_force_fade_sec", 0.3)

        # pre-contact gating
        self.declare_parameter("precontact_gating", True)
        self.declare_parameter("fz_on", 5.0)
        self.declare_parameter("consec_on", 10)

        # pose smoothing (pre)
        self.declare_parameter("hampel_enable", True)
        self.declare_parameter("hampel_win", 16)
        self.declare_parameter("hampel_sig", 2.0)

        # D2 (acc-penalty) smoothing (pre)
        self.declare_parameter("lam_pos_d2", 250000.0)
        self.declare_parameter("lam_ang_d2", 6000.0)
        self.declare_parameter("pose_ema_enable", True)
        self.declare_parameter("pose_ema_alpha", 0.10)

        # retime fixed x2
        self.retime_enable = True
        self.retime_k = 2

        # post-retime jerk penalty (D3)
        self.declare_parameter("post_enable", True)
        self.declare_parameter("lam_pos_d3", 2.0e7)
        self.declare_parameter("lam_ang_d3", 6.0e5)

        # QP-guard loop
        self.declare_parameter("qp_guard_enable", True)
        self.declare_parameter("qp_guard_safety", 0.75)  # 컨트롤러가 더 빡빡할 가능성 -> 보수적으로
        self.declare_parameter("qp_guard_max_iter", 8)
        self.declare_parameter("qp_guard_growth", 2.2)
        self.declare_parameter("max_dev_pos_mm", 8.0)
        self.declare_parameter("max_dev_ang_rad", 0.06)

        # CG
        self.declare_parameter("cg_iters", 400)
        self.declare_parameter("cg_tol", 1e-8)

        # QP-proxy limits (네 컨트롤러에 맞게 더 줄여도 됨)
        self.declare_parameter("pos_vmax", 30.0)
        self.declare_parameter("pos_amax", 120.0)
        self.declare_parameter("ang_vmax", 0.6)
        self.declare_parameter("ang_amax", 3.0)
        self.declare_parameter("pos_jmax", 5000.0)
        self.declare_parameter("ang_jmax", 80.0)

        # load params
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)

        self.record_hz = float(self.get_parameter("record_hz").value)
        self.dt = 1.0 / max(1e-9, self.record_hz)
        self.require_fresh_sec = float(self.get_parameter("require_fresh_sec").value)

        self.save_path = str(self.get_parameter("save_path").value)
        self.viz_root = str(self.get_parameter("viz_root").value)

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
        self.consec_on = int(self.get_parameter("consec_on").value)

        self.hampel_enable = bool(self.get_parameter("hampel_enable").value)
        self.hampel_win = int(self.get_parameter("hampel_win").value)
        self.hampel_sig = float(self.get_parameter("hampel_sig").value)

        self.lam_pos_d2 = float(self.get_parameter("lam_pos_d2").value)
        self.lam_ang_d2 = float(self.get_parameter("lam_ang_d2").value)

        self.pose_ema_enable = bool(self.get_parameter("pose_ema_enable").value)
        self.pose_ema_alpha = float(self.get_parameter("pose_ema_alpha").value)

        self.post_enable = bool(self.get_parameter("post_enable").value)
        self.lam_pos_d3 = float(self.get_parameter("lam_pos_d3").value)
        self.lam_ang_d3 = float(self.get_parameter("lam_ang_d3").value)

        self.qp_guard_enable = bool(self.get_parameter("qp_guard_enable").value)
        self.qp_guard_safety = float(self.get_parameter("qp_guard_safety").value)
        self.qp_guard_max_iter = int(self.get_parameter("qp_guard_max_iter").value)
        self.qp_guard_growth = float(self.get_parameter("qp_guard_growth").value)
        self.max_dev_pos_mm = float(self.get_parameter("max_dev_pos_mm").value)
        self.max_dev_ang_rad = float(self.get_parameter("max_dev_ang_rad").value)

        self.cg_iters = int(self.get_parameter("cg_iters").value)
        self.cg_tol = float(self.get_parameter("cg_tol").value)

        self.lim = Limits(
            pos_vmax=float(self.get_parameter("pos_vmax").value),
            pos_amax=float(self.get_parameter("pos_amax").value),
            ang_vmax=float(self.get_parameter("ang_vmax").value),
            ang_amax=float(self.get_parameter("ang_amax").value),
            pos_jmax=float(self.get_parameter("pos_jmax").value),
            ang_jmax=float(self.get_parameter("ang_jmax").value),
        )

        # latest samples
        self.latest_pose6_mm_rad: Optional[np.ndarray] = None
        self.latest_force3_N: Optional[np.ndarray] = None
        self.latest_pose_t: float = 0.0
        self.latest_force_t: float = 0.0

        # buffers
        self.episode_active = False
        self.finishing_ = False
        self.buf_pose = []
        self.buf_force = []

        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self.cb_pose, 50)
        self.sub_force = self.create_subscription(Wrench, self.force_topic, self.cb_force, 10)
        self.timer = self.create_timer(self.dt, self.cb_timer)

        self.get_logger().info(f"[RETIME] fixed x2 enabled. dt={self.dt:.6f}s, save={self.save_path}")

    def cb_pose(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        x, y, z, rx, ry, rz = msg.data[:6]
        self.latest_pose6_mm_rad = np.array([1000.0*x, 1000.0*y, 1000.0*z, rx, ry, rz], dtype=np.float64)
        self.latest_pose_t = time.time()

    def cb_force(self, msg: Wrench):
        fx = float(msg.force.x)
        fy = float(msg.force.y)
        fz = float(msg.force.z)
        self.latest_force3_N = np.array([fx, fy, fz], dtype=np.float64)
        self.latest_force_t = time.time()

        if self.finishing_:
            return

        if (not self.episode_active) and (abs(fx) >= self.start_abs_fx):
            self.episode_active = True
            self.buf_pose.clear()
            self.buf_force.clear()
            self.get_logger().info("=== EPISODE STARTED (|fx| >= start_abs_fx) ===")
            return

        if self.episode_active and (abs(fy) >= self.stop_abs_fy):
            self.get_logger().info("=== EPISODE ENDED (|fy| >= stop_abs_fy) ===")
            self.finish_episode()
            return

    def cb_timer(self):
        if (not self.episode_active) or self.finishing_:
            return
        now = time.time()
        if self.latest_pose6_mm_rad is None or (now - self.latest_pose_t) > self.require_fresh_sec:
            return
        if self.latest_force3_N is None or (now - self.latest_force_t) > self.require_fresh_sec:
            return
        self.buf_pose.append(self.latest_pose6_mm_rad.copy())
        self.buf_force.append(self.latest_force3_N.copy())

    # ---------- force processing ----------
    def _force_process(self, F: np.ndarray) -> np.ndarray:
        Fp = np.clip(F.copy(), -self.force_clamp_abs, self.force_clamp_abs)
        if self.zero_xy_forces:
            Fp[:, 0] = 0.0
            Fp[:, 1] = 0.0
        if 0.0 < self.force_ema_alpha < 1.0:
            # EMA
            for i in range(1, Fp.shape[0]):
                Fp[i] = self.force_ema_alpha * Fp[i] + (1.0 - self.force_ema_alpha) * Fp[i - 1]
        return Fp

    def _apply_edge_force_window(self, F: np.ndarray) -> np.ndarray:
        out = F.copy()
        n = out.shape[0]
        zN = int(round(self.edge_force_zero_sec * self.record_hz))
        fN = int(round(self.edge_force_fade_sec * self.record_hz))
        zN = max(0, min(n, zN))
        fN = max(0, min(n, fN))

        if zN > 0:
            out[:zN, :] = 0.0
            out[n - zN:, :] = 0.0
        if fN > 0 and (zN + fN) < n:
            w = np.linspace(0.0, 1.0, fN, dtype=np.float64).reshape(-1, 1)
            out[zN:zN + fN, :] *= w
        if fN > 0 and (n - zN - fN) > 0:
            w = np.linspace(1.0, 0.0, fN, dtype=np.float64).reshape(-1, 1)
            out[n - zN - fN:n - zN, :] *= w
        return out

    # ---------- pose smoothing ----------
    def _pose_pre_smooth(self, P: np.ndarray) -> np.ndarray:
        P0 = P.copy()
        if self.hampel_enable:
            P0 = hampel_nd(P0, win=self.hampel_win, n_sigmas=self.hampel_sig)

        # D2 smoothing
        P1 = P0.copy()
        P1[:, :3] = whittaker_cg_nd(P1[:, :3], lam=self.lam_pos_d2, cg_iters=self.cg_iters, tol=self.cg_tol)
        P1[:, 3:] = whittaker_cg_nd(P1[:, 3:], lam=self.lam_ang_d2, cg_iters=self.cg_iters, tol=self.cg_tol)

        if self.pose_ema_enable:
            P1 = ema_nd(P1, alpha=self.pose_ema_alpha)
        return P1

    def _retime_x2(self, P: np.ndarray, F: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # fixed x2
        Pr = upsample_linear(P, 2)
        Fr = upsample_linear(F, 2)
        return Pr, Fr

    def _pose_post_smooth_d3(self, P: np.ndarray, lam_pos_d3: float, lam_ang_d3: float) -> np.ndarray:
        if not self.post_enable:
            return P
        P2 = P.copy()
        P2[:, :3] = whittaker_jerk_cg_nd(P2[:, :3], lam=lam_pos_d3, cg_iters=self.cg_iters, tol=self.cg_tol)
        P2[:, 3:] = whittaker_jerk_cg_nd(P2[:, 3:], lam=lam_ang_d3, cg_iters=self.cg_iters, tol=self.cg_tol)
        return P2

    def _qp_guard(self, Pref: np.ndarray) -> np.ndarray:
        """
        Pref (retimed + pre-smoothed) -> (post D3) -> eval -> violation 있으면 lam을 키워 반복
        """
        if not self.qp_guard_enable:
            return self._pose_post_smooth_d3(Pref, self.lam_pos_d3, self.lam_ang_d3)

        lam_p = self.lam_pos_d3
        lam_a = self.lam_ang_d3
        best = None
        best_score = 1e18

        for it in range(max(1, self.qp_guard_max_iter)):
            Pk = self._pose_post_smooth_d3(Pref, lam_p, lam_a)

            # deviation guard (너무 원본에서 멀어지면 중단)
            dpos = norm_rows(Pk[:, :3] - Pref[:, :3])
            dang = norm_rows(Pk[:, 3:] - Pref[:, 3:])
            if float(dpos.max()) > self.max_dev_pos_mm or float(dang.max()) > self.max_dev_ang_rad:
                self.get_logger().warn(
                    f"[QP-GUARD] stop by deviation: max_dpos={float(dpos.max()):.3f}mm (allow {self.max_dev_pos_mm}), "
                    f"max_dang={float(dang.max()):.4f}rad (allow {self.max_dev_ang_rad})"
                )
                break

            st, dbg = eval_qp_proxy(Pk, self.dt, self.lim, safety=self.qp_guard_safety)
            print_eval(self.get_logger(), f"QP-GUARD iter={it} (lam_p={lam_p:.3e}, lam_a={lam_a:.3e}, safety={self.qp_guard_safety})", st, self.lim, self.qp_guard_safety)

            # score: jerk 위주로
            score = max(
                st.jpos_p95 / (self.lim.pos_jmax * self.qp_guard_safety + 1e-9),
                st.jang_p95 / (self.lim.ang_jmax * self.qp_guard_safety + 1e-9),
                st.apos_p95 / (self.lim.pos_amax * self.qp_guard_safety + 1e-9),
                st.aang_p95 / (self.lim.ang_amax * self.qp_guard_safety + 1e-9),
            )
            if score < best_score:
                best_score = score
                best = Pk

            if constraints_ok(st):
                self.get_logger().info("[QP-GUARD] constraints satisfied.")
                return Pk

            # 어디서 터지는지 top-k 출력
            log_topk_violations(self.get_logger(), dbg, dt=self.dt, k=6)

            # grow lambda (jerk 더 강하게)
            lam_p *= self.qp_guard_growth
            lam_a *= self.qp_guard_growth

        self.get_logger().warn("[QP-GUARD] could not fully satisfy constraints within limits. Returning best smoothed.")
        return best if best is not None else self._pose_post_smooth_d3(Pref, self.lam_pos_d3, self.lam_ang_d3)

    # ---------- viz ----------
    def _make_viz_dir(self) -> str:
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        out_dir = os.path.join(self.viz_root, ts)
        os.makedirs(out_dir, exist_ok=True)
        return out_dir

    def _save_viz(self, out_dir: str, rawP: np.ndarray, rawF: np.ndarray, filtP: np.ndarray, filtF: np.ndarray):
        try:
            save_plot_1_lin_kinematics(out_dir, self.dt, rawP, filtP)
            save_plot_2_rotvec_kinematics(out_dir, self.dt, rawP, filtP)
            save_plot_3_forces(out_dir, self.dt, rawF, filtF)
            self.get_logger().info(f"[VIZ] Saved plots to: {out_dir}")
        except Exception as e:
            self.get_logger().error(f"[VIZ] Failed to save plots: {e}")

    # ---------- episode finalize ----------
    def finish_episode(self):
        if self.finishing_:
            return
        self.finishing_ = True
        self.episode_active = False

        if len(self.buf_pose) < 10:
            self.get_logger().warn("Episode too short. Discarding.")
            rclpy.shutdown()
            return

        rawP = np.asarray(self.buf_pose, dtype=np.float64)
        rawF = np.asarray(self.buf_force, dtype=np.float64)

        st0, dbg0 = eval_qp_proxy(rawP, self.dt, self.lim, safety=1.0)
        print_eval(self.get_logger(), "RAW (before any smoothing)", st0, self.lim, 1.0)
        log_topk_violations(self.get_logger(), dbg0, dt=self.dt, k=4)

        # forces
        Fp = self._force_process(rawF)

        # pre smooth
        Ps = self._pose_pre_smooth(rawP)
        st1, dbg1 = eval_qp_proxy(Ps, self.dt, self.lim, safety=1.0)
        print_eval(self.get_logger(), "AFTER pre-smooth (D2)", st1, self.lim, 1.0)
        log_topk_violations(self.get_logger(), dbg1, dt=self.dt, k=4)

        # retime x2
        Pr, Fr = self._retime_x2(Ps, Fp)
        self.get_logger().info(f"[RETIME] x2 applied: rows {Ps.shape[0]} -> {Pr.shape[0]}")

        # post jerk smoothing with QP guard
        Pr2 = self._qp_guard(Pr)

        st2, dbg2 = eval_qp_proxy(Pr2, self.dt, self.lim, safety=self.qp_guard_safety)
        print_eval(self.get_logger(), "FINAL pose (retime x2 + D3 jerk smoothing)", st2, self.lim, self.qp_guard_safety)
        log_topk_violations(self.get_logger(), dbg2, dt=self.dt, k=6)

        # contact gating (forces only)
        if self.precontact_gating:
            cidx = detect_contact_idx(Fr[:, 2], self.fz_on, self.consec_on)
            if cidx is not None and cidx > 0:
                self.get_logger().info(f"[CONTACT] at idx={cidx} (t={cidx*self.dt:.3f}s) -> zero forces before contact")
                Fr[:cidx, :] = 0.0

        # edge force window
        Fr = self._apply_edge_force_window(Fr)

        # save txt (pose+force)
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        out9 = np.hstack([Pr2, Fr])
        with open(self.save_path, "w") as f:
            for row in out9:
                f.write("\t".join([f"{v:.6f}" for v in row.tolist()]) + "\n")
        self.get_logger().info(f"Saved: {self.save_path}  (rows={out9.shape[0]})")

        # viz
        viz_dir = self._make_viz_dir()
        self._save_viz(viz_dir, rawP, rawF, Pr2, Fr)

        # transfer
        if self.transfer_enable:
            self._transfer_file()

        self.get_logger().info("Shutting down (end condition met).")
        rclpy.shutdown()

    def _transfer_file(self):
        try:
            self.get_logger().info(f"Sending file to Control PC ({self.remote_ip})...")
            dst = f"{self.remote_user}@{self.remote_ip}:{self.remote_dir}"
            subprocess.run(["scp", self.save_path, dst], check=True)
            self.get_logger().info(f"SUCCESS: transferred to {self.remote_dir}")
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
