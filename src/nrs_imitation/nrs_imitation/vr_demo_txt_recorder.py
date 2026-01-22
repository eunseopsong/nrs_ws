#!/usr/bin/env python3
# ============================================================
# vr_demo_txt_recorder.py
#
# Goals (this version):
# - Keep N raw (125 Hz 그대로, 행 개수 유지)
# - Pre-contact force gating: filtered fz로 contact 감지 전까지 fx,fy,fz=0
# - Force filtering pipeline 유지 (Whittaker + EMA + clamp + edge window ...)
# - Pose smoothing to reduce QP blow-up:
#     (1) Hampel outlier removal (per channel)
#     (2) Whittaker smoothing (2nd-derivative penalty, per channel)
#     (3) Auto-increase smoothing strength until max(v/a/w/alpha) under limits*safety
# - Print QP-proxy evaluation metrics + TOP violation indices (p2p vs vr 비교용)
# - Save txt and SCP to control PC
# ============================================================

import os
import threading
import numpy as np
import subprocess

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
# Robust outlier removal (Hampel filter)
# ---------------------------
def hampel_filter_1d(x: np.ndarray, win: int, n_sigmas: float = 3.0) -> np.ndarray:
    """
    Hampel filter: replace outliers with local median.
    - win: half-window size (radius). actual window size = 2*win+1
    """
    x = x.astype(np.float64, copy=False)
    n = x.shape[0]
    if n < 5 or win <= 0:
        return x.copy()

    win = int(win)
    out = x.copy()
    k = 1.4826  # scale factor for MAD
    for i in range(n):
        s = max(0, i - win)
        e = min(n, i + win + 1)
        w = x[s:e]
        med = np.median(w)
        mad = np.median(np.abs(w - med)) + 1e-12
        sigma = k * mad
        if np.abs(x[i] - med) > n_sigmas * sigma:
            out[i] = med
    return out


def hampel_pose(pose: np.ndarray, win: int, n_sigmas: float) -> np.ndarray:
    out = pose.copy().astype(np.float64)
    for j in range(out.shape[1]):
        out[:, j] = hampel_filter_1d(out[:, j], win=win, n_sigmas=n_sigmas)
    return out


# ---------------------------
# QP-proxy evaluation
# ---------------------------
def _grad1(y: np.ndarray, dt: float) -> np.ndarray:
    # numpy gradient gives same length, stable edges
    return np.gradient(y, dt, axis=0)


def qp_proxy_metrics(pose: np.ndarray, dt: float):
    """
    pose: (N,6) [x y z mm, wx wy wz rad]
    returns dict with v/a/w/alpha etc.
    """
    pos = pose[:, 0:3]
    ang = pose[:, 3:6]

    v = _grad1(pos, dt)
    a = _grad1(v, dt)
    w = _grad1(ang, dt)
    alpha = _grad1(w, dt)

    speed = np.linalg.norm(v, axis=1)
    acc = np.linalg.norm(a, axis=1)
    wmag = np.linalg.norm(w, axis=1)
    alphamag = np.linalg.norm(alpha, axis=1)

    # "reference jerk" (for comparing roughness)
    jerk = _grad1(a, dt)
    jpos = np.linalg.norm(jerk, axis=1)
    jalpha = _grad1(alpha, dt)
    jang = np.linalg.norm(jalpha, axis=1)

    def stats(x):
        return {
            "max": float(np.max(x)),
            "p95": float(np.percentile(x, 95.0)),
            "mean": float(np.mean(x)),
        }

    return {
        "N": int(pose.shape[0]),
        "dt": float(dt),
        "T": float((pose.shape[0]-1) * dt),
        "speed": speed,
        "acc": acc,
        "wmag": wmag,
        "alphamag": alphamag,
        "jpos": jpos,
        "jang": jang,
        "speed_stats": stats(speed),
        "acc_stats": stats(acc),
        "w_stats": stats(wmag),
        "alpha_stats": stats(alphamag),
        "jpos_max": float(np.max(jpos)),
        "jang_max": float(np.max(jang)),
    }


def violation_indices(x: np.ndarray, lim: float):
    return np.where(x > lim)[0]


def topk_indices(x: np.ndarray, k: int = 8):
    if x.size == 0:
        return np.array([], dtype=int)
    k = int(max(1, k))
    idx = np.argpartition(-x, min(k, x.size-1))[:k]
    idx = idx[np.argsort(-x[idx])]
    return idx


def format_eval_block(m, lims, safety, title: str):
    """
    lims: dict {pos_vmax,pos_amax,ang_vmax,ang_amax}
    """
    N = m["N"]
    dt = m["dt"]
    T = m["T"]

    pv = m["speed_stats"]
    pa = m["acc_stats"]
    av = m["w_stats"]
    aa = m["alpha_stats"]

    vpos_lim = lims["pos_vmax"] * safety
    apos_lim = lims["pos_amax"] * safety
    vang_lim = lims["ang_vmax"] * safety
    aang_lim = lims["ang_amax"] * safety

    vpos_vrate = 100.0 * float(np.mean(m["speed"] > vpos_lim))
    apos_vrate = 100.0 * float(np.mean(m["acc"] > apos_lim))
    vang_vrate = 100.0 * float(np.mean(m["wmag"] > vang_lim))
    aang_vrate = 100.0 * float(np.mean(m["alphamag"] > aang_lim))

    s = []
    s.append(f"[QP-EVAL] ===== {title} =====")
    s.append(f"  N={N}  dt={dt:.6f}s  T={T:.3f}s")
    s.append(f"  pos |v|: max={pv['max']:.3f} (lim {lims['pos_vmax']:.3f}, {pv['max']/max(lims['pos_vmax'],1e-9):.3f}x), p95={pv['p95']:.3f}, mean={pv['mean']:.3f}  [mm/s]")
    s.append(f"  pos |a|: max={pa['max']:.3f} (lim {lims['pos_amax']:.3f}, {pa['max']/max(lims['pos_amax'],1e-9):.3f}x), p95={pa['p95']:.3f}, mean={pa['mean']:.3f}  [mm/s^2]")
    s.append(f"  ang |w|: max={av['max']:.3f} (lim {lims['ang_vmax']:.3f}, {av['max']/max(lims['ang_vmax'],1e-9):.3f}x), p95={av['p95']:.3f}, mean={av['mean']:.3f}  [rad/s]")
    s.append(f"  ang |alpha|: max={aa['max']:.3f} (lim {lims['ang_amax']:.3f}, {aa['max']/max(lims['ang_amax'],1e-9):.3f}x), p95={aa['p95']:.3f}, mean={aa['mean']:.3f}  [rad/s^2]")
    s.append(f"  jerk(ref): pos max={m['jpos_max']:.3f} [mm/s^3], ang max={m['jang_max']:.3f} [rad/s^3]")
    s.append(f"  violation_rate(safety={safety:.3f}): vpos={vpos_vrate:.3f}%, apos={apos_vrate:.3f}%, vang={vang_vrate:.3f}%, aang={aang_vrate:.3f}%")
    return "\n".join(s)


def log_top_violations(node_logger, m, lims, safety, topk=6):
    dt = m["dt"]
    vpos_lim = lims["pos_vmax"] * safety
    apos_lim = lims["pos_amax"] * safety
    vang_lim = lims["ang_vmax"] * safety
    aang_lim = lims["ang_amax"] * safety

    def _log(name, arr, lim):
        bad = violation_indices(arr, lim)
        if bad.size == 0:
            node_logger.info(f"[QP-EVAL] {name}: no violations over lim*safety ({lim:.3f}).")
            return
        # show worst K among violating
        vals = arr[bad]
        order = np.argsort(-vals)
        pick = bad[order[:topk]]
        msg = f"[QP-EVAL] {name} top{min(topk,pick.size)} violating indices:\n"
        for idx in pick:
            msg += f"    idx={int(idx):6d}, t={idx*dt:8.3f}s, value={float(arr[idx]):.6f}\n"
        node_logger.info(msg.rstrip())

    _log("pos|v|", m["speed"], vpos_lim)
    _log("pos|a|", m["acc"], apos_lim)
    _log("ang|w|", m["wmag"], vang_lim)
    _log("ang|alpha|", m["alphamag"], aang_lim)


# ---------------------------
# Contact gating
# ---------------------------
def find_contact_index_fz(fz: np.ndarray, th_on: float, th_off: float, consecutive_on: int, consecutive_off: int) -> int:
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
# Pose smoothing (N 유지, max 기준으로 auto 강화)
# ---------------------------
def smooth_pose_auto(
    pose_raw: np.ndarray,
    dt: float,
    lims: dict,
    safety: float,
    cg_tol: float,
    cg_max_iter: int,
    # Hampel
    hampel_enable: bool,
    hampel_win: int,
    hampel_sigmas: float,
    # Whittaker auto
    wh_enable: bool,
    lam_pos_init: float,
    lam_ang_init: float,
    lam_growth: float,
    lam_pos_max: float,
    lam_ang_max: float,
    auto_iters: int,
    # optional EMA on pose
    pose_ema_enable: bool,
    pose_ema_alpha: float,
):
    """
    Returns pose_f, eval_before, eval_after, used_lams
    """
    pose0 = pose_raw.copy().astype(np.float64)

    eval0 = qp_proxy_metrics(pose0, dt)

    work = pose0.copy()
    if hampel_enable:
        work = hampel_pose(work, win=hampel_win, n_sigmas=hampel_sigmas)
        # endpoints preserve
        work[0, :] = pose0[0, :]
        work[-1, :] = pose0[-1, :]

    if not wh_enable:
        eval1 = qp_proxy_metrics(work, dt)
        return work, eval0, eval1, {"lam_pos": 0.0, "lam_ang": 0.0}

    lam_pos = float(lam_pos_init)
    lam_ang = float(lam_ang_init)
    lam_growth = float(max(1.1, lam_growth))

    best = work.copy()
    best_eval = qp_proxy_metrics(best, dt)
    best_score = max(
        best_eval["speed_stats"]["max"] / max(lims["pos_vmax"], 1e-9),
        best_eval["acc_stats"]["max"] / max(lims["pos_amax"], 1e-9),
        best_eval["w_stats"]["max"] / max(lims["ang_vmax"], 1e-9),
        best_eval["alpha_stats"]["max"] / max(lims["ang_amax"], 1e-9),
    )

    for _ in range(int(max(1, auto_iters))):
        cur = work.copy()

        # Whittaker per channel
        for j in range(3):  # xyz
            cur[:, j] = whittaker_smooth_1d(cur[:, j], lam_pos, cg_tol=cg_tol, cg_max_iter=cg_max_iter)
        for j in range(3, 6):  # angles
            cur[:, j] = whittaker_smooth_1d(cur[:, j], lam_ang, cg_tol=cg_tol, cg_max_iter=cg_max_iter)

        if pose_ema_enable:
            for j in range(6):
                cur[:, j] = ema_filtfilt_1d(cur[:, j], pose_ema_alpha)

        # endpoints exact preserve
        cur[0, :] = pose0[0, :]
        cur[-1, :] = pose0[-1, :]

        ev = qp_proxy_metrics(cur, dt)
        score = max(
            ev["speed_stats"]["max"] / max(lims["pos_vmax"], 1e-9),
            ev["acc_stats"]["max"] / max(lims["pos_amax"], 1e-9),
            ev["w_stats"]["max"] / max(lims["ang_vmax"], 1e-9),
            ev["alpha_stats"]["max"] / max(lims["ang_amax"], 1e-9),
        )

        if score < best_score:
            best_score = score
            best = cur
            best_eval = ev

        # stop if all max under lim*safety
        if (ev["speed_stats"]["max"] <= lims["pos_vmax"] * safety and
            ev["acc_stats"]["max"] <= lims["pos_amax"] * safety and
            ev["w_stats"]["max"] <= lims["ang_vmax"] * safety and
            ev["alpha_stats"]["max"] <= lims["ang_amax"] * safety):
            return cur, eval0, ev, {"lam_pos": lam_pos, "lam_ang": lam_ang}

        # otherwise increase lambda (but cap)
        lam_pos = min(lam_pos * lam_growth, lam_pos_max)
        lam_ang = min(lam_ang * lam_growth, lam_ang_max)

    # if not satisfied, return best (minimize worst ratio)
    return best, eval0, best_eval, {"lam_pos": lam_pos, "lam_ang": lam_ang}


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

        # ===== pose unwrap =====
        self.declare_parameter('omega_unwrap', True)

        # ===== force filtering (keep) =====
        self.declare_parameter('lam_force', 3000.0)
        self.declare_parameter('cg_tol', 1e-10)
        self.declare_parameter('cg_max_iter', 200)
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

        # ===== QP-proxy limits =====
        self.declare_parameter('qp_pos_vmax', 30.0)   # mm/s
        self.declare_parameter('qp_pos_amax', 120.0)  # mm/s^2
        self.declare_parameter('qp_ang_vmax', 0.6)    # rad/s
        self.declare_parameter('qp_ang_amax', 3.0)    # rad/s^2
        self.declare_parameter('qp_safety', 1.05)
        self.declare_parameter('qp_topk', 6)

        # ===== Pose smoothing (NEW) =====
        self.declare_parameter('pose_hampel_enable', True)
        self.declare_parameter('pose_hampel_win', 6)          # radius (2*win+1)
        self.declare_parameter('pose_hampel_sigmas', 3.0)

        self.declare_parameter('pose_whittaker_enable', True)
        self.declare_parameter('pose_lam_pos_init', 8000.0)   # 시작값 (필요시 조정)
        self.declare_parameter('pose_lam_ang_init', 200.0)
        self.declare_parameter('pose_lam_growth', 2.5)        # auto 강화 비율
        self.declare_parameter('pose_lam_pos_max', 5e7)
        self.declare_parameter('pose_lam_ang_max', 5e6)
        self.declare_parameter('pose_auto_iters', 6)

        self.declare_parameter('pose_ema_enable', False)      # 필요시 True
        self.declare_parameter('pose_ema_alpha', 0.20)

        # ===== plot optional =====
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

        self.qp_lims = {
            "pos_vmax": float(self.get_parameter('qp_pos_vmax').value),
            "pos_amax": float(self.get_parameter('qp_pos_amax').value),
            "ang_vmax": float(self.get_parameter('qp_ang_vmax').value),
            "ang_amax": float(self.get_parameter('qp_ang_amax').value),
        }
        self.qp_safety = float(self.get_parameter('qp_safety').value)
        self.qp_topk = int(self.get_parameter('qp_topk').value)

        self.pose_hampel_enable = bool(self.get_parameter('pose_hampel_enable').value)
        self.pose_hampel_win = int(self.get_parameter('pose_hampel_win').value)
        self.pose_hampel_sigmas = float(self.get_parameter('pose_hampel_sigmas').value)

        self.pose_wh_enable = bool(self.get_parameter('pose_whittaker_enable').value)
        self.pose_lam_pos_init = float(self.get_parameter('pose_lam_pos_init').value)
        self.pose_lam_ang_init = float(self.get_parameter('pose_lam_ang_init').value)
        self.pose_lam_growth = float(self.get_parameter('pose_lam_growth').value)
        self.pose_lam_pos_max = float(self.get_parameter('pose_lam_pos_max').value)
        self.pose_lam_ang_max = float(self.get_parameter('pose_lam_ang_max').value)
        self.pose_auto_iters = int(self.get_parameter('pose_auto_iters').value)

        self.pose_ema_enable = bool(self.get_parameter('pose_ema_enable').value)
        self.pose_ema_alpha = float(self.get_parameter('pose_ema_alpha').value)

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

        # subs
        self.create_subscription(Float64MultiArray, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(Wrench, self.ft_topic, self.ft_callback, 10)

        if self.record_hz <= 0.0:
            raise ValueError("record_hz must be > 0")
        self.timer = self.create_timer(1.0 / self.record_hz, self.main_loop)

        self.get_logger().info(f"Initialized. Local save path: {self.file_path}")
        self.get_logger().info(
            f"Pose: keep N raw. Hampel={self.pose_hampel_enable}(win={self.pose_hampel_win}, sig={self.pose_hampel_sigmas}) "
            f"+ WhittakerAuto={self.pose_wh_enable}(lam_pos_init={self.pose_lam_pos_init}, lam_ang_init={self.pose_lam_ang_init}, growth={self.pose_lam_growth}, iters={self.pose_auto_iters}) "
            f"+ PoseEMA={self.pose_ema_enable}(alpha={self.pose_ema_alpha})"
        )
        self.get_logger().info(
            f"Pre-contact force gating: {self.precontact_force_zero}  "
            f"(fz_on={self.contact_fz_on}, fz_off={self.contact_fz_off}, consec_on={self.contact_consec_on}, consec_off={self.contact_consec_off})"
        )
        self.get_logger().info(
            f"QP-proxy limits: pos_vmax={self.qp_lims['pos_vmax']} mm/s, pos_amax={self.qp_lims['pos_amax']} mm/s^2, "
            f"ang_vmax={self.qp_lims['ang_vmax']} rad/s, ang_amax={self.qp_lims['ang_amax']} rad/s^2, safety={self.qp_safety}"
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

        # episode start/end trigger (유지)
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

        # unwrap angles BEFORE any pose smoothing
        if self.omega_unwrap:
            for k in range(3, 6):
                pose_raw[:, k] = unwrap_angle_series(pose_raw[:, k], discont=np.pi)

        dt = 1.0 / float(self.playback_hz if self.playback_hz > 1e-9 else 125.0)

        # -------------------------
        # (A) Pose smoothing (N 유지)
        # -------------------------
        pose_f, ev_before, ev_after, used_lams = smooth_pose_auto(
            pose_raw,
            dt=dt,
            lims=self.qp_lims,
            safety=self.qp_safety,
            cg_tol=self.cg_tol,
            cg_max_iter=self.cg_max_iter,
            hampel_enable=self.pose_hampel_enable,
            hampel_win=self.pose_hampel_win,
            hampel_sigmas=self.pose_hampel_sigmas,
            wh_enable=self.pose_wh_enable,
            lam_pos_init=self.pose_lam_pos_init,
            lam_ang_init=self.pose_lam_ang_init,
            lam_growth=self.pose_lam_growth,
            lam_pos_max=self.pose_lam_pos_max,
            lam_ang_max=self.pose_lam_ang_max,
            auto_iters=self.pose_auto_iters,
            pose_ema_enable=self.pose_ema_enable,
            pose_ema_alpha=self.pose_ema_alpha,
        )

        self.get_logger().info(format_eval_block(ev_before, self.qp_lims, self.qp_safety, "BEFORE pose smoothing (RAW)"))
        log_top_violations(self.get_logger(), ev_before, self.qp_lims, self.qp_safety, topk=self.qp_topk)

        self.get_logger().info(format_eval_block(ev_after, self.qp_lims, self.qp_safety, "AFTER pose smoothing"))
        self.get_logger().info(f"[POSE-SMOOTH] used_lams={used_lams}")
        log_top_violations(self.get_logger(), ev_after, self.qp_lims, self.qp_safety, topk=self.qp_topk)

        # pose delta info
        pos_err = pose_f[:, 0:3] - pose_raw[:, 0:3]
        ang_err = pose_f[:, 3:6] - pose_raw[:, 3:6]
        self.get_logger().info(
            "[POSE-DELTA] pos: rms=%.3f mm, max=%.3f mm | ang: rms=%.6f rad, max=%.6f rad" % (
                float(np.sqrt(np.mean(np.sum(pos_err**2, axis=1)))),
                float(np.max(np.linalg.norm(pos_err, axis=1))),
                float(np.sqrt(np.mean(np.sum(ang_err**2, axis=1)))),
                float(np.max(np.linalg.norm(ang_err, axis=1))),
            )
        )

        # -------------------------
        # (B) Force filtering (keep pipeline, length unchanged)
        # -------------------------
        force_f = force_raw.copy()

        # Whittaker + EMA for force
        for j in range(3):
            force_f[:, j] = whittaker_smooth_1d(force_f[:, j], self.lam_force, cg_tol=self.cg_tol, cg_max_iter=self.cg_max_iter)
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

        # -------------------------
        # (C) Pre-contact force gating
        # -------------------------
        contact_idx = -1
        if self.precontact_force_zero:
            fz = force_f[:, 2]
            contact_idx = find_contact_index_fz(
                fz=fz,
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
                    f"[CONTACT] Detected at idx={contact_idx}/{force_f.shape[0]} (t={contact_idx*dt:.3f}s) -> "
                    f"Zeroing forces for [0:{contact_idx})"
                )
                force_f = zero_force_before_contact(force_f, contact_idx)

        # final (N unchanged)
        out = np.hstack([pose_f, force_f]).astype(np.float64)

        # save
        np.savetxt(self.file_path, out, fmt="%.10f")
        self.get_logger().info(
            f"Saved local file: {self.file_path}  (raw rows={raw.shape[0]} -> out rows={out.shape[0]})"
        )

        if self.plot_enable:
            self.save_plots(out, dt, contact_idx=contact_idx)

        # send to B PC
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

        plt.figure()
        plt.title("Pose XYZ")
        plt.plot(t, x, label="x")
        plt.plot(t, y, label="y")
        plt.plot(t, z, label="z")
        if contact_idx >= 0:
            plt.axvline(t[contact_idx], linestyle="--", label="contact")
        plt.xlabel("time [s]")
        plt.ylabel("pos [mm]")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_pose_xyz.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("Pose Omega (rad)")
        plt.plot(t, wx, label="wx")
        plt.plot(t, wy, label="wy")
        plt.plot(t, wz, label="wz")
        if contact_idx >= 0:
            plt.axvline(t[contact_idx], linestyle="--", label="contact")
        plt.xlabel("time [s]")
        plt.ylabel("rad")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{prefix}_pose_omega.png", dpi=150, bbox_inches="tight")
        plt.close()

        plt.figure()
        plt.title("Force (filtered + precontact zero)")
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
        plt.savefig(f"{prefix}_force.png", dpi=150, bbox_inches="tight")
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
