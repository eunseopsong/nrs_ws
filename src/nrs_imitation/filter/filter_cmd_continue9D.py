#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
filter_cmd_continue9D.py

[요구사항 반영]
- input: cmd_continue9D.txt (raw trajectory)
- 실행하면:
  (1) raw를 non_filtered_cmd_continue9D.txt 로 백업
  (2) VrDemoTxtRecorder.finish_episode()의 후처리 파이프라인을 그대로 적용
  (3) 결과를 cmd_continue9D.txt 에 덮어쓰기 저장
  (4) viz_YYYYMMDD_HHMMSS 폴더 생성
  (5) 9개 채널 각각에 대해 before/after를 한 figure(단일 subplot)로 저장 (png)
      x, y, z, wx, wy, wz, fx, fy, fz 각각 1장씩
  (+) summary.png: 9개 subplot 한 장(편의용)

Input TXT format per row (9 floats):
  x y z [mm], wx wy wz [rad], fx fy fz [N]

Output TXT format per row (9 floats):
  same as above (tab-separated, %.6f)
"""

import argparse
import math
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple, Dict

import numpy as np

# headless 환경에서도 png 저장 가능하게
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


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
    return x[:-2] - 2.0 * x[1:-1] + x[2:]


def _apply_D2t(u: np.ndarray, n: int) -> np.ndarray:
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


def print_eval(title: str, st: EvalStats, lim: Limits, safety: float):
    print(f"[QP-EVAL] ===== {title} =====")
    print(
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
# Contact detection (identical)
# ----------------------------
def detect_contact_idx(fz: np.ndarray, fz_on: float, fz_off: float, consec_on: int, consec_off: int) -> Optional[int]:
    """
    Returns first index where contact is considered ON.
    (Recorder 코드와 동일하게 fz_on + consec_on만 실제로 사용)
    """
    on = False
    cnt_on = 0
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
            break

    return first_on_idx


# ----------------------------
# Force window (identical)
# ----------------------------
def apply_edge_force_window(F: np.ndarray, hz: float, edge_force_zero_sec: float, edge_force_fade_sec: float) -> np.ndarray:
    out = F.copy()
    n = out.shape[0]
    zN = int(round(edge_force_zero_sec * hz))
    fN = int(round(edge_force_fade_sec * hz))
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


# ----------------------------
# Pose smoothing (identical)
# ----------------------------
def pose_smooth(
    P: np.ndarray,
    dt: float,
    lim: Limits,
    safety: float,
    hampel_enable: bool,
    hampel_win: int,
    hampel_sig: float,
    whittaker_auto: bool,
    lam_pos_init: float,
    lam_ang_init: float,
    lam_growth: float,
    lam_iters: int,
    cg_iters: int,
    cg_tol: float,
    pose_ema_enable: bool,
    pose_ema_alpha: float
) -> Tuple[np.ndarray, Dict[str, float]]:
    P0 = P.copy()

    if hampel_enable:
        P0 = hampel_nd(P0, win=hampel_win, n_sigmas=hampel_sig)

    if not whittaker_auto:
        Pp = P0.copy()
        Pp[:, :3] = whittaker_cg_nd(Pp[:, :3], lam=lam_pos_init, cg_iters=cg_iters, tol=cg_tol)
        Pp[:, 3:] = whittaker_cg_nd(Pp[:, 3:], lam=lam_ang_init, cg_iters=cg_iters, tol=cg_tol)
        if pose_ema_enable:
            Pp = ema_nd(Pp, alpha=pose_ema_alpha)
        info = {"lam_pos": lam_pos_init, "lam_ang": lam_ang_init}
        return Pp, info

    lam_pos = lam_pos_init
    lam_ang = lam_ang_init

    best = None
    best_score = 1e18
    best_info = {"lam_pos": lam_pos, "lam_ang": lam_ang}

    max_pos_delta_allow = 5.0      # mm
    max_ang_delta_allow = 0.03     # rad

    for _ in range(max(1, lam_iters)):
        Pp = P0.copy()
        Pp[:, :3] = whittaker_cg_nd(Pp[:, :3], lam=lam_pos, cg_iters=cg_iters, tol=cg_tol)
        Pp[:, 3:] = whittaker_cg_nd(Pp[:, 3:], lam=lam_ang, cg_iters=cg_iters, tol=cg_tol)

        if pose_ema_enable:
            Pp = ema_nd(Pp, alpha=pose_ema_alpha)

        dpos = norm_rows(Pp[:, :3] - P[:, :3])
        dang = norm_rows(Pp[:, 3:] - P[:, 3:])
        if float(dpos.max()) > max_pos_delta_allow or float(dang.max()) > max_ang_delta_allow:
            break

        st, _ = eval_qp_proxy(Pp, dt, lim, safety=safety)
        score = max(st.apos_p95 / (lim.pos_amax + 1e-9),
                    st.aang_p95 / (lim.ang_amax + 1e-9),
                    st.jpos_p95 / (lim.pos_jmax + 1e-9),
                    st.jang_p95 / (lim.ang_jmax + 1e-9)) \
                + 0.05 * (float(dpos.mean()) / 1.0)

        if score < best_score:
            best_score = score
            best = Pp
            best_info = {"lam_pos": lam_pos, "lam_ang": lam_ang}

        lam_pos *= lam_growth
        lam_ang *= lam_growth

    if best is None:
        best = P0
    return best, best_info


# ----------------------------
# Force process (identical)
# ----------------------------
def force_process(F: np.ndarray, zero_xy_forces: bool, force_clamp_abs: float, force_ema_alpha: float) -> np.ndarray:
    Fp = F.copy()
    Fp = np.clip(Fp, -force_clamp_abs, force_clamp_abs)
    if zero_xy_forces:
        Fp[:, 0] = 0.0
        Fp[:, 1] = 0.0
    if 0.0 < force_ema_alpha < 1.0:
        Fp = ema_nd(Fp, alpha=force_ema_alpha)
    return Fp


# ----------------------------
# Retime (identical)
# ----------------------------
def retime_uniform(
    P: np.ndarray, F: np.ndarray,
    dt: float, lim: Limits, safety: float,
    retime_enable: bool, retime_use_jerk: bool,
    retime_max_k: int, retime_passes: int
) -> Tuple[np.ndarray, np.ndarray, int]:
    if not retime_enable:
        return P, F, 1

    Pcur = P.copy()
    Fcur = F.copy()
    k_total = 1

    for _ in range(max(1, retime_passes)):
        st, _ = eval_qp_proxy(Pcur, dt, lim, safety=safety)

        rv = max(
            st.vpos_max / (lim.pos_vmax * safety + 1e-9),
            st.vang_max / (lim.ang_vmax * safety + 1e-9),
        )
        ra = max(
            math.sqrt(st.apos_max / (lim.pos_amax * safety + 1e-9)),
            math.sqrt(st.aang_max / (lim.ang_amax * safety + 1e-9)),
        )

        rj = 1.0
        if retime_use_jerk:
            rj = max(
                (st.jpos_max / (lim.pos_jmax * safety + 1e-9)) ** (1.0 / 3.0),
                (st.jang_max / (lim.ang_jmax * safety + 1e-9)) ** (1.0 / 3.0),
            )

        r_need = max(1.0, rv, ra, rj)
        k_need = int(math.ceil(r_need))

        remaining = max(1, retime_max_k // max(1, k_total))
        k_need = min(k_need, remaining)

        if k_need <= 1:
            break

        Pcur = upsample_linear(Pcur, k_need)
        Fcur = upsample_linear(Fcur, k_need)
        k_total *= k_need

    return Pcur, Fcur, k_total


# ----------------------------
# TXT I/O
# ----------------------------
def read_txt9(path: str) -> np.ndarray:
    rows = []
    with open(path, "r") as f:
        for ln, line in enumerate(f, 1):
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.replace(",", " ").split()
            if len(parts) < 9:
                raise RuntimeError(f"Line {ln}: need >=9 floats, got={len(parts)} -> '{s}'")
            rows.append(list(map(float, parts[:9])))
    data = np.asarray(rows, dtype=np.float64)
    if data.ndim != 2 or data.shape[1] != 9 or data.shape[0] == 0:
        raise RuntimeError(f"Invalid data shape: {data.shape}")
    return data


def write_txt9_tab6(path: str, out: np.ndarray):
    with open(path, "w") as f:
        for row in out:
            f.write("\t".join([f"{v:.6f}" for v in row.tolist()]) + "\n")


# ----------------------------
# Visualization
# ----------------------------
def save_one_channel_png(outdir: str, name: str, y_raw: np.ndarray, y_filt: np.ndarray, unit: str, dt: float):
    n_raw = y_raw.size
    n_f = y_filt.size

    # raw time (reference)
    t_raw = np.arange(n_raw) * dt
    T_raw = t_raw[-1] if n_raw > 0 else 0.0

    # after time: FORCE match the same total duration as raw
    # (so both end at T_raw)
    if n_f <= 1:
        t_f = np.array([0.0])
    else:
        t_f = np.linspace(0.0, T_raw, n_f)

    plt.figure(figsize=(12, 4))
    plt.plot(t_raw, y_raw, label="before")
    plt.plot(t_f,   y_filt, label="after")
    plt.xlabel("time [s]")
    plt.ylabel(f"{name} [{unit}]")
    plt.title(f"{name}: before vs after (time-aligned)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, f"{name}.png"), dpi=200)
    plt.close()



def save_summary_png(outdir: str, raw9: np.ndarray, filt9: np.ndarray, dt: float):
    names_units = [
        ("x_mm", "mm"), ("y_mm", "mm"), ("z_mm", "mm"),
        ("wx_rad", "rad"), ("wy_rad", "rad"), ("wz_rad", "rad"),
        ("fx_N", "N"), ("fy_N", "N"), ("fz_N", "N"),
    ]

    n_raw = raw9.shape[0]
    n_f   = filt9.shape[0]

    tr = np.arange(n_raw) * dt
    T_raw = tr[-1] if n_raw > 0 else 0.0
    tf = np.linspace(0.0, T_raw, n_f) if n_f > 1 else np.array([0.0])

    plt.figure(figsize=(14, 12))
    for i, (nm, unit) in enumerate(names_units):
        plt.subplot(3, 3, i + 1)
        plt.plot(tr, raw9[:, i], label="before")
        plt.plot(tf, filt9[:, i], label="after")
        plt.title(nm)
        plt.grid(True)
        if i == 0:
            plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "summary.png"), dpi=200)
    plt.close()



# ----------------------------
# Main
# ----------------------------
def main():
    ap = argparse.ArgumentParser()

    # 파일은 기본적으로 cmd_continue9D.txt를 사용 (요구사항)
    ap.add_argument("--dir", default=".", help="cmd_continue9D.txt 가 있는 디렉토리 (default: .)")
    ap.add_argument("--infile", default="cmd_continue9D.txt", help="입력 파일명 (default: cmd_continue9D.txt)")
    ap.add_argument("--backup", default="non_filtered_cmd_continue9D.txt", help="백업 파일명")
    ap.add_argument("--record_hz", type=float, default=125.0)

    # force shaping
    ap.add_argument("--zero_xy_forces", type=int, default=1, help="1이면 fx,fy=0 강제 (default=1)")
    ap.add_argument("--force_clamp_abs", type=float, default=200.0)
    ap.add_argument("--force_ema_alpha", type=float, default=0.2)
    ap.add_argument("--edge_force_zero_sec", type=float, default=0.5)
    ap.add_argument("--edge_force_fade_sec", type=float, default=0.3)

    # precontact gating
    ap.add_argument("--precontact_gating", type=int, default=1, help="1이면 contact 이전 force=0 (default=1)")
    ap.add_argument("--fz_on", type=float, default=5.0)
    ap.add_argument("--fz_off", type=float, default=3.0)
    ap.add_argument("--consec_on", type=int, default=10)
    ap.add_argument("--consec_off", type=int, default=10)

    # pose smoothing
    ap.add_argument("--hampel_enable", type=int, default=1)
    ap.add_argument("--hampel_win", type=int, default=6)
    ap.add_argument("--hampel_sig", type=float, default=3.0)

    ap.add_argument("--whittaker_auto", type=int, default=1)
    ap.add_argument("--lam_pos_init", type=float, default=20000.0)
    ap.add_argument("--lam_ang_init", type=float, default=200.0)
    ap.add_argument("--lam_growth", type=float, default=3.0)
    ap.add_argument("--lam_iters", type=int, default=6)
    ap.add_argument("--cg_iters", type=int, default=200)
    ap.add_argument("--cg_tol", type=float, default=1e-8)

    ap.add_argument("--pose_ema_enable", type=int, default=0)
    ap.add_argument("--pose_ema_alpha", type=float, default=0.2)

    # QP-proxy limits + safety
    ap.add_argument("--pos_vmax", type=float, default=30.0)
    ap.add_argument("--pos_amax", type=float, default=120.0)
    ap.add_argument("--ang_vmax", type=float, default=0.6)
    ap.add_argument("--ang_amax", type=float, default=3.0)
    ap.add_argument("--pos_jmax", type=float, default=5000.0)
    ap.add_argument("--ang_jmax", type=float, default=80.0)
    ap.add_argument("--safety", type=float, default=1.05)

    # retime
    ap.add_argument("--retime_enable", type=int, default=1)
    ap.add_argument("--retime_use_jerk", type=int, default=1)
    ap.add_argument("--retime_max_k", type=int, default=20)
    ap.add_argument("--retime_passes", type=int, default=3)

    args = ap.parse_args()

    base_dir = os.path.abspath(args.dir)
    in_path = os.path.join(base_dir, args.infile)
    backup_path = os.path.join(base_dir, args.backup)

    if not os.path.isfile(in_path):
        raise FileNotFoundError(f"Input not found: {in_path}")

    dt = 1.0 / max(1e-9, args.record_hz)
    lim = Limits(
        pos_vmax=args.pos_vmax,
        pos_amax=args.pos_amax,
        ang_vmax=args.ang_vmax,
        ang_amax=args.ang_amax,
        pos_jmax=args.pos_jmax,
        ang_jmax=args.ang_jmax,
    )

    # -------------------------
    # (1) read raw
    # -------------------------
    raw9 = read_txt9(in_path)
    rawN = int(raw9.shape[0])
    print(f"[INFO] Read raw: {in_path} (N={rawN})")

    # -------------------------
    # (2) backup raw -> non_filtered_cmd_continue9D.txt
    # -------------------------
    write_txt9_tab6(backup_path, raw9)
    print(f"[INFO] Backup saved: {backup_path}")

    P = raw9[:, :6].copy()
    F = raw9[:, 6:].copy()

    # --- eval raw (same as recorder)
    st_raw, _ = eval_qp_proxy(P, dt, lim, safety=args.safety)
    print_eval("BEFORE pose smoothing (RAW)", st_raw, lim, args.safety)

    # -------------------------
    # (3) force process
    # -------------------------
    Fp = force_process(
        F,
        zero_xy_forces=bool(args.zero_xy_forces),
        force_clamp_abs=args.force_clamp_abs,
        force_ema_alpha=args.force_ema_alpha
    )

    # -------------------------
    # (4) pose smoothing
    # -------------------------
    Ps, info = pose_smooth(
        P, dt, lim, args.safety,
        hampel_enable=bool(args.hampel_enable),
        hampel_win=args.hampel_win,
        hampel_sig=args.hampel_sig,
        whittaker_auto=bool(args.whittaker_auto),
        lam_pos_init=args.lam_pos_init,
        lam_ang_init=args.lam_ang_init,
        lam_growth=args.lam_growth,
        lam_iters=args.lam_iters,
        cg_iters=args.cg_iters,
        cg_tol=args.cg_tol,
        pose_ema_enable=bool(args.pose_ema_enable),
        pose_ema_alpha=args.pose_ema_alpha
    )
    st_sm, _ = eval_qp_proxy(Ps, dt, lim, safety=args.safety)
    print_eval("AFTER pose smoothing", st_sm, lim, args.safety)
    print(f"[POSE-SMOOTH] used_lams={info}")

    dpos = norm_rows(Ps[:, :3] - P[:, :3])
    dang = norm_rows(Ps[:, 3:] - P[:, 3:])
    print(
        f"[POSE-DELTA] pos: rms={float(np.sqrt(np.mean(dpos**2))):.3f} mm, max={float(dpos.max()):.3f} mm | "
        f"ang: rms={float(np.sqrt(np.mean(dang**2))):.6f} rad, max={float(dang.max()):.6f} rad"
    )

    # -------------------------
    # (5) retime
    # -------------------------
    Pr, Fr, k_total = retime_uniform(
        Ps, Fp,
        dt=dt, lim=lim, safety=args.safety,
        retime_enable=bool(args.retime_enable),
        retime_use_jerk=bool(args.retime_use_jerk),
        retime_max_k=args.retime_max_k,
        retime_passes=args.retime_passes
    )
    st_rt, _ = eval_qp_proxy(Pr, dt, lim, safety=args.safety)
    print_eval("AFTER retiming (pose)", st_rt, lim, args.safety)
    if k_total > 1:
        print(f"[QP-EVAL] Applied time-scale k_total={k_total}  (rows: {Ps.shape[0]} -> {Pr.shape[0]})")

    # -------------------------
    # (6) contact gating
    # -------------------------
    if bool(args.precontact_gating):
        cidx = detect_contact_idx(Fr[:, 2], args.fz_on, args.fz_off, args.consec_on, args.consec_off)
        if cidx is not None and cidx > 0:
            print(f"[CONTACT] Detected at idx={cidx}/{Pr.shape[0]} (t={cidx*dt:.3f}s) -> Zeroing forces for [0:{cidx})")
            Fr[:cidx, :] = 0.0

    # -------------------------
    # (7) edge force window
    # -------------------------
    Fr = apply_edge_force_window(
        Fr, hz=args.record_hz,
        edge_force_zero_sec=args.edge_force_zero_sec,
        edge_force_fade_sec=args.edge_force_fade_sec
    )

    # -------------------------
    # (8) save filtered -> cmd_continue9D.txt (overwrite)
    # -------------------------
    filt9 = np.hstack([Pr, Fr])
    write_txt9_tab6(in_path, filt9)
    print(f"[DONE] Overwrote filtered file: {in_path} (rawN={rawN} -> outN={filt9.shape[0]})")

    # -------------------------
    # (9) visualization folder + per-channel pngs
    # -------------------------
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    viz_dir = os.path.join(base_dir, f"viz_{ts}")
    os.makedirs(viz_dir, exist_ok=True)

    # channel names/units + column index
    meta = [
        ("x_mm", 0, "mm"),
        ("y_mm", 1, "mm"),
        ("z_mm", 2, "mm"),
        ("wx_rad", 3, "rad"),
        ("wy_rad", 4, "rad"),
        ("wz_rad", 5, "rad"),
        ("fx_N", 6, "N"),
        ("fy_N", 7, "N"),
        ("fz_N", 8, "N"),
    ]

    for nm, idx, unit in meta:
        save_one_channel_png(
            outdir=viz_dir,
            name=nm,
            y_raw=raw9[:, idx],
            y_filt=filt9[:, idx],
            unit=unit,
            dt=dt
        )

    # optional summary (9-subplots in one)
    save_summary_png(viz_dir, raw9, filt9, dt)

    print(f"[VIZ] Saved per-channel pngs + summary.png to: {viz_dir}")


if __name__ == "__main__":
    main()
