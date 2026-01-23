#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vr_demo_hdf5_recorder.py

- VR tracker: /calibrated_pose (Float64MultiArray: [x y z wx wy wz])   input: (m, rad)
- FT sensor : /ftsensor/measured_Cvalue (geometry_msgs/Wrench)        input: (N)

Episode rule (same as your baseline):
  start: |fx| >= start_abs_fx
  end  : |fy| >= stop_abs_fy

Unlike txt recorder: repeats episodes and saves into ONE HDF5.
Filtering / path generation pipeline is copied from your vr_demo_txt_recorder baseline:
  Hampel -> Whittaker(CG) auto-lambda -> (optional EMA) -> retime_uniform -> contact gating -> edge force window
"""

import os
import sys
import time
import math
import json
import atexit
import threading
import select
import termios
import tty
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List

import numpy as np
import h5py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


# ============================================================
# Utilities
# ============================================================
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
    if win <= 0:
        return x.copy()
    n = x.size
    y = x.copy()
    k = 1.4826  # MAD -> std approx
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
#   => (I + lam*D2^T D2) z = y
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
    N = int(pose6.shape[0])
    T = dt * max(0, (N - 1))

    dp = pose6[1:, :3] - pose6[:-1, :3]
    dw = pose6[1:, 3:] - pose6[:-1, 3:]
    vpos = norm_rows(dp) / dt
    vang = norm_rows(dw) / dt

    v = (pose6[1:, :] - pose6[:-1, :]) / dt
    a = (v[1:, :] - v[:-1, :]) / dt
    apos = norm_rows(a[:, :3])
    aang = norm_rows(a[:, 3:])

    j = (a[1:, :] - a[:-1, :]) / dt
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
# Contact detection (fz hysteresis + consecutive)
# ----------------------------
def detect_contact_idx(fz: np.ndarray, fz_on: float, fz_off: float, consec_on: int, consec_off: int) -> Optional[int]:
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
# Edge force window (same as baseline)
# ----------------------------
def apply_edge_force_window(F: np.ndarray, hz: float, edge_zero_sec: float, edge_fade_sec: float) -> np.ndarray:
    out = F.copy()
    n = out.shape[0]
    zN = int(round(edge_zero_sec * hz))
    fN = int(round(edge_fade_sec * hz))
    zN = max(0, min(n, zN))
    fN = max(0, min(n, fN))

    if n == 0:
        return out

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


# ============================================================
# Keyboard watcher (press 'q' to quit; no Enter)
# ============================================================
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
            tty.setcbreak(self._fd)
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
            pass
        finally:
            self._restore_term()


# ============================================================
# Main Node
# ============================================================
class VRDemoHDF5Recorder(Node):
    def __init__(self):
        super().__init__("vr_demo_hdf5_recorder")

        # -------------------------
        # HDF5 / run control
        # -------------------------
        self.declare_parameter("save_dir", "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/datasets")
        self.declare_parameter("hdf5_name", "vr_demo_stage1.hdf5")
        self.declare_parameter("overwrite", True)
        self.declare_parameter("flush_each_episode", True)

        self.declare_parameter("num_episodes", 50)
        self.declare_parameter("min_raw_samples", 10)
        self.declare_parameter("quit_key", "q")

        # -------------------------
        # topics
        # -------------------------
        self.declare_parameter("pose_topic", "/calibrated_pose")             # Float64MultiArray [x y z wx wy wz] (m,rad)
        self.declare_parameter("force_topic", "/ftsensor/measured_Cvalue")  # geometry_msgs/Wrench

        # -------------------------
        # recorder timing (same as baseline)
        # -------------------------
        self.declare_parameter("record_hz", 125.0)
        self.declare_parameter("require_fresh_sec", 0.2)

        # -------------------------
        # episode rule (same as baseline)
        # -------------------------
        self.declare_parameter("start_abs_fx", 10.0)
        self.declare_parameter("stop_abs_fy", 10.0)

        # -------------------------
        # force shaping (same as baseline)
        # -------------------------
        self.declare_parameter("zero_xy_forces", True)
        self.declare_parameter("force_clamp_abs", 200.0)
        self.declare_parameter("force_ema_alpha", 0.2)
        self.declare_parameter("edge_force_zero_sec", 0.5)
        self.declare_parameter("edge_force_fade_sec", 0.3)

        # -------------------------
        # pre-contact gating (same as baseline)
        # -------------------------
        self.declare_parameter("precontact_gating", True)
        self.declare_parameter("fz_on", 5.0)
        self.declare_parameter("fz_off", 3.0)
        self.declare_parameter("consec_on", 10)
        self.declare_parameter("consec_off", 10)

        # -------------------------
        # pose smoothing (same as baseline)
        # -------------------------
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

        # -------------------------
        # QP-proxy limits (same as baseline)
        # -------------------------
        self.declare_parameter("pos_vmax", 30.0)
        self.declare_parameter("pos_amax", 120.0)
        self.declare_parameter("ang_vmax", 0.6)
        self.declare_parameter("ang_amax", 3.0)
        self.declare_parameter("pos_jmax", 5000.0)
        self.declare_parameter("ang_jmax", 80.0)
        self.declare_parameter("safety", 1.05)

        # -------------------------
        # retiming (same as baseline)
        # -------------------------
        self.declare_parameter("retime_enable", True)
        self.declare_parameter("retime_use_jerk", True)
        self.declare_parameter("retime_max_k", 20)
        self.declare_parameter("retime_passes", 3)

        # -------------------------
        # load params
        # -------------------------
        self.save_dir = str(self.get_parameter("save_dir").value)
        self.hdf5_name = str(self.get_parameter("hdf5_name").value)
        self.overwrite = bool(self.get_parameter("overwrite").value)
        self.flush_each_episode = bool(self.get_parameter("flush_each_episode").value)

        self.num_episodes = int(self.get_parameter("num_episodes").value)
        self.min_raw_samples = int(self.get_parameter("min_raw_samples").value)
        self.quit_key = str(self.get_parameter("quit_key").value)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)

        self.record_hz = float(self.get_parameter("record_hz").value)
        self.dt = 1.0 / max(1e-9, self.record_hz)
        self.require_fresh_sec = float(self.get_parameter("require_fresh_sec").value)

        self.start_abs_fx = float(self.get_parameter("start_abs_fx").value)
        self.stop_abs_fy = float(self.get_parameter("stop_abs_fy").value)

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

        # -------------------------
        # file open
        # -------------------------
        os.makedirs(self.save_dir, exist_ok=True)
        self.hdf5_path = os.path.join(self.save_dir, self.hdf5_name)
        if self.overwrite and os.path.exists(self.hdf5_path):
            os.remove(self.hdf5_path)

        self.h5_lock = threading.Lock()
        self.h5 = h5py.File(self.hdf5_path, "a")
        self.grp_eps = self.h5.require_group("episodes")

        self.episode_count = self._detect_existing_episode_count()
        self._write_root_meta_once()

        # -------------------------
        # runtime state (same style as baseline)
        # -------------------------
        self.state_lock = threading.Lock()

        self.latest_pose6_mm_rad: Optional[np.ndarray] = None
        self.latest_force3_N: Optional[np.ndarray] = None
        self.latest_pose_t: float = 0.0
        self.latest_force_t: float = 0.0

        self.episode_active = False
        self.finishing_ = False

        self.buf_pose: List[np.ndarray] = []
        self.buf_force: List[np.ndarray] = []
        self.buf_t: List[float] = []

        self.stop_requested = False
        self.stop_reason = ""

        # -------------------------
        # ROS IO
        # -------------------------
        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self.cb_pose, 50)
        self.sub_force = self.create_subscription(Wrench, self.force_topic, self.cb_force, 10)
        self.timer = self.create_timer(self.dt, self.cb_timer)
        self.timer_stop = self.create_timer(0.05, self._check_stop)

        # keyboard
        self.kb = _KeyboardQuitter(quit_key=self.quit_key)
        enabled = self.kb.start()
        atexit.register(self.kb.stop)

        # logs
        self.get_logger().info("============================================================")
        self.get_logger().info("VRDemoHDF5Recorder initialized (multi-episode -> single HDF5)")
        self.get_logger().info(f"  HDF5: {self.hdf5_path}")
        self.get_logger().info(f"  Topics: pose={self.pose_topic} (Float64MultiArray), force={self.force_topic} (Wrench)")
        self.get_logger().info(f"  record_hz={self.record_hz}, require_fresh_sec={self.require_fresh_sec}")
        self.get_logger().info(f"  Episode rule: start=|fx|>={self.start_abs_fx}, end=|fy|>={self.stop_abs_fy}")
        self.get_logger().info(f"  Target episodes: {self.num_episodes} (current existing: {self.episode_count})")
        self.get_logger().info(
            f"  Pose smoothing: Hampel={self.hampel_enable}(win={self.hampel_win}, sig={self.hampel_sig}) + "
            f"WhittakerAuto={self.whittaker_auto}(lam_pos_init={self.lam_pos_init}, lam_ang_init={self.lam_ang_init}, "
            f"growth={self.lam_growth}, iters={self.lam_iters}) + PoseEMA={self.pose_ema_enable}(alpha={self.pose_ema_alpha})"
        )
        self.get_logger().info(
            f"  Pre-contact gating: {self.precontact_gating} (fz_on={self.fz_on}, fz_off={self.fz_off}, consec_on={self.consec_on})"
        )
        self.get_logger().info(
            f"  QP-proxy limits: pos_vmax={self.lim.pos_vmax} mm/s, pos_amax={self.lim.pos_amax} mm/s^2, "
            f"ang_vmax={self.lim.ang_vmax} rad/s, ang_amax={self.lim.ang_amax} rad/s^2, safety={self.safety}"
        )
        self.get_logger().info(
            f"  Retime: enable={self.retime_enable}, use_jerk={self.retime_use_jerk}, max_k={self.retime_max_k}, passes={self.retime_passes}"
        )
        if enabled:
            self.get_logger().info(f"  Press '{self.quit_key}' to stop (no Enter). Ctrl+C also works.")
        else:
            self.get_logger().warn("  stdin is not a TTY -> 'q' quit disabled. Use Ctrl+C instead.")
        self.get_logger().info("============================================================")

    # ============================================================
    # HDF5 helpers
    # ============================================================
    def _detect_existing_episode_count(self) -> int:
        max_idx = -1
        for k in self.grp_eps.keys():
            if k.startswith("ep_"):
                try:
                    idx = int(k.split("_")[1])
                    max_idx = max(max_idx, idx)
                except Exception:
                    pass
        return max_idx + 1

    def _write_root_meta_once(self):
        # do not overwrite if already exists (append mode)
        if "created_unix" not in self.h5.attrs:
            self.h5.attrs["created_unix"] = float(time.time())
        self.h5.attrs["columns"] = np.string_("x_mm,y_mm,z_mm,wx,wy,wz,fx,fy,fz")
        self.h5.attrs["note_pose"] = np.string_("pose xyz input meters -> stored millimeters; omega stored radians")
        self.h5.attrs["record_hz"] = float(self.record_hz)
        self.h5.attrs["dt"] = float(self.dt)
        self.h5.attrs["episode_rule"] = np.string_(f"start=|fx|>={self.start_abs_fx}, end=|fy|>={self.stop_abs_fy}")
        self.h5.flush()

    # ============================================================
    # stop control
    # ============================================================
    def request_stop(self, reason: str = "user_request"):
        self.stop_requested = True
        self.stop_reason = str(reason)
        self.get_logger().warn(f"[STOP REQUEST] reason={self.stop_reason}")

    def _check_stop(self):
        if self.stop_requested and (not self.finishing_) and (not self.episode_active):
            self.finalize_and_shutdown()

    def finalize_and_shutdown(self):
        self.get_logger().warn("Finalizing HDF5 and shutting down...")
        try:
            with self.h5_lock:
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

    # ============================================================
    # ROS callbacks (collect raw)
    # ============================================================
    def cb_pose(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        x, y, z, wx, wy, wz = msg.data[:6]
        pose = np.array([1000.0 * x, 1000.0 * y, 1000.0 * z, wx, wy, wz], dtype=np.float64)
        with self.state_lock:
            self.latest_pose6_mm_rad = pose
            self.latest_pose_t = time.time()

    def cb_force(self, msg: Wrench):
        fx = float(msg.force.x)
        fy = float(msg.force.y)
        fz = float(msg.force.z)
        F = np.array([fx, fy, fz], dtype=np.float64)

        with self.state_lock:
            self.latest_force3_N = F
            self.latest_force_t = time.time()

        if self.stop_requested:
            return

        # keyboard stop check (fast path)
        if self.kb.hit() and (not self.stop_requested):
            self.request_stop(reason=f"keyboard_{self.quit_key}")

        if self.finishing_:
            return

        abs_fx_over_start = abs(fx) >= self.start_abs_fx
        abs_fy_over_end = abs(fy) >= self.stop_abs_fy

        # start
        if (not self.episode_active) and abs_fx_over_start:
            with self.state_lock:
                self.episode_active = True
                self.buf_pose.clear()
                self.buf_force.clear()
                self.buf_t.clear()
            self.get_logger().info(f"=== EPISODE STARTED (idx={self.episode_count:04d}) ===")
            return

        # end
        if self.episode_active and abs_fy_over_end:
            self.get_logger().info(f"=== EPISODE ENDED (idx={self.episode_count:04d}) by |fy| threshold ===")
            self._start_finish_thread(reason="fy_threshold")
            return

    def cb_timer(self):
        if self.stop_requested and self.episode_active and (not self.finishing_):
            # stop requested: try to finalize current episode too
            self.get_logger().warn("Stop requested while recording -> closing current episode.")
            self._start_finish_thread(reason=self.stop_reason or "stop_requested")
            return

        if (not self.episode_active) or self.finishing_ or self.stop_requested:
            return

        now = time.time()
        with self.state_lock:
            if self.latest_pose6_mm_rad is None or (now - self.latest_pose_t) > self.require_fresh_sec:
                return
            if self.latest_force3_N is None or (now - self.latest_force_t) > self.require_fresh_sec:
                return

            self.buf_pose.append(self.latest_pose6_mm_rad.copy())
            self.buf_force.append(self.latest_force3_N.copy())
            self.buf_t.append(now)

    # ============================================================
    # Finish episode (threaded)
    # ============================================================
    def _start_finish_thread(self, reason: str):
        if self.finishing_:
            return
        self.finishing_ = True

        with self.state_lock:
            self.episode_active = False
            P_list = self.buf_pose.copy()
            F_list = self.buf_force.copy()
            self.buf_pose.clear()
            self.buf_force.clear()
            self.buf_t.clear()

        th = threading.Thread(target=self._finish_episode_worker, args=(P_list, F_list, reason), daemon=True)
        th.start()

    def _finish_episode_worker(self, P_list: List[np.ndarray], F_list: List[np.ndarray], reason: str):
        try:
            if len(P_list) < max(1, self.min_raw_samples):
                self.get_logger().warn(
                    f"Episode dropped (too short): raw_len={len(P_list)} < {self.min_raw_samples}, reason={reason}"
                )
                return

            P = np.asarray(P_list, dtype=np.float64)   # (N,6) [mm, rad]
            F = np.asarray(F_list, dtype=np.float64)   # (N,3) [N]
            rawN = int(P.shape[0])

            # 1) force process
            Fp = self._force_process(F)

            # 2) pose smoothing (auto lambda)
            Ps, info = self._pose_smooth(P)

            # 3) retime uniform (time dilation)
            Pr, Fr, k_total = self._retime_uniform(Ps, Fp)

            # 4) contact gating
            if self.precontact_gating:
                cidx = detect_contact_idx(Fr[:, 2], self.fz_on, self.fz_off, self.consec_on, self.consec_off)
                if cidx is not None and cidx > 0:
                    Fr[:cidx, :] = 0.0

            # 5) edge force window
            Fr = apply_edge_force_window(
                Fr, hz=self.record_hz,
                edge_zero_sec=self.edge_force_zero_sec,
                edge_fade_sec=self.edge_force_fade_sec
            )

            out = np.hstack([Pr, Fr]).astype(np.float32)  # (N,9)

            # save
            ep_idx = self.episode_count
            self._save_episode_to_hdf5(ep_idx, out, reason=reason, raw_len=rawN, k_total=k_total, used_lams=info)
            self.episode_count += 1

            self.get_logger().info(
                f"=== EPISODE SAVED (idx={ep_idx:04d}) raw_len={rawN} -> out_len={out.shape[0]} "
                f"k_total={k_total} reason={reason} ==="
            )

            if self.episode_count >= self.num_episodes:
                self.request_stop(reason="reached_num_episodes")

        except Exception as e:
            self.get_logger().error(f"Episode processing failed: {e}")
        finally:
            self.finishing_ = False

    # ============================================================
    # Baseline pipeline parts
    # ============================================================
    def _force_process(self, F: np.ndarray) -> np.ndarray:
        Fp = F.copy()
        Fp = np.clip(Fp, -self.force_clamp_abs, self.force_clamp_abs)
        if self.zero_xy_forces:
            Fp[:, 0] = 0.0
            Fp[:, 1] = 0.0
        if 0.0 < self.force_ema_alpha < 1.0:
            Fp = ema_nd(Fp, alpha=self.force_ema_alpha)
        return Fp

    def _pose_smooth(self, P: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
        P0 = P.copy()

        if self.hampel_enable:
            P0 = hampel_nd(P0, win=self.hampel_win, n_sigmas=self.hampel_sig)

        if not self.whittaker_auto:
            Pp = P0.copy()
            Pp[:, :3] = whittaker_cg_nd(Pp[:, :3], lam=self.lam_pos_init, cg_iters=self.cg_iters, tol=self.cg_tol)
            Pp[:, 3:] = whittaker_cg_nd(Pp[:, 3:], lam=self.lam_ang_init, cg_iters=self.cg_iters, tol=self.cg_tol)
            if self.pose_ema_enable:
                Pp = ema_nd(Pp, alpha=self.pose_ema_alpha)
            return Pp, {"lam_pos": self.lam_pos_init, "lam_ang": self.lam_ang_init}

        lam_pos = self.lam_pos_init
        lam_ang = self.lam_ang_init

        best = None
        best_score = 1e18
        best_info = {"lam_pos": lam_pos, "lam_ang": lam_ang}

        # same constraints as baseline
        max_pos_delta_allow = 5.0   # mm
        max_ang_delta_allow = 0.03  # rad

        for _ in range(max(1, self.lam_iters)):
            Pp = P0.copy()
            Pp[:, :3] = whittaker_cg_nd(Pp[:, :3], lam=lam_pos, cg_iters=self.cg_iters, tol=self.cg_tol)
            Pp[:, 3:] = whittaker_cg_nd(Pp[:, 3:], lam=lam_ang, cg_iters=self.cg_iters, tol=self.cg_tol)

            if self.pose_ema_enable:
                Pp = ema_nd(Pp, alpha=self.pose_ema_alpha)

            dpos = norm_rows(Pp[:, :3] - P[:, :3])
            dang = norm_rows(Pp[:, 3:] - P[:, 3:])
            if float(dpos.max()) > max_pos_delta_allow or float(dang.max()) > max_ang_delta_allow:
                break

            st, _ = eval_qp_proxy(Pp, self.dt, self.lim, safety=self.safety)
            score = max(
                st.apos_p95 / (self.lim.pos_amax + 1e-9),
                st.aang_p95 / (self.lim.ang_amax + 1e-9),
                st.jpos_p95 / (self.lim.pos_jmax + 1e-9),
                st.jang_p95 / (self.lim.ang_jmax + 1e-9),
            ) + 0.05 * (float(dpos.mean()) / 1.0)

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
        if not self.retime_enable:
            return P, F, 1

        Pcur = P.copy()
        Fcur = F.copy()
        k_total = 1

        for _ in range(max(1, self.retime_passes)):
            st, _ = eval_qp_proxy(Pcur, self.dt, self.lim, safety=self.safety)

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

            remaining = max(1, self.retime_max_k // max(1, k_total))
            k_need = min(k_need, remaining)

            if k_need <= 1:
                break

            Pcur = upsample_linear(Pcur, k_need)
            Fcur = upsample_linear(Fcur, k_need)
            k_total *= k_need

        return Pcur, Fcur, k_total

    # ============================================================
    # HDF5 save
    # ============================================================
    def _save_episode_to_hdf5(
        self,
        ep_idx: int,
        out: np.ndarray,
        reason: str,
        raw_len: int,
        k_total: int,
        used_lams: Dict[str, float],
    ):
        ep_name = f"ep_{ep_idx:04d}"
        with self.h5_lock:
            if ep_name in self.grp_eps:
                del self.grp_eps[ep_name]
            g = self.grp_eps.create_group(ep_name)
            g.attrs["saved_unix"] = float(time.time())
            g.attrs["reason"] = np.string_(reason)
            g.attrs["raw_len"] = int(raw_len)
            g.attrs["out_len"] = int(out.shape[0])
            g.attrs["dtype"] = np.string_(str(out.dtype))
            g.attrs["record_hz"] = float(self.record_hz)
            g.attrs["dt"] = float(self.dt)
            g.attrs["k_total"] = int(k_total)
            g.attrs["used_lams_json"] = np.string_(json.dumps(used_lams))

            # also store key params for reproducibility
            g.attrs["force_clamp_abs"] = float(self.force_clamp_abs)
            g.attrs["force_ema_alpha"] = float(self.force_ema_alpha)
            g.attrs["edge_force_zero_sec"] = float(self.edge_force_zero_sec)
            g.attrs["edge_force_fade_sec"] = float(self.edge_force_fade_sec)

            g.attrs["precontact_gating"] = int(self.precontact_gating)
            g.attrs["fz_on"] = float(self.fz_on)
            g.attrs["fz_off"] = float(self.fz_off)
            g.attrs["consec_on"] = int(self.consec_on)
            g.attrs["consec_off"] = int(self.consec_off)

            g.attrs["safety"] = float(self.safety)
            g.attrs["pos_vmax"] = float(self.lim.pos_vmax)
            g.attrs["pos_amax"] = float(self.lim.pos_amax)
            g.attrs["ang_vmax"] = float(self.lim.ang_vmax)
            g.attrs["ang_amax"] = float(self.lim.ang_amax)
            g.attrs["pos_jmax"] = float(self.lim.pos_jmax)
            g.attrs["ang_jmax"] = float(self.lim.ang_jmax)

            g.create_dataset(
                "traj",
                data=out,
                compression="gzip",
                compression_opts=4,
                shuffle=True,
            )

            if self.flush_each_episode:
                self.h5.flush()


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoHDF5Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C: stop gracefully
        try:
            node.request_stop(reason="KeyboardInterrupt")
        except Exception:
            pass
        # give time for current finishing thread
        time.sleep(0.1)
        try:
            if rclpy.ok():
                node.finalize_and_shutdown()
        except Exception:
            pass
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
