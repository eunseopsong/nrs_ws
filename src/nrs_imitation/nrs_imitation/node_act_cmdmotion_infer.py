#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
node_act_cmdmotion_infer.py (BASELINE + STATE MACHINE)
APPROACH -> PRELOAD(PRESS) -> TRACK -> (optional RELEASE)

Baseline guarantees kept:
1) FIRST publish MUST equal current pose (/ur10skku/currentP) for pose6.
2) After that, command follows inference output SLOWLY (QP-safe):
   - Temporal aggregation across multiple plans (exp decay)
   - Anchor offset (current_pose - pred_pose at first usage)
   - EMA smoothing (tau_sec) + per-tick step caps + startup ramp

Patch focus:
- Start MUST be APPROACH.
- PRELOAD(PRESS) transition ONLY after robust "touch" detection:
    touch = (meas_fz - fz_baseline) >= touch_fz_thr
    AND touch_ok_count consecutive
    AND after touch_min_after_start_sec from first publish
- During PRELOAD: stop inference(plan generation), hold XY/RPY, z-servo to raise meas_fz to preload target.
- When preload achieved (within tol for ok_count) -> TRACK (resume inference).

NEW PATCH (requested):
- If robot stays still (pose change small) for stall_sec or longer,
  inject cmd_fz = fz_kick_N for fz_kick_dur_sec, then clear plans to trigger replan.
- No forced z decrease; only force kick to push contact/touch transition.
"""

import os
import sys
import time
import math
import pickle
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Deque, List
from enum import Enum

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image


# ============================================================
# Helpers (QoS / time / math)
# ============================================================

def _monotonic() -> float:
    return time.monotonic()

def _reliability_from_str(s: str) -> ReliabilityPolicy:
    s = (s or "").strip().lower()
    if s in ["reliable", "rel"]:
        return ReliabilityPolicy.RELIABLE
    if s in ["best_effort", "besteffort", "best"]:
        return ReliabilityPolicy.BEST_EFFORT
    return ReliabilityPolicy.BEST_EFFORT

def _qos(depth: int, reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )

def _exp_decay_weight(age_steps: int, tau_steps: float) -> float:
    if tau_steps <= 1e-9:
        return 1.0
    age_steps = max(0, int(age_steps))
    return float(math.exp(-float(age_steps) / float(tau_steps)))

def _beta_from_tau(dt: float, tau: float) -> float:
    # EMA beta: x <- x + beta*(target-x)
    if tau <= 1e-9:
        return 1.0
    return float(1.0 - math.exp(-float(dt) / float(tau)))


# ============================================================
# Helpers (Image decode)
# ============================================================

def _img_to_rgb_numpy(msg: Image) -> np.ndarray:
    """
    Convert sensor_msgs/Image -> np.uint8 (H,W,3) RGB
    Supports: rgb8, bgr8, rgba8, bgra8
    """
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or "").lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == "rgb8":
        return buf.reshape((h, w, 3))
    if enc == "bgr8":
        img = buf.reshape((h, w, 3))
        return img[..., ::-1].copy()
    if enc == "rgba8":
        return buf.reshape((h, w, 4))[..., :3]
    if enc == "bgra8":
        img = buf.reshape((h, w, 4))[..., :3]
        return img[..., ::-1].copy()

    # fallback
    try:
        return buf.reshape((h, w, 3))
    except Exception as e:
        raise RuntimeError(f"Unsupported image encoding={msg.encoding}, size=({h},{w}), err={e}")

def _to_tensor_image_stack(top_rgb: np.ndarray, ee_rgb: np.ndarray, device: torch.device,
                           resize_hw: int = 0) -> torch.Tensor:
    """
    (H,W,3) -> (1,2,3,H,W) float in [0,1]
    cam order fixed: [top, ee]
    Optional resize to (resize_hw, resize_hw) if resize_hw>0
    """
    if top_rgb is None or ee_rgb is None:
        raise RuntimeError("top/ee image is None")

    if resize_hw and resize_hw > 0:
        try:
            import cv2
            top_rgb = cv2.resize(top_rgb, (resize_hw, resize_hw), interpolation=cv2.INTER_LINEAR)
            ee_rgb  = cv2.resize(ee_rgb,  (resize_hw, resize_hw), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            raise RuntimeError(f"cv2 resize failed (resize_hw={resize_hw}): {e}")
    else:
        if top_rgb.shape != ee_rgb.shape:
            raise RuntimeError(f"Top/Ee image size mismatch: top={top_rgb.shape}, ee={ee_rgb.shape}")

    top = np.transpose(top_rgb, (2, 0, 1))
    ee  = np.transpose(ee_rgb,  (2, 0, 1))

    img = np.stack([top, ee], axis=0).astype(np.float32) / 255.0  # (2,3,H,W)
    img_t = torch.from_numpy(img).unsqueeze(0).to(device=device, dtype=torch.float32)  # (1,2,3,H,W)
    return img_t


# ============================================================
# Helpers (Stats)
# ============================================================

@dataclass
class StatsPack:
    qpos_mean: np.ndarray
    qpos_std:  np.ndarray
    act_mean:  np.ndarray
    act_std:   np.ndarray

def _sanitize_std(x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    x = np.asarray(x, dtype=np.float32).reshape(-1)
    return np.maximum(x, eps)

def _load_dataset_stats(ckpt_dir: str) -> Optional[StatsPack]:
    p = os.path.join(ckpt_dir, "dataset_stats.pkl")
    if not os.path.exists(p):
        return None
    with open(p, "rb") as f:
        st = pickle.load(f)

    if all(k in st for k in ["qpos_mean", "qpos_std", "action_mean", "action_std"]):
        qm = np.asarray(st["qpos_mean"], dtype=np.float32).reshape(9)
        qs = _sanitize_std(np.asarray(st["qpos_std"], dtype=np.float32).reshape(9))
        am = np.asarray(st["action_mean"], dtype=np.float32).reshape(9)
        astd = _sanitize_std(np.asarray(st["action_std"], dtype=np.float32).reshape(9))
        return StatsPack(qm, qs, am, astd)

    return None

def _normalize_qpos(q: torch.Tensor, stats: StatsPack) -> torch.Tensor:
    mu = torch.tensor(stats.qpos_mean, dtype=torch.float32, device=q.device).view(1, 9)
    sd = torch.tensor(stats.qpos_std,  dtype=torch.float32, device=q.device).view(1, 9)
    return (q - mu) / sd

def _denorm_action_seq(seq: torch.Tensor, stats: StatsPack) -> torch.Tensor:
    mu = torch.tensor(stats.act_mean, dtype=torch.float32, device=seq.device)
    sd = torch.tensor(stats.act_std,  dtype=torch.float32, device=seq.device)
    if seq.dim() == 2:
        return seq * sd.view(1, 9) + mu.view(1, 9)
    if seq.dim() == 3:
        return seq * sd.view(1, 1, 9) + mu.view(1, 1, 9)
    raise RuntimeError(f"unexpected seq dim: {seq.shape}")


# ============================================================
# Helpers (Policy output shape)
# ============================================================

def _fix_a_hat_shape(a_hat: torch.Tensor, chunk_size: int) -> torch.Tensor:
    """
    Standardize output to (T,9) with T=chunk_size.
    Handles:
      - (1,T,9)
      - (T,1,9)
      - (T,9)
    """
    if a_hat.dim() == 2:
        return a_hat
    if a_hat.dim() != 3:
        raise RuntimeError(f"Unexpected a_hat dim: {a_hat.shape}")

    B0, B1, B2 = a_hat.shape
    if B2 != 9:
        raise RuntimeError(f"Unexpected last dim (need 9): {a_hat.shape}")

    if B0 == 1 and B1 == chunk_size:
        return a_hat[0]
    if B0 == chunk_size and B1 == 1:
        return a_hat[:, 0, :]
    if B1 == chunk_size:
        return a_hat[0]
    raise RuntimeError(f"Cannot interpret a_hat shape={a_hat.shape} with chunk_size={chunk_size}")


# ============================================================
# Plan buffer entry
# ============================================================

@dataclass
class Plan:
    t0: float
    seq_den: np.ndarray  # (T,9) denorm


# ============================================================
# Stage machine
# ============================================================

class Stage(Enum):
    APPROACH = 0
    PRELOAD = 1
    TRACK = 2
    RELEASE = 3


# ============================================================
# State dict compatibility loader
# ============================================================

def _strip_prefix_from_state_dict(sd: dict, prefixes: List[str]) -> dict:
    out = {}
    for k, v in sd.items():
        nk = k
        for p in prefixes:
            if nk.startswith(p):
                nk = nk[len(p):]
        out[nk] = v
    return out

def _try_load_state_dict_compat(model: torch.nn.Module, state_dict: dict):
    """
    Try several key transforms and pick the best (min missing+unexpected).
    Returns: (missing, unexpected)
    """
    candidates = []
    candidates.append(("orig", state_dict))
    candidates.append(("strip_model.", _strip_prefix_from_state_dict(state_dict, ["model."])))
    candidates.append(("strip_module.", _strip_prefix_from_state_dict(state_dict, ["module."])))
    candidates.append(("strip_model+module", _strip_prefix_from_state_dict(state_dict, ["module.", "model."])))
    candidates.append(("strip_policy.", _strip_prefix_from_state_dict(state_dict, ["policy."])))
    candidates.append(("strip_model_module", _strip_prefix_from_state_dict(state_dict, ["model.", "module."])))

    best_missing = None
    best_unexpected = None
    best_score = None

    for _, sd in candidates:
        try:
            missing, unexpected = model.load_state_dict(sd, strict=False)
            score = len(missing) + len(unexpected)
            if (best_score is None) or (score < best_score):
                best_score = score
                best_missing = missing
                best_unexpected = unexpected
        except Exception:
            continue

    if best_missing is None:
        missing, unexpected = model.load_state_dict(state_dict, strict=False)
        return missing, unexpected

    return best_missing, best_unexpected


# ============================================================
# ROS2 Node
# ============================================================

class NodeActCmdMotionInfer(Node):
    def __init__(self):
        super().__init__("node_act_cmdmotion_infer")

        # -----------------------------
        # Parameters (paths / IO)
        # -----------------------------
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("act_root", "")
        self.declare_parameter("chunk_size", 100)

        self.declare_parameter("pose_topic", "/ur10skku/currentP")
        self.declare_parameter("force_topic", "/ur10skku/currentF")
        self.declare_parameter("top_img_topic", "/realsense/top/color/image_raw")
        self.declare_parameter("ee_img_topic",  "/realsense/ee/color/image_raw")
        self.declare_parameter("cmd_topic", "/ur10skku/cmdMotion")

        self.declare_parameter("image_qos", "best_effort")
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("infer_hz", 7.5)

        # -----------------------------
        # Baseline safety (QP-safe)
        # -----------------------------
        self.declare_parameter("tau_sec", 0.35)
        self.declare_parameter("startup_ramp_sec", 1.5)
        self.declare_parameter("step_cap_pos_mm", 0.30)
        self.declare_parameter("step_cap_ang_rad", 0.0004)
        self.declare_parameter("step_cap_fz", 0.20)

        self.declare_parameter("use_temporal_agg", True)
        self.declare_parameter("temporal_agg_mode", "exp")
        self.declare_parameter("temporal_agg_tau_steps", 20.0)
        self.declare_parameter("pred_step_offset", 1)
        self.declare_parameter("max_plans", 6)

        # contact gating (hysteresis)
        self.declare_parameter("contact_on_thr", 2.0)
        self.declare_parameter("contact_off_thr", 1.0)
        self.declare_parameter("clear_plans_on_contact_change", False)

        # touch detection (robust PRESS trigger)
        self.declare_parameter("touch_fz_thr", 0.5)
        self.declare_parameter("touch_ok_count", 3)
        self.declare_parameter("touch_min_after_start_sec", 1.0)
        self.declare_parameter("touch_baseline_tau_sec", 0.5)
        self.declare_parameter("touch_use_delta", True)

        # preload (PRESS) behavior
        self.declare_parameter("preload_target_source", "stats_mean")  # stats_mean | fixed
        self.declare_parameter("preload_fixed_N", 10.0)
        self.declare_parameter("preload_target_scale", 1.0)
        self.declare_parameter("preload_min_N", 10.0)
        self.declare_parameter("preload_timeout_sec", 5.0)
        self.declare_parameter("preload_ok_count", 10)
        self.declare_parameter("preload_kp_mm_per_N", 0.02)
        self.declare_parameter("preload_dz_max_mm", 0.08)
        self.declare_parameter("preload_tol_N", 0.2)
        self.declare_parameter("press_force_cmd_mode", "target")  # keep|zero|target
        self.declare_parameter("press_hold_xy", True)
        self.declare_parameter("press_hold_rpy", True)

        # optional release assist
        self.declare_parameter("release_assist_enable", False)
        self.declare_parameter("release_ramp_sec", 1.0)

        # I/O shaping
        self.declare_parameter("force_indices", [0, 1, 2])
        self.declare_parameter("first_cmd_fz", 0.0)
        self.declare_parameter("action_type", "absolute")  # absolute | delta
        self.declare_parameter("normalize_qpos", True)
        self.declare_parameter("denorm_action", True)
        self.declare_parameter("resize_hw", 0)
        self.declare_parameter("debug_every_n", 30)

        # force safety
        self.declare_parameter("fz_hard_limit", 25.0)

        # policy config
        self.declare_parameter("kl_weight", 10.0)
        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("lr_backbone", 1e-5)
        self.declare_parameter("backbone", "resnet18")
        self.declare_parameter("enc_layers", 4)
        self.declare_parameter("dec_layers", 7)
        self.declare_parameter("nheads", 8)
        self.declare_parameter("image_resize_hw", 256)
        self.declare_parameter("image_pool_hw", 4)
        self.declare_parameter("pretrained_backbone", True)

        # -----------------------------
        # NEW: Stall -> FZ KICK
        # -----------------------------
        self.declare_parameter("stall_sec", 0.8)
        self.declare_parameter("stall_pos_eps_mm", 0.02)
        self.declare_parameter("stall_ang_eps_rad", 0.0002)
        self.declare_parameter("stall_min_after_start_sec", 1.0)
        self.declare_parameter("fz_kick_N", 1.5)
        self.declare_parameter("fz_kick_dur_sec", 0.35)
        self.declare_parameter("fz_kick_cooldown_sec", 0.8)
        self.declare_parameter("fz_kick_enable_in_track", False)

        # -----------------------------
        # Read params
        # -----------------------------
        self.ckpt_dir = str(self.get_parameter("ckpt_dir").value)
        self.act_root = str(self.get_parameter("act_root").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)
        self.top_img_topic = str(self.get_parameter("top_img_topic").value)
        self.ee_img_topic = str(self.get_parameter("ee_img_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.image_qos_str = str(self.get_parameter("image_qos").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.infer_hz = float(self.get_parameter("infer_hz").value)

        self.tau_sec = float(self.get_parameter("tau_sec").value)
        self.startup_ramp_sec = float(self.get_parameter("startup_ramp_sec").value)
        self.step_cap_pos_mm = float(self.get_parameter("step_cap_pos_mm").value)
        self.step_cap_ang_rad = float(self.get_parameter("step_cap_ang_rad").value)
        self.step_cap_fz = float(self.get_parameter("step_cap_fz").value)

        self.use_temporal_agg = bool(self.get_parameter("use_temporal_agg").value)
        self.temporal_agg_mode = str(self.get_parameter("temporal_agg_mode").value).strip().lower()
        self.temporal_agg_tau_steps = float(self.get_parameter("temporal_agg_tau_steps").value)
        self.pred_step_offset = int(self.get_parameter("pred_step_offset").value)
        self.max_plans = int(self.get_parameter("max_plans").value)

        self.contact_on_thr = float(self.get_parameter("contact_on_thr").value)
        self.contact_off_thr = float(self.get_parameter("contact_off_thr").value)
        self.clear_plans_on_contact_change = bool(self.get_parameter("clear_plans_on_contact_change").value)

        self.touch_fz_thr = float(self.get_parameter("touch_fz_thr").value)
        self.touch_ok_count = int(self.get_parameter("touch_ok_count").value)
        self.touch_min_after_start_sec = float(self.get_parameter("touch_min_after_start_sec").value)
        self.touch_baseline_tau_sec = float(self.get_parameter("touch_baseline_tau_sec").value)
        self.touch_use_delta = bool(self.get_parameter("touch_use_delta").value)

        self.preload_target_source = str(self.get_parameter("preload_target_source").value).strip().lower()
        self.preload_fixed_N = float(self.get_parameter("preload_fixed_N").value)
        self.preload_target_scale = float(self.get_parameter("preload_target_scale").value)
        self.preload_min_N = float(self.get_parameter("preload_min_N").value)
        self.preload_timeout_sec = float(self.get_parameter("preload_timeout_sec").value)
        self.preload_ok_count = int(self.get_parameter("preload_ok_count").value)
        self.preload_kp_mm_per_N = float(self.get_parameter("preload_kp_mm_per_N").value)
        self.preload_dz_max_mm = float(self.get_parameter("preload_dz_max_mm").value)
        self.preload_tol_N = float(self.get_parameter("preload_tol_N").value)
        self.press_force_cmd_mode = str(self.get_parameter("press_force_cmd_mode").value).strip().lower()
        self.press_hold_xy = bool(self.get_parameter("press_hold_xy").value)
        self.press_hold_rpy = bool(self.get_parameter("press_hold_rpy").value)

        self.release_assist_enable = bool(self.get_parameter("release_assist_enable").value)
        self.release_ramp_sec = float(self.get_parameter("release_ramp_sec").value)

        self.force_indices = tuple(int(x) for x in self.get_parameter("force_indices").value)
        self.first_cmd_fz = float(self.get_parameter("first_cmd_fz").value)
        self.action_type = str(self.get_parameter("action_type").value).strip().lower()

        self.normalize_qpos_enabled = bool(self.get_parameter("normalize_qpos").value)
        self.denorm_action_enabled = bool(self.get_parameter("denorm_action").value)

        self.resize_hw = int(self.get_parameter("resize_hw").value)
        self.debug_every_n = max(1, int(self.get_parameter("debug_every_n").value))

        self.fz_hard_limit = float(self.get_parameter("fz_hard_limit").value)

        # NEW: stall/fz kick
        self.stall_sec = float(self.get_parameter("stall_sec").value)
        self.stall_pos_eps_mm = float(self.get_parameter("stall_pos_eps_mm").value)
        self.stall_ang_eps_rad = float(self.get_parameter("stall_ang_eps_rad").value)
        self.stall_min_after_start_sec = float(self.get_parameter("stall_min_after_start_sec").value)
        self.fz_kick_N = float(self.get_parameter("fz_kick_N").value)
        self.fz_kick_dur_sec = float(self.get_parameter("fz_kick_dur_sec").value)
        self.fz_kick_cooldown_sec = float(self.get_parameter("fz_kick_cooldown_sec").value)
        self.fz_kick_enable_in_track = bool(self.get_parameter("fz_kick_enable_in_track").value)

        # device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"[INFO] Using device: {self.device}")

        # validate paths
        if not self.ckpt_dir or not os.path.isdir(self.ckpt_dir):
            raise RuntimeError(f"ckpt_dir invalid: {self.ckpt_dir}")
        if not self.act_root or not os.path.isdir(self.act_root):
            raise RuntimeError(f"act_root invalid: {self.act_root}")

        # stats
        self.stats = _load_dataset_stats(self.ckpt_dir)
        if self.stats is None:
            self.get_logger().warn("[STATS] dataset_stats.pkl missing/invalid -> disable normalize/denorm.")
            self.normalize_qpos_enabled = False
            self.denorm_action_enabled = False
        else:
            self.get_logger().info(f"[STATS] Loaded dataset_stats.pkl from {self.ckpt_dir}")

        # policy
        self.policy = self._load_policy_and_ckpt_from_act_root()

        # -----------------------------
        # State buffers
        # -----------------------------
        self._lock = threading.Lock()
        self._pose6: Optional[np.ndarray] = None
        self._force: Optional[np.ndarray] = None
        self._img_top: Optional[np.ndarray] = None
        self._img_ee: Optional[np.ndarray] = None

        # baseline state
        self._sent_first_cmd = False
        self.prev_cmd: Optional[np.ndarray] = None  # (9,)
        self._t_start = _monotonic()
        self._t_first_pub = None

        # contact hysteresis state
        self._contact = False
        self._last_contact = False

        # stage (start APPROACH)
        self.stage = Stage.APPROACH

        # anchor offset
        self._anchor_ready = False
        self._anchor_offset6 = np.zeros(6, dtype=np.float32)

        # plan buffer
        self.plans: Deque[Plan] = deque(maxlen=max(1, self.max_plans))

        # touch baseline + counter
        self._fz_base = 0.0
        self._fz_base_init = False
        self._touch_ok = 0

        # preload bookkeeping
        self._preload_t0 = 0.0
        self._preload_ok = 0
        self._preload_hold_pose6 = None  # (6,)
        self._preload_target_N = max(self.preload_min_N, 10.0)

        # release bookkeeping
        self._release_t0 = 0.0
        self._release_start_fz_cmd = 0.0

        # -----------------------------
        # NEW: Stall / FZ kick state
        # -----------------------------
        self._stall_last_pose6: Optional[np.ndarray] = None
        self._stall_last_move_t: float = _monotonic()
        self._fz_kick_active: bool = False
        self._fz_kick_t0: float = 0.0
        self._fz_kick_last_end_t: float = -1e9

        # -----------------------------
        # ROS I/O
        # -----------------------------
        img_rel = _reliability_from_str(self.image_qos_str)
        img_qos = _qos(depth=1, reliability=img_rel)
        vec_qos = _qos(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Float64MultiArray, self.pose_topic, self._on_pose, vec_qos)
        self.create_subscription(Float64MultiArray, self.force_topic, self._on_force, vec_qos)
        self.create_subscription(Image, self.top_img_topic, self._on_top_img, img_qos)
        self.create_subscription(Image, self.ee_img_topic,  self._on_ee_img,  img_qos)

        self.pub_cmd = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # timers
        self.dt_control = 1.0 / max(1e-6, self.control_hz)
        self.dt_infer = 1.0 / max(1e-6, self.infer_hz)

        self.timer_control = self.create_timer(self.dt_control, self._on_control_timer)
        self.timer_infer   = self.create_timer(self.dt_infer,   self._on_infer_timer)

        self.get_logger().info(
            "[INFO] ✅ Ready.\n"
            f"  stage_start={self.stage.name}\n"
            f"  pose_topic={self.pose_topic}\n"
            f"  force_topic={self.force_topic}\n"
            f"  top_img_topic={self.top_img_topic}\n"
            f"  ee_img_topic={self.ee_img_topic}\n"
            f"  cmd_topic={self.cmd_topic}\n"
            f"  image_qos={self.image_qos_str}\n"
            f"  control_hz={self.control_hz} infer_hz={self.infer_hz}\n"
            f"  tau_sec={self.tau_sec} startup_ramp_sec={self.startup_ramp_sec}\n"
            f"  step_caps(pos_mm={self.step_cap_pos_mm}, ang_rad={self.step_cap_ang_rad}, fz={self.step_cap_fz})\n"
            f"  temporal_agg={int(self.use_temporal_agg)} mode={self.temporal_agg_mode} tau_steps={self.temporal_agg_tau_steps} max_plans={self.max_plans}\n"
            f"  contact_gate(on={self.contact_on_thr}, off={self.contact_off_thr}) clear_on_change={int(self.clear_plans_on_contact_change)}\n"
            f"  touch(delta={int(self.touch_use_delta)}, thr={self.touch_fz_thr}, ok={self.touch_ok_count}, min_after={self.touch_min_after_start_sec}s, base_tau={self.touch_baseline_tau_sec}s)\n"
            f"  PRELOAD(src={self.preload_target_source}, min={self.preload_min_N}N, scale={self.preload_target_scale}, tol={self.preload_tol_N}N, ok={self.preload_ok_count}, timeout={self.preload_timeout_sec}s, kp={self.preload_kp_mm_per_N}mm/N, dz_max={self.preload_dz_max_mm}mm, fcmd={self.press_force_cmd_mode})\n"
            f"  STALL(sec={self.stall_sec}, pos_eps_mm={self.stall_pos_eps_mm}, ang_eps_rad={self.stall_ang_eps_rad}, min_after={self.stall_min_after_start_sec}s, kick={self.fz_kick_N}N/{self.fz_kick_dur_sec}s, cooldown={self.fz_kick_cooldown_sec}s, enable_in_track={int(self.fz_kick_enable_in_track)})\n"
            f"  RELEASE(enable={int(self.release_assist_enable)}, ramp_sec={self.release_ramp_sec})\n"
        )

    # ------------------------------------------------------------
    # Load policy (act_root/policy.py) + ckpt (policy_best.ckpt)
    # ------------------------------------------------------------
    def _load_policy_and_ckpt_from_act_root(self):
        if self.act_root not in sys.path:
            sys.path.insert(0, self.act_root)

        try:
            from policy import ACTPolicy
        except Exception as e:
            raise RuntimeError(f"Failed to import ACTPolicy from {self.act_root}/policy.py : {e}")

        args_override = {
            "kl_weight": float(self.get_parameter("kl_weight").value),
            "num_queries": int(self.chunk_size),

            "lr": 1e-4,
            "hidden_dim": int(self.get_parameter("hidden_dim").value),
            "dim_feedforward": int(self.get_parameter("dim_feedforward").value),
            "lr_backbone": float(self.get_parameter("lr_backbone").value),
            "backbone": str(self.get_parameter("backbone").value),
            "enc_layers": int(self.get_parameter("enc_layers").value),
            "dec_layers": int(self.get_parameter("dec_layers").value),
            "nheads": int(self.get_parameter("nheads").value),

            "camera_names": ["cam_top", "cam_ee"],
            "state_dim": 9,
            "action_dim": 9,

            "image_resize_hw": int(self.get_parameter("image_resize_hw").value),
            "image_pool_hw": int(self.get_parameter("image_pool_hw").value),
            "pretrained_backbone": bool(self.get_parameter("pretrained_backbone").value),
        }

        self.get_logger().info("[INFO] Loading policy (training-time policy.py)...")
        policy = ACTPolicy(args_override).to(self.device)
        policy.eval()

        ckpt_path = os.path.join(self.ckpt_dir, "policy_best.ckpt")
        if not os.path.exists(ckpt_path):
            raise RuntimeError(f"policy_best.ckpt not found: {ckpt_path}")

        ckpt_obj = torch.load(ckpt_path, map_location=self.device)
        state_dict = ckpt_obj["state_dict"] if isinstance(ckpt_obj, dict) and "state_dict" in ckpt_obj else ckpt_obj

        model = policy.model if hasattr(policy, "model") else policy
        missing, unexpected = _try_load_state_dict_compat(model, state_dict)

        self.get_logger().info(f"[INFO] Loaded ckpt into policy. missing={len(missing)}, unexpected={len(unexpected)}")
        self.get_logger().info("[INFO] camera_names = ['cam_top','cam_ee']")
        return policy

    # ------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------
    def _on_pose(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32).reshape(-1)
        if arr.size >= 6:
            with self._lock:
                self._pose6 = arr[:6].copy()

    def _on_force(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32).reshape(-1)
        with self._lock:
            self._force = arr.copy()

    def _on_top_img(self, msg: Image):
        try:
            rgb = _img_to_rgb_numpy(msg)
            with self._lock:
                self._img_top = rgb
        except Exception as e:
            self.get_logger().error(f"[TOP IMG] decode failed: {e}")

    def _on_ee_img(self, msg: Image):
        try:
            rgb = _img_to_rgb_numpy(msg)
            with self._lock:
                self._img_ee = rgb
        except Exception as e:
            self.get_logger().error(f"[EE IMG] decode failed: {e}")

    # ------------------------------------------------------------
    # Contact update (hysteresis)
    # ------------------------------------------------------------
    def _update_contact(self, meas_fz: float) -> bool:
        prev = self._contact
        if (not self._contact) and (meas_fz >= self.contact_on_thr):
            self._contact = True
        elif self._contact and (meas_fz <= self.contact_off_thr):
            self._contact = False
        return (prev != self._contact)

    # ------------------------------------------------------------
    # Compute preload target
    # ------------------------------------------------------------
    def _compute_preload_target(self) -> float:
        tgt = self.preload_fixed_N
        if (self.preload_target_source == "stats_mean") and (self.stats is not None):
            mean_fz = float(self.stats.qpos_mean[8])
            tgt = abs(mean_fz) * float(self.preload_target_scale)
        tgt = max(float(self.preload_min_N), float(tgt))
        return float(tgt)

    # ------------------------------------------------------------
    # Stage transitions
    # ------------------------------------------------------------
    def _enter_preload(self, pose6_now: np.ndarray):
        self.stage = Stage.PRELOAD
        self._preload_t0 = _monotonic()
        self._preload_ok = 0
        self._preload_hold_pose6 = pose6_now.astype(np.float32).copy()
        self._preload_target_N = self._compute_preload_target()

        self.plans.clear()
        self._anchor_ready = False

        self.get_logger().warn(f"[STAGE] -> PRELOAD(PRESS) target={self._preload_target_N:.2f}N (touch confirmed)")

    def _enter_track(self):
        self.stage = Stage.TRACK
        self.plans.clear()
        self._anchor_ready = False
        self.get_logger().warn("[STAGE] -> TRACK (resume inference)")

    def _enter_release(self, fz_cmd_start: float):
        self.stage = Stage.RELEASE
        self._release_t0 = _monotonic()
        self._release_start_fz_cmd = float(max(0.0, fz_cmd_start))
        self.get_logger().warn(f"[STAGE] -> RELEASE (fz ramp {self._release_start_fz_cmd:.3f} -> 0 in {self.release_ramp_sec:.2f}s)")

    # ------------------------------------------------------------
    # Infer timer: generate plan and push (disabled during PRELOAD)
    # ------------------------------------------------------------
    def _on_infer_timer(self):
        if self.stage == Stage.PRELOAD:
            return

        with self._lock:
            pose6 = None if self._pose6 is None else self._pose6.copy()
            force = None if self._force is None else self._force.copy()
            top = None if self._img_top is None else self._img_top.copy()
            ee  = None if self._img_ee is None else self._img_ee.copy()

        if pose6 is None or force is None or top is None or ee is None:
            return
        if force.size < 3:
            return

        idx = list(self.force_indices)
        f3 = np.zeros(3, dtype=np.float32)
        for i, k in enumerate(idx):
            if k < force.size:
                f3[i] = float(force[k])

        q_np = np.concatenate([pose6[:6], f3], axis=0).astype(np.float32)  # (9,)
        q_t = torch.from_numpy(q_np).unsqueeze(0).to(self.device, dtype=torch.float32)

        if self.normalize_qpos_enabled and self.stats is not None:
            q_t = _normalize_qpos(q_t, self.stats)

        try:
            img_t = _to_tensor_image_stack(top, ee, device=self.device, resize_hw=self.resize_hw)
        except Exception as e:
            self.get_logger().error(f"[INFER] image stack failed: {e}")
            return

        try:
            with torch.inference_mode():
                a_hat = self.policy(q_t, img_t)
            seq = _fix_a_hat_shape(a_hat, self.chunk_size)  # (T,9)

            if self.denorm_action_enabled and self.stats is not None:
                seq = _denorm_action_seq(seq, self.stats)

            seq_den = seq.detach().cpu().numpy().astype(np.float32)
            seq_den[:, 8] = np.clip(seq_den[:, 8], -self.fz_hard_limit, self.fz_hard_limit)

        except Exception as e:
            self.get_logger().error(f"[INFER] policy forward failed: {e}")
            return

        self.plans.append(Plan(t0=_monotonic(), seq_den=seq_den))

    # ------------------------------------------------------------
    # Temporal aggregation (denorm)
    # ------------------------------------------------------------
    def _temporal_agg_cmd(self, now_t: float) -> Optional[np.ndarray]:
        if not self.plans:
            return None

        vals: List[np.ndarray] = []
        wts: List[float] = []

        for p in list(self.plans):
            age_steps = int((now_t - p.t0) * self.control_hz)
            k = age_steps + int(self.pred_step_offset)
            if 0 <= k < p.seq_den.shape[0]:
                v = p.seq_den[k]
                if self.use_temporal_agg and self.temporal_agg_mode == "exp":
                    w = _exp_decay_weight(age_steps, self.temporal_agg_tau_steps)
                else:
                    w = 1.0
                vals.append(v.astype(np.float32))
                wts.append(float(w))

        if len(vals) == 0:
            p = self.plans[-1]
            age_steps = int((now_t - p.t0) * self.control_hz)
            k = int(np.clip(age_steps + int(self.pred_step_offset), 0, p.seq_den.shape[0] - 1))
            return p.seq_den[k].astype(np.float32)

        W = float(np.sum(wts))
        if W <= 1e-9:
            return vals[-1].astype(np.float32)

        acc = np.zeros(9, dtype=np.float32)
        for v, w in zip(vals, wts):
            acc += (w / W) * v
        return acc.astype(np.float32)

    # ------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------
    def _publish_cmd(self, cmd9: np.ndarray):
        m = Float64MultiArray()
        m.data = [float(x) for x in cmd9.reshape(-1).tolist()]
        self.pub_cmd.publish(m)

    def _startup_ramp(self) -> float:
        if self.startup_ramp_sec <= 1e-6:
            return 1.0
        t = _monotonic() - self._t_start
        return float(np.clip(t / self.startup_ramp_sec, 0.0, 1.0))

    # ------------------------------------------------------------
    # PRELOAD control (Z-servo + optional cmd_fz)
    # ------------------------------------------------------------
    def _preload_control_step(self, pose6_now: np.ndarray, meas_fz: float) -> np.ndarray:
        hold = self._preload_hold_pose6 if self._preload_hold_pose6 is not None else pose6_now.astype(np.float32)

        cmd = np.zeros(9, dtype=np.float32)
        cmd[0:6] = pose6_now.astype(np.float32)
        cmd[6] = 0.0
        cmd[7] = 0.0
        cmd[8] = 0.0

        if self.press_hold_xy:
            cmd[0] = hold[0]
            cmd[1] = hold[1]
        if self.press_hold_rpy:
            cmd[3] = hold[3]
            cmd[4] = hold[4]
            cmd[5] = hold[5]

        target = float(self._preload_target_N)
        err = float(target - meas_fz)

        dz = self.preload_kp_mm_per_N * max(0.0, err)
        dz = float(np.clip(dz, 0.0, self.preload_dz_max_mm))
        cmd[2] = float(cmd[2] - dz)

        mode = self.press_force_cmd_mode
        if mode == "zero":
            cmd[8] = 0.0
        elif mode == "target":
            cmd[8] = float(target)
        else:
            prev_fz = float(self.prev_cmd[8]) if (self.prev_cmd is not None) else 0.0
            cmd[8] = float(prev_fz)

        cmd[8] = float(np.clip(cmd[8], 0.0, self.fz_hard_limit))
        return cmd

    # ------------------------------------------------------------
    # RELEASE force shaping
    # ------------------------------------------------------------
    def _release_force(self, cmd_target: np.ndarray) -> np.ndarray:
        cmd = cmd_target.astype(np.float32).copy()
        t = _monotonic() - self._release_t0
        if self.release_ramp_sec <= 1e-6:
            s = 1.0
        else:
            s = float(np.clip(t / self.release_ramp_sec, 0.0, 1.0))
        fz = (1.0 - s) * float(self._release_start_fz_cmd)
        cmd[6] = 0.0
        cmd[7] = 0.0
        cmd[8] = float(max(0.0, fz))
        return cmd

    # ------------------------------------------------------------
    # Control timer
    # ------------------------------------------------------------
    def _on_control_timer(self):
        now_t = _monotonic()

        with self._lock:
            pose6 = None if self._pose6 is None else self._pose6.copy()
            force = None if self._force is None else self._force.copy()

        if pose6 is None:
            return

        meas_fz = 0.0
        if force is not None and force.size >= 3:
            meas_fz = float(force[2])  # normal force

        # (1) FIRST publish = current pose hold
        if not self._sent_first_cmd:
            cmd0 = np.zeros(9, dtype=np.float32)
            cmd0[0:6] = pose6.astype(np.float32)
            cmd0[6] = 0.0
            cmd0[7] = 0.0
            cmd0[8] = float(self.first_cmd_fz)

            self.prev_cmd = cmd0.copy()
            self._sent_first_cmd = True
            self._t_first_pub = now_t
            self._t_start = now_t

            self.stage = Stage.APPROACH

            self._fz_base = max(0.0, meas_fz)
            self._fz_base_init = True
            self._touch_ok = 0

            # NEW: init stall trackers
            self._stall_last_pose6 = pose6.astype(np.float32).copy()
            self._stall_last_move_t = now_t
            self._fz_kick_active = False
            self._fz_kick_last_end_t = -1e9

            self._publish_cmd(cmd0)
            self.get_logger().info("[START] First publish = current pose. stage=APPROACH")
            return

        if self.prev_cmd is None:
            return

        # contact hysteresis (logging / optional release)
        changed = self._update_contact(meas_fz)
        if changed:
            if self.clear_plans_on_contact_change:
                self.plans.clear()
                self._anchor_ready = False
            self.get_logger().warn(f"[CONTACT] changed -> {int(self._contact)} | meas_fz={meas_fz:.3f} | stage={self.stage.name}")

            if self.release_assist_enable:
                if (not self._contact) and self._last_contact and (self.stage == Stage.TRACK):
                    fz_start = float(self.prev_cmd[8]) if self.prev_cmd is not None else 0.0
                    self._enter_release(fz_start)

        self._last_contact = self._contact

        # -----------------------------
        # Stall detect -> FZ KICK (NEW)
        # -----------------------------
        if self._t_first_pub is not None:
            elapsed_since_start = (now_t - self._t_first_pub)

            # update last_move_t by pose change
            if self._stall_last_pose6 is None:
                self._stall_last_pose6 = pose6.astype(np.float32).copy()
                self._stall_last_move_t = now_t
            else:
                dp = float(np.linalg.norm(pose6[:3] - self._stall_last_pose6[:3]))
                da = float(np.linalg.norm(pose6[3:6] - self._stall_last_pose6[3:6]))
                if (dp >= self.stall_pos_eps_mm) or (da >= self.stall_ang_eps_rad):
                    self._stall_last_pose6 = pose6.astype(np.float32).copy()
                    self._stall_last_move_t = now_t

            kick_stage_ok = (self.stage == Stage.APPROACH) or (self.fz_kick_enable_in_track and self.stage == Stage.TRACK)

            if (kick_stage_ok and (not self._contact) and (not self._fz_kick_active)
                and (elapsed_since_start >= self.stall_min_after_start_sec)
                and ((now_t - self._stall_last_move_t) >= self.stall_sec)
                and ((now_t - self._fz_kick_last_end_t) >= self.fz_kick_cooldown_sec)):

                stall_t = now_t - self._stall_last_move_t
                self._fz_kick_active = True
                self._fz_kick_t0 = now_t
                self.get_logger().warn(
                    f"[STALL] no motion for {stall_t:.2f}s -> FZ KICK start "
                    f"(fz={self.fz_kick_N:.2f}N, dur={self.fz_kick_dur_sec:.2f}s)"
                )

            if self._fz_kick_active and ((now_t - self._fz_kick_t0) >= self.fz_kick_dur_sec):
                self._fz_kick_active = False
                self._fz_kick_last_end_t = now_t

                # trigger fresh plan usage
                self.plans.clear()
                self._anchor_ready = False

                self.get_logger().warn("[STALL] FZ KICK end -> replan requested")

        # -----------------------------
        # Touch detector
        # -----------------------------
        # baseline update only in APPROACH and NOT during kick
        if self.stage == Stage.APPROACH and (not self._fz_kick_active):
            if not self._fz_base_init:
                self._fz_base = max(0.0, meas_fz)
                self._fz_base_init = True
            else:
                beta_base = _beta_from_tau(self.dt_control, self.touch_baseline_tau_sec)
                self._fz_base = float((1.0 - beta_base) * self._fz_base + beta_base * max(0.0, meas_fz))

            if self.touch_use_delta:
                touch_sig = max(0.0, meas_fz - self._fz_base)
            else:
                touch_sig = max(0.0, meas_fz)

            elapsed = (now_t - self._t_first_pub) if (self._t_first_pub is not None) else 0.0
            allow_touch = (elapsed >= self.touch_min_after_start_sec)

            if allow_touch and (touch_sig >= self.touch_fz_thr):
                self._touch_ok += 1
            else:
                self._touch_ok = 0

            if self._touch_ok >= self.touch_ok_count:
                self._touch_ok = 0
                self._enter_preload(pose6.astype(np.float32))

        # -----------------------------
        # Build cmd_target by stage
        # -----------------------------
        cmd_target = None

        if self.stage == Stage.PRELOAD:
            cmd_target = self._preload_control_step(pose6.astype(np.float32), meas_fz)

            if abs(meas_fz - self._preload_target_N) <= self.preload_tol_N:
                self._preload_ok += 1
            else:
                self._preload_ok = 0

            if self._preload_ok >= self.preload_ok_count:
                self.get_logger().warn(f"[PRELOAD] OK (meas_fz~{self._preload_target_N:.2f}N) for {self.preload_ok_count} ticks -> TRACK")
                self._enter_track()
            else:
                if (_monotonic() - self._preload_t0) >= self.preload_timeout_sec:
                    self.get_logger().warn(f"[PRELOAD] TIMEOUT {self.preload_timeout_sec:.2f}s (meas_fz={meas_fz:.2f}) -> TRACK anyway")
                    self._enter_track()

        else:
            cmd_pred = self._temporal_agg_cmd(now_t)

            if cmd_pred is None:
                self._publish_cmd(self.prev_cmd)
                return

            cmd_target = cmd_pred.astype(np.float32).copy()

            if self.action_type == "delta":
                cmd_target = (self.prev_cmd + cmd_target).astype(np.float32)

            if not self._anchor_ready:
                self._anchor_offset6 = (pose6.astype(np.float32) - cmd_target[0:6]).astype(np.float32)
                self._anchor_ready = True
                self.get_logger().info("[ANCHOR] initialized")

            cmd_target[0:6] = (cmd_target[0:6] + self._anchor_offset6).astype(np.float32)

            if self.stage == Stage.APPROACH:
                cmd_target[6] = 0.0
                cmd_target[7] = 0.0
                cmd_target[8] = 0.0

            if self.stage == Stage.RELEASE:
                cmd_target = self._release_force(cmd_target)
                if (_monotonic() - self._release_t0) >= max(1e-6, self.release_ramp_sec):
                    self.stage = Stage.APPROACH
                    self.plans.clear()
                    self._anchor_ready = False
                    self._touch_ok = 0
                    self.get_logger().warn("[STAGE] RELEASE done -> APPROACH")

        # -----------------------------
        # Apply FZ KICK (NEW): keep pose from model, inject cmd_fz
        # -----------------------------
        if self._fz_kick_active:
            cmd_target[8] = float(max(cmd_target[8], self.fz_kick_N))

        # safety clamp
        cmd_target[8] = float(np.clip(cmd_target[8], 0.0, self.fz_hard_limit))

        # -----------------------------
        # QP-safe slow-follow: EMA + step caps + startup ramp
        # -----------------------------
        dt = self.dt_control
        beta = _beta_from_tau(dt, self.tau_sec)

        ramp = self._startup_ramp()
        cap_pos = max(1e-9, self.step_cap_pos_mm * ramp)
        cap_ang = max(1e-12, self.step_cap_ang_rad * ramp)
        cap_fz  = max(1e-9, self.step_cap_fz * ramp)

        d = (cmd_target - self.prev_cmd).astype(np.float32)
        d = (beta * d).astype(np.float32)

        for i in range(3):
            di = float(d[i])
            if abs(di) > cap_pos:
                d[i] = float(np.sign(di) * cap_pos)
        for i in range(3, 6):
            di = float(d[i])
            if abs(di) > cap_ang:
                d[i] = float(np.sign(di) * cap_ang)
        for i in (6, 7):
            di = float(d[i])
            if abs(di) > cap_pos:
                d[i] = float(np.sign(di) * cap_pos)
        di = float(d[8])
        if abs(di) > cap_fz:
            d[8] = float(np.sign(di) * cap_fz)

        cmd_next = (self.prev_cmd + d).astype(np.float32)

        for i in range(9):
            a0 = float(self.prev_cmd[i])
            a1 = float(cmd_next[i])
            tg = float(cmd_target[i])
            if (a0 - tg) * (a1 - tg) < 0.0:
                cmd_next[i] = tg

        self._publish_cmd(cmd_next)
        self.prev_cmd = cmd_next

        # debug
        if (int(now_t * self.control_hz) % self.debug_every_n) == 0:
            base = self._fz_base if self._fz_base_init else 0.0
            touch_sig = max(0.0, meas_fz - base) if self.touch_use_delta else max(0.0, meas_fz)
            stall_t = now_t - self._stall_last_move_t if self._stall_last_move_t is not None else 0.0
            self.get_logger().info(
                f"[CTRL] stage={self.stage.name} contact={int(self._contact)} meas_fz={meas_fz:.3f} "
                f"fz_base={base:.3f} touch_sig={touch_sig:.3f} touch_ok={self._touch_ok} | "
                f"stall={stall_t:.2f}s kick={int(self._fz_kick_active)} | "
                f"beta={beta:.4f} ramp={ramp:.3f} cap(pos={cap_pos:.4f}, ang={cap_ang:.6f}, fz={cap_fz:.4f}) | "
                f"cmd_xyz=[{cmd_next[0]:.3f},{cmd_next[1]:.3f},{cmd_next[2]:.3f}] cmd_fz={cmd_next[8]:.3f}"
            )


# ============================================================
# main
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NodeActCmdMotionInfer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
