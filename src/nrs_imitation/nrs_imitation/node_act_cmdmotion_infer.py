#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
node_act_cmdmotion_infer.py (REBUILT: first-hold + slow-follow + exp temporal-agg)

Key guarantees (QP-safe):
1) FIRST publish MUST equal current pose (/ur10skku/currentP) for 6D pose part.
   -> publish one "hold" command immediately once pose arrives.
2) After that, command follows inference output SLOWLY:
   - Temporal aggregation with exponential decay across multiple plans
   - Anchor offset to align predicted pose to current pose at first plan usage
   - EMA smoothing using tau_sec + per-tick step caps (pos/ang/fz)

ACT input matches training / node_check_inference:
- qpos = [pose6 + force3] -> (1,9)
- image = stack([top, ee]) -> (1,2,3,H,W) float in [0,1]
- camera_names fixed to ["cam_top","cam_ee"]

Example:
ros2 run nrs_imitation node_act_cmdmotion_infer --ros-args \
  -p ckpt_dir:=/home/eunseop/nrs_lab2/checkpoints/ur10e_swing/20260208_1536 \
  -p act_root:=/home/eunseop/nrs_lab2/nrs_act \
  -p image_qos:=reliable \
  -p cmd_topic:=/ur10skku/cmdMotion \
  -p control_hz:=30.0 \
  -p infer_hz:=7.5 \
  -p tau_sec:=0.35 \
  -p max_plans:=6 \
  -p contact_on_thr:=5.0 \
  -p contact_off_thr:=2.0 \
  -p clear_plans_on_contact_change:=True
"""

import os
import sys
import time
import math
import pickle
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple, Deque, List

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

    # fallback: try 3-channel
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

    # (H,W,3)->(3,H,W)
    top = np.transpose(top_rgb, (2, 0, 1))
    ee  = np.transpose(ee_rgb,  (2, 0, 1))

    img = np.stack([top, ee], axis=0).astype(np.float32) / 255.0  # (2,3,H,W)
    img_t = torch.from_numpy(img).unsqueeze(0).to(device=device, dtype=torch.float32)  # (1,2,3,H,W)
    return img_t


# ============================================================
# Helpers (Stats: supports both 9D and groupwise keys)
# ============================================================

@dataclass
class StatsPack:
    # All are np.float32 shape (9,)
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

    # Preferred: 9D vectors
    if all(k in st for k in ["qpos_mean", "qpos_std", "action_mean", "action_std"]):
        qm = np.asarray(st["qpos_mean"], dtype=np.float32).reshape(9)
        qs = _sanitize_std(np.asarray(st["qpos_std"], dtype=np.float32).reshape(9))
        am = np.asarray(st["action_mean"], dtype=np.float32).reshape(9)
        astd = _sanitize_std(np.asarray(st["action_std"], dtype=np.float32).reshape(9))
        return StatsPack(qm, qs, am, astd)

    # Fallback: groupwise keys (xyz/rpy/force each size 1 or 3)
    keys_q = ["qpos_mean_xyz","qpos_std_xyz","qpos_mean_rpy","qpos_std_rpy","qpos_mean_force","qpos_std_force"]
    keys_a = ["action_mean_xyz","action_std_xyz","action_mean_rpy","action_std_rpy","action_mean_force","action_std_force"]
    if all(k in st for k in keys_q + keys_a):
        def _to3(v):
            a = np.asarray(v, dtype=np.float32).reshape(-1)
            if a.size == 1:
                return np.array([a.item(), a.item(), a.item()], dtype=np.float32)
            if a.size == 3:
                return a.astype(np.float32)
            raise ValueError("size must be 1 or 3")

        q_mean = np.concatenate([_to3(st["qpos_mean_xyz"]), _to3(st["qpos_mean_rpy"]), _to3(st["qpos_mean_force"])], axis=0)
        q_std  = np.concatenate([_sanitize_std(_to3(st["qpos_std_xyz"])),
                                 _sanitize_std(_to3(st["qpos_std_rpy"])),
                                 _sanitize_std(_to3(st["qpos_std_force"]))], axis=0)
        a_mean = np.concatenate([_to3(st["action_mean_xyz"]), _to3(st["action_mean_rpy"]), _to3(st["action_mean_force"])], axis=0)
        a_std  = np.concatenate([_sanitize_std(_to3(st["action_std_xyz"])),
                                 _sanitize_std(_to3(st["action_std_rpy"])),
                                 _sanitize_std(_to3(st["action_std_force"]))], axis=0)
        return StatsPack(q_mean.reshape(9), q_std.reshape(9), a_mean.reshape(9), a_std.reshape(9))

    return None

def _normalize_qpos(q: torch.Tensor, stats: StatsPack) -> torch.Tensor:
    mu = torch.tensor(stats.qpos_mean, dtype=torch.float32, device=q.device).view(1, 9)
    sd = torch.tensor(stats.qpos_std,  dtype=torch.float32, device=q.device).view(1, 9)
    return (q - mu) / sd

def _denorm_action_seq(seq: torch.Tensor, stats: StatsPack) -> torch.Tensor:
    # seq: (T,9) or (1,T,9)
    mu = torch.tensor(stats.act_mean, dtype=torch.float32, device=seq.device)
    sd = torch.tensor(stats.act_std,  dtype=torch.float32, device=seq.device)
    if seq.dim() == 2:
        return seq * sd.view(1,9) + mu.view(1,9)
    if seq.dim() == 3:
        return seq * sd.view(1,1,9) + mu.view(1,1,9)
    raise RuntimeError(f"unexpected seq dim: {seq.shape}")


# ============================================================
# Helpers (Policy output shape)
# ============================================================

def _fix_a_hat_shape(a_hat: torch.Tensor, chunk_size: int) -> torch.Tensor:
    """
    Standardize output to (T,9) with T=chunk_size.
    Handles common variants:
      - (1,T,9)
      - (T,1,9)
      - (T,9)
    """
    if a_hat.dim() == 2:
        # (T,9)
        return a_hat
    if a_hat.dim() != 3:
        raise RuntimeError(f"Unexpected a_hat dim: {a_hat.shape}")

    B0, B1, B2 = a_hat.shape
    if B2 != 9:
        raise RuntimeError(f"Unexpected last dim (need 9): {a_hat.shape}")

    # (1,T,9)
    if B0 == 1 and B1 == chunk_size:
        return a_hat[0]
    # (T,1,9)
    if B0 == chunk_size and B1 == 1:
        return a_hat[:, 0, :]
    # (B,T,9) pick first batch
    if B1 == chunk_size:
        return a_hat[0]
    raise RuntimeError(f"Cannot interpret a_hat shape={a_hat.shape} with chunk_size={chunk_size}")


# ============================================================
# Plan buffer entry
# ============================================================

@dataclass
class Plan:
    t0: float            # monotonic start time (when plan was generated)
    seq_den: np.ndarray  # (T,9) in denormalized space


# ============================================================
# ROS2 Node
# ============================================================

class NodeActCmdMotionInfer(Node):
    def __init__(self):
        super().__init__("node_act_cmdmotion_infer")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("act_root", "")
        self.declare_parameter("chunk_size", 100)

        self.declare_parameter("pose_topic", "/ur10skku/currentP")
        self.declare_parameter("force_topic", "/ur10skku/currentF")
        self.declare_parameter("top_img_topic", "/realsense/top/color/image_raw")
        self.declare_parameter("ee_img_topic",  "/realsense/ee/color/image_raw")

        self.declare_parameter("cmd_topic", "/ur10skku/cmdMotion")

        self.declare_parameter("image_qos", "best_effort")  # reliable | best_effort
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("infer_hz", 7.5)

        # 핵심: "천천히" 추종
        self.declare_parameter("tau_sec", 0.35)  # EMA time-constant for cmd following
        self.declare_parameter("startup_ramp_sec", 1.5)  # step caps ramp-up duration

        # per-tick step caps (QP-safe)
        self.declare_parameter("pos_step_cap_mm", 0.30)
        self.declare_parameter("ang_step_cap_rad", 0.0004)
        self.declare_parameter("fz_step_cap", 0.20)

        # temporal aggregation
        self.declare_parameter("use_temporal_agg", True)
        self.declare_parameter("temporal_agg_mode", "exp")  # exp | mean
        self.declare_parameter("temporal_agg_tau_steps", 20.0)  # exp weight tau in "control steps"
        self.declare_parameter("pred_step_offset", 1)
        self.declare_parameter("max_plans", 6)

        # contact gating
        self.declare_parameter("contact_on_thr", 5.0)
        self.declare_parameter("contact_off_thr", 2.0)
        self.declare_parameter("clear_plans_on_contact_change", True)

        # I/O shaping
        self.declare_parameter("force_indices", [0, 1, 2])  # use Fx,Fy,Fz in obs qpos
        self.declare_parameter("first_cmd_fz", 0.0)         # first publish desired fz (0 recommended)
        self.declare_parameter("action_type", "absolute")   # absolute | delta

        # normalization
        self.declare_parameter("normalize_qpos", True)
        self.declare_parameter("denorm_action", True)

        # image optional resize
        self.declare_parameter("resize_hw", 0)  # 0: keep raw size, else resize to (resize_hw, resize_hw)

        # model hyperparams (must match training)
        self.declare_parameter("kl_weight", 10.0)
        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("nheads", 8)
        self.declare_parameter("enc_layers", 4)
        self.declare_parameter("dec_layers", 7)
        self.declare_parameter("backbone", "resnet18")
        self.declare_parameter("lr_backbone", 1e-5)
        self.declare_parameter("no_pretrained", False)
        self.declare_parameter("image_resize_hw", 256)
        self.declare_parameter("image_pool_hw", 4)

        # debug
        self.declare_parameter("debug_every_n", 30)

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

        self.pos_step_cap_mm = float(self.get_parameter("pos_step_cap_mm").value)
        self.ang_step_cap_rad = float(self.get_parameter("ang_step_cap_rad").value)
        self.fz_step_cap = float(self.get_parameter("fz_step_cap").value)

        self.use_temporal_agg = bool(self.get_parameter("use_temporal_agg").value)
        self.temporal_agg_mode = str(self.get_parameter("temporal_agg_mode").value).strip().lower()
        self.temporal_agg_tau_steps = float(self.get_parameter("temporal_agg_tau_steps").value)
        self.pred_step_offset = int(self.get_parameter("pred_step_offset").value)
        self.max_plans = int(self.get_parameter("max_plans").value)

        self.contact_on_thr = float(self.get_parameter("contact_on_thr").value)
        self.contact_off_thr = float(self.get_parameter("contact_off_thr").value)
        self.clear_plans_on_contact_change = bool(self.get_parameter("clear_plans_on_contact_change").value)

        self.force_indices = tuple(int(x) for x in self.get_parameter("force_indices").value)
        self.first_cmd_fz = float(self.get_parameter("first_cmd_fz").value)
        self.action_type = str(self.get_parameter("action_type").value).strip().lower()

        self.normalize_qpos_enabled = bool(self.get_parameter("normalize_qpos").value)
        self.denorm_action_enabled = bool(self.get_parameter("denorm_action").value)

        self.resize_hw = int(self.get_parameter("resize_hw").value)
        self.debug_every_n = max(1, int(self.get_parameter("debug_every_n").value))

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

        # policy (training-time policy.py)
        self.policy = self._load_policy_and_ckpt()

        # -----------------------------
        # State buffers
        # -----------------------------
        self._lock = threading.Lock()

        self._pose6: Optional[np.ndarray] = None
        self._force6: Optional[np.ndarray] = None
        self._img_top: Optional[np.ndarray] = None
        self._img_ee: Optional[np.ndarray] = None

        # command state
        self._sent_first_cmd = False
        self.prev_cmd: Optional[np.ndarray] = None  # (9,)
        self._t_start = _monotonic()

        # contact
        self._contact = False

        # anchor offset to prevent jumps
        self._anchor_ready = False
        self._anchor_offset6 = np.zeros(6, dtype=np.float32)

        # plan buffer (denormed sequences)
        self.plans: Deque[Plan] = deque(maxlen=max(1, self.max_plans))

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
            f"  pose_topic={self.pose_topic}\n"
            f"  force_topic={self.force_topic}\n"
            f"  top_img_topic={self.top_img_topic}\n"
            f"  ee_img_topic={self.ee_img_topic}\n"
            f"  cmd_topic={self.cmd_topic}\n"
            f"  image_qos={self.image_qos_str}\n"
            f"  control_hz={self.control_hz} infer_hz={self.infer_hz}\n"
            f"  tau_sec={self.tau_sec} startup_ramp_sec={self.startup_ramp_sec}\n"
            f"  step_caps(pos_mm={self.pos_step_cap_mm}, ang_rad={self.ang_step_cap_rad}, fz={self.fz_step_cap})\n"
            f"  temporal_agg={int(self.use_temporal_agg)} mode={self.temporal_agg_mode} tau_steps={self.temporal_agg_tau_steps} max_plans={self.max_plans}\n"
            f"  contact_gate(on={self.contact_on_thr}, off={self.contact_off_thr}) clear_on_change={int(self.clear_plans_on_contact_change)}\n"
        )

    # ------------------------------------------------------------
    # Load policy
    # ------------------------------------------------------------
    def _load_policy_and_ckpt(self):
        if self.act_root not in sys.path:
            sys.path.insert(0, self.act_root)

        try:
            from policy import ACTPolicy
        except Exception as e:
            raise RuntimeError(f"Failed to import ACTPolicy from {self.act_root}/policy.py : {e}")

        args_override = {
            "lr": 1e-4,
            "num_queries": int(self.chunk_size),
            "kl_weight": float(self.get_parameter("kl_weight").value),
            "hidden_dim": int(self.get_parameter("hidden_dim").value),
            "dim_feedforward": int(self.get_parameter("dim_feedforward").value),
            "lr_backbone": float(self.get_parameter("lr_backbone").value),
            "backbone": str(self.get_parameter("backbone").value),
            "enc_layers": int(self.get_parameter("enc_layers").value),
            "dec_layers": int(self.get_parameter("dec_layers").value),
            "nheads": int(self.get_parameter("nheads").value),

            # critical: fixed order
            "camera_names": ["cam_top", "cam_ee"],

            "state_dim": 9,
            "action_dim": 9,

            "image_resize_hw": int(self.get_parameter("image_resize_hw").value),
            "image_pool_hw": int(self.get_parameter("image_pool_hw").value),
            "pretrained_backbone": (not bool(self.get_parameter("no_pretrained").value)),
        }

        self.get_logger().info("[INFO] Loading policy (training-time policy.py)...")
        policy = ACTPolicy(args_override).to(self.device)
        policy.eval()

        ckpt_path = os.path.join(self.ckpt_dir, "policy_best.ckpt")
        if not os.path.exists(ckpt_path):
            raise RuntimeError(f"policy_best.ckpt not found: {ckpt_path}")

        ckpt = torch.load(ckpt_path, map_location=self.device)
        # support both raw state_dict and dict with 'state_dict'
        state_dict = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt
        missing, unexpected = policy.model.load_state_dict(state_dict, strict=False)
        self.get_logger().info(f"[INFO] Loaded ckpt. missing={len(missing)}, unexpected={len(unexpected)}")
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
        if arr.size >= 3:
            with self._lock:
                self._force6 = arr.copy()

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
    # Infer timer: generate a new plan and push to buffer
    # ------------------------------------------------------------
    def _on_infer_timer(self):
        with self._lock:
            pose6 = None if self._pose6 is None else self._pose6.copy()
            force6 = None if self._force6 is None else self._force6.copy()
            top = None if self._img_top is None else self._img_top.copy()
            ee  = None if self._img_ee is None else self._img_ee.copy()

        if pose6 is None or force6 is None or top is None or ee is None:
            return

        # build qpos (1,9)
        f3 = force6[list(self.force_indices)].astype(np.float32)
        q_np = np.concatenate([pose6[:6], f3], axis=0).astype(np.float32)  # (9,)
        q_t = torch.from_numpy(q_np).unsqueeze(0).to(self.device, dtype=torch.float32)
        if self.normalize_qpos_enabled and self.stats is not None:
            q_t = _normalize_qpos(q_t, self.stats)

        # build imgs (1,2,3,H,W)
        try:
            img_t = _to_tensor_image_stack(top, ee, device=self.device, resize_hw=self.resize_hw)
        except Exception as e:
            self.get_logger().error(f"[INFER] image stack failed: {e}")
            return

        # forward -> (T,9) (normalized)
        try:
            with torch.inference_mode():
                a_hat = self.policy(q_t, img_t)   # expected (1,T,9) or (T,1,9) or (T,9)
            seq = _fix_a_hat_shape(a_hat, self.chunk_size)  # (T,9) normalized
            if self.denorm_action_enabled and self.stats is not None:
                seq = _denorm_action_seq(seq, self.stats)   # (T,9) denorm
            seq_den = seq.detach().cpu().numpy().astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"[INFER] policy forward failed: {e}")
            return

        # push plan
        self.plans.append(Plan(t0=_monotonic(), seq_den=seq_den))

    # ------------------------------------------------------------
    # Temporal aggregation (denorm space)
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
                if self.use_temporal_agg:
                    if self.temporal_agg_mode == "exp":
                        w = _exp_decay_weight(age_steps, self.temporal_agg_tau_steps)
                    else:
                        w = 1.0
                else:
                    w = 1.0
                vals.append(v.astype(np.float32))
                wts.append(float(w))

        if len(vals) == 0:
            # fallback: use newest plan's last valid index
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
    # Control timer: first hold, then slow-follow
    # ------------------------------------------------------------
    def _on_control_timer(self):
        now_t = _monotonic()

        # read latest pose/force for contact update & first-hold
        with self._lock:
            pose6 = None if self._pose6 is None else self._pose6.copy()
            force6 = None if self._force6 is None else self._force6.copy()

        if pose6 is None:
            return

        meas_fz = 0.0
        if force6 is not None and force6.size >= 3:
            meas_fz = float(force6[2])

        # contact change handling
        changed = self._update_contact(meas_fz)
        if changed and self.clear_plans_on_contact_change:
            self.plans.clear()
            self._anchor_ready = False
            # jump 방지: contact 변화 순간에는 현재 pose로 "hold" 한번 더 강제
            if self.prev_cmd is not None:
                self.prev_cmd[0:6] = pose6
                self._publish_cmd(self.prev_cmd)
            self.get_logger().warn(f"[CONTACT] changed -> {int(self._contact)} | cleared plans & reset anchor")

        # (1) FIRST publish = current pose hold (MUST)
        if not self._sent_first_cmd:
            cmd0 = np.zeros(9, dtype=np.float32)
            cmd0[0:6] = pose6.astype(np.float32)
            cmd0[6] = 0.0
            cmd0[7] = 0.0
            cmd0[8] = float(self.first_cmd_fz)  # recommended 0.0

            self.prev_cmd = cmd0.copy()
            self._sent_first_cmd = True
            self._publish_cmd(cmd0)

            self.get_logger().info(
                f"[START] First publish = current pose HOLD. "
                f"pose6=[{pose6[0]:.3f},{pose6[1]:.3f},{pose6[2]:.3f},{pose6[3]:.4f},{pose6[4]:.4f},{pose6[5]:.4f}] "
                f"fz0={cmd0[8]:.3f}"
            )
            return

        if self.prev_cmd is None:
            return

        # (2) build cmd_target from temporal agg (denorm)
        cmd_pred = self._temporal_agg_cmd(now_t)
        if cmd_pred is None:
            # no plan -> hold
            self._publish_cmd(self.prev_cmd)
            return

        cmd_target = cmd_pred.astype(np.float32).copy()

        # action_type handling
        if self.action_type == "delta":
            cmd_target = (self.prev_cmd + cmd_target).astype(np.float32)

        # anchor: align predicted pose to current pose at first usage (prevents first jump)
        if not self._anchor_ready:
            self._anchor_offset6 = (pose6.astype(np.float32) - cmd_target[0:6]).astype(np.float32)
            self._anchor_ready = True
            self.get_logger().info("[ANCHOR] initialized: (current_pose6 - pred_pose6) applied to all future targets")

        cmd_target[0:6] = (cmd_target[0:6] + self._anchor_offset6).astype(np.float32)

        # (3) slow-follow: EMA(beta from tau_sec) + per-tick caps (QP-safe)
        dt = self.dt_control
        beta = _beta_from_tau(dt, self.tau_sec)

        # startup ramp scales caps
        ramp = self._startup_ramp()
        cap_pos = max(1e-9, self.pos_step_cap_mm * ramp)
        cap_ang = max(1e-12, self.ang_step_cap_rad * ramp)
        cap_fz  = max(1e-9, self.fz_step_cap * ramp)

        # desired delta after EMA
        d = (cmd_target - self.prev_cmd).astype(np.float32)
        d = (beta * d).astype(np.float32)

        # cap per component
        # xyz
        for i in range(3):
            di = float(d[i])
            if abs(di) > cap_pos:
                d[i] = float(np.sign(di) * cap_pos)
        # rpy
        for i in range(3, 6):
            di = float(d[i])
            if abs(di) > cap_ang:
                d[i] = float(np.sign(di) * cap_ang)
        # fx, fy (same as pos cap)
        for i in (6, 7):
            di = float(d[i])
            if abs(di) > cap_pos:
                d[i] = float(np.sign(di) * cap_pos)
        # fz
        di = float(d[8])
        if abs(di) > cap_fz:
            d[8] = float(np.sign(di) * cap_fz)

        cmd_next = (self.prev_cmd + d).astype(np.float32)

        # overshoot snap (if crosses target, snap to target)
        for i in range(9):
            a0 = float(self.prev_cmd[i])
            a1 = float(cmd_next[i])
            tg = float(cmd_target[i])
            if (a0 - tg) * (a1 - tg) < 0.0:
                cmd_next[i] = tg

        # publish
        self._publish_cmd(cmd_next)
        self.prev_cmd = cmd_next

        # debug
        if (int(now_t * self.control_hz) % self.debug_every_n) == 0:
            self.get_logger().info(
                f"[CTRL] contact={int(self._contact)} meas_fz={meas_fz:.3f} beta={beta:.4f} ramp={ramp:.3f} "
                f"cap(pos={cap_pos:.4f}, ang={cap_ang:.6f}, fz={cap_fz:.4f}) | "
                f"cmd_xyz=[{cmd_next[0]:.3f},{cmd_next[1]:.3f},{cmd_next[2]:.3f}]"
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
