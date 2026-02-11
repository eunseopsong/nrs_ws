#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import threading
import pickle
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import torch
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ============================================================
# Utilities
# ============================================================

def _now() -> float:
    return time.time()

def _clip(x: float, lo: float, hi: float) -> float:
    return float(np.clip(x, lo, hi))

def _sign(x: float) -> float:
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0

def _safe_resize_rgb(img_bgr: np.ndarray, out_hw: Tuple[int, int]) -> Optional[np.ndarray]:
    if img_bgr is None:
        return None
    if img_bgr.dtype != np.uint8:
        img_bgr = cv2.convertScaleAbs(img_bgr)
    if img_bgr.ndim == 2:
        img_bgr = cv2.cvtColor(img_bgr, cv2.COLOR_GRAY2BGR)
    if img_bgr.shape[2] == 1:
        img_bgr = np.repeat(img_bgr, 3, axis=2)
    if img_bgr.shape[2] > 3:
        img_bgr = img_bgr[:, :, :3]

    H, W = out_hw
    if img_bgr.shape[0] != H or img_bgr.shape[1] != W:
        img_bgr = cv2.resize(img_bgr, (W, H), interpolation=cv2.INTER_LINEAR)

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    return img_rgb

def _imagenet_norm(t: torch.Tensor) -> torch.Tensor:
    mean = torch.tensor([0.485, 0.456, 0.406], dtype=t.dtype, device=t.device)[:, None, None]
    std  = torch.tensor([0.229, 0.224, 0.225], dtype=t.dtype, device=t.device)[:, None, None]
    return (t - mean) / std

def _ensure_act_paths(act_root: str) -> None:
    if not act_root or not os.path.isdir(act_root):
        raise FileNotFoundError(f"act_root not found: {act_root}")

    candidates = [
        os.path.join(act_root, "custom"),
        os.path.join(act_root, "act", "detr", "util"),
        os.path.join(act_root, "act", "detr"),
        act_root,
        os.path.join(act_root, "act"),
    ]
    for p in candidates:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


# ============================================================
# Stats handling (group-wise: [0:3],[3:6],[6:9])
# ============================================================

@dataclass
class GroupStats3:
    mean_xyz: np.ndarray  # (3,)
    std_xyz:  np.ndarray  # (3,)
    mean_rpy: np.ndarray  # (3,)
    std_rpy:  np.ndarray  # (3,)
    mean_frc: np.ndarray  # (3,)
    std_frc:  np.ndarray  # (3,)

def _to_3(x: Any, name: str) -> np.ndarray:
    a = np.asarray(x, dtype=np.float32).reshape(-1)
    if a.size == 1:
        return np.array([a.item(), a.item(), a.item()], dtype=np.float32)
    if a.size == 3:
        return a.astype(np.float32)
    raise ValueError(f"{name} must be size 1 or 3. got {a.size}")

def _sanitize_std(s: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    s = np.asarray(s, dtype=np.float32)
    return np.maximum(s, eps)

def _build_group_stats_from_9(mean9: np.ndarray, std9: np.ndarray) -> GroupStats3:
    mean9 = np.asarray(mean9, dtype=np.float32).reshape(9)
    std9  = np.asarray(std9, dtype=np.float32).reshape(9)
    return GroupStats3(
        mean_xyz=mean9[0:3], std_xyz=_sanitize_std(std9[0:3]),
        mean_rpy=mean9[3:6], std_rpy=_sanitize_std(std9[3:6]),
        mean_frc=mean9[6:9], std_frc=_sanitize_std(std9[6:9]),
    )

def _apply_norm_groupwise(v9: np.ndarray, gs: GroupStats3) -> np.ndarray:
    v9 = np.asarray(v9, dtype=np.float32).reshape(9)
    out = np.empty_like(v9)
    out[0:3] = (v9[0:3] - gs.mean_xyz) / gs.std_xyz
    out[3:6] = (v9[3:6] - gs.mean_rpy) / gs.std_rpy
    out[6:9] = (v9[6:9] - gs.mean_frc) / gs.std_frc
    return out

def _apply_denorm_groupwise(v9n: np.ndarray, gs: GroupStats3) -> np.ndarray:
    v9n = np.asarray(v9n, dtype=np.float32).reshape(9)
    out = np.empty_like(v9n)
    out[0:3] = v9n[0:3] * gs.std_xyz + gs.mean_xyz
    out[3:6] = v9n[3:6] * gs.std_rpy + gs.mean_rpy
    out[6:9] = v9n[6:9] * gs.std_frc + gs.mean_frc
    return out

def _load_dataset_stats_groupwise(ckpt_dir: str) -> Tuple[Optional[GroupStats3], Optional[GroupStats3], Dict[str, Any]]:
    stats_path = os.path.join(ckpt_dir, "dataset_stats.pkl")
    if not os.path.exists(stats_path):
        return None, None, {}

    with open(stats_path, "rb") as f:
        st = pickle.load(f)

    def has(*keys):
        return all(k in st for k in keys)

    qgs = None
    ags = None

    try:
        if has("qpos_mean_xyz", "qpos_std_xyz", "qpos_mean_rpy", "qpos_std_rpy", "qpos_mean_force", "qpos_std_force"):
            qgs = GroupStats3(
                mean_xyz=_to_3(st["qpos_mean_xyz"], "qpos_mean_xyz"),
                std_xyz=_sanitize_std(_to_3(st["qpos_std_xyz"], "qpos_std_xyz")),
                mean_rpy=_to_3(st["qpos_mean_rpy"], "qpos_mean_rpy"),
                std_rpy=_sanitize_std(_to_3(st["qpos_std_rpy"], "qpos_std_rpy")),
                mean_frc=_to_3(st["qpos_mean_force"], "qpos_mean_force"),
                std_frc=_sanitize_std(_to_3(st["qpos_std_force"], "qpos_std_force")),
            )
        if has("action_mean_xyz", "action_std_xyz", "action_mean_rpy", "action_std_rpy", "action_mean_force", "action_std_force"):
            ags = GroupStats3(
                mean_xyz=_to_3(st["action_mean_xyz"], "action_mean_xyz"),
                std_xyz=_sanitize_std(_to_3(st["action_std_xyz"], "action_std_xyz")),
                mean_rpy=_to_3(st["action_mean_rpy"], "action_mean_rpy"),
                std_rpy=_sanitize_std(_to_3(st["action_std_rpy"], "action_std_rpy")),
                mean_frc=_to_3(st["action_mean_force"], "action_mean_force"),
                std_frc=_sanitize_std(_to_3(st["action_std_force"], "action_std_force")),
            )
    except Exception:
        qgs, ags = None, None

    if qgs is None and ("qpos_mean" in st and "qpos_std" in st):
        qm = np.asarray(st["qpos_mean"], dtype=np.float32).reshape(-1)
        qs = np.asarray(st["qpos_std"], dtype=np.float32).reshape(-1)
        if qm.size == 9 and qs.size == 9:
            qgs = _build_group_stats_from_9(qm, qs)

    if ags is None and ("action_mean" in st and "action_std" in st):
        am = np.asarray(st["action_mean"], dtype=np.float32).reshape(-1)
        astd = np.asarray(st["action_std"], dtype=np.float32).reshape(-1)
        if am.size == 9 and astd.size == 9:
            ags = _build_group_stats_from_9(am, astd)

    return qgs, ags, st


# ============================================================
# Policy building
# ============================================================

def build_policy_and_load_ckpt_programmatic(
    ckpt_dir: str,
    act_root: str,
    device: torch.device,
    hidden_dim: int,
    dim_feedforward: int,
    chunk_size: int,
    kl_weight: float,
    enc_layers: int,
    dec_layers: int,
    nheads: int,
    dropout: float,
    backbone: str,
    camera_names: Tuple[str, str] = ("cam_top", "cam_ee"),
):
    _ensure_act_paths(act_root)

    try:
        from act.detr.models.detr_vae import build as build_act_model
    except Exception as e:
        raise ImportError(
            f"Failed to import ACT build() from act_root={act_root}. "
            f"Check that act_root has act/detr/models/detr_vae.py. Original error: {e}"
        )

    import argparse
    args = argparse.Namespace(
        lr=1e-4,
        lr_backbone=1e-5,
        batch_size=1,
        weight_decay=1e-4,
        epochs=1,
        lr_drop=200,
        clip_max_norm=0.1,

        backbone=str(backbone),
        dilation=False,
        position_embedding="sine",
        masks=False,

        enc_layers=int(enc_layers),
        dec_layers=int(dec_layers),
        nheads=int(nheads),
        dropout=float(dropout),
        pre_norm=False,

        hidden_dim=int(hidden_dim),
        dim_feedforward=int(dim_feedforward),
        num_queries=int(chunk_size),
        camera_names=list(camera_names),

        kl_weight=float(kl_weight),
        eval=True,
        onscreen_render=False,
    )

    model = build_act_model(args).to(device).eval()

    ckpt_path = os.path.join(ckpt_dir, "policy_best.ckpt")
    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"policy_best.ckpt not found: {ckpt_path}")

    ckpt = torch.load(ckpt_path, map_location=device)
    state_dict = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt

    missing, unexpected = model.load_state_dict(state_dict, strict=False)
    return model, missing, unexpected


# ============================================================
# Temporal aggregation (denormed space)
# ============================================================

def _exp_weight(k: int, tau: float) -> float:
    if tau <= 1e-9:
        return 1.0
    return float(math.exp(-float(max(0, k)) / float(tau)))


# ============================================================
# ROS2 Node
# ============================================================

class ActCmdMotionInferNode(Node):
    def __init__(self):
        super().__init__("act_cmdmotion_infer_node")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("act_root", "/home/eunseop/nrs_lab2/nrs_act")
        self.declare_parameter("robot_name", "ur10skku")
        self.declare_parameter("hz", 25.0)

        self.declare_parameter("pose_topic", "")
        self.declare_parameter("force_topic", "")
        self.declare_parameter("cmd_topic", "")
        self.declare_parameter("image_topic_top", "/realsense/top/color/image_raw")
        self.declare_parameter("image_topic_ee",  "/realsense/ee/color/image_raw")

        # ACT model hyperparams
        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("chunk_size", 100)
        self.declare_parameter("kl_weight", 10.0)
        self.declare_parameter("enc_layers", 4)
        self.declare_parameter("dec_layers", 7)
        self.declare_parameter("nheads", 8)
        self.declare_parameter("dropout", 0.1)
        self.declare_parameter("backbone", "resnet18")

        # Image
        self.declare_parameter("img_h", 480)
        self.declare_parameter("img_w", 640)
        self.declare_parameter("use_imagenet_norm", True)

        # Normalization behavior
        self.declare_parameter("normalize_qpos", True)
        self.declare_parameter("denorm_action", True)

        # action type
        self.declare_parameter("action_type", "auto")  # absolute | delta | auto

        # Temporal aggregation (denorm 이후)
        self.declare_parameter("use_temporal_agg", True)
        self.declare_parameter("temporal_agg_mode", "exp")  # exp | mean
        self.declare_parameter("temporal_agg_tau", 20.0)
        self.declare_parameter("pred_step_offset", 1)
        self.declare_parameter("temporal_agg_maxlen", 80)

        # Rate limiting / smoothing
        self.declare_parameter("smooth_alpha_action", 0.6)
        self.declare_parameter("startup_ramp_sec", 2.0)

        # Command step caps (틱당 변화량 상한)
        self.declare_parameter("pos_step_cap_mm", 0.31)
        self.declare_parameter("ang_step_cap_rad", 0.0004)
        self.declare_parameter("fz_step_cap", 0.2)
        self.declare_parameter("target_update_every", 10)

        # Contact detection
        self.declare_parameter("contact_fz_on", 5.0)
        self.declare_parameter("contact_fz_off", 3.0)

        # Safety clamp
        self.declare_parameter("fz_min", 0.0)
        self.declare_parameter("fz_max", 30.0)

        # Wrench filtering (obs)
        self.declare_parameter("wrench_clip_fxy", 50.0)
        self.declare_parameter("wrench_clip_m", 5.0)
        self.declare_parameter("wrench_obs_ema", 0.2)

        # -------------------------
        # Stall debugging + prevention
        # -------------------------
        self.declare_parameter("stall_enable", True)
        self.declare_parameter("stall_window_sec", 2.0)
        self.declare_parameter("stall_pose_tol_mm", 2.0)
        self.declare_parameter("stall_pose_tol_rad", 0.01)
        self.declare_parameter("stall_fz_min_rise", 1.0)
        self.declare_parameter("stall_force_boost_target", 7.0)
        self.declare_parameter("stall_force_boost_rate", 2.0)
        self.declare_parameter("stall_force_boost_max", 12.0)
        self.declare_parameter("stall_force_pred_min", 1.0)
        self.declare_parameter("stall_debug", True)

        # -------------------------
        # NEW: Approach slowdown + pre-contact soft force + impact guard
        # -------------------------
        # (A) pre-contact 판단: meas_fz가 조금이라도 생기면 "닿기 시작"
        self.declare_parameter("precontact_meas_fz_on", 0.8)   # N
        self.declare_parameter("precontact_meas_fz_off", 0.3)  # N

        # (B) 접촉점(앵커 타겟) 근처에서 감속
        self.declare_parameter("approach_enable", True)
        self.declare_parameter("approach_dist_slow_start_mm", 25.0)  # 이 거리 이내부터 감속 시작
        self.declare_parameter("approach_min_scale", 0.08)           # 최저 스케일(속도 최소 비율)
        self.declare_parameter("approach_z_extra_scale", 0.5)        # z는 더 느리게(추가 배율)

        # (C) pre-contact fz soft-start (정책이 fz를 늦게 내도 충돌 방지)
        self.declare_parameter("precontact_force_softstart_enable", True)
        self.declare_parameter("precontact_fz_target", 2.0)          # N (contact 전에는 이 정도까지만 천천히)
        self.declare_parameter("precontact_fz_rate", 1.5)            # N/s (contact 전 상승 속도)
        self.declare_parameter("precontact_hold_z_when_fz_on", True) # 닿기 시작하면 z를 거의 고정

        # (D) impact spike guard
        self.declare_parameter("impact_guard_enable", True)
        self.declare_parameter("impact_meas_fz_spike", 40.0)          # N 넘으면 충돌로 판단
        self.declare_parameter("impact_hold_sec", 0.25)               # 잠깐 멈춤
        self.declare_parameter("impact_retract_mm", 0.10)             # hold 중 z를 살짝 올려서 파고듦 방지 (mm/tick)
        self.declare_parameter("impact_drop_fz_rate", 40.0)           # N/s (충돌 시 cmd_fz 빠르게 내리기)

        # Debug logging
        self.declare_parameter("debug_print_every_n", 25)
        self.declare_parameter("debug_seq_preview_len", 10)

        # -------------------------
        # Read params
        # -------------------------
        self.ckpt_dir = str(self.get_parameter("ckpt_dir").value)
        self.act_root = str(self.get_parameter("act_root").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.hz = float(self.get_parameter("hz").value)

        pose_topic = str(self.get_parameter("pose_topic").value).strip()
        force_topic = str(self.get_parameter("force_topic").value).strip()
        cmd_topic = str(self.get_parameter("cmd_topic").value).strip()

        self.pose_topic = pose_topic if pose_topic else f"/{self.robot_name}/currentP"
        self.force_topic = force_topic if force_topic else f"/{self.robot_name}/currentF"
        self.cmd_topic = cmd_topic if cmd_topic else f"/{self.robot_name}/cmdMotion"

        self.image_topic_top = str(self.get_parameter("image_topic_top").value)
        self.image_topic_ee  = str(self.get_parameter("image_topic_ee").value)

        self.hidden_dim = int(self.get_parameter("hidden_dim").value)
        self.dim_feedforward = int(self.get_parameter("dim_feedforward").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.kl_weight = float(self.get_parameter("kl_weight").value)
        self.enc_layers = int(self.get_parameter("enc_layers").value)
        self.dec_layers = int(self.get_parameter("dec_layers").value)
        self.nheads = int(self.get_parameter("nheads").value)
        self.dropout = float(self.get_parameter("dropout").value)
        self.backbone = str(self.get_parameter("backbone").value)

        self.img_h = int(self.get_parameter("img_h").value)
        self.img_w = int(self.get_parameter("img_w").value)
        self.use_imnet = bool(self.get_parameter("use_imagenet_norm").value)

        self.normalize_qpos = bool(self.get_parameter("normalize_qpos").value)
        self.denorm_action = bool(self.get_parameter("denorm_action").value)

        self.action_type = str(self.get_parameter("action_type").value).lower().strip()

        self.use_temporal_agg = bool(self.get_parameter("use_temporal_agg").value)
        self.temporal_agg_mode = str(self.get_parameter("temporal_agg_mode").value).lower().strip()
        self.temporal_agg_tau = float(self.get_parameter("temporal_agg_tau").value)
        self.pred_step_offset = int(self.get_parameter("pred_step_offset").value)
        self.temporal_agg_maxlen = int(self.get_parameter("temporal_agg_maxlen").value)

        self.smooth_alpha_action = float(np.clip(float(self.get_parameter("smooth_alpha_action").value), 0.0, 1.0))
        self.startup_ramp_sec = float(self.get_parameter("startup_ramp_sec").value)

        self.pos_step_cap_mm = float(self.get_parameter("pos_step_cap_mm").value)
        self.ang_step_cap_rad = float(self.get_parameter("ang_step_cap_rad").value)
        self.fz_step_cap = float(self.get_parameter("fz_step_cap").value)
        self.target_update_every = max(1, int(self.get_parameter("target_update_every").value))

        self.contact_fz_on = float(self.get_parameter("contact_fz_on").value)
        self.contact_fz_off = float(self.get_parameter("contact_fz_off").value)

        self.fz_min = float(self.get_parameter("fz_min").value)
        self.fz_max = float(self.get_parameter("fz_max").value)

        self.wrench_clip_fxy = float(self.get_parameter("wrench_clip_fxy").value)
        self.wrench_clip_m = float(self.get_parameter("wrench_clip_m").value)
        self.wrench_obs_ema = float(np.clip(float(self.get_parameter("wrench_obs_ema").value), 0.0, 1.0))

        self.stall_enable = bool(self.get_parameter("stall_enable").value)
        self.stall_window_sec = float(self.get_parameter("stall_window_sec").value)
        self.stall_pose_tol_mm = float(self.get_parameter("stall_pose_tol_mm").value)
        self.stall_pose_tol_rad = float(self.get_parameter("stall_pose_tol_rad").value)
        self.stall_fz_min_rise = float(self.get_parameter("stall_fz_min_rise").value)
        self.stall_force_boost_target = float(self.get_parameter("stall_force_boost_target").value)
        self.stall_force_boost_rate = float(self.get_parameter("stall_force_boost_rate").value)
        self.stall_force_boost_max = float(self.get_parameter("stall_force_boost_max").value)
        self.stall_force_pred_min = float(self.get_parameter("stall_force_pred_min").value)
        self.stall_debug = bool(self.get_parameter("stall_debug").value)

        # NEW approach/impact params
        self.precontact_meas_fz_on = float(self.get_parameter("precontact_meas_fz_on").value)
        self.precontact_meas_fz_off = float(self.get_parameter("precontact_meas_fz_off").value)
        self.approach_enable = bool(self.get_parameter("approach_enable").value)
        self.approach_dist_slow_start_mm = float(self.get_parameter("approach_dist_slow_start_mm").value)
        self.approach_min_scale = float(self.get_parameter("approach_min_scale").value)
        self.approach_z_extra_scale = float(self.get_parameter("approach_z_extra_scale").value)
        self.precontact_force_softstart_enable = bool(self.get_parameter("precontact_force_softstart_enable").value)
        self.precontact_fz_target = float(self.get_parameter("precontact_fz_target").value)
        self.precontact_fz_rate = float(self.get_parameter("precontact_fz_rate").value)
        self.precontact_hold_z_when_fz_on = bool(self.get_parameter("precontact_hold_z_when_fz_on").value)
        self.impact_guard_enable = bool(self.get_parameter("impact_guard_enable").value)
        self.impact_meas_fz_spike = float(self.get_parameter("impact_meas_fz_spike").value)
        self.impact_hold_sec = float(self.get_parameter("impact_hold_sec").value)
        self.impact_retract_mm = float(self.get_parameter("impact_retract_mm").value)
        self.impact_drop_fz_rate = float(self.get_parameter("impact_drop_fz_rate").value)

        self.debug_print_every_n = max(1, int(self.get_parameter("debug_print_every_n").value))
        self.debug_seq_preview_len = max(1, int(self.get_parameter("debug_seq_preview_len").value))

        if not self.ckpt_dir or not os.path.isdir(self.ckpt_dir):
            raise FileNotFoundError(f"ckpt_dir not found: {self.ckpt_dir}")

        # -------------------------
        # Buffers / ROS I/O
        # -------------------------
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        self._have_pose = False
        self._have_force = False
        self._have_img_top = False
        self._have_img_ee = False

        self._pose6 = np.zeros(6, dtype=np.float32)
        self._wrench6 = np.zeros(6, dtype=np.float32)
        self._wrench6_filt = None

        self._img_top_bgr = None
        self._img_ee_bgr  = None

        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self._cb_pose, 10)
        self.sub_force = self.create_subscription(Float64MultiArray, self.force_topic, self._cb_force, 10)
        self.sub_img_top = self.create_subscription(Image, self.image_topic_top, self._cb_img_top, 10)
        self.sub_img_ee  = self.create_subscription(Image, self.image_topic_ee,  self._cb_img_ee,  10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # -------------------------
        # Device / Load stats / Load policy
        # -------------------------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"[INFO] Using device: {self.device}")
        self.get_logger().info(f"[INFO] ckpt_dir={self.ckpt_dir}")
        self.get_logger().info(f"[INFO] act_root={self.act_root}")
        self.get_logger().info(f"[INFO] pose_topic={self.pose_topic}, force_topic={self.force_topic}")
        self.get_logger().info(f"[INFO] image_topics=[{self.image_topic_top}, {self.image_topic_ee}]")
        self.get_logger().info(f"[INFO] cmd_topic={self.cmd_topic} @ {self.hz} Hz (dt={1.0/self.hz:.4f}s)")
        self.get_logger().info(
            f"[INFO] action_type={self.action_type} | temporal_agg={int(self.use_temporal_agg)} "
            f"mode={self.temporal_agg_mode} tau={self.temporal_agg_tau} offset={self.pred_step_offset}"
        )

        self.qpos_stats_g, self.act_stats_g, self._raw_stats = _load_dataset_stats_groupwise(self.ckpt_dir)
        if self.qpos_stats_g is None or self.act_stats_g is None:
            raise RuntimeError("dataset_stats.pkl missing or invalid (need qpos/action mean/std 9D or group-wise keys).")

        qgs = self.qpos_stats_g
        ags = self.act_stats_g
        self.get_logger().info(
            f"[STATS] qpos_mean_xyz={qgs.mean_xyz.tolist()} qpos_std_xyz_min={float(np.min(qgs.std_xyz)):.6f} | "
            f"qpos_mean_rpy={qgs.mean_rpy.tolist()} qpos_std_rpy_min={float(np.min(qgs.std_rpy)):.6f} | "
            f"qpos_mean_frc={qgs.mean_frc.tolist()} qpos_std_frc_min={float(np.min(qgs.std_frc)):.6f}"
        )
        self.get_logger().info(
            f"[STATS] act_mean_xyz={ags.mean_xyz.tolist()} act_std_xyz_min={float(np.min(ags.std_xyz)):.6f} | "
            f"act_mean_rpy={ags.mean_rpy.tolist()} act_std_rpy_min={float(np.min(ags.std_rpy)):.6f} | "
            f"act_mean_frc={ags.mean_frc.tolist()} act_std_frc_min={float(np.min(ags.std_frc)):.6f}"
        )

        self.get_logger().info("[INFO] Loading policy (programmatic)...")
        self.policy, missing, unexpected = build_policy_and_load_ckpt_programmatic(
            ckpt_dir=self.ckpt_dir,
            act_root=self.act_root,
            device=self.device,
            hidden_dim=self.hidden_dim,
            dim_feedforward=self.dim_feedforward,
            chunk_size=self.chunk_size,
            kl_weight=self.kl_weight,
            enc_layers=self.enc_layers,
            dec_layers=self.dec_layers,
            nheads=self.nheads,
            dropout=self.dropout,
            backbone=self.backbone,
            camera_names=("cam_top", "cam_ee"),
        )
        self.get_logger().info(f"[INFO] Loaded policy. missing={len(missing)}, unexpected={len(unexpected)}")

        # -------------------------
        # Control states
        # -------------------------
        self._tick = 0
        self._contact = False
        self._precontact = False

        self.prev_cmd = None
        self._vel_step = None
        self._vel_step_last_update_tick = -1

        self._sent_first_cmd = False
        self._first_cmd_time = None

        self._anchor_ready = False
        self._anchor_offset6 = np.zeros(6, dtype=np.float32)

        self._prev_action_den = None

        self.pred_buffer = deque(maxlen=max(1, self.temporal_agg_maxlen))
        self._last_seq_den = None
        self._last_seq_start_tick = 0

        self._stall_ref_pose6 = None
        self._stall_fz_hist = deque()
        self._stall_boost_active = False
        self._stall_boost_until = 0.0

        # NEW: impact hold
        self._impact_hold_until = 0.0

        self.timer = self.create_timer(1.0 / self.hz, self._on_timer)
        self.get_logger().info("✅ Model ready. Waiting for topics...")

    # -------------------------
    # Callbacks
    # -------------------------
    def _cb_pose(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        with self._lock:
            self._pose6[:] = np.asarray(msg.data[:6], dtype=np.float32)
            self._have_pose = True

    def _cb_force(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        w = np.asarray(msg.data[:6], dtype=np.float32)

        w[0] = float(np.clip(w[0], -self.wrench_clip_fxy, self.wrench_clip_fxy))
        w[1] = float(np.clip(w[1], -self.wrench_clip_fxy, self.wrench_clip_fxy))
        w[2] = float(w[2])
        w[3] = float(np.clip(w[3], -self.wrench_clip_m, self.wrench_clip_m))
        w[4] = float(np.clip(w[4], -self.wrench_clip_m, self.wrench_clip_m))
        w[5] = float(np.clip(w[5], -self.wrench_clip_m, self.wrench_clip_m))

        with self._lock:
            self._wrench6[:] = w
            self._have_force = True
            if self._wrench6_filt is None:
                self._wrench6_filt = w.copy()
            else:
                a = self.wrench_obs_ema
                self._wrench6_filt = a * w + (1.0 - a) * self._wrench6_filt

    def _cb_img_top(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._img_top_bgr = img
                self._have_img_top = True
        except Exception as e:
            self.get_logger().error(f"Top image convert error: {e}")

    def _cb_img_ee(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._img_ee_bgr = img
                self._have_img_ee = True
        except Exception as e:
            self.get_logger().error(f"EE image convert error: {e}")

    # -------------------------
    # Helpers
    # -------------------------
    def _update_contact(self, meas_fz: float):
        if (not self._contact) and (meas_fz >= self.contact_fz_on):
            self._contact = True
        elif self._contact and (meas_fz <= self.contact_fz_off):
            self._contact = False

    def _update_precontact(self, meas_fz: float):
        if (not self._precontact) and (meas_fz >= self.precontact_meas_fz_on):
            self._precontact = True
        elif self._precontact and (meas_fz <= self.precontact_meas_fz_off):
            self._precontact = False

    def _ramp_scale(self, tick: int) -> float:
        if self.startup_ramp_sec <= 1e-6:
            return 1.0
        total_ticks = max(1, int(self.startup_ramp_sec * self.hz))
        return float(np.clip((tick + 1) / total_ticks, 0.0, 1.0))

    def _publish_cmd(self, cmd9: np.ndarray):
        m = Float64MultiArray()
        m.data = [float(v) for v in cmd9.tolist()]
        self.pub_cmd.publish(m)

    def _prepare_inputs(self) -> Optional[Tuple[torch.Tensor, torch.Tensor, np.ndarray, float]]:
        with self._lock:
            if not (self._have_pose and self._have_force and self._have_img_top and self._have_img_ee):
                return None
            pose6 = self._pose6.copy()
            w6 = self._wrench6_filt.copy() if self._wrench6_filt is not None else self._wrench6.copy()
            top_bgr = None if self._img_top_bgr is None else self._img_top_bgr.copy()
            ee_bgr  = None if self._img_ee_bgr  is None else self._img_ee_bgr.copy()

        if top_bgr is None or ee_bgr is None:
            return None

        fx, fy, fz, mx, my, mz = [float(v) for v in w6.tolist()]

        qpos_np = np.concatenate([pose6, np.asarray([fx, fy, fz], dtype=np.float32)], axis=0).astype(np.float32)
        if self.normalize_qpos:
            qpos_np = _apply_norm_groupwise(qpos_np, self.qpos_stats_g)

        top_rgb = _safe_resize_rgb(top_bgr, (self.img_h, self.img_w))
        ee_rgb  = _safe_resize_rgb(ee_bgr,  (self.img_h, self.img_w))
        if top_rgb is None or ee_rgb is None:
            return None

        top_t = torch.from_numpy(top_rgb).permute(2, 0, 1).float() / 255.0
        ee_t  = torch.from_numpy(ee_rgb).permute(2, 0, 1).float() / 255.0
        if self.use_imnet:
            top_t = _imagenet_norm(top_t)
            ee_t  = _imagenet_norm(ee_t)

        cams = torch.stack([top_t, ee_t], dim=0)  # (2,3,H,W)
        imgs = cams.unsqueeze(0).to(self.device)  # (1,2,3,H,W)
        qpos = torch.from_numpy(qpos_np).unsqueeze(0).to(self.device)  # (1,9)
        return imgs, qpos, pose6, float(fz)

    def _policy_forward_seq_norm(self, imgs: torch.Tensor, qpos: torch.Tensor) -> np.ndarray:
        with torch.no_grad():
            out = self.policy(qpos, imgs)
            action_t = out[0] if isinstance(out, (tuple, list)) else out
        action_t = action_t.detach()
        if action_t.ndim == 3:
            seq = action_t[0]
        elif action_t.ndim == 2:
            seq = action_t
        else:
            seq = action_t.reshape(-1).view(1, -1)
        seq = seq.cpu().numpy().astype(np.float32)
        if seq.shape[1] != 9:
            raise RuntimeError(f"Policy output dim != 9. got {seq.shape}. Check model build/config.")
        return seq

    def _denorm_action_seq(self, seq_norm: np.ndarray) -> np.ndarray:
        if not self.denorm_action:
            return seq_norm.astype(np.float32)
        H = seq_norm.shape[0]
        out = np.empty_like(seq_norm, dtype=np.float32)
        for i in range(H):
            out[i] = _apply_denorm_groupwise(seq_norm[i], self.act_stats_g)
        return out

    def _temporal_aggregate_den(self, tick_now: int) -> Optional[np.ndarray]:
        if not self.pred_buffer:
            return None

        vals: List[np.ndarray] = []
        wts: List[float] = []
        for (t0, seq_den) in self.pred_buffer:
            k = (tick_now - t0) + int(self.pred_step_offset)
            if 0 <= k < seq_den.shape[0]:
                v = seq_den[k]
                w = _exp_weight(k, self.temporal_agg_tau) if self.temporal_agg_mode == "exp" else 1.0
                vals.append(v)
                wts.append(w)

        if len(vals) == 0:
            if self._last_seq_den is not None:
                k = (tick_now - self._last_seq_start_tick) + int(self.pred_step_offset)
                k = int(np.clip(k, 0, self._last_seq_den.shape[0] - 1))
                return self._last_seq_den[k].astype(np.float32)
            return None

        W = float(np.sum(wts))
        if W <= 1e-9:
            return vals[-1].astype(np.float32)

        acc = np.zeros(9, dtype=np.float32)
        for v, w in zip(vals, wts):
            acc += (w / W) * v.astype(np.float32)
        return acc.astype(np.float32)

    def _near_pose(self, pose6: np.ndarray, ref6: np.ndarray, tol_mm: float, tol_rad: float) -> bool:
        dp = np.linalg.norm((pose6[0:3] - ref6[0:3]).astype(np.float32))
        dr = np.linalg.norm((pose6[3:6] - ref6[3:6]).astype(np.float32))
        return (dp <= tol_mm) and (dr <= tol_rad)

    def _approach_scale_from_dist(self, pose6: np.ndarray) -> float:
        """
        앵커 타겟(=대략 접촉점) 근처에서 속도 스케일을 줄인다.
        """
        if (not self.approach_enable) or (self._stall_ref_pose6 is None):
            return 1.0
        d = float(np.linalg.norm((pose6[0:3] - self._stall_ref_pose6[0:3]).astype(np.float32)))
        if d >= self.approach_dist_slow_start_mm:
            return 1.0
        # d=0 -> min_scale, d=slow_start -> 1.0
        s = self.approach_min_scale + (1.0 - self.approach_min_scale) * (d / max(1e-6, self.approach_dist_slow_start_mm))
        return float(np.clip(s, self.approach_min_scale, 1.0))

    def _stall_update_hist(self, cmd_fz: float):
        t = _now()
        self._stall_fz_hist.append((t, float(cmd_fz)))
        while self._stall_fz_hist and (t - self._stall_fz_hist[0][0] > max(0.5, self.stall_window_sec * 2.0)):
            self._stall_fz_hist.popleft()

    def _stall_detect(self, current_pose6: np.ndarray) -> bool:
        if not self.stall_enable:
            return False
        if self._contact:
            return False
        if self._stall_ref_pose6 is None:
            return False
        if not self._near_pose(current_pose6, self._stall_ref_pose6, self.stall_pose_tol_mm, self.stall_pose_tol_rad):
            return False

        t = _now()
        recent = [(tt, fz) for (tt, fz) in self._stall_fz_hist if (t - tt) <= self.stall_window_sec]
        if len(recent) < 3:
            return False
        fz_vals = [fz for (_, fz) in recent]
        rise = float(max(fz_vals) - min(fz_vals))
        return (rise < self.stall_fz_min_rise)

    # -------------------------
    # Main loop
    # -------------------------
    def _on_timer(self):
        pack = self._prepare_inputs()
        if pack is None:
            return
        imgs, qpos, current_pose6, meas_fz = pack

        self._update_contact(meas_fz)
        self._update_precontact(meas_fz)

        dt = 1.0 / float(self.hz)

        # impact guard: meas_fz 스파이크면 잠깐 홀드
        if self.impact_guard_enable and (meas_fz >= self.impact_meas_fz_spike) and (not self._contact):
            self._impact_hold_until = max(self._impact_hold_until, _now() + self.impact_hold_sec)

        # 첫 cmd
        if not self._sent_first_cmd:
            cmd0 = np.zeros(9, dtype=np.float32)
            cmd0[0:6] = current_pose6
            cmd0[6] = 0.0
            cmd0[7] = 0.0
            cmd0[8] = _clip(float(meas_fz), self.fz_min, self.fz_max)

            self.prev_cmd = cmd0.copy()
            self._vel_step = np.zeros(9, dtype=np.float32)
            self._vel_step_last_update_tick = self._tick
            self._sent_first_cmd = True
            self._first_cmd_time = _now()

            # 임시 stall ref (anchor 초기화 후 갱신됨)
            self._stall_ref_pose6 = current_pose6.copy()

            self._publish_cmd(cmd0)
            self.get_logger().info(
                f"[START] First cmd=current. cmd=[{cmd0[0]:.3f},{cmd0[1]:.3f},{cmd0[2]:.3f},"
                f"{cmd0[3]:.4f},{cmd0[4]:.4f},{cmd0[5]:.4f},{cmd0[6]:.3f},{cmd0[7]:.3f},{cmd0[8]:.3f}]"
            )
            self._tick += 1
            return

        # Inference
        try:
            pred_seq_norm = self._policy_forward_seq_norm(imgs, qpos)
            pred_seq_den = self._denorm_action_seq(pred_seq_norm)
        except Exception as e:
            self.get_logger().error(f"Policy forward failed: {e}")
            return

        self.pred_buffer.append((self._tick, pred_seq_den))
        self._last_seq_den = pred_seq_den
        self._last_seq_start_tick = self._tick

        # denorm 이후 temporal agg
        if self.use_temporal_agg:
            action_den = self._temporal_aggregate_den(self._tick)
        else:
            k = (self._tick - self._last_seq_start_tick) + int(self.pred_step_offset)
            k = int(np.clip(k, 0, pred_seq_den.shape[0] - 1))
            action_den = pred_seq_den[k].astype(np.float32)

        if action_den is None:
            return

        # action smoothing (denormed)
        if self._prev_action_den is None:
            action_den_s = action_den
        else:
            a = float(self.smooth_alpha_action)
            action_den_s = a * action_den + (1.0 - a) * self._prev_action_den
        self._prev_action_den = action_den_s.astype(np.float32)

        # action_type
        if self.action_type == "delta":
            cmd_target = self.prev_cmd.copy()
            cmd_target[:] = cmd_target + action_den_s.astype(np.float32)
        else:
            cmd_target = action_den_s.astype(np.float32).copy()

        # clamp fz
        cmd_target[8] = _clip(float(cmd_target[8]), self.fz_min, self.fz_max)

        # anchor (pose offset)
        if not self._anchor_ready:
            self._anchor_offset6 = (current_pose6 - cmd_target[0:6]).astype(np.float32)
            self._anchor_ready = True
            self._stall_ref_pose6 = (cmd_target[0:6] + self._anchor_offset6).copy()
            self.get_logger().info("[ANCHOR] pose offset initialized (current - pred). stall_ref_pose set.")
        cmd_target[0:6] = cmd_target[0:6] + self._anchor_offset6

        # ------------------------------------------------------------
        # NEW: pre-contact soft-start force + approach slowdown
        # ------------------------------------------------------------
        approach_scale = self._approach_scale_from_dist(current_pose6)

        # precontact이면 더 강하게 감속(특히 z)
        if self._precontact and (not self._contact):
            approach_scale = min(approach_scale, 0.25)

            # 닿기 시작하면 z 거의 고정(충돌 방지)
            if self.precontact_hold_z_when_fz_on:
                cmd_target[2] = self.prev_cmd[2]  # z target을 현재 cmd로 고정(파고들지 않게)

            # contact 전 soft-start fz: 0에서 갑자기 15N 가는 걸 방지
            if self.precontact_force_softstart_enable:
                desired = float(np.clip(self.precontact_fz_target, self.fz_min, self.fz_max))
                cur = float(self.prev_cmd[8])
                if cur < desired:
                    cur = min(desired, cur + float(self.precontact_fz_rate) * dt)
                cmd_target[8] = max(cmd_target[8], cur)

        # ------------------------------------------------------------
        # 기존 stall boost는 "정지"를 위한 것이고,
        # 충돌 방지는 위 precontact/approach가 담당.
        # ------------------------------------------------------------
        self._stall_update_hist(self.prev_cmd[8] if self.prev_cmd is not None else 0.0)
        stalled = self._stall_detect(current_pose6)
        if stalled and (not self._precontact) and (not self._contact):
            pred_fz_max = float(np.max(pred_seq_den[:, 8]))
            if pred_fz_max < self.stall_force_pred_min and self.stall_debug:
                self.get_logger().warn(
                    f"[STALL] pred_fz_max={pred_fz_max:.3f} < {self.stall_force_pred_min:.3f}. "
                    f"Model might not be producing force-rise (early phase)."
                )
            # stall 상태면 fz만 아주 천천히 올려서 contact 진입 유도
            target = float(np.clip(self.stall_force_boost_target, self.fz_min, min(self.fz_max, self.stall_force_boost_max)))
            if cmd_target[8] < target:
                cmd_target[8] = min(target, cmd_target[8] + float(self.stall_force_boost_rate) * dt)
            self._stall_boost_active = True
            self._stall_boost_until = _now() + 1.0

        if self._stall_boost_active and _now() > self._stall_boost_until:
            self._stall_boost_active = False

        # ------------------------------------------------------------
        # Step caps + update scheduling (approach_scale 반영)
        # ------------------------------------------------------------
        ramp = self._ramp_scale(self._tick)

        cap_pos = max(1e-6, self.pos_step_cap_mm * ramp * approach_scale)
        cap_ang = max(1e-8, self.ang_step_cap_rad * ramp * approach_scale)
        cap_fz  = max(1e-6, self.fz_step_cap * ramp)  # force는 approach_scale로 줄이면 너무 늦게 오를 수 있어 유지

        # z는 더 느리게
        cap_z = cap_pos * float(np.clip(self.approach_z_extra_scale, 0.05, 1.0))

        prev = self.prev_cmd.copy()
        need_update = (self._vel_step is None) or (self._tick - self._vel_step_last_update_tick >= self.target_update_every)

        if need_update:
            step = np.zeros(9, dtype=np.float32)

            # xyz
            for i in range(3):
                d = float(cmd_target[i] - prev[i])
                cap_i = cap_z if i == 2 else cap_pos
                step[i] = d if abs(d) <= cap_i else (_sign(d) * cap_i)

            # rpy
            for i in range(3, 6):
                d = float(cmd_target[i] - prev[i])
                step[i] = d if abs(d) <= cap_ang else (_sign(d) * cap_ang)

            # fx, fy
            for i in (6, 7):
                d = float(cmd_target[i] - prev[i])
                step[i] = d if abs(d) <= cap_pos else (_sign(d) * cap_pos)

            # fz
            d = float(cmd_target[8] - prev[8])
            step[8] = d if abs(d) <= cap_fz else (_sign(d) * cap_fz)

            self._vel_step = step
            self._vel_step_last_update_tick = self._tick

        cmd_next = prev + self._vel_step

        # overshoot snap
        for i in range(9):
            a0, a1, tg = float(prev[i]), float(cmd_next[i]), float(cmd_target[i])
            if (a0 - tg) * (a1 - tg) < 0.0:
                cmd_next[i] = tg

        # ------------------------------------------------------------
        # NEW: impact hold 동안 motion 억제 + 살짝 후퇴 + fz 급강하
        # ------------------------------------------------------------
        if self.impact_guard_enable and (_now() < self._impact_hold_until):
            # pose hold
            cmd_next[0:6] = prev[0:6]
            # z retract (파고듦 방지)
            cmd_next[2] = prev[2] + float(self.impact_retract_mm)
            # drop fz fast
            drop = float(self.impact_drop_fz_rate) * dt
            cmd_next[8] = max(self.fz_min, float(prev[8]) - drop)

        cmd_next[8] = _clip(float(cmd_next[8]), self.fz_min, self.fz_max)

        # publish
        self._publish_cmd(cmd_next)
        self.prev_cmd = cmd_next

        # Debug prints
        if (self._tick % self.debug_print_every_n) == 0:
            pred_fz_now = float(action_den_s[8])
            pred_fz_max = float(np.max(pred_seq_den[:, 8]))
            pred_fz_mean = float(np.mean(pred_seq_den[:, 8]))
            self.get_logger().info(
                f"tick={self._tick} contact={1 if self._contact else 0} preC={1 if self._precontact else 0} "
                f"impactHold={1 if (_now() < self._impact_hold_until) else 0} boost={1 if self._stall_boost_active else 0} | "
                f"meas_fz={meas_fz:.3f} cmd_fz={cmd_next[8]:.3f} | pred_fz_now={pred_fz_now:.3f} "
                f"pred_fz_max={pred_fz_max:.3f} pred_fz_mean={pred_fz_mean:.3f} | "
                f"approachScale={approach_scale:.3f} cap_z={cap_z:.4f} | "
                f"cmd_xyz=[{cmd_next[0]:.3f},{cmd_next[1]:.3f},{cmd_next[2]:.3f}]"
            )
            L = min(self.debug_seq_preview_len, pred_seq_den.shape[0])
            preview = pred_seq_den[:L, 8].tolist()
            self.get_logger().info(f"[SEQ] fz_den[0:{L}]={['%.3f'%v for v in preview]}")

        self._tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ActCmdMotionInferNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # shutdown 중복 호출 방지
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
