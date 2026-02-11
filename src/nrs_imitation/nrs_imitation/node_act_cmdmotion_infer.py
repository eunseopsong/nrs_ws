#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading
import pickle
from collections import deque
from typing import Optional, Tuple

import numpy as np
import torch
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ============================================================
# Utils
# ============================================================

def _ensure_act_paths(act_root: str) -> None:
    if not act_root or not os.path.isdir(act_root):
        raise FileNotFoundError(f"act_root not found: {act_root}")

    candidates = [
        act_root,
        os.path.join(act_root, "act"),
        os.path.join(act_root, "act", "model"),
        os.path.join(act_root, "act", "detr"),
        os.path.join(act_root, "act", "detr", "util"),
        os.path.join(act_root, "custom"),
    ]
    for p in candidates:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.append(p)


def _load_dataset_stats(ckpt_dir: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    stats_path = os.path.join(ckpt_dir, "dataset_stats.pkl")
    if not os.path.exists(stats_path):
        return None, None
    try:
        with open(stats_path, "rb") as f:
            stats = pickle.load(f)
        if "action_mean" in stats and "action_std" in stats:
            mean = np.asarray(stats["action_mean"], dtype=np.float32)
            std  = np.asarray(stats["action_std"], dtype=np.float32)
            std = np.maximum(std, 1e-6)
            return mean, std
        return None, None
    except Exception:
        return None, None


def _imagenet_norm(t: torch.Tensor) -> torch.Tensor:
    mean = torch.tensor([0.485, 0.456, 0.406], dtype=t.dtype, device=t.device)[:, None, None]
    std  = torch.tensor([0.229, 0.224, 0.225], dtype=t.dtype, device=t.device)[:, None, None]
    return (t - mean) / std


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


def _clip(v: float, lo: float, hi: float) -> float:
    return float(np.clip(v, lo, hi))


def _sign(x: float) -> float:
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0


def _find_first_linear_in_features(model: torch.nn.Module) -> Optional[int]:
    """
    ACT 코드 구조가 버전에 따라 달라질 수 있어서,
    "qpos가 들어가는 첫 Linear"을 100% 특정하긴 어렵지만,
    실무적으로는 '가장 먼저 나오는 Linear 중 in_features가 9 또는 12인 것'을
    qpos embedding으로 보는 게 안전하다.
    """
    for m in model.modules():
        if isinstance(m, torch.nn.Linear):
            if m.in_features in (9, 12):
                return int(m.in_features)
    return None


# ============================================================
# Policy Loader
# ============================================================

def build_policy_and_load_ckpt_programmatic(
    ckpt_dir: str,
    act_root: str,
    device: torch.device,
    hidden_dim: int,
    dim_feedforward: int,
    chunk_size: int,
    kl_weight: float,
    camera_names: Tuple[str, str] = ("cam_top", "cam_ee"),
):
    _ensure_act_paths(act_root)

    try:
        from act.detr.models.detr_vae import build as build_act_model
    except Exception as e:
        raise ImportError(
            f"Failed to import ACT build() from act_root={act_root}. "
            f"Check that {act_root} contains act/detr/models/detr_vae.py. "
            f"Original error: {e}"
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

        backbone="resnet18",
        dilation=False,
        position_embedding="sine",
        masks=False,

        enc_layers=6,
        dec_layers=6,
        nheads=8,
        dropout=0.1,
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

    # security warning 대응: 네가 "내가 만든 ckpt"면 현 상태도 OK.
    # 그래도 경고 없애려면 weights_only=True + state_dict 형태로 저장되어 있어야 함.
    ckpt = torch.load(ckpt_path, map_location=device)
    state_dict = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt

    missing, unexpected = model.load_state_dict(state_dict, strict=False)
    return model, missing, unexpected


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
        self.declare_parameter("robot_name", "ur10skku")
        self.declare_parameter("hz", 25.0)
        self.declare_parameter("act_root", "/home/eunseop/nrs_lab2/nrs_act")

        self.declare_parameter("pose_topic", "")
        self.declare_parameter("force_topic", "")
        self.declare_parameter("cmd_topic", "")
        self.declare_parameter("image_topic_top", "/realsense/top/color/image_raw")
        self.declare_parameter("image_topic_ee",  "/realsense/ee/color/image_raw")

        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("chunk_size", 100)
        self.declare_parameter("kl_weight", 10.0)

        self.declare_parameter("img_h", 480)
        self.declare_parameter("img_w", 640)
        self.declare_parameter("use_imagenet_norm", True)

        self.declare_parameter("warmup_sec", 1.0)
        self.declare_parameter("startup_ramp_sec", 2.0)
        self.declare_parameter("anchor_to_current", True)

        self.declare_parameter("use_temporal_agg", True)
        self.declare_parameter("temporal_agg_maxlen", 60)

        self.declare_parameter("smooth_alpha", 0.6)

        self.declare_parameter("pos_step_cap_mm", 0.31)
        self.declare_parameter("ang_step_cap_rad", 0.0004)
        self.declare_parameter("fz_step_cap", 0.2)
        self.declare_parameter("target_update_every", 10)

        self.declare_parameter("inference_every", 1)
        self.declare_parameter("step_ema_beta", 0.25)

        self.declare_parameter("contact_fz_on", 5.0)
        self.declare_parameter("contact_fz_off", 3.0)
        self.declare_parameter("contact_cap_scale", 0.5)
        self.declare_parameter("contact_update_mult", 2.0)

        # 이 파라미터는 남겨두되, "모델 입력 차원"이 9면 자동으로 무시됨.
        self.declare_parameter("use_wrench_obs", True)

        self.declare_parameter("wrench_clip_fxy", 20.0)
        self.declare_parameter("wrench_clip_m", 2.0)
        self.declare_parameter("wrench_obs_ema", 0.2)

        self.declare_parameter("fz_min", 0.0)
        self.declare_parameter("fz_max", 30.0)

        # -------------------------
        # Read params
        # -------------------------
        self.ckpt_dir = str(self.get_parameter("ckpt_dir").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.hz = float(self.get_parameter("hz").value)
        self.act_root = str(self.get_parameter("act_root").value)

        pose_topic = str(self.get_parameter("pose_topic").value).strip()
        force_topic = str(self.get_parameter("force_topic").value).strip()
        cmd_topic = str(self.get_parameter("cmd_topic").value).strip()
        self.pose_topic = pose_topic if pose_topic else f"/{self.robot_name}/currentP"
        self.force_topic = force_topic if force_topic else f"/{self.robot_name}/currentF"
        self.cmd_topic = cmd_topic if cmd_topic else f"/{self.robot_name}/cmdMotion"

        self.image_topic_top = str(self.get_parameter("image_topic_top").value)
        self.image_topic_ee = str(self.get_parameter("image_topic_ee").value)

        self.hidden_dim = int(self.get_parameter("hidden_dim").value)
        self.dim_feedforward = int(self.get_parameter("dim_feedforward").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.kl_weight = float(self.get_parameter("kl_weight").value)

        self.img_h = int(self.get_parameter("img_h").value)
        self.img_w = int(self.get_parameter("img_w").value)
        self.use_imnet = bool(self.get_parameter("use_imagenet_norm").value)

        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        self.startup_ramp_sec = float(self.get_parameter("startup_ramp_sec").value)
        self.anchor_to_current = bool(self.get_parameter("anchor_to_current").value)

        self.use_temporal_agg = bool(self.get_parameter("use_temporal_agg").value)
        self.temporal_agg_maxlen = int(self.get_parameter("temporal_agg_maxlen").value)

        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)

        self.pos_step_cap_mm = float(self.get_parameter("pos_step_cap_mm").value)
        self.ang_step_cap_rad = float(self.get_parameter("ang_step_cap_rad").value)
        self.fz_step_cap = float(self.get_parameter("fz_step_cap").value)

        self.target_update_every = max(1, int(self.get_parameter("target_update_every").value))
        self.inference_every = max(1, int(self.get_parameter("inference_every").value))

        self.step_ema_beta = float(np.clip(float(self.get_parameter("step_ema_beta").value), 0.0, 1.0))

        self.contact_fz_on = float(self.get_parameter("contact_fz_on").value)
        self.contact_fz_off = float(self.get_parameter("contact_fz_off").value)
        self.contact_cap_scale = float(self.get_parameter("contact_cap_scale").value)
        self.contact_update_mult = float(self.get_parameter("contact_update_mult").value)

        self.use_wrench_obs_param = bool(self.get_parameter("use_wrench_obs").value)

        self.wrench_clip_fxy = float(self.get_parameter("wrench_clip_fxy").value)
        self.wrench_clip_m = float(self.get_parameter("wrench_clip_m").value)
        self.wrench_obs_ema = float(np.clip(float(self.get_parameter("wrench_obs_ema").value), 0.0, 1.0))

        self.fz_min = float(self.get_parameter("fz_min").value)
        self.fz_max = float(self.get_parameter("fz_max").value)

        if not self.ckpt_dir:
            raise RuntimeError("ckpt_dir is empty.")
        if not os.path.isdir(self.ckpt_dir):
            raise FileNotFoundError(f"ckpt_dir not found: {self.ckpt_dir}")

        # -------------------------
        # State buffers
        # -------------------------
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        self._have_pose = False
        self._have_force = False
        self._pose6 = np.zeros(6, dtype=np.float32)      # [x y z rx ry rz]  (mm / rad)
        self._wrench6 = np.zeros(6, dtype=np.float32)    # [Fx Fy Fz Mx My Mz]
        self._wrench6_filt = None                        # EMA filtered

        self._img_top_bgr = None
        self._img_ee_bgr = None
        self._have_img_top = False
        self._have_img_ee = False

        self._contact = False

        # ROS I/O
        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self._cb_pose, 10)
        self.sub_force = self.create_subscription(Float64MultiArray, self.force_topic, self._cb_force, 10)
        self.sub_img_top = self.create_subscription(Image, self.image_topic_top, self._cb_img_top, 10)
        self.sub_img_ee  = self.create_subscription(Image, self.image_topic_ee,  self._cb_img_ee,  10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # Device / policy
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"[INFO] Using device: {self.device}")
        self.get_logger().info(f"[INFO] ckpt_dir={self.ckpt_dir}")
        self.get_logger().info(f"[INFO] act_root={self.act_root}")
        self.get_logger().info(f"[INFO] pose_topic={self.pose_topic}, force_topic={self.force_topic}")
        self.get_logger().info(f"[INFO] image_topics=[{self.image_topic_top}, {self.image_topic_ee}]")
        self.get_logger().info(f"[INFO] cmd_topic={self.cmd_topic} @ {self.hz} Hz (dt={1.0/self.hz:.4f}s)")

        self.action_mean, self.action_std = _load_dataset_stats(self.ckpt_dir)
        if self.action_mean is None:
            self.get_logger().warn("[WARN] dataset_stats.pkl missing/invalid -> denormalization disabled (mean=0,std=1).")
        else:
            self.get_logger().info(f"[INFO] Loaded action_mean/std (len={len(self.action_mean)})")

        self.get_logger().info("[INFO] Loading policy (programmatic, no argparse CLI)...")
        self.policy, missing, unexpected = build_policy_and_load_ckpt_programmatic(
            ckpt_dir=self.ckpt_dir,
            act_root=self.act_root,
            device=self.device,
            hidden_dim=self.hidden_dim,
            dim_feedforward=self.dim_feedforward,
            chunk_size=self.chunk_size,
            kl_weight=self.kl_weight,
            camera_names=("cam_top", "cam_ee"),
        )
        self.get_logger().info(f"[INFO] Loaded policy. missing={len(missing)}, unexpected={len(unexpected)}")
        if len(missing) > 0:
            self.get_logger().warn("[WARN] Non-empty missing keys. Verify checkpoint/model config match if behavior is odd.")

        # ✅ model이 기대하는 qpos 차원 자동 감지
        in_dim = _find_first_linear_in_features(self.policy)
        if in_dim is None:
            # 그래도 ACT 기본은 9가 많으니 fallback 9
            in_dim = 9
            self.get_logger().warn("[WARN] Could not detect qpos dim from model. Fallback qpos_in_dim=9.")
        self.qpos_in_dim = int(in_dim)

        # 파라미터 요청(use_wrench_obs)과 모델 입력이 다르면 자동으로 강제
        self.use_wrench_obs = bool(self.use_wrench_obs_param and self.qpos_in_dim == 12)

        self.get_logger().info(f"[INFO] Detected qpos_in_dim={self.qpos_in_dim} -> use_wrench_obs_effective={self.use_wrench_obs}")

        # buffers
        self.pred_buffer = deque(maxlen=max(1, self.temporal_agg_maxlen))
        self.prev_action_norm = None
        self._last_seq_norm = None
        self._last_seq_start_tick = 0

        # command state
        self._sent_first_cmd = False
        self._first_cmd_time = None
        self.prev_cmd = None
        self._vel_step = None
        self._vel_step_ema = None
        self._tick = 0

        self._anchor_ready = False
        self._anchor_offset6 = np.zeros(6, dtype=np.float32)

        self._last_info_t = time.time()

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
        w[2] = float(w[2])  # fz raw
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
    def _denorm(self, a_norm: np.ndarray) -> np.ndarray:
        if self.action_mean is None or self.action_std is None:
            return a_norm
        D = a_norm.shape[0]
        if len(self.action_mean) != D:
            return a_norm
        return a_norm * self.action_std + self.action_mean

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

        # ✅ 핵심: 모델 입력 차원에 맞춰 qpos 구성
        # - qpos_in_dim==9  : [pose6, 0,0,fz]  (기존 학습과 동일)
        # - qpos_in_dim==12 : [pose6, fx,fy,fz,mx,my,mz] (이렇게 학습된 ckpt에서만)
        if self.qpos_in_dim == 12 and self.use_wrench_obs:
            qpos_np = np.concatenate([pose6, np.asarray([fx, fy, fz, mx, my, mz], dtype=np.float32)], axis=0)  # (12,)
        else:
            qpos_np = np.concatenate([pose6, np.asarray([0.0, 0.0, fz], dtype=np.float32)], axis=0)            # (9,)

        top_rgb = _safe_resize_rgb(top_bgr, (self.img_h, self.img_w))
        ee_rgb  = _safe_resize_rgb(ee_bgr,  (self.img_h, self.img_w))
        if top_rgb is None or ee_rgb is None:
            return None

        top_t = torch.from_numpy(top_rgb).permute(2, 0, 1).float() / 255.0
        ee_t  = torch.from_numpy(ee_rgb).permute(2, 0, 1).float() / 255.0
        cams = torch.stack([top_t, ee_t], dim=0)              # (2,3,H,W)
        imgs = cams.unsqueeze(0).to(self.device)              # (1,2,3,H,W)

        if self.use_imnet:
            imgs = imgs.clone()
            imgs[:, 0] = _imagenet_norm(imgs[:, 0])
            imgs[:, 1] = _imagenet_norm(imgs[:, 1])

        qpos = torch.from_numpy(qpos_np).unsqueeze(0).to(self.device)
        return imgs, qpos, pose6, fz

    def _policy_forward_seq(self, imgs: torch.Tensor, qpos: torch.Tensor) -> np.ndarray:
        with torch.no_grad():
            out = self.policy(qpos, imgs)
            action_t = out[0] if isinstance(out, (tuple, list)) else out
        action_t = action_t.detach()

        if action_t.ndim == 3:
            seq = action_t[0]                  # (H,D)
        elif action_t.ndim == 2:
            seq = action_t
        else:
            flat = action_t.reshape(-1)
            seq = flat[:9].view(1, -1)

        return seq.cpu().numpy().astype(np.float32)

    def _temporal_aggregate(self, t_now: int) -> Optional[np.ndarray]:
        if not self.pred_buffer:
            return None

        acc = []
        for (t0, seq) in self.pred_buffer:
            k = t_now - t0
            if 0 <= k < seq.shape[0]:
                acc.append(seq[k])

        if acc:
            return np.mean(np.stack(acc, axis=0), axis=0).astype(np.float32)

        # fallback: last seq
        if self._last_seq_norm is not None:
            k = t_now - self._last_seq_start_tick
            k = int(np.clip(k, 0, self._last_seq_norm.shape[0]-1))
            return self._last_seq_norm[k].astype(np.float32)

        return self.pred_buffer[-1][1][0].astype(np.float32)

    def _ramp_scale(self, t_now: int) -> float:
        if self.startup_ramp_sec <= 1e-6:
            return 1.0
        total_ticks = max(1, int(self.startup_ramp_sec * self.hz))
        return float(np.clip((t_now + 1) / total_ticks, 0.0, 1.0))

    def _publish_cmd(self, cmd9: np.ndarray):
        m = Float64MultiArray()
        m.data = [float(v) for v in cmd9.tolist()]
        self.pub_cmd.publish(m)

    def _update_contact(self, fz: float):
        if (not self._contact) and (fz >= self.contact_fz_on):
            self._contact = True
        elif self._contact and (fz <= self.contact_fz_off):
            self._contact = False

    # -------------------------
    # Main timer loop
    # -------------------------
    def _on_timer(self):
        pack = self._prepare_inputs()
        if pack is None:
            return
        imgs, qpos, current_pose6, current_fz = pack

        self._update_contact(current_fz)

        # FIRST PUBLISH
        if not self._sent_first_cmd:
            cmd0 = np.zeros(9, dtype=np.float32)
            cmd0[:6] = current_pose6
            cmd0[6] = 0.0
            cmd0[7] = 0.0
            cmd0[8] = _clip(current_fz, self.fz_min, self.fz_max)

            self.prev_cmd = cmd0.copy()
            self._vel_step = np.zeros(9, dtype=np.float32)
            self._vel_step_ema = np.zeros(9, dtype=np.float32)
            self._tick = 0

            self.pred_buffer.clear()
            self.prev_action_norm = None
            self._last_seq_norm = None
            self._last_seq_start_tick = 0

            self._anchor_ready = False
            self._anchor_offset6[:] = 0.0

            self._sent_first_cmd = True
            self._first_cmd_time = time.time()

            self._publish_cmd(cmd0)
            self.get_logger().info(
                f"[START] First cmd=current. cmd=[{cmd0[0]:.3f},{cmd0[1]:.3f},{cmd0[2]:.3f},"
                f"{cmd0[3]:.4f},{cmd0[4]:.4f},{cmd0[5]:.4f},0,0,{cmd0[8]:.3f}]"
            )
            return

        # warmup hold
        if self.warmup_sec > 1e-6 and (time.time() - self._first_cmd_time) < self.warmup_sec:
            self._publish_cmd(self.prev_cmd)
            return

        # inference
        do_infer = (self._tick % self.inference_every == 0) or (self._last_seq_norm is None)
        if do_infer:
            try:
                pred_seq_norm = self._policy_forward_seq(imgs, qpos)
            except Exception as e:
                self.get_logger().error(f"Policy forward failed: {e}")
                return

            if pred_seq_norm.ndim != 2 or pred_seq_norm.shape[0] < 1:
                self.get_logger().warn("[WARN] pred_seq_norm invalid.")
                return

            D = pred_seq_norm.shape[1]
            if D < 9:
                pad = np.zeros((pred_seq_norm.shape[0], 9), dtype=np.float32)
                pad[:, :D] = pred_seq_norm
                pred_seq_norm = pad

            self.pred_buffer.append((self._tick, pred_seq_norm))
            self._last_seq_norm = pred_seq_norm
            self._last_seq_start_tick = self._tick

        # choose action at tick
        if self.use_temporal_agg:
            agg_norm = self._temporal_aggregate(self._tick)
        else:
            k = self._tick - self._last_seq_start_tick
            k = int(np.clip(k, 0, self._last_seq_norm.shape[0]-1))
            agg_norm = self._last_seq_norm[k]

        if agg_norm is None:
            return

        # light norm smoothing
        if self.prev_action_norm is None:
            smooth_norm = agg_norm
        else:
            a = float(np.clip(self.smooth_alpha, 0.0, 1.0))
            smooth_norm = a * agg_norm + (1.0 - a) * self.prev_action_norm
        self.prev_action_norm = smooth_norm

        action = self._denorm(smooth_norm.astype(np.float32))
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] < 9:
            pad = np.zeros(9, dtype=np.float32)
            pad[: action.shape[0]] = action
            action = pad
        else:
            action = action[:9]

        cmd_target = np.zeros(9, dtype=np.float32)
        cmd_target[:6] = action[:6]
        cmd_target[6] = 0.0
        cmd_target[7] = 0.0
        cmd_target[8] = _clip(float(action[8]), self.fz_min, self.fz_max)

        # anchor
        if self.anchor_to_current and (not self._anchor_ready):
            self._anchor_offset6 = (current_pose6 - cmd_target[:6]).astype(np.float32)
            self._anchor_ready = True
            self.get_logger().info("[ANCHOR] pose offset initialized (current - pred)")

        if self.anchor_to_current and self._anchor_ready:
            cmd_target[:6] = cmd_target[:6] + self._anchor_offset6

        # caps + contact scaling
        ramp = self._ramp_scale(self._tick)
        cap_pos = max(1e-6, self.pos_step_cap_mm * ramp)
        cap_ang = max(1e-8, self.ang_step_cap_rad * ramp)
        cap_fz  = max(1e-6, self.fz_step_cap * ramp)

        update_every = self.target_update_every
        if self._contact:
            cap_pos *= self.contact_cap_scale
            cap_ang *= self.contact_cap_scale
            cap_fz  *= self.contact_cap_scale
            update_every = int(max(1, round(update_every * self.contact_update_mult)))

        prev = self.prev_cmd.copy()
        need_update = (self._vel_step is None) or (update_every <= 1) or (self._tick % update_every == 0)

        if need_update:
            step_raw = np.zeros(9, dtype=np.float32)
            for i in range(3):
                d = float(cmd_target[i] - prev[i])
                step_raw[i] = d if abs(d) <= cap_pos else (_sign(d) * cap_pos)
            for i in range(3, 6):
                d = float(cmd_target[i] - prev[i])
                step_raw[i] = d if abs(d) <= cap_ang else (_sign(d) * cap_ang)
            step_raw[6] = 0.0
            step_raw[7] = 0.0
            d = float(cmd_target[8] - prev[8])
            step_raw[8] = d if abs(d) <= cap_fz else (_sign(d) * cap_fz)

            if self._vel_step_ema is None:
                self._vel_step_ema = step_raw.copy()
            else:
                b = self.step_ema_beta
                self._vel_step_ema = b * step_raw + (1.0 - b) * self._vel_step_ema

            self._vel_step = self._vel_step_ema.copy()

        cmd_next = prev + self._vel_step

        # overshoot snap
        for i in range(9):
            if i in (6, 7):
                cmd_next[i] = 0.0
                continue
            a0, a1, tg = float(prev[i]), float(cmd_next[i]), float(cmd_target[i])
            if (a0 - tg) * (a1 - tg) < 0.0:
                cmd_next[i] = tg

        cmd_next[8] = _clip(float(cmd_next[8]), self.fz_min, self.fz_max)

        self._publish_cmd(cmd_next)
        self.prev_cmd = cmd_next
        self._tick += 1

        # log 1Hz
        t = time.time()
        if t - self._last_info_t > 1.0:
            self._last_info_t = t
            self.get_logger().info(
                f"contact={1 if self._contact else 0} | infer={'Y' if do_infer else 'N'} | "
                f"cmd=[{cmd_next[0]:.3f},{cmd_next[1]:.3f},{cmd_next[2]:.3f},"
                f"{cmd_next[3]:.4f},{cmd_next[4]:.4f},{cmd_next[5]:.4f},0,0,{cmd_next[8]:.3f}]"
            )


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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
