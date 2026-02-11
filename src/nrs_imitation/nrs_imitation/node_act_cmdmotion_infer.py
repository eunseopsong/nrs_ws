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
    """
    act_root 예: /home/eunseop/nrs_lab2/nrs_act
    이 경로를 sys.path에 넣어서 `from act.detr.models.detr_vae import build`가 되게 만든다.
    """
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
    """
    ckpt_dir/dataset_stats.pkl에서 action_mean, action_std 로드
    """
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
    """
    t: (3,H,W), range [0,1], RGB
    """
    mean = torch.tensor([0.485, 0.456, 0.406], dtype=t.dtype, device=t.device)[:, None, None]
    std  = torch.tensor([0.229, 0.224, 0.225], dtype=t.dtype, device=t.device)[:, None, None]
    return (t - mean) / std


def _safe_resize_rgb(img_bgr: np.ndarray, out_hw: Tuple[int, int]) -> Optional[np.ndarray]:
    """
    img_bgr -> RGB uint8, resize to (H,W)
    out_hw: (H,W)
    """
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


# ============================================================
# Policy Loader (NO argparse CLI / NO training script)
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
    """
    - act_root를 sys.path에 올린 후
    - act.detr.models.detr_vae.build()로 모델을 구성하고
    - ckpt_dir/policy_best.ckpt의 state_dict를 로드한다.
    """
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
        num_queries=int(chunk_size),     # == H
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
# ROS2 Node
# ============================================================

class ActCmdMotionInferNode(Node):
    """
    요구사항:
      1) 첫 publish 값은 current 값과 동일
      2) 이후는 temporal-agg + 매우 느린 이동 (QP-safe)
      3) 이동 과정에서 (구간 내) 가속도/저크 = 0  => step(=속도) 고정 유지, 일정 주기마다만 갱신
         - target_update_every ticks 동안은 "속도(step)"를 고정
    """

    def __init__(self):
        super().__init__("act_cmdmotion_infer_node")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("robot_name", "ur10skku")
        self.declare_parameter("hz", 25.0)
        self.declare_parameter("act_root", "/home/eunseop/nrs_lab2/nrs_act")

        # topics
        self.declare_parameter("pose_topic", "")     # default: /<robot_name>/currentP
        self.declare_parameter("force_topic", "")    # default: /<robot_name>/currentF
        self.declare_parameter("cmd_topic", "")      # default: /<robot_name>/cmdMotion
        self.declare_parameter("image_topic_top", "/realsense/top/color/image_raw")
        self.declare_parameter("image_topic_ee",  "/realsense/ee/color/image_raw")

        # model hyperparams
        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("chunk_size", 100)
        self.declare_parameter("kl_weight", 10.0)

        # image preprocess
        self.declare_parameter("img_h", 480)
        self.declare_parameter("img_w", 640)
        self.declare_parameter("use_imagenet_norm", True)

        # startup behavior
        self.declare_parameter("warmup_sec", 1.0)          # first cmd=current 이후 홀드 시간
        self.declare_parameter("startup_ramp_sec", 2.0)    # step cap scale 0->1 램프
        self.declare_parameter("anchor_to_current", True)  # 첫 inference에서 current-pred 오프셋 고정

        # temporal aggregation
        self.declare_parameter("use_temporal_agg", True)
        self.declare_parameter("temporal_agg_maxlen", 60)  # 버퍼 최대 엔트리 수 (tick 기준)

        # smoothing in norm space (추가 안정화)
        self.declare_parameter("smooth_alpha", 0.4)  # 0~1 (1=노스무딩)

        # step caps (QP-safe, "아주 느리게")
        self.declare_parameter("pos_step_cap_mm", 0.31)     # mm / tick
        self.declare_parameter("ang_step_cap_rad", 0.0004)  # rad / tick
        self.declare_parameter("fz_step_cap", 0.2)          # N / tick

        # "가속도/저크 0"을 위해 step(속도) 고정 유지 주기
        self.declare_parameter("target_update_every", 10)   # ticks (>=2 권장). 이 구간 내 step 고정.

        # force clamp
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

        self.target_update_every = int(self.get_parameter("target_update_every").value)
        self.target_update_every = max(1, self.target_update_every)

        self.fz_min = float(self.get_parameter("fz_min").value)
        self.fz_max = float(self.get_parameter("fz_max").value)

        if not self.ckpt_dir:
            raise RuntimeError("ckpt_dir is empty. Example: -p ckpt_dir:=/home/eunseop/.../20260208_1536")
        if not os.path.isdir(self.ckpt_dir):
            raise FileNotFoundError(f"ckpt_dir not found: {self.ckpt_dir}")

        # -------------------------
        # State buffers
        # -------------------------
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        self._have_pose = False
        self._have_force = False
        self._pose6 = np.zeros(6, dtype=np.float32)      # [x y z rx ry rz] (네 currentP 형태 그대로)
        self._force3 = np.zeros(3, dtype=np.float32)     # [fx fy fz], fx/fy는 0으로 강제

        self._img_top_bgr = None
        self._img_ee_bgr = None
        self._have_img_top = False
        self._have_img_ee = False

        # -------------------------
        # ROS I/O
        # -------------------------
        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self._cb_pose, 10)
        self.sub_force = self.create_subscription(Float64MultiArray, self.force_topic, self._cb_force, 10)
        self.sub_img_top = self.create_subscription(Image, self.image_topic_top, self._cb_img_top, 10)
        self.sub_img_ee  = self.create_subscription(Image, self.image_topic_ee,  self._cb_img_ee,  10)

        self.pub_cmd = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # -------------------------
        # Device / policy / stats
        # -------------------------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"[INFO] Using device: {self.device}")
        self.get_logger().info(f"[INFO] ckpt_dir={self.ckpt_dir}")
        self.get_logger().info(f"[INFO] act_root={self.act_root}")
        self.get_logger().info(f"[INFO] pose_topic={self.pose_topic}, force_topic={self.force_topic}")
        self.get_logger().info(f"[INFO] image_topics=[{self.image_topic_top}, {self.image_topic_ee}]")
        self.get_logger().info(f"[INFO] cmd_topic={self.cmd_topic} @ {self.hz} Hz (dt={1.0/self.hz:.4f}s)")
        self.get_logger().info(f"[SAFE] step caps: pos={self.pos_step_cap_mm}mm, ang={self.ang_step_cap_rad}rad, fz={self.fz_step_cap}N")
        self.get_logger().info(f"[SAFE] target_update_every={self.target_update_every} ticks (step fixed in-between)")

        # dataset stats (action denorm)
        self.action_mean, self.action_std = _load_dataset_stats(self.ckpt_dir)
        if self.action_mean is None:
            self.get_logger().warn("[WARN] dataset_stats.pkl missing/invalid -> denormalization disabled (mean=0,std=1).")
        else:
            self.get_logger().info(f"[INFO] Loaded action_mean/std (len={len(self.action_mean)})")

        # policy load
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

        # -------------------------
        # Temporal agg buffer: store (t0, pred_seq_norm(H,D))
        # -------------------------
        self.pred_buffer = deque(maxlen=max(1, self.temporal_agg_maxlen))

        # smoothing in norm space (after temporal agg)
        self.prev_action_norm = None  # (D,)

        # -------------------------
        # Command state (핵심!)
        # -------------------------
        self._sent_first_cmd = False
        self._first_cmd_time = None

        self.prev_cmd = None          # (9,) last published cmd
        self._vel_step = None         # (9,) fixed step vector for "0-acc/0-jerk within segment"
        self._tick = 0                # inference tick count (after warmup)

        # anchor offset (pose6 only)
        self._anchor_ready = False
        self._anchor_offset6 = np.zeros(6, dtype=np.float32)

        # throttle log
        self._last_info_t = time.time()

        # timer
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
        if len(msg.data) < 3:
            return
        # fx,fy는 0으로 강제, fz만 사용
        fz = float(msg.data[2])
        with self._lock:
            self._force3[:] = np.asarray([0.0, 0.0, fz], dtype=np.float32)
            self._have_force = True

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

    def _prepare_inputs(self) -> Optional[Tuple[torch.Tensor, torch.Tensor, np.ndarray]]:
        """
        return:
          imgs_tensor: (1,2,3,H,W)
          qpos_tensor: (1,9)
          current_qpos_np: (9,)
        """
        with self._lock:
            if not (self._have_pose and self._have_force and self._have_img_top and self._have_img_ee):
                return None
            pose6 = self._pose6.copy()
            force3 = self._force3.copy()
            top_bgr = None if self._img_top_bgr is None else self._img_top_bgr.copy()
            ee_bgr  = None if self._img_ee_bgr  is None else self._img_ee_bgr.copy()

        if top_bgr is None or ee_bgr is None:
            return None

        qpos_np = np.concatenate([pose6, force3], axis=0).astype(np.float32)  # (9,)

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

        qpos = torch.from_numpy(qpos_np).unsqueeze(0).to(self.device)  # (1,9)
        return imgs, qpos, qpos_np

    def _policy_forward_seq(self, imgs: torch.Tensor, qpos: torch.Tensor) -> np.ndarray:
        """
        return pred_seq_norm: (H, D)  (D>=9 가정)
        """
        with torch.no_grad():
            out = self.policy(qpos, imgs)
            action_t = out[0] if isinstance(out, (tuple, list)) else out
        action_t = action_t.detach()

        # 가능한 shape들 처리:
        # (B,H,D) / (H,D) / (B,D) / (flat)
        if action_t.ndim == 3:
            seq = action_t[0]                  # (H,D)
        elif action_t.ndim == 2:
            # (H,D) or (B,D)
            # 여기서는 chunk가 나오면 H가 100 같은 값이므로 H로 해석
            seq = action_t
        elif action_t.ndim == 1:
            flat = action_t.reshape(-1)
            D_guess = 9
            if flat.numel() % D_guess == 0:
                seq = flat.view(-1, D_guess)   # (H,D_guess)
            else:
                seq = flat[:D_guess].view(1, -1)
        else:
            flat = action_t.reshape(-1)
            seq = flat[:9].view(1, -1)

        seq_np = seq.cpu().numpy().astype(np.float32)
        return seq_np

    def _temporal_aggregate(self, t_now: int) -> np.ndarray:
        """
        pred_buffer에 쌓인 (t0, seq(H,D)) 에서
        현재 시점 t_now의 action을 겹치는 예측들 평균으로 만든다.
        """
        if not self.pred_buffer:
            return None

        acc = []
        for (t0, seq) in self.pred_buffer:
            k = t_now - t0
            if 0 <= k < seq.shape[0]:
                acc.append(seq[k])

        if not acc:
            # fallback: 가장 최신 seq의 0번
            return self.pred_buffer[-1][1][0]

        return np.mean(np.stack(acc, axis=0), axis=0).astype(np.float32)

    def _ramp_scale(self, t_now: int) -> float:
        """
        startup_ramp_sec 동안 step cap을 0->1로 선형 스케일.
        """
        if self.startup_ramp_sec <= 1e-6:
            return 1.0
        total_ticks = max(1, int(self.startup_ramp_sec * self.hz))
        return float(np.clip((t_now + 1) / total_ticks, 0.0, 1.0))

    def _publish_cmd(self, cmd9: np.ndarray):
        m = Float64MultiArray()
        m.data = [float(v) for v in cmd9.tolist()]
        self.pub_cmd.publish(m)

    # -------------------------
    # Main timer loop
    # -------------------------
    def _on_timer(self):
        pack = self._prepare_inputs()
        if pack is None:
            return
        imgs, qpos, current_qpos = pack

        current_pose6 = current_qpos[:6].copy()
        current_fz = float(current_qpos[8])

        # ======================================================
        # (1) FIRST PUBLISH = CURRENT (요구사항 #1)
        # ======================================================
        if not self._sent_first_cmd:
            cmd0 = np.zeros(9, dtype=np.float32)
            cmd0[:6] = current_pose6
            cmd0[6] = 0.0
            cmd0[7] = 0.0
            cmd0[8] = _clip(current_fz, self.fz_min, self.fz_max)

            self.prev_cmd = cmd0.copy()
            self._vel_step = np.zeros(9, dtype=np.float32)
            self._tick = 0
            self.pred_buffer.clear()
            self.prev_action_norm = None
            self._anchor_ready = False
            self._anchor_offset6[:] = 0.0

            self._sent_first_cmd = True
            self._first_cmd_time = time.time()

            self._publish_cmd(cmd0)
            self.get_logger().info(
                f"[START] First cmd = current (pose6 + fz). "
                f"cmd=[{cmd0[0]:.3f},{cmd0[1]:.3f},{cmd0[2]:.3f},"
                f"{cmd0[3]:.4f},{cmd0[4]:.4f},{cmd0[5]:.4f},0,0,{cmd0[8]:.3f}]"
            )
            return

        # ======================================================
        # Warmup hold (첫 cmd 유지)
        # ======================================================
        if self.warmup_sec > 1e-6 and (time.time() - self._first_cmd_time) < self.warmup_sec:
            self._publish_cmd(self.prev_cmd)
            return

        # ======================================================
        # (2) Inference: get sequence (H,D)
        # ======================================================
        try:
            pred_seq_norm = self._policy_forward_seq(imgs, qpos)  # (H,D)
        except Exception as e:
            self.get_logger().error(f"Policy forward failed: {e}")
            return

        if pred_seq_norm.ndim != 2 or pred_seq_norm.shape[0] < 1:
            self.get_logger().warn("[WARN] pred_seq_norm has invalid shape.")
            return

        # ensure D>=9
        D = pred_seq_norm.shape[1]
        if D < 9:
            pad = np.zeros((pred_seq_norm.shape[0], 9), dtype=np.float32)
            pad[:, :D] = pred_seq_norm
            pred_seq_norm = pad
            D = 9

        # buffer push
        self.pred_buffer.append((self._tick, pred_seq_norm))

        # ======================================================
        # (temporal agg) aggregated action in norm space
        # ======================================================
        if self.use_temporal_agg:
            agg_norm = self._temporal_aggregate(self._tick)
        else:
            agg_norm = pred_seq_norm[0]

        if agg_norm is None:
            return

        # ======================================================
        # extra smoothing (EMA) in norm space
        # ======================================================
        if self.prev_action_norm is None:
            smooth_norm = agg_norm
        else:
            a = float(np.clip(self.smooth_alpha, 0.0, 1.0))
            smooth_norm = a * agg_norm + (1.0 - a) * self.prev_action_norm
        self.prev_action_norm = smooth_norm

        # ======================================================
        # denorm -> target action (9,)
        # ======================================================
        action = self._denorm(smooth_norm.astype(np.float32))
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] < 9:
            pad = np.zeros(9, dtype=np.float32)
            pad[: action.shape[0]] = action
            action = pad
        else:
            action = action[:9]

        # cmd_target = [pose6, 0,0,fz]
        cmd_target = np.zeros(9, dtype=np.float32)
        cmd_target[:6] = action[:6]
        cmd_target[6] = 0.0
        cmd_target[7] = 0.0
        cmd_target[8] = _clip(float(action[8]), self.fz_min, self.fz_max)

        # ======================================================
        # (anchor) 첫 inference에서 current - pred 고정
        # ======================================================
        if self.anchor_to_current and (not self._anchor_ready):
            self._anchor_offset6 = (current_pose6 - cmd_target[:6]).astype(np.float32)
            self._anchor_ready = True
            self.get_logger().info("[ANCHOR] pose offset initialized (current - pred)")

        if self.anchor_to_current and self._anchor_ready:
            cmd_target[:6] = cmd_target[:6] + self._anchor_offset6

        # ======================================================
        # (3) 매우 느리게, 그리고 "구간 내 accel/jerk=0"
        #     => step(=속도) 벡터를 target_update_every tick 동안 고정
        # ======================================================
        ramp = self._ramp_scale(self._tick)
        cap_pos = self.pos_step_cap_mm * ramp
        cap_ang = self.ang_step_cap_rad * ramp
        cap_fz  = self.fz_step_cap * ramp

        # 너무 작은 cap으로 0이 되는 경우 방지(초반 완전 정지 방지용)
        cap_pos = max(1e-6, cap_pos)
        cap_ang = max(1e-8, cap_ang)
        cap_fz  = max(1e-6, cap_fz)

        prev = self.prev_cmd.copy()

        # 속도(step) 갱신 조건:
        need_update = (self._vel_step is None) or (self.target_update_every <= 1) or (self._tick % self.target_update_every == 0)

        if need_update:
            step = np.zeros(9, dtype=np.float32)

            # pos: "상수 속도"를 위해 보통은 ±cap로 고정 (남은 거리가 cap보다 작으면 그만큼)
            for i in range(3):
                d = float(cmd_target[i] - prev[i])
                if abs(d) <= cap_pos:
                    step[i] = d
                else:
                    step[i] = _sign(d) * cap_pos

            # rot
            for i in range(3, 6):
                d = float(cmd_target[i] - prev[i])
                if abs(d) <= cap_ang:
                    step[i] = d
                else:
                    step[i] = _sign(d) * cap_ang

            # fx, fy: 0 고정
            step[6] = 0.0
            step[7] = 0.0

            # fz
            d = float(cmd_target[8] - prev[8])
            if abs(d) <= cap_fz:
                step[8] = d
            else:
                step[8] = _sign(d) * cap_fz

            self._vel_step = step

        # "고정 속도"로 한 tick 전진
        cmd_next = prev + self._vel_step

        # overshoot 방지: 목표를 지나치면 목표로 스냅
        for i in range(9):
            if i in (6, 7):
                cmd_next[i] = 0.0
                continue
            a0 = float(prev[i])
            a1 = float(cmd_next[i])
            tg = float(cmd_target[i])
            # 같은 방향으로 가다가 목표를 넘으면 tg로
            if (a0 - tg) * (a1 - tg) <= 0.0 and abs(a1 - tg) <= abs(a0 - tg):
                # ok
                pass
            # 목표를 넘어섰다(방향 바뀜)면 tg로 스냅
            if (a0 - tg) * (a1 - tg) < 0.0:
                cmd_next[i] = tg

        # fz clamp
        cmd_next[8] = _clip(float(cmd_next[8]), self.fz_min, self.fz_max)

        # publish + update
        self._publish_cmd(cmd_next)
        self.prev_cmd = cmd_next
        self._tick += 1

        # throttled log
        t = time.time()
        if t - self._last_info_t > 1.0:
            self._last_info_t = t
            self.get_logger().info(
                f"cmd=[{cmd_next[0]:.3f},{cmd_next[1]:.3f},{cmd_next[2]:.3f},"
                f"{cmd_next[3]:.4f},{cmd_next[4]:.4f},{cmd_next[5]:.4f},0,0,{cmd_next[8]:.3f}]"
            )


# ============================================================
# Main
# ============================================================

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
