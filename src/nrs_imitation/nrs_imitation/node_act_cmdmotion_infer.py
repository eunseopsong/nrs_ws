#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading
import pickle
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

def _now_str() -> str:
    return time.strftime("%m%d_%H%M%S")


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
            std = np.asarray(stats["action_std"], dtype=np.float32)
            std = np.maximum(std, 1e-6)  # divide-by-zero guard
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


def _safe_resize_rgb(img_bgr: np.ndarray, out_hw: Tuple[int, int]) -> np.ndarray:
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


def _clip_step(target: float, current: float, max_step: float) -> float:
    return current + float(np.clip(target - current, -max_step, +max_step))


# ============================================================
# Policy Loader (NO argparse / NO training script import)
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

    # build()가 기대하는 args를 Namespace 형태로 구성 (학습 CLI parser 절대 안 씀)
    import argparse
    args = argparse.Namespace(
        # 아래 값들은 build() 호환용 (대부분은 build에서만 사용)
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
        num_queries=int(chunk_size),         # == H
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
    def __init__(self):
        super().__init__("act_cmdmotion_infer_node")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("robot_name", "ur10skku")
        self.declare_parameter("hz", 25.0)

        # ACT code root (so you don't need export PYTHONPATH)
        self.declare_parameter("act_root", "/home/eunseop/nrs_lab2/nrs_act")

        # Topics (override 가능)
        self.declare_parameter("pose_topic", "")     # default: /<robot_name>/currentP
        self.declare_parameter("force_topic", "")    # default: /<robot_name>/currentF
        self.declare_parameter("cmd_topic", "")      # default: /<robot_name>/cmdMotion
        self.declare_parameter("image_topic_top", "/realsense/top/color/image_raw")
        self.declare_parameter("image_topic_ee",  "/realsense/ee/color/image_raw")

        # Model hyperparams
        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("chunk_size", 100)
        self.declare_parameter("kl_weight", 10.0)

        # Image preprocess
        self.declare_parameter("img_h", 480)
        self.declare_parameter("img_w", 640)
        self.declare_parameter("use_imagenet_norm", True)

        # Smoothing (normalized action space EMA)
        self.declare_parameter("smooth_alpha", 0.4)  # 0~1 (1=노스무딩)

        # Safety clamp (현재값 대비 변화량 제한)
        self.declare_parameter("max_pos_step_mm", 1.5)      # mm per tick @25Hz
        self.declare_parameter("max_rot_step_rad", 0.02)    # rad per tick
        self.declare_parameter("max_fz_step", 1.0)          # N per tick

        # Force clamp
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

        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)

        self.max_pos_step_mm = float(self.get_parameter("max_pos_step_mm").value)
        self.max_rot_step_rad = float(self.get_parameter("max_rot_step_rad").value)
        self.max_fz_step = float(self.get_parameter("max_fz_step").value)

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
        self._pose6 = np.zeros(6, dtype=np.float32)      # [x y z wx wy wz]
        self._force3 = np.zeros(3, dtype=np.float32)     # [fx fy fz] but fx,fy will be forced 0

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
        self.get_logger().info(f"[INFO] cmd_topic={self.cmd_topic} @ {self.hz} Hz")

        # dataset stats (action denorm)
        self.action_mean, self.action_std = _load_dataset_stats(self.ckpt_dir)
        if self.action_mean is None:
            self.get_logger().warn("[WARN] dataset_stats.pkl missing or invalid -> denormalization disabled.")
        else:
            self.get_logger().info(f"[INFO] Loaded action_mean/std (len={len(self.action_mean)})")

        # policy load
        self.get_logger().info("[INFO] Loading policy (programmatic, no argparse)...")
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

        # smoothing in normalized space
        self.prev_action_norm = None  # (D,)

        # timer
        self.timer = self.create_timer(1.0 / self.hz, self._on_timer)

        # throttle
        self._last_info_t = time.time()

        self.get_logger().info("✅ Model ready for inference! (publishing /cmdMotion)")

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
        # 입력으로는 fx,fy=0 강제, fz만 사용 (데이터셋과 맞춤)
        fx = 0.0
        fy = 0.0
        fz = float(msg.data[2])
        with self._lock:
            self._force3[:] = np.asarray([fx, fy, fz], dtype=np.float32)
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
    # Inference helpers
    # -------------------------
    def _denorm_action(self, a_norm: np.ndarray) -> np.ndarray:
        if self.action_mean is None or self.action_std is None:
            return a_norm
        D = a_norm.shape[0]
        if len(self.action_mean) != D:
            # 차원 불일치면 denorm 비활성화
            return a_norm
        return a_norm * self.action_std + self.action_mean

    def _prepare_inputs(self) -> Optional[Tuple[torch.Tensor, torch.Tensor, np.ndarray]]:
        """
        return:
          imgs_tensor: (1,2,3,H,W)
          qpos_tensor: (1,9)
          current_qpos_np: (9,)  (for safety clamp reference)
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

        # qpos = [pose6, force3]  (9,)
        qpos_np = np.concatenate([pose6, force3], axis=0).astype(np.float32)

        # images -> RGB, resize
        top_rgb = _safe_resize_rgb(top_bgr, (self.img_h, self.img_w))
        ee_rgb  = _safe_resize_rgb(ee_bgr,  (self.img_h, self.img_w))
        if top_rgb is None or ee_rgb is None:
            return None

        # (3,H,W) float [0,1]
        top_t = torch.from_numpy(top_rgb).permute(2, 0, 1).float() / 255.0
        ee_t  = torch.from_numpy(ee_rgb).permute(2, 0, 1).float() / 255.0

        # stack cams in correct order: [top, ee]
        cams = torch.stack([top_t, ee_t], dim=0)  # (2,3,H,W)
        imgs = cams.unsqueeze(0).to(self.device)  # (1,2,3,H,W)

        if self.use_imnet:
            # apply per-camera ImageNet norm
            imgs = imgs.clone()
            imgs[:, 0] = _imagenet_norm(imgs[:, 0])
            imgs[:, 1] = _imagenet_norm(imgs[:, 1])

        qpos = torch.from_numpy(qpos_np).unsqueeze(0).to(self.device)  # (1,9)

        return imgs, qpos, qpos_np

    def _policy_forward_one(self, imgs: torch.Tensor, qpos: torch.Tensor) -> np.ndarray:
        """
        return action_norm_vec: (D,)
        """
        with torch.no_grad():
            out = self.policy(qpos, imgs)
            if isinstance(out, (tuple, list)):
                action_t = out[0]
            else:
                action_t = out

        # 다양한 shape 대응
        # - (B,H,D) -> take [0,0]
        # - (B,D)   -> take [0]
        # - (H,D)   -> take [0]
        # - (flat)  -> reshape
        action_t = action_t.detach()

        if action_t.ndim == 3:
            a = action_t[0, 0, :]
        elif action_t.ndim == 2:
            # could be (B,D) or (H,D)
            a = action_t[0, :]
        elif action_t.ndim == 1:
            a = action_t
        else:
            # fallback: flatten then try reshape as (H,D)
            flat = action_t.reshape(-1)
            # try infer D=9 by default
            D_guess = 9
            if flat.numel() % D_guess == 0:
                a = flat.view(-1, D_guess)[0]
            else:
                a = flat[:D_guess]

        return a.cpu().numpy().astype(np.float32)

    # -------------------------
    # Main timer loop
    # -------------------------
    def _on_timer(self):
        pack = self._prepare_inputs()
        if pack is None:
            return
        imgs, qpos, current_qpos = pack

        # forward
        try:
            action_norm = self._policy_forward_one(imgs, qpos)  # (D,)
        except Exception as e:
            self.get_logger().error(f"Policy forward failed: {e}")
            return

        # smoothing in norm space
        if self.prev_action_norm is None:
            smoothed_norm = action_norm
        else:
            a = float(self.smooth_alpha)
            smoothed_norm = a * action_norm + (1.0 - a) * self.prev_action_norm
        self.prev_action_norm = smoothed_norm

        # denorm
        action = self._denorm_action(smoothed_norm)  # (D,)
        action = np.asarray(action, dtype=np.float32)

        # ensure length >= 9
        if action.shape[0] < 9:
            pad = np.zeros(9, dtype=np.float32)
            pad[: action.shape[0]] = action
            action = pad
        else:
            action = action[:9]

        # command = [x,y,z,wx,wy,wz,fx,fy,fz]
        cmd = action.copy()

        # fx, fy always 0
        cmd[6] = 0.0
        cmd[7] = 0.0

        # clamp fz range
        cmd[8] = float(np.clip(cmd[8], self.fz_min, self.fz_max))

        # -------------------------
        # Safety: clamp step vs current
        # current_qpos: [pose6, force3] where force3 already fx=fy=0, fz=measured
        # We clamp pose and fz only (fx/fy forced 0)
        # -------------------------
        cur_x, cur_y, cur_z = float(current_qpos[0]), float(current_qpos[1]), float(current_qpos[2])
        cur_wx, cur_wy, cur_wz = float(current_qpos[3]), float(current_qpos[4]), float(current_qpos[5])
        cur_fz = float(current_qpos[8])

        cmd[0] = _clip_step(cmd[0], cur_x, self.max_pos_step_mm)
        cmd[1] = _clip_step(cmd[1], cur_y, self.max_pos_step_mm)
        cmd[2] = _clip_step(cmd[2], cur_z, self.max_pos_step_mm)

        cmd[3] = _clip_step(cmd[3], cur_wx, self.max_rot_step_rad)
        cmd[4] = _clip_step(cmd[4], cur_wy, self.max_rot_step_rad)
        cmd[5] = _clip_step(cmd[5], cur_wz, self.max_rot_step_rad)

        cmd[8] = _clip_step(cmd[8], cur_fz, self.max_fz_step)
        cmd[8] = float(np.clip(cmd[8], self.fz_min, self.fz_max))

        # publish
        m = Float64MultiArray()
        m.data = [float(v) for v in cmd.tolist()]
        self.pub_cmd.publish(m)

        # throttled info log
        t = time.time()
        if t - self._last_info_t > 1.0:
            self._last_info_t = t
            self.get_logger().info(
                f"cmd=[{cmd[0]:.3f},{cmd[1]:.3f},{cmd[2]:.3f},"
                f"{cmd[3]:.4f},{cmd[4]:.4f},{cmd[5]:.4f},0,0,{cmd[8]:.3f}]"
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
