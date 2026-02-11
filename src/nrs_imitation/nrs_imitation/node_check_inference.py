#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import pickle
from typing import Optional, Tuple

import numpy as np
import torch
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
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


def _find_first_linear_in_features(model: torch.nn.Module) -> Optional[int]:
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
    camera_names=("cam_top", "cam_ee"),
):
    _ensure_act_paths(act_root)

    from act.detr.models.detr_vae import build as build_act_model

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

    ckpt = torch.load(ckpt_path, map_location=device)
    state_dict = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt
    missing, unexpected = model.load_state_dict(state_dict, strict=False)
    return model, missing, unexpected


# ============================================================
# Node
# ============================================================

class ActCheckInferenceNode(Node):
    def __init__(self):
        super().__init__("act_check_inference_node")

        # Params (match infer node)
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("robot_name", "ur10skku")
        self.declare_parameter("act_root", "/home/eunseop/nrs_lab2/nrs_act")

        self.declare_parameter("pose_topic", "")
        self.declare_parameter("force_topic", "")
        self.declare_parameter("image_topic_top", "/realsense/top/color/image_raw")
        self.declare_parameter("image_topic_ee",  "/realsense/ee/color/image_raw")

        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("chunk_size", 100)
        self.declare_parameter("kl_weight", 10.0)

        self.declare_parameter("img_h", 480)
        self.declare_parameter("img_w", 640)
        self.declare_parameter("use_imagenet_norm", True)

        self.declare_parameter("use_wrench_obs", False)  # will be auto-adjusted by detected qpos dim
        self.declare_parameter("wrench_clip_fxy", 20.0)
        self.declare_parameter("wrench_clip_m", 2.0)
        self.declare_parameter("wrench_obs_ema", 0.2)

        self.declare_parameter("fz_min", 0.0)
        self.declare_parameter("fz_max", 30.0)

        # Behavior
        self.declare_parameter("auto_once", True)            # if True: run once when ready, then shutdown
        self.declare_parameter("save_dir", "/tmp/act_check") # outputs
        self.declare_parameter("print_k", 10)                # print first/last K steps

        # Read
        self.ckpt_dir = str(self.get_parameter("ckpt_dir").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.act_root = str(self.get_parameter("act_root").value)

        pose_topic = str(self.get_parameter("pose_topic").value).strip()
        force_topic = str(self.get_parameter("force_topic").value).strip()
        self.pose_topic = pose_topic if pose_topic else f"/{self.robot_name}/currentP"
        self.force_topic = force_topic if force_topic else f"/{self.robot_name}/currentF"

        self.image_topic_top = str(self.get_parameter("image_topic_top").value)
        self.image_topic_ee = str(self.get_parameter("image_topic_ee").value)

        self.hidden_dim = int(self.get_parameter("hidden_dim").value)
        self.dim_feedforward = int(self.get_parameter("dim_feedforward").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.kl_weight = float(self.get_parameter("kl_weight").value)

        self.img_h = int(self.get_parameter("img_h").value)
        self.img_w = int(self.get_parameter("img_w").value)
        self.use_imnet = bool(self.get_parameter("use_imagenet_norm").value)

        self.use_wrench_obs_param = bool(self.get_parameter("use_wrench_obs").value)
        self.wrench_clip_fxy = float(self.get_parameter("wrench_clip_fxy").value)
        self.wrench_clip_m = float(self.get_parameter("wrench_clip_m").value)
        self.wrench_obs_ema = float(np.clip(float(self.get_parameter("wrench_obs_ema").value), 0.0, 1.0))

        self.fz_min = float(self.get_parameter("fz_min").value)
        self.fz_max = float(self.get_parameter("fz_max").value)

        self.auto_once = bool(self.get_parameter("auto_once").value)
        self.save_dir = str(self.get_parameter("save_dir").value)
        self.print_k = int(self.get_parameter("print_k").value)

        if not self.ckpt_dir or not os.path.isdir(self.ckpt_dir):
            raise FileNotFoundError(f"ckpt_dir not found: {self.ckpt_dir}")

        os.makedirs(self.save_dir, exist_ok=True)

        # State buffers
        self.bridge = CvBridge()
        self._have_pose = False
        self._have_force = False
        self._have_img_top = False
        self._have_img_ee = False

        self._pose6 = np.zeros(6, dtype=np.float32)
        self._wrench6 = np.zeros(6, dtype=np.float32)
        self._wrench6_filt = None

        self._img_top_bgr = None
        self._img_ee_bgr = None

        # ROS
        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self._cb_pose, 10)
        self.sub_force = self.create_subscription(Float64MultiArray, self.force_topic, self._cb_force, 10)
        self.sub_img_top = self.create_subscription(Image, self.image_topic_top, self._cb_img_top, 10)
        self.sub_img_ee  = self.create_subscription(Image, self.image_topic_ee,  self._cb_img_ee,  10)

        # Trigger service (for "mid-position check")
        self.srv = self.create_service(Trigger, "check_inference", self._on_trigger)

        # Load policy
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"[INFO] device={self.device}")
        self.get_logger().info(f"[INFO] ckpt_dir={self.ckpt_dir}")
        self.get_logger().info(f"[INFO] topics pose={self.pose_topic}, force={self.force_topic}")
        self.get_logger().info(f"[INFO] image_topics top={self.image_topic_top}, ee={self.image_topic_ee}")
        self.get_logger().info("[INFO] Loading policy...")

        self.action_mean, self.action_std = _load_dataset_stats(self.ckpt_dir)
        if self.action_mean is None:
            self.get_logger().warn("[WARN] dataset_stats.pkl missing -> denorm disabled.")

        self.policy, missing, unexpected = build_policy_and_load_ckpt_programmatic(
            ckpt_dir=self.ckpt_dir,
            act_root=self.act_root,
            device=self.device,
            hidden_dim=self.hidden_dim,
            dim_feedforward=self.dim_feedforward,
            chunk_size=self.chunk_size,
            kl_weight=self.kl_weight,
        )
        self.get_logger().info(f"[INFO] Loaded policy. missing={len(missing)}, unexpected={len(unexpected)}")

        in_dim = _find_first_linear_in_features(self.policy)
        if in_dim is None:
            in_dim = 9
            self.get_logger().warn("[WARN] qpos dim detect failed -> fallback 9")
        self.qpos_in_dim = int(in_dim)
        self.use_wrench_obs = bool(self.use_wrench_obs_param and self.qpos_in_dim == 12)
        self.get_logger().info(f"[INFO] Detected qpos_in_dim={self.qpos_in_dim}, use_wrench_obs_effective={self.use_wrench_obs}")

        # Auto once timer
        self._auto_done = False
        self.timer = self.create_timer(0.1, self._auto_spin_once)

        self.get_logger().info("✅ Ready. Waiting for topics... (service: /check_inference)")

    # -------------------------
    # Callbacks
    # -------------------------
    def _cb_pose(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
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
            self._img_top_bgr = img
            self._have_img_top = True
        except Exception as e:
            self.get_logger().error(f"Top image convert error: {e}")

    def _cb_img_ee(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._img_ee_bgr = img
            self._have_img_ee = True
        except Exception as e:
            self.get_logger().error(f"EE image convert error: {e}")

    # -------------------------
    # Inference core
    # -------------------------
    def _ready(self) -> bool:
        return self._have_pose and self._have_force and self._have_img_top and self._have_img_ee

    def _build_inputs(self) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray, np.ndarray]:
        pose6 = self._pose6.copy()
        w6 = (self._wrench6_filt.copy() if self._wrench6_filt is not None else self._wrench6.copy())
        top_bgr = self._img_top_bgr.copy()
        ee_bgr  = self._img_ee_bgr.copy()

        fx, fy, fz, mx, my, mz = [float(v) for v in w6.tolist()]

        if self.qpos_in_dim == 12 and self.use_wrench_obs:
            qpos_np = np.concatenate([pose6, np.asarray([fx, fy, fz, mx, my, mz], dtype=np.float32)], axis=0)
        else:
            qpos_np = np.concatenate([pose6, np.asarray([0.0, 0.0, fz], dtype=np.float32)], axis=0)

        top_rgb = _safe_resize_rgb(top_bgr, (self.img_h, self.img_w))
        ee_rgb  = _safe_resize_rgb(ee_bgr,  (self.img_h, self.img_w))
        if top_rgb is None or ee_rgb is None:
            raise RuntimeError("Image preprocessing failed")

        top_t = torch.from_numpy(top_rgb).permute(2, 0, 1).float() / 255.0
        ee_t  = torch.from_numpy(ee_rgb).permute(2, 0, 1).float() / 255.0
        cams = torch.stack([top_t, ee_t], dim=0)
        imgs = cams.unsqueeze(0).to(self.device)

        if self.use_imnet:
            imgs = imgs.clone()
            imgs[:, 0] = _imagenet_norm(imgs[:, 0])
            imgs[:, 1] = _imagenet_norm(imgs[:, 1])

        qpos = torch.from_numpy(qpos_np).unsqueeze(0).to(self.device)
        return imgs, qpos, pose6, w6

    def _denorm(self, a_norm: np.ndarray) -> np.ndarray:
        if self.action_mean is None or self.action_std is None:
            return a_norm
        D = a_norm.shape[-1]
        if len(self.action_mean) != D:
            return a_norm
        return a_norm * self.action_std + self.action_mean

    def _run_inference_and_save(self, tag: str) -> str:
        imgs, qpos, pose6, w6 = self._build_inputs()
        fx, fy, fz, mx, my, mz = [float(v) for v in w6.tolist()]

        with torch.no_grad():
            out = self.policy(qpos, imgs)
            action_t = out[0] if isinstance(out, (tuple, list)) else out

        # expected shape (B, T, 9) or (T,9) ...
        if action_t.ndim == 3:
            seq_norm = action_t[0].detach().cpu().numpy().astype(np.float32)
        elif action_t.ndim == 2:
            seq_norm = action_t.detach().cpu().numpy().astype(np.float32)
        else:
            flat = action_t.reshape(-1).detach().cpu().numpy().astype(np.float32)
            seq_norm = flat[:9].reshape(1, 9)

        # pad/truncate to (T,9)
        if seq_norm.shape[1] < 9:
            pad = np.zeros((seq_norm.shape[0], 9), dtype=np.float32)
            pad[:, :seq_norm.shape[1]] = seq_norm
            seq_norm = pad
        else:
            seq_norm = seq_norm[:, :9]

        seq = self._denorm(seq_norm)

        # basic sanity metrics
        # (1) overall movement range
        xyz = seq[:, 0:3]
        rpy = seq[:, 3:6]
        fz_seq = seq[:, 8]
        rng_xyz = (xyz.max(axis=0) - xyz.min(axis=0)).tolist()
        rng_rpy = (rpy.max(axis=0) - rpy.min(axis=0)).tolist()
        rng_fz = float(fz_seq.max() - fz_seq.min())

        # Save files
        ts = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.save_dir, f"{ts}_{tag}")
        npy_path = base + "_seq.npy"
        txt_path = base + "_seq.txt"
        png_path = base + "_plot.png"
        meta_path = base + "_meta.txt"

        np.save(npy_path, seq.astype(np.float32))
        np.savetxt(txt_path, seq.astype(np.float32), fmt="%.6f")

        # Plot
        t = np.arange(seq.shape[0], dtype=np.int32)

        fig = plt.figure(figsize=(14, 10))
        ax1 = plt.subplot(3, 1, 1)
        ax1.plot(t, seq[:, 0], label="x(mm)")
        ax1.plot(t, seq[:, 1], label="y(mm)")
        ax1.plot(t, seq[:, 2], label="z(mm)")
        ax1.grid(True); ax1.legend()

        ax2 = plt.subplot(3, 1, 2)
        ax2.plot(t, seq[:, 3], label="rx(rad)")
        ax2.plot(t, seq[:, 4], label="ry(rad)")
        ax2.plot(t, seq[:, 5], label="rz(rad)")
        ax2.grid(True); ax2.legend()

        ax3 = plt.subplot(3, 1, 3)
        ax3.plot(t, seq[:, 8], label="Fz(N)")
        ax3.grid(True); ax3.legend()

        plt.tight_layout()
        plt.savefig(png_path, dpi=160)
        plt.close(fig)

        # Meta
        with open(meta_path, "w") as f:
            f.write(f"tag={tag}\n")
            f.write(f"qpos_in_dim={self.qpos_in_dim}\n")
            f.write(f"use_wrench_obs_effective={self.use_wrench_obs}\n")
            f.write(f"pose6_current={pose6.tolist()}\n")
            f.write(f"wrench6_filt=[{fx},{fy},{fz},{mx},{my},{mz}]\n")
            f.write(f"range_xyz(mm)={rng_xyz}\n")
            f.write(f"range_rpy(rad)={rng_rpy}\n")
            f.write(f"range_fz(N)={rng_fz}\n")
            f.write(f"seq_shape={seq.shape}\n")
            f.write(f"saved_npy={npy_path}\n")
            f.write(f"saved_txt={txt_path}\n")
            f.write(f"saved_png={png_path}\n")

        # Print summary to console
        k = max(1, min(self.print_k, seq.shape[0] // 2))
        self.get_logger().info(f"[{tag}] current pose6={pose6.tolist()}, current fz={fz:.3f}")
        self.get_logger().info(f"[{tag}] seq shape={seq.shape} | range_xyz(mm)={rng_xyz} | range_rpy(rad)={rng_rpy} | range_fz(N)={rng_fz:.3f}")

        self.get_logger().info(f"[{tag}] first {k} steps (x y z rx ry rz ... fz):")
        for i in range(k):
            s = seq[i]
            self.get_logger().info(f"  t={i:03d}: [{s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f}, {s[3]:.5f}, {s[4]:.5f}, {s[5]:.5f}, fz={s[8]:.3f}]")

        self.get_logger().info(f"[{tag}] last {k} steps:")
        for i in range(seq.shape[0]-k, seq.shape[0]):
            s = seq[i]
            self.get_logger().info(f"  t={i:03d}: [{s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f}, {s[3]:.5f}, {s[4]:.5f}, {s[5]:.5f}, fz={s[8]:.3f}]")

        self.get_logger().info(f"[{tag}] Saved plot: {png_path}")
        return png_path

    # -------------------------
    # Modes
    # -------------------------
    def _auto_spin_once(self):
        if not self.auto_once or self._auto_done:
            return
        if not self._ready():
            return

        self._auto_done = True
        try:
            self._run_inference_and_save(tag="auto_once")
        except Exception as e:
            self.get_logger().error(f"Auto inference failed: {e}")
        finally:
            self.get_logger().info("Auto-once done. Shutting down...")
            rclpy.shutdown()

    def _on_trigger(self, request, response):
        if not self._ready():
            response.success = False
            response.message = "Not ready: waiting for pose/force/images"
            return response
        try:
            png_path = self._run_inference_and_save(tag="trigger")
            response.success = True
            response.message = f"Saved: {png_path}"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ActCheckInferenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
