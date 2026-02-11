#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 ACT inference node (UR10e): (pos6 + force3 + 2cam images) -> (pos6 + force3) cmdMotion

✅ Fixes your current crash:
- Uses the SAME custom policy.py (dict args) API you used for training
- Loads ACT "model-only" checkpoint (policy_best.ckpt) into policy.model
- Avoids Namespace/argparse mismatches that caused:
  - KeyError('kl_weight')
  - AttributeError("'dict' object has no attribute 'hidden_dim'")

✅ Includes your runtime controls:
- normalize_qpos / denorm_action
- temporal aggregation (exp EMA, tau)
- pred_step_offset, target_update_every
- contact_fz_threshold, min_contact_fz_cmd (>= 5N)
- fz_step_cap, z_step_cap_mm
- impact_hold_ticks
- stall detection + force boost
- debug_print_every_n, debug_seq_preview_len

Topics (defaults match your logs):
- pose_topic:  /ur10skku/currentP  (Float64MultiArray, len=6: [x,y,z,r,p,y])
- force_topic: /ur10skku/currentF  (Float64MultiArray, len=3: [fx,fy,fz])
- image_topics: [/realsense/top/color/image_raw, /realsense/ee/color/image_raw] (sensor_msgs/Image)
- cmd_topic:   /ur10skku/cmdMotion (Float64MultiArray, len=9: [x,y,z,r,p,y,fx,fy,fz])

Build/Run:
- put this file in: nrs_imitation/nrs_imitation/node_act_cmdmotion_infer.py
- ensure entry_point console_scripts points to nrs_imitation.node_act_cmdmotion_infer:main
"""

import os
import sys
import glob
import time
import math
import pickle
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2


# -----------------------------
# Helpers
# -----------------------------
def _now_sec() -> float:
    return time.time()


def _clamp(x: float, lo: float, hi: float) -> float:
    return float(max(lo, min(hi, x)))


def _vec(x) -> np.ndarray:
    return np.asarray(x, dtype=np.float64).reshape(-1)


def resolve_ckpt_path(ckpt_dir: str) -> str:
    # training saved: policy_best.ckpt
    p = os.path.join(ckpt_dir, "policy_best.ckpt")
    if os.path.exists(p):
        return p
    # fallback: last epoch ckpt
    eps = sorted(glob.glob(os.path.join(ckpt_dir, "policy_epoch_*_seed_*.ckpt")))
    if len(eps) > 0:
        return eps[-1]
    raise FileNotFoundError(f"[ACT-INFER] No checkpoint found under: {ckpt_dir}")


def load_stats(stats_path: str) -> dict:
    with open(stats_path, "rb") as f:
        stats = pickle.load(f)
    # expect keys: qpos_mean/std, action_mean/std
    return stats


def to_torch_image_rgb01(cv_bgr: np.ndarray) -> torch.Tensor:
    """
    cv_bgr: HxWx3 uint8 (BGR)
    returns: (3,H,W) float32 in [0,1] RGB
    """
    cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
    t = torch.from_numpy(cv_rgb).permute(2, 0, 1).contiguous().float() / 255.0
    return t


def fmt_arr(a: np.ndarray, n: int = 3) -> str:
    a = _vec(a)
    return "[" + ",".join([f"{a[i]:.3f}" for i in range(min(n, a.size))]) + ("" if a.size <= n else ",...") + "]"


# -----------------------------
# Runtime state
# -----------------------------
@dataclass
class ContactState:
    contact: bool = False
    prev_contact: bool = False
    impact_hold_ticks_left: int = 0


# -----------------------------
# Node
# -----------------------------
class ActCmdMotionInferNode(Node):
    def __init__(self):
        super().__init__("act_cmdmotion_infer_node")

        # -----------------------------
        # Params (core)
        # -----------------------------
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("act_root", "")

        self.declare_parameter("device", "cuda")  # "cuda" or "cpu"
        self.declare_parameter("cmd_hz", 25.0)

        self.declare_parameter("pose_topic", "/ur10skku/currentP")
        self.declare_parameter("force_topic", "/ur10skku/currentF")
        self.declare_parameter("cmd_topic", "/ur10skku/cmdMotion")

        self.declare_parameter(
            "image_topics",
            ["/realsense/top/color/image_raw", "/realsense/ee/color/image_raw"],
        )

        # -----------------------------
        # Params (policy config) - 학습 기본값
        # -----------------------------
        self.declare_parameter("chunk_size", 100)       # num_queries
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
        self.declare_parameter("lr", 1e-4)  # build 함수가 요구할 수 있어 둠

        # -----------------------------
        # Params (inference behavior)
        # -----------------------------
        self.declare_parameter("action_type", "absolute")  # "absolute" or "delta"
        self.declare_parameter("normalize_qpos", True)
        self.declare_parameter("denorm_action", True)

        self.declare_parameter("use_temporal_agg", True)
        self.declare_parameter("temporal_agg_mode", "exp")  # exp only
        self.declare_parameter("temporal_agg_tau", 20.0)    # ticks

        self.declare_parameter("pred_step_offset", 1)       # use a_hat[:, offset]
        self.declare_parameter("target_update_every", 1)    # ticks

        # contact / force shaping
        self.declare_parameter("contact_fz_threshold", 5.0)
        self.declare_parameter("min_contact_fz_cmd", 5.0)

        self.declare_parameter("fz_step_cap", 2.0)          # N per tick
        self.declare_parameter("z_step_cap_mm", 0.8)        # mm per tick
        self.declare_parameter("impact_hold_ticks", 10)     # ticks

        # stall boost
        self.declare_parameter("stall_enable", True)
        self.declare_parameter("stall_pred_fz_thr", 1.0)
        self.declare_parameter("stall_force_boost_target", 7.0)
        self.declare_parameter("stall_force_boost_rate", 2.0)  # N/sec
        self.declare_parameter("stall_force_boost_max", 12.0)

        self.declare_parameter("startup_ramp_sec", 0.0)

        # debug
        self.declare_parameter("debug_print_every_n", 25)
        self.declare_parameter("debug_seq_preview_len", 10)

        # -----------------------------
        # Device
        # -----------------------------
        dev = str(self.get_parameter("device").value).lower()
        if dev == "cuda" and torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        self.get_logger().info(f"[INFO] Using device: {self.device}")

        # -----------------------------
        # Resolve paths + sys.path for act_root/custom
        # -----------------------------
        self.ckpt_dir = str(self.get_parameter("ckpt_dir").value)
        self.act_root = str(self.get_parameter("act_root").value)
        if not self.ckpt_dir or not os.path.isdir(self.ckpt_dir):
            raise RuntimeError(f"[ACT-INFER] Invalid ckpt_dir: {self.ckpt_dir}")
        if not self.act_root or not os.path.isdir(self.act_root):
            raise RuntimeError(f"[ACT-INFER] Invalid act_root: {self.act_root}")

        # Your training structure had:
        # act_root/custom/policy.py  (and it appends act_root/act and act_root/act/detr)
        custom_dir = os.path.join(self.act_root, "custom")
        act_dir = os.path.join(self.act_root, "act")
        detr_dir = os.path.join(act_dir, "detr")

        for p in [custom_dir, self.act_root, act_dir, detr_dir]:
            if p not in sys.path:
                sys.path.insert(0, p)

        self.get_logger().info(f"[ACT] sys.path[0:5]={sys.path[0:5]}")

        # -----------------------------
        # Import training-time custom policy.py (dict args)
        # -----------------------------
        try:
            from policy import ACTPolicy as CustomACTPolicy  # noqa
            from policy import CNNMLPPolicy as CustomCNNMLPPolicy  # noqa
            self.CustomACTPolicy = CustomACTPolicy
            self.CustomCNNMLPPolicy = CustomCNNMLPPolicy
        except Exception as e:
            raise RuntimeError(
                "[ACT] Failed to import training-time custom policy.py. "
                "Check act_root/custom exists and is on sys.path.\n"
                f"Error: {e}"
            )

        # -----------------------------
        # Load stats + policy
        # -----------------------------
        stats_path = os.path.join(self.ckpt_dir, "dataset_stats.pkl")
        if not os.path.exists(stats_path):
            raise RuntimeError(f"[ACT] dataset_stats.pkl not found: {stats_path}")
        self.stats = load_stats(stats_path)

        # print stats (like your logs)
        q_mu = _vec(self.stats["qpos_mean"])
        q_sd = _vec(self.stats["qpos_std"])
        a_mu = _vec(self.stats["action_mean"])
        a_sd = _vec(self.stats["action_std"])

        self.get_logger().info(
            "[STATS] "
            f"qpos_mean_xyz={q_mu[0:3].tolist()} qpos_std_xyz_min={float(np.min(q_sd[0:3])):.6f} | "
            f"qpos_mean_rpy={q_mu[3:6].tolist()} qpos_std_rpy_min={float(np.min(q_sd[3:6])):.6f} | "
            f"qpos_mean_frc={q_mu[6:9].tolist()} qpos_std_frc_min={float(np.min(q_sd[6:9])):.6f}"
        )
        self.get_logger().info(
            "[STATS] "
            f"act_mean_xyz={a_mu[0:3].tolist()} act_std_xyz_min={float(np.min(a_sd[0:3])):.6f} | "
            f"act_mean_rpy={a_mu[3:6].tolist()} act_std_rpy_min={float(np.min(a_sd[3:6])):.6f} | "
            f"act_mean_frc={a_mu[6:9].tolist()} act_std_frc_min={float(np.min(a_sd[6:9])):.6f}"
        )

        self.policy = None
        self._load_policy()

        # -----------------------------
        # ROS I/O
        # -----------------------------
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.image_topics = list(self.get_parameter("image_topics").value)

        self.sub_pose = self.create_subscription(Float64MultiArray, self.pose_topic, self.cb_pose, 10)
        self.sub_force = self.create_subscription(Float64MultiArray, self.force_topic, self.cb_force, 10)

        self.bridge = CvBridge()
        self.img_subs = []
        for i, t in enumerate(self.image_topics):
            self.img_subs.append(self.create_subscription(Image, t, lambda msg, k=i: self.cb_image(msg, k), 10))

        self.pub_cmd = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # -----------------------------
        # State buffers
        # -----------------------------
        self.have_pose = False
        self.have_force = False
        self.have_imgs = [False for _ in self.image_topics]

        self.cur_pose6 = np.zeros((6,), dtype=np.float64)
        self.cur_force3 = np.zeros((3,), dtype=np.float64)
        self.cur_imgs = [None for _ in self.image_topics]  # torch (3,H,W)

        self.tick = 0
        self.start_time = _now_sec()

        # anchor offset (current - pred) to align
        self.anchor_inited = False
        self.pose_offset6 = np.zeros((6,), dtype=np.float64)

        # command state
        self.last_cmd9 = None  # np(9,)
        self.temporal_ema9 = None  # EMA of predicted action (selected step)
        self.stall_boost = 0.0

        self.contact_state = ContactState()

        # timer
        hz = float(self.get_parameter("cmd_hz").value)
        self.dt = 1.0 / max(1e-6, hz)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info("[INFO] ✅ Model ready. Waiting for topics...")

    # -----------------------------
    # Policy load
    # -----------------------------
    def build_policy_config(self) -> dict:
        camera_names = ["cam_top", "cam_ee"]

        cfg = {
            "lr": float(self.get_parameter("lr").value),
            "num_queries": int(self.get_parameter("chunk_size").value),
            "kl_weight": float(self.get_parameter("kl_weight").value),
            "hidden_dim": int(self.get_parameter("hidden_dim").value),
            "dim_feedforward": int(self.get_parameter("dim_feedforward").value),
            "lr_backbone": float(self.get_parameter("lr_backbone").value),
            "backbone": str(self.get_parameter("backbone").value),
            "enc_layers": int(self.get_parameter("enc_layers").value),
            "dec_layers": int(self.get_parameter("dec_layers").value),
            "nheads": int(self.get_parameter("nheads").value),

            "camera_names": camera_names,
            "state_dim": 9,
            "action_dim": 9,

            "image_resize_hw": int(self.get_parameter("image_resize_hw").value),
            "image_pool_hw": int(self.get_parameter("image_pool_hw").value),
            "pretrained_backbone": (not bool(self.get_parameter("no_pretrained").value)),
        }
        return cfg

    def _load_policy(self):
        self.get_logger().info("[INFO] Loading policy (training-time custom policy.py)...")

        cfg = self.build_policy_config()
        # build ACT policy (dict args)
        self.policy = self.CustomACTPolicy(cfg).to(self.device)
        self.policy.eval()

        ckpt_path = resolve_ckpt_path(self.ckpt_dir)
        ckpt = torch.load(ckpt_path, map_location=self.device)

        # ACT training saved model-only state_dict -> load into policy.model
        missing, unexpected = self.policy.model.load_state_dict(ckpt, strict=False)
        self.get_logger().info(f"[INFO] Loaded policy from {ckpt_path}. missing={len(missing)}, unexpected={len(unexpected)}")

    # -----------------------------
    # Subscribers
    # -----------------------------
    def cb_pose(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float64).reshape(-1)
        if arr.size < 6:
            return
        self.cur_pose6 = arr[:6].copy()
        self.have_pose = True

    def cb_force(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float64).reshape(-1)
        if arr.size < 3:
            return
        self.cur_force3 = arr[:3].copy()
        self.have_force = True

    def cb_image(self, msg: Image, cam_idx: int):
        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            t = to_torch_image_rgb01(cv_bgr)  # (3,H,W) RGB [0,1]
            self.cur_imgs[cam_idx] = t
            self.have_imgs[cam_idx] = True
        except Exception:
            return

    # -----------------------------
    # Core logic
    # -----------------------------
    def ready(self) -> bool:
        return self.have_pose and self.have_force and all(self.have_imgs)

    def normalize_qpos9(self, qpos9: np.ndarray) -> np.ndarray:
        mu = _vec(self.stats["qpos_mean"])
        sd = _vec(self.stats["qpos_std"])
        return (qpos9 - mu) / sd

    def denorm_action9(self, a9_norm: np.ndarray) -> np.ndarray:
        mu = _vec(self.stats["action_mean"])
        sd = _vec(self.stats["action_std"])
        return a9_norm * sd + mu

    def temporal_aggregate(self, a9_new: np.ndarray) -> np.ndarray:
        use = bool(self.get_parameter("use_temporal_agg").value)
        if not use:
            return a9_new

        tau = float(self.get_parameter("temporal_agg_tau").value)
        # EMA alpha per tick
        alpha = 1.0 - math.exp(-1.0 / max(1e-6, tau))

        if self.temporal_ema9 is None:
            self.temporal_ema9 = a9_new.copy()
        else:
            self.temporal_ema9 = (1.0 - alpha) * self.temporal_ema9 + alpha * a9_new
        return self.temporal_ema9.copy()

    def contact_update(self, meas_fz: float):
        thr = float(self.get_parameter("contact_fz_threshold").value)
        self.contact_state.prev_contact = self.contact_state.contact
        self.contact_state.contact = (meas_fz >= thr)

        if (not self.contact_state.prev_contact) and self.contact_state.contact:
            # just impacted
            self.contact_state.impact_hold_ticks_left = int(self.get_parameter("impact_hold_ticks").value)

    def apply_step_caps(self, cmd9: np.ndarray, last_cmd9: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        returns (cmd9_capped, cap_z_applied_mm)
        """
        cap_z_mm = float(self.get_parameter("z_step_cap_mm").value)
        cap_fz = float(self.get_parameter("fz_step_cap").value)

        out = cmd9.copy()
        dz = out[2] - last_cmd9[2]
        dz_c = _clamp(dz, -cap_z_mm, cap_z_mm)
        out[2] = last_cmd9[2] + dz_c

        dfz = out[8] - last_cmd9[8]
        dfz_c = _clamp(dfz, -cap_fz, cap_fz)
        out[8] = last_cmd9[8] + dfz_c

        # optionally cap other force components too (safer)
        dfx = out[6] - last_cmd9[6]
        dfy = out[7] - last_cmd9[7]
        out[6] = last_cmd9[6] + _clamp(dfx, -cap_fz, cap_fz)
        out[7] = last_cmd9[7] + _clamp(dfy, -cap_fz, cap_fz)

        return out, float(dz_c)

    def stall_logic(self, pred_seq_denorm: np.ndarray, meas_fz: float) -> Tuple[float, float]:
        """
        pred_seq_denorm: (num_queries,9) in real scale
        returns (stall_boost, pred_fz_max)
        """
        stall_enable = bool(self.get_parameter("stall_enable").value)
        if not stall_enable:
            self.stall_boost = 0.0
            return 0.0, float(np.max(pred_seq_denorm[:, 8]))

        pred_fz_max = float(np.max(pred_seq_denorm[:, 8]))
        pred_thr = float(self.get_parameter("stall_pred_fz_thr").value)

        # apply only before contact
        if meas_fz >= float(self.get_parameter("contact_fz_threshold").value):
            self.stall_boost = 0.0
            return 0.0, pred_fz_max

        if pred_fz_max < pred_thr:
            # ramp boost up
            target = float(self.get_parameter("stall_force_boost_target").value)
            rate = float(self.get_parameter("stall_force_boost_rate").value)  # N/sec
            mx = float(self.get_parameter("stall_force_boost_max").value)

            self.stall_boost = min(target, self.stall_boost + rate * self.dt)
            self.stall_boost = min(mx, self.stall_boost)
        else:
            # decay boost
            self.stall_boost = max(0.0, self.stall_boost - 2.0 * float(self.get_parameter("stall_force_boost_rate").value) * self.dt)

        return float(self.stall_boost), pred_fz_max

    # -----------------------------
    # Timer
    # -----------------------------
    def on_timer(self):
        if not self.ready():
            return

        self.tick += 1

        # current state
        cur9 = np.concatenate([self.cur_pose6, self.cur_force3], axis=0).astype(np.float64)
        meas_fz = float(cur9[8])

        # contact update
        self.contact_update(meas_fz)

        # first cmd = current
        if self.last_cmd9 is None:
            self.last_cmd9 = cur9.copy()
            self.publish_cmd(self.last_cmd9)
            self.get_logger().info(f"[START] First cmd=current. cmd={fmt_arr(self.last_cmd9, n=9)}")
            return

        # build inputs
        qpos9 = cur9.copy()
        if bool(self.get_parameter("normalize_qpos").value):
            qpos9_in = self.normalize_qpos9(qpos9)
        else:
            qpos9_in = qpos9

        # images: stack as (B,K,3,H,W)
        imgs = torch.stack(self.cur_imgs, dim=0)  # (K,3,H,W)
        imgs = imgs.unsqueeze(0).to(self.device)  # (1,K,3,H,W)

        qpos_t = torch.from_numpy(qpos9_in.astype(np.float32)).unsqueeze(0).to(self.device)  # (1,9)

        # policy forward
        with torch.inference_mode():
            a_hat = self.policy(qpos_t, imgs, actions=None, is_pad=None)  # (1,num_queries,9)
        a_hat = a_hat[0].detach().float().cpu().numpy()  # (Q,9) normalized scale output (training target scale)

        # denorm full sequence for stall check
        if bool(self.get_parameter("denorm_action").value):
            a_seq_den = self.denorm_action9(a_hat)
        else:
            a_seq_den = a_hat.copy()

        # select step
        offset = int(self.get_parameter("pred_step_offset").value)
        offset = int(_clamp(offset, 0, a_seq_den.shape[0] - 1))
        a_sel = a_seq_den[offset].copy()

        # anchor offset init once: align predicted pose to current pose
        if not self.anchor_inited:
            self.pose_offset6 = (self.cur_pose6 - a_sel[0:6]).copy()
            self.anchor_inited = True
            self.get_logger().info("[ANCHOR] pose offset initialized (current - pred).")

        # apply pose offset
        a_sel[0:6] = a_sel[0:6] + self.pose_offset6

        # temporal aggregation on selected action
        a_sel = self.temporal_aggregate(a_sel)

        # action_type
        action_type = str(self.get_parameter("action_type").value).lower()
        if action_type == "delta":
            # delta around current
            target9 = cur9 + a_sel
        else:
            target9 = a_sel

        # impact hold: freeze xy/rpy, only allow force to rise minimally
        if self.contact_state.impact_hold_ticks_left > 0:
            self.contact_state.impact_hold_ticks_left -= 1
            target9[0:6] = self.last_cmd9[0:6]  # hold pose
            # force: keep last fx/fy, allow fz to increase
            target9[6] = 0.0
            target9[7] = 0.0

        # contact force floor (your request: teach used >=5N)
        if self.contact_state.contact:
            target9[8] = max(float(target9[8]), float(self.get_parameter("min_contact_fz_cmd").value))

        # stall boost (if pred fz never rises)
        boost, pred_fz_max = self.stall_logic(a_seq_den, meas_fz)
        if boost > 0.0:
            target9[8] = max(target9[8], boost)

        # Safety: in your pipeline fx/fy often should be 0
        # (keep as commanded if you want; default keep)
        # target9[6] = 0.0
        # target9[7] = 0.0

        # startup ramp (blend from current to target)
        ramp_sec = float(self.get_parameter("startup_ramp_sec").value)
        if ramp_sec > 1e-6:
            t = _now_sec() - self.start_time
            w = _clamp(t / ramp_sec, 0.0, 1.0)
            target9 = (1.0 - w) * cur9 + w * target9

        # step caps
        cmd9, cap_z = self.apply_step_caps(target9, self.last_cmd9)

        # publish
        self.publish_cmd(cmd9)
        self.last_cmd9 = cmd9.copy()

        # debug
        dbg_n = int(self.get_parameter("debug_print_every_n").value)
        if dbg_n > 0 and (self.tick % dbg_n == 0):
            meas_fz = float(self.cur_force3[2])
            self.get_logger().info(
                f"tick={self.tick} contact={int(self.contact_state.contact)} "
                f"impactHold={self.contact_state.impact_hold_ticks_left} boost={boost:.3f} | "
                f"meas_fz={meas_fz:.3f} cmd_fz={cmd9[8]:.3f} | "
                f"pred_fz_max={pred_fz_max:.3f} pred_fz_now={float(a_seq_den[0,8]):.3f} "
                f"| cap_z={cap_z:.4f} | cmd_xyz={fmt_arr(cmd9[0:3], n=3)}"
            )

            L = int(self.get_parameter("debug_seq_preview_len").value)
            L = int(_clamp(L, 1, a_seq_den.shape[0]))
            preview = [f"{a_seq_den[i,8]:+.3f}" for i in range(L)]
            self.get_logger().info(f"[SEQ] fz_den[0:{L}]={preview}")

            # warning message (your log style)
            stall_thr = float(self.get_parameter("stall_pred_fz_thr").value)
            if bool(self.get_parameter("stall_enable").value) and pred_fz_max < stall_thr:
                self.get_logger().warn(
                    f"[STALL] pred_fz_max={pred_fz_max:.3f} < {stall_thr:.3f}. Model might not be producing force-rise (early phase)."
                )

    def publish_cmd(self, cmd9: np.ndarray):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in _vec(cmd9)]
        self.pub_cmd.publish(msg)


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
