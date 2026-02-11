#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
node_act_cmdmotion_infer.py (REBUILT)

Goal:
- Subscribe ONLY 4 topics (same as node_check_inference):
    /ur10skku/currentP                  (Float64MultiArray)  -> pose 6D
    /ur10skku/currentF                  (Float64MultiArray)  -> force 6D (use first 3 by default)
    /realsense/ee/color/image_raw       (sensor_msgs/Image)  -> ee cam
    /realsense/top/color/image_raw      (sensor_msgs/Image)  -> top cam

- Build ACT input exactly like training:
    qpos = [pose6 + force3] => (9,)
    image = stack([top, ee]) => (B=1,K=2,3,H,W) ; float in [0,1]
    camera_names MUST be ["cam_top","cam_ee"] (same order)

- Load ckpt_dir/policy_best.ckpt (model-only state_dict) using training-time policy.py in act_root
  (policy.py is NOT in ckpt_dir)

- Run inference at infer_hz, publish command at control_hz:
    * Inference produces sequence (T=chunk_size, 9)
    * Control publishes 9D action on cmd_topic

- Temporal Aggregation (Exponential Decay):
    At each control tick, aggregate multiple recent predicted sequences:
        action_now = sum_i w_i * seq_i[k_i] / sum_i w_i
    where w_i = exp(-age_i / tau_sec), age_i = now - t_infer_i
    and k_i = int((now - t_infer_i) * control_hz)  (step index since that inference)
    (clamped to [0, chunk_size-1])

- Optional contact gate:
    Detect contact by force_z (from force_indices) with hysteresis.
    When contact state changes, you may clear previous plans (default True).

Usage:
ros2 run nrs_imitation node_act_cmdmotion_infer --ros-args \
  -p ckpt_dir:=/home/eunseop/nrs_lab2/checkpoints/ur10e_swing/20260208_1536 \
  -p act_root:=/home/eunseop/nrs_lab2/nrs_act \
  -p image_qos:=reliable \
  -p cmd_topic:=/nrs_imitation/cmd_action \
  -p control_hz:=30.0 \
  -p infer_hz:=7.5
"""

import os
import sys
import time
import pickle
from dataclasses import dataclass
from typing import Optional, Tuple, List

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image


# -----------------------------
# Helpers (same as node_check_inference style)
# -----------------------------
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


def _img_to_rgb_numpy(msg: Image) -> np.ndarray:
    """
    Convert sensor_msgs/Image to np.uint8 (H,W,3) in RGB order.

    Supports:
    - rgb8
    - bgr8
    - rgba8 / bgra8 (alpha dropped)
    """
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or "").lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == "rgb8":
        img = buf.reshape((h, w, 3))
        return img
    if enc == "bgr8":
        img = buf.reshape((h, w, 3))
        return img[..., ::-1].copy()
    if enc == "rgba8":
        img = buf.reshape((h, w, 4))[..., :3]
        return img
    if enc == "bgra8":
        img = buf.reshape((h, w, 4))[..., :3]
        return img[..., ::-1].copy()

    # fallback attempt: assume 3-channel
    try:
        img = buf.reshape((h, w, 3))
        return img
    except Exception as e:
        raise RuntimeError(f"Unsupported image encoding={msg.encoding}, size=({h},{w}), err={e}")


def _to_tensor_image_stack(top_rgb: np.ndarray, ee_rgb: np.ndarray, device: torch.device) -> torch.Tensor:
    """
    Make (B=1,K=2,3,H,W) float tensor in [0,1].
    Order MUST match training camera_names=["cam_top","cam_ee"] => [top, ee]
    """
    if top_rgb.shape != ee_rgb.shape:
        raise RuntimeError(f"Top/Ee image size mismatch: top={top_rgb.shape} ee={ee_rgb.shape}")

    top = np.transpose(top_rgb, (2, 0, 1))
    ee = np.transpose(ee_rgb, (2, 0, 1))
    img = np.stack([top, ee], axis=0).astype(np.float32) / 255.0
    img_t = torch.from_numpy(img).unsqueeze(0).to(device=device, dtype=torch.float32)
    return img_t


def _to_tensor_qpos(pose6: np.ndarray, force6: np.ndarray, force_indices=(0, 1, 2),
                    device: torch.device = torch.device("cpu")) -> torch.Tensor:
    """
    qpos = [pose6 + force3] => (1,9)
    """
    pose6 = np.asarray(pose6, dtype=np.float32).reshape(-1)
    force6 = np.asarray(force6, dtype=np.float32).reshape(-1)
    if pose6.size < 6:
        raise RuntimeError(f"pose6 size < 6: {pose6.size}")
    if force6.size < max(force_indices) + 1:
        raise RuntimeError(f"force6 size < needed idx: force size={force6.size}, idx={force_indices}")

    f3 = force6[list(force_indices)].astype(np.float32)
    q = np.concatenate([pose6[:6], f3], axis=0).astype(np.float32)  # (9,)
    q_t = torch.from_numpy(q).unsqueeze(0).to(device=device, dtype=torch.float32)  # (1,9)
    return q_t


def _load_stats_if_exists(ckpt_dir: str) -> Optional[dict]:
    p = os.path.join(ckpt_dir, "dataset_stats.pkl")
    if not os.path.exists(p):
        return None
    with open(p, "rb") as f:
        stats = pickle.load(f)
    return stats


def _normalize_qpos(q_t: torch.Tensor, stats: dict) -> torch.Tensor:
    mu = torch.tensor(stats["qpos_mean"], dtype=torch.float32, device=q_t.device).view(1, 9)
    sd = torch.tensor(stats["qpos_std"], dtype=torch.float32, device=q_t.device).view(1, 9)
    return (q_t - mu) / sd


def _denormalize_action(a_t: torch.Tensor, stats: dict) -> torch.Tensor:
    """
    a_t: (...,9) normalized
    """
    mu = torch.tensor(stats["action_mean"], dtype=torch.float32, device=a_t.device).view(*(1,) * (a_t.dim() - 1), 9)
    sd = torch.tensor(stats["action_std"], dtype=torch.float32, device=a_t.device).view(*(1,) * (a_t.dim() - 1), 9)
    return a_t * sd + mu


def _fix_a_hat_shape(a_hat: torch.Tensor, chunk_size: int) -> torch.Tensor:
    """
    Standardize output to (T,9) with T=chunk_size.
    Handles common variants:
      - (B,T,9)
      - (T,B,9)
      - (T,9) already
    """
    if a_hat.dim() == 2:
        return a_hat
    if a_hat.dim() != 3:
        raise RuntimeError(f"Unexpected a_hat dim: {a_hat.shape}")

    B0, B1, B2 = a_hat.shape

    if B0 == 1 and B1 == chunk_size and B2 == 9:
        return a_hat[0]  # (T,9)
    if B0 == chunk_size and B1 == 1 and B2 == 9:
        return a_hat[:, 0, :]  # (T,9)
    if B2 == 9 and B1 == chunk_size:
        return a_hat[0]

    raise RuntimeError(f"Cannot interpret a_hat shape={a_hat.shape} with chunk_size={chunk_size}")


# -----------------------------
# Temporal Agg container
# -----------------------------
@dataclass
class PredPlan:
    t_infer: float            # wall time (sec)
    seq: np.ndarray           # (T,9) float32
    contact_state: int        # 0/1 at inference time


def _exp_weight(age: float, tau: float) -> float:
    if tau <= 1e-6:
        return 0.0
    # stable
    x = -max(0.0, age) / tau
    if x < -50.0:
        return 0.0
    return float(np.exp(x))


# -----------------------------
# Node
# -----------------------------
class NodeActCmdMotionInfer(Node):
    def __init__(self):
        super().__init__("node_act_cmdmotion_infer")

        # ---- params (paths) ----
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("act_root", "")
        self.declare_parameter("chunk_size", 100)

        # ---- topics (same defaults as node_check_inference) ----
        self.declare_parameter("pose_topic", "/ur10skku/currentP")
        self.declare_parameter("force_topic", "/ur10skku/currentF")
        self.declare_parameter("ee_img_topic", "/realsense/ee/color/image_raw")
        self.declare_parameter("top_img_topic", "/realsense/top/color/image_raw")

        # ---- qos ----
        self.declare_parameter("image_qos", "best_effort")  # reliable | best_effort

        # ---- publish ----
        self.declare_parameter("cmd_topic", "/nrs_imitation/cmd_action")
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("infer_hz", 7.5)

        # ---- model hyperparams (match training) ----
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

        # ---- normalization ----
        self.declare_parameter("normalize_qpos", True)
        self.declare_parameter("denorm_action", True)
        self.declare_parameter("force_indices", [0, 1, 2])

        # ---- temporal agg ----
        self.declare_parameter("tau_sec", 0.35)               # exp decay time constant
        self.declare_parameter("max_plans", 6)                # keep last N inferred plans
        self.declare_parameter("min_weight_sum", 1e-6)
        self.declare_parameter("hold_last_on_empty", True)    # if no plans available, hold last cmd
        self.declare_parameter("max_step_clamp", 99)          # clamp step index within [0,chunk_size-1]

        # ---- contact gate (optional) ----
        self.declare_parameter("use_contact_gate", True)
        self.declare_parameter("contact_axis_in_force3", 2)   # 0/1/2 index in selected force3
        self.declare_parameter("contact_on_thr", 5.0)         # hysteresis ON
        self.declare_parameter("contact_off_thr", 2.0)        # hysteresis OFF
        self.declare_parameter("clear_plans_on_contact_change", True)

        # ---- read params ----
        self.ckpt_dir = str(self.get_parameter("ckpt_dir").value)
        self.act_root = str(self.get_parameter("act_root").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)
        self.ee_img_topic = str(self.get_parameter("ee_img_topic").value)
        self.top_img_topic = str(self.get_parameter("top_img_topic").value)

        self.image_qos_str = str(self.get_parameter("image_qos").value)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.infer_hz = float(self.get_parameter("infer_hz").value)

        self.normalize_qpos_enabled = bool(self.get_parameter("normalize_qpos").value)
        self.denorm_action_enabled = bool(self.get_parameter("denorm_action").value)
        self.force_indices = tuple(int(x) for x in self.get_parameter("force_indices").value)

        self.tau_sec = float(self.get_parameter("tau_sec").value)
        self.max_plans = int(self.get_parameter("max_plans").value)
        self.min_weight_sum = float(self.get_parameter("min_weight_sum").value)
        self.hold_last_on_empty = bool(self.get_parameter("hold_last_on_empty").value)
        self.max_step_clamp = int(self.get_parameter("max_step_clamp").value)

        self.use_contact_gate = bool(self.get_parameter("use_contact_gate").value)
        self.contact_axis_in_force3 = int(self.get_parameter("contact_axis_in_force3").value)
        self.contact_on_thr = float(self.get_parameter("contact_on_thr").value)
        self.contact_off_thr = float(self.get_parameter("contact_off_thr").value)
        self.clear_plans_on_contact_change = bool(self.get_parameter("clear_plans_on_contact_change").value)

        # ---- device ----
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"[INFO] Using device: {self.device}")

        # ---- validate paths ----
        if not self.ckpt_dir or not os.path.isdir(self.ckpt_dir):
            raise RuntimeError(f"ckpt_dir invalid: {self.ckpt_dir}")
        if not self.act_root or not os.path.isdir(self.act_root):
            raise RuntimeError(f"act_root invalid: {self.act_root}")

        # ---- load stats ----
        self.stats = _load_stats_if_exists(self.ckpt_dir)
        if self.stats is None:
            self.get_logger().warn("[STATS] dataset_stats.pkl not found. normalize_qpos/denorm_action will be disabled.")
            self.normalize_qpos_enabled = False
            self.denorm_action_enabled = False
        else:
            self.get_logger().info(f"[STATS] Loaded dataset stats: {os.path.join(self.ckpt_dir, 'dataset_stats.pkl')}")

        # ---- load policy ----
        self.policy = self._load_policy_and_ckpt()

        # ---- buffers ----
        self._pose6: Optional[np.ndarray] = None
        self._force6: Optional[np.ndarray] = None
        self._img_top: Optional[np.ndarray] = None
        self._img_ee: Optional[np.ndarray] = None

        self._plans: List[PredPlan] = []
        self._last_cmd: Optional[np.ndarray] = None

        # contact state (0/1)
        self._contact_state = 0  # start non-contact

        # ---- QoS ----
        img_rel = _reliability_from_str(self.image_qos_str)
        img_qos = _qos(depth=1, reliability=img_rel)
        vec_qos = _qos(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ---- subs ----
        self.create_subscription(Float64MultiArray, self.pose_topic, self._on_pose, vec_qos)
        self.create_subscription(Float64MultiArray, self.force_topic, self._on_force, vec_qos)
        self.create_subscription(Image, self.top_img_topic, self._on_top_img, img_qos)
        self.create_subscription(Image, self.ee_img_topic, self._on_ee_img, img_qos)

        # ---- pub ----
        self.pub_cmd = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # ---- timers ----
        self.dt_control = 1.0 / max(1e-6, self.control_hz)
        self.dt_infer = 1.0 / max(1e-6, self.infer_hz)
        self.timer_control = self.create_timer(self.dt_control, self._control_loop)
        self.timer_infer = self.create_timer(self.dt_infer, self._infer_loop)

        self.get_logger().info(
            "[INFO] ✅ Ready.\n"
            f"  pose_topic={self.pose_topic}\n"
            f"  force_topic={self.force_topic}\n"
            f"  ee_img_topic={self.ee_img_topic}\n"
            f"  top_img_topic={self.top_img_topic}\n"
            f"  image_qos={self.image_qos_str}\n"
            f"  force_indices={self.force_indices}\n"
            f"  cmd_topic={self.cmd_topic}\n"
            f"  control_hz={self.control_hz}\n"
            f"  infer_hz={self.infer_hz}\n"
            f"  tau_sec={self.tau_sec}\n"
            f"  max_plans={self.max_plans}\n"
            f"  contact_gate={self.use_contact_gate} (on={self.contact_on_thr}, off={self.contact_off_thr})\n"
        )

    # -----------------------------
    # Policy load (same philosophy as node_check_inference)
    # -----------------------------
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

            # CRITICAL
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
        missing, unexpected = policy.model.load_state_dict(ckpt, strict=False)
        self.get_logger().info(f"[INFO] Loaded ckpt. missing={len(missing)}, unexpected={len(unexpected)}")

        try:
            cam_names = list(policy.model.camera_names)
        except Exception:
            cam_names = args_override["camera_names"]
        self.get_logger().info(f"[INFO] camera_names in model = {cam_names}")

        return policy

    # -----------------------------
    # Callbacks
    # -----------------------------
    def _on_pose(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32).reshape(-1)
        if arr.size >= 6:
            self._pose6 = arr[:6].copy()

    def _on_force(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32).reshape(-1)
        if arr.size >= 3:
            self._force6 = arr.copy()
            if self.use_contact_gate:
                self._update_contact_state_from_force()

    def _on_top_img(self, msg: Image):
        try:
            self._img_top = _img_to_rgb_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"[TOP IMG] decode failed: {e}")

    def _on_ee_img(self, msg: Image):
        try:
            self._img_ee = _img_to_rgb_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"[EE IMG] decode failed: {e}")

    # -----------------------------
    # State checks
    # -----------------------------
    def _have_all(self) -> bool:
        return (self._pose6 is not None) and (self._force6 is not None) and (self._img_top is not None) and (self._img_ee is not None)

    def _force3(self) -> Optional[np.ndarray]:
        if self._force6 is None:
            return None
        f6 = np.asarray(self._force6, dtype=np.float32).reshape(-1)
        if f6.size < max(self.force_indices) + 1:
            return None
        return f6[list(self.force_indices)].astype(np.float32)

    # -----------------------------
    # Contact gate (hysteresis)
    # -----------------------------
    def _update_contact_state_from_force(self):
        f3 = self._force3()
        if f3 is None:
            return
        axis = int(np.clip(self.contact_axis_in_force3, 0, 2))
        val = float(f3[axis])

        prev = self._contact_state
        now = prev

        if prev == 0:
            if val >= self.contact_on_thr:
                now = 1
        else:
            if val <= self.contact_off_thr:
                now = 0

        if now != prev:
            self._contact_state = now
            self.get_logger().info(f"[CONTACT] state change: {prev} -> {now} (val={val:+0.3f})")
            if self.clear_plans_on_contact_change:
                self._plans.clear()
                self.get_logger().info("[CONTACT] cleared plans due to contact state change")

    # -----------------------------
    # Inference loop (infer_hz)
    # -----------------------------
    def _infer_loop(self):
        if not self._have_all():
            return

        try:
            # Build inputs
            q_t = _to_tensor_qpos(self._pose6, self._force6, force_indices=self.force_indices, device=self.device)
            img_t = _to_tensor_image_stack(self._img_top, self._img_ee, device=self.device)

            if self.normalize_qpos_enabled and self.stats is not None:
                q_t = _normalize_qpos(q_t, self.stats)

            with torch.inference_mode():
                a_hat = self.policy(q_t, img_t)

            seq = _fix_a_hat_shape(a_hat, self.chunk_size)  # (T,9)

            if self.denorm_action_enabled and self.stats is not None:
                seq = _denormalize_action(seq, self.stats)

            seq_np = seq.detach().cpu().numpy().astype(np.float32)

            # store plan
            plan = PredPlan(
                t_infer=time.time(),
                seq=seq_np,
                contact_state=int(self._contact_state),
            )
            self._plans.append(plan)
            if len(self._plans) > self.max_plans:
                self._plans = self._plans[-self.max_plans:]

            self.get_logger().debug(f"[INFER] stored plan. num_plans={len(self._plans)}")

        except Exception as e:
            self.get_logger().error(f"[INFER] failed: {e}")

    # -----------------------------
    # Control loop (control_hz)
    # -----------------------------
    def _control_loop(self):
        now = time.time()

        cmd = self._compute_temporal_agg_cmd(now)
        if cmd is None:
            if self.hold_last_on_empty and (self._last_cmd is not None):
                cmd = self._last_cmd
            else:
                return

        self._publish_cmd(cmd)
        self._last_cmd = cmd

    def _compute_temporal_agg_cmd(self, now: float) -> Optional[np.ndarray]:
        if len(self._plans) == 0:
            return None

        # aggregate
        w_sum = 0.0
        a_sum = np.zeros((9,), dtype=np.float64)

        # step index mapping: elapsed seconds -> elapsed control steps
        # k_i = int((now - t_infer) * control_hz)
        for plan in reversed(self._plans):
            age = now - float(plan.t_infer)
            if age < 0.0:
                age = 0.0

            # optional: if contact state mismatch, downweight heavily
            if self.use_contact_gate and int(plan.contact_state) != int(self._contact_state):
                # mismatch => effectively ignore (or very small weight)
                w = _exp_weight(age, self.tau_sec) * 0.05
            else:
                w = _exp_weight(age, self.tau_sec)

            if w <= 0.0:
                continue

            k = int(age * self.control_hz)
            k = int(np.clip(k, 0, min(self.chunk_size - 1, self.max_step_clamp)))

            a = plan.seq[k].astype(np.float64)  # (9,)
            a_sum += w * a
            w_sum += w

        if w_sum < self.min_weight_sum:
            return None

        a_now = (a_sum / w_sum).astype(np.float32)
        return a_now

    def _publish_cmd(self, a9: np.ndarray):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in a9.reshape(-1).tolist()]
        self.pub_cmd.publish(msg)


# -----------------------------
# main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = NodeActCmdMotionInfer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
