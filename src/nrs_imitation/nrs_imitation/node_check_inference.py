#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
node_check_inference.py

Purpose
- Debug ACT inference by generating a rollout trajectory (default 100 steps)
  from the current ROS topics (pose/force/images), saving plots + csv.
- Additionally runs a minimal "validation-style" sanity check if dataset_stats.pkl
  contains usable samples/paths (best-effort, optional).

Fixes included
- Load normalization stats from ckpt_dir/dataset_stats.pkl
- Avoid DETR argparse errors by never calling act/detr/main.py build_*_and_optimizer()
- Robust ACTPolicy instantiation via introspection and safe checkpoint loading
"""

import os
import sys
import time
import json
import pickle
import random
import inspect
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, List

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

try:
    from sensor_msgs.msg import Image
    HAVE_IMAGE_MSG = True
except Exception:
    HAVE_IMAGE_MSG = False

try:
    import torch
    HAVE_TORCH = True
except Exception:
    HAVE_TORCH = False

# Headless plotting
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# -----------------------------
# Data classes / utilities
# -----------------------------
@dataclass
class NormStats:
    mean: np.ndarray
    std: np.ndarray

    def normalize(self, x: np.ndarray) -> np.ndarray:
        return (x - self.mean) / (self.std + 1e-8)

    def denormalize(self, x: np.ndarray) -> np.ndarray:
        return x * (self.std + 1e-8) + self.mean


def _as_np(x: Any) -> np.ndarray:
    if isinstance(x, np.ndarray):
        return x
    if HAVE_TORCH and torch.is_tensor(x):
        return x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def _safe_mkdir(p: str) -> None:
    Path(p).mkdir(parents=True, exist_ok=True)


def _now_string() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


def _clip_std(std: np.ndarray, min_std: float = 1e-6) -> np.ndarray:
    std = np.asarray(std, dtype=np.float64)
    std = np.where(np.abs(std) < min_std, min_std, std)
    return std


# -----------------------------
# Stats loader (dataset_stats.pkl)
# -----------------------------
def load_dataset_stats_pkl(ckpt_dir: str) -> Tuple[NormStats, NormStats, Dict[str, Any]]:
    """
    Loads normalization stats from ckpt_dir/dataset_stats.pkl

    Expected (common) keys (best-effort):
      - qpos_mean, qpos_std, act_mean, act_std
    Other possible keys:
      - state_mean/state_std, action_mean/action_std
      - obs_mean/obs_std, action_mean/action_std
      - mean/std dicts nested
    """
    ckpt = Path(ckpt_dir)
    pkl_path = ckpt / "dataset_stats.pkl"
    if not pkl_path.is_file():
        raise FileNotFoundError(f"dataset_stats.pkl not found in ckpt_dir: {pkl_path}")

    with open(pkl_path, "rb") as f:
        obj = pickle.load(f)

    if not isinstance(obj, dict):
        raise RuntimeError(f"dataset_stats.pkl content is not a dict. type={type(obj)}")

    # Helper: try candidate key pairs
    def pick_pair(mean_keys: List[str], std_keys: List[str]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        for mk in mean_keys:
            for sk in std_keys:
                if mk in obj and sk in obj:
                    m = _as_np(obj[mk]).astype(np.float64)
                    s = _clip_std(_as_np(obj[sk]).astype(np.float64))
                    return m, s
        return None

    # qpos/state/obs
    qpos_pair = pick_pair(
        mean_keys=["qpos_mean", "state_mean", "obs_mean", "qpos_mu", "state_mu", "obs_mu"],
        std_keys=["qpos_std", "state_std", "obs_std", "qpos_sigma", "state_sigma", "obs_sigma"],
    )

    # action
    act_pair = pick_pair(
        mean_keys=["act_mean", "action_mean", "target_mean", "act_mu", "action_mu", "target_mu"],
        std_keys=["act_std", "action_std", "target_std", "act_sigma", "action_sigma", "target_sigma"],
    )

    # Nested dict fallback: obj["qpos"]["mean"], ...
    if qpos_pair is None and "qpos" in obj and isinstance(obj["qpos"], dict):
        d = obj["qpos"]
        if "mean" in d and "std" in d:
            qpos_pair = (_as_np(d["mean"]).astype(np.float64), _clip_std(_as_np(d["std"]).astype(np.float64)))

    if act_pair is None and "action" in obj and isinstance(obj["action"], dict):
        d = obj["action"]
        if "mean" in d and "std" in d:
            act_pair = (_as_np(d["mean"]).astype(np.float64), _clip_std(_as_np(d["std"]).astype(np.float64)))

    if act_pair is None and "act" in obj and isinstance(obj["act"], dict):
        d = obj["act"]
        if "mean" in d and "std" in d:
            act_pair = (_as_np(d["mean"]).astype(np.float64), _clip_std(_as_np(d["std"]).astype(np.float64)))

    if qpos_pair is None or act_pair is None:
        keys = sorted(list(obj.keys()))
        raise RuntimeError(
            "Failed to find normalization stats in dataset_stats.pkl.\n"
            f"Found keys={keys}\n"
            "Expected qpos_mean/qpos_std + act_mean/act_std (or similar)."
        )

    qpos_mean, qpos_std = qpos_pair
    act_mean, act_std = act_pair

    qpos_stats = NormStats(mean=qpos_mean, std=qpos_std)
    act_stats = NormStats(mean=act_mean, std=act_std)
    return qpos_stats, act_stats, obj


# -----------------------------
# Policy loader (safe, no argparse)
# -----------------------------
def _import_policy_module(act_root: str):
    """
    Import act_root/policy.py as module "policy".
    """
    act_root = str(Path(act_root).resolve())
    if act_root not in sys.path:
        sys.path.insert(0, act_root)

    try:
        import policy  # type: ignore
        return policy
    except Exception as e:
        raise RuntimeError(f"Failed to import policy.py from act_root={act_root}. err={e}")


def _select_ckpt_file(ckpt_dir: str) -> str:
    ckpt = Path(ckpt_dir)
    cands = [
        ckpt / "policy_best.ckpt",
        ckpt / "policy_last.ckpt",
        ckpt / "policy.ckpt",
        ckpt / "model_best.ckpt",
        ckpt / "model.ckpt",
    ]
    for p in cands:
        if p.is_file():
            return str(p)
    # fallback: any .ckpt
    any_ckpt = list(ckpt.glob("*.ckpt"))
    if len(any_ckpt) > 0:
        return str(any_ckpt[0])
    raise FileNotFoundError(f"No checkpoint file found in ckpt_dir={ckpt_dir}")


def _extract_state_dict(ckpt_obj: Any) -> Dict[str, Any]:
    """
    Best-effort extraction of state_dict from various checkpoint formats.
    """
    if isinstance(ckpt_obj, dict):
        # Common keys
        for k in ["state_dict", "model", "policy", "ema", "net", "actor"]:
            if k in ckpt_obj and isinstance(ckpt_obj[k], (dict,)):
                # Sometimes nested: ckpt["model"]["state_dict"]
                if "state_dict" in ckpt_obj[k] and isinstance(ckpt_obj[k]["state_dict"], dict):
                    return ckpt_obj[k]["state_dict"]
                return ckpt_obj[k]
        # If dict itself looks like state_dict (tensor values)
        tensor_like = 0
        for v in ckpt_obj.values():
            if HAVE_TORCH and torch.is_tensor(v):
                tensor_like += 1
        if tensor_like > 0:
            return ckpt_obj
    raise RuntimeError("Unrecognized checkpoint format: cannot extract state_dict.")


def instantiate_act_policy(
    act_root: str,
    ckpt_dir: str,
    device: str,
    state_dim: int,
    action_dim: int,
    camera_names: Optional[List[str]] = None,
) -> Any:
    """
    Robustly instantiate ACTPolicy without touching DETR argparse code.

    Strategy:
    1) Import policy.py from act_root
    2) Try helper factory functions if they exist:
       - load_policy / make_policy / create_policy / get_policy / load_act_policy
    3) Else instantiate ACTPolicy class via signature inspection.
    """
    if not HAVE_TORCH:
        raise RuntimeError("PyTorch not available in this environment.")

    polmod = _import_policy_module(act_root)

    # 1) Prefer a factory helper if provided
    helper_names = [
        "load_policy",
        "load_act_policy",
        "make_policy",
        "make_act_policy",
        "create_policy",
        "get_policy",
    ]
    for hn in helper_names:
        if hasattr(polmod, hn):
            fn = getattr(polmod, hn)
            if callable(fn):
                try:
                    # Try keyword-based call (most robust)
                    return fn(
                        ckpt_dir=ckpt_dir,
                        device=device,
                        state_dim=state_dim,
                        action_dim=action_dim,
                        camera_names=camera_names or [],
                    )
                except TypeError:
                    # Try minimal
                    try:
                        return fn(ckpt_dir=ckpt_dir, device=device)
                    except TypeError:
                        # Try positional
                        try:
                            return fn(ckpt_dir, device)
                        except Exception as e:
                            raise RuntimeError(f"Factory {hn} exists but failed to run. err={e}")
                except Exception as e:
                    raise RuntimeError(f"Factory {hn} exists but failed. err={e}")

    # 2) ACTPolicy class fallback
    if not hasattr(polmod, "ACTPolicy"):
        raise RuntimeError(f"policy.py has no ACTPolicy and no usable factory. act_root={act_root}")

    ACTPolicyCls = getattr(polmod, "ACTPolicy")

    sig = inspect.signature(ACTPolicyCls.__init__)
    params = list(sig.parameters.values())  # includes self
    # Remove self
    params = [p for p in params if p.name != "self"]

    # Build candidate kwargs
    kwargs = {}
    if camera_names is not None:
        kwargs["camera_names"] = camera_names
    kwargs["device"] = device
    kwargs["ckpt_dir"] = ckpt_dir
    kwargs["state_dim"] = state_dim
    kwargs["action_dim"] = action_dim

    # Instantiate by matching parameter names
    try_orders = []

    # (a) __init__(self, ckpt_dir=..., device=...)
    if any(p.name == "ckpt_dir" for p in params) or any(p.name == "device" for p in params):
        try_orders.append(("kw_ckpt_device", {"ckpt_dir": ckpt_dir, "device": device}))

    # (b) __init__(self, args_override_dict)
    if len(params) == 1:
        try_orders.append(("single_dict", {"args": {"ckpt_dir": ckpt_dir, "device": device, "camera_names": camera_names or []}}))

    # (c) __init__(self, state_dim, action_dim, ...)
    if any(p.name in ["state_dim", "action_dim"] for p in params) or len(params) >= 2:
        try_orders.append(("kw_dims", {"state_dim": state_dim, "action_dim": action_dim, "device": device, "camera_names": camera_names or []}))

    # (d) empty init
    try_orders.append(("empty", {}))

    last_err = None
    for tag, cand in try_orders:
        try:
            if tag == "single_dict":
                # If ACTPolicy expects a single positional "args_override"
                policy = ACTPolicyCls(cand["args"])
            else:
                # Only pass kwargs that exist in signature, if it doesn't accept **kwargs
                has_var_kw = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values())
                if has_var_kw:
                    policy = ACTPolicyCls(**cand)
                else:
                    filtered = {k: v for k, v in cand.items() if k in sig.parameters}
                    policy = ACTPolicyCls(**filtered)
            return policy
        except Exception as e:
            last_err = e

    raise RuntimeError(f"Failed to instantiate ACTPolicy. last_err={last_err}")


def load_policy_weights(policy: Any, ckpt_file: str, device: str) -> Tuple[List[str], List[str]]:
    """
    Loads checkpoint weights into policy with strict=False.
    """
    if not HAVE_TORCH:
        raise RuntimeError("PyTorch not available.")

    ckpt_obj = torch.load(ckpt_file, map_location=device)
    state_dict = _extract_state_dict(ckpt_obj)

    # Some checkpoints have module prefixes
    # We keep as-is and rely on strict=False.
    missing, unexpected = [], []
    if hasattr(policy, "load_state_dict"):
        ret = policy.load_state_dict(state_dict, strict=False)
        if hasattr(ret, "missing_keys"):
            missing = list(ret.missing_keys)
            unexpected = list(ret.unexpected_keys)
    else:
        raise RuntimeError("Policy object has no load_state_dict().")

    if hasattr(policy, "to"):
        policy.to(device)
    if hasattr(policy, "eval"):
        policy.eval()
    return missing, unexpected


# -----------------------------
# Policy inference wrapper
# -----------------------------
def _to_torch(x: np.ndarray, device: str) -> "torch.Tensor":
    t = torch.from_numpy(x).float()
    return t.to(device)


def _prep_images(images: List[np.ndarray], device: str) -> Optional["torch.Tensor"]:
    """
    Convert list of HxWxC uint8 images to torch tensor:
      [1, num_cams, 3, H, W] float in [0,1]
    """
    if not HAVE_TORCH:
        return None
    if images is None or len(images) == 0:
        return None

    proc = []
    for im in images:
        if im is None:
            continue
        im = np.asarray(im)
        if im.ndim == 2:
            im = np.stack([im, im, im], axis=-1)
        if im.shape[-1] == 4:
            im = im[..., :3]
        if im.shape[-1] != 3:
            # best-effort
            im = im[..., :3]
        im_f = im.astype(np.float32) / 255.0
        # HWC -> CHW
        im_f = np.transpose(im_f, (2, 0, 1))
        proc.append(im_f)

    if len(proc) == 0:
        return None

    arr = np.stack(proc, axis=0)  # [num_cams, 3, H, W]
    arr = arr[None, ...]          # [1, num_cams, 3, H, W]
    return _to_torch(arr, device)


def run_policy_once(policy: Any, qpos_normed: np.ndarray, images_tensor: Optional["torch.Tensor"], device: str) -> np.ndarray:
    """
    Tries common call patterns. Returns action in normalized space (numpy).
    """
    if not HAVE_TORCH:
        raise RuntimeError("PyTorch not available.")

    q = _to_torch(qpos_normed[None, :], device)  # [1, D]

    # Try methods in order
    candidates = []

    # direct call
    candidates.append(("__call__(q, images)", lambda: policy(q, images_tensor) if images_tensor is not None else policy(q)))
    # forward
    if hasattr(policy, "forward"):
        candidates.append(("forward(q, images)", lambda: policy.forward(q, images_tensor) if images_tensor is not None else policy.forward(q)))
    # act
    if hasattr(policy, "act"):
        candidates.append(("act(q, images)", lambda: policy.act(q, images_tensor) if images_tensor is not None else policy.act(q)))
    # get_action
    if hasattr(policy, "get_action"):
        candidates.append(("get_action(q, images)", lambda: policy.get_action(q, images_tensor) if images_tensor is not None else policy.get_action(q)))
    # predict
    if hasattr(policy, "predict"):
        candidates.append(("predict(q, images)", lambda: policy.predict(q, images_tensor) if images_tensor is not None else policy.predict(q)))

    last_err = None
    out = None
    for name, fn in candidates:
        try:
            out = fn()
            break
        except Exception as e:
            last_err = (name, e)

    if out is None:
        raise RuntimeError(f"All policy call patterns failed. last_err={last_err}")

    # If output is tuple/list, take first
    if isinstance(out, (tuple, list)):
        out = out[0]

    # If dict, try known keys
    if isinstance(out, dict):
        for k in ["action", "actions", "a", "pred_action"]:
            if k in out:
                out = out[k]
                break

    if not (HAVE_TORCH and torch.is_tensor(out)):
        out = torch.as_tensor(out)

    out_np = out.detach().cpu().numpy()
    # ensure shape [D]
    out_np = np.squeeze(out_np)
    return out_np.astype(np.float64)


# -----------------------------
# ROS Node
# -----------------------------
class ActCheckInferenceNode(Node):
    def __init__(self):
        super().__init__("act_check_inference_node")

        # ---- params ----
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("act_root", "")
        self.declare_parameter("out_dir", str(Path.home() / "debug_check_inference"))
        self.declare_parameter("device", "cuda")
        self.declare_parameter("pose_topic", "/ur10skku/currentP")
        self.declare_parameter("force_topic", "/ur10skku/currentF")
        self.declare_parameter("image_topics", ["/realsense/top/color/image_raw", "/realsense/ee/color/image_raw"])
        self.declare_parameter("camera_names", ["top", "ee"])

        self.declare_parameter("do_rollout", True)
        self.declare_parameter("rollout_horizon", 100)
        self.declare_parameter("action_is_delta", True)
        self.declare_parameter("sleep_between_steps_sec", 0.0)  # offline rollout, default 0

        self.declare_parameter("wait_timeout_sec", 10.0)  # wait for first msgs
        self.declare_parameter("seed", 0)

        # minimal dataset sanity check
        self.declare_parameter("do_min_dataset_check", True)
        self.declare_parameter("dataset_check_samples", 16)

        self.ckpt_dir = self.get_parameter("ckpt_dir").value
        self.act_root = self.get_parameter("act_root").value
        self.out_dir = self.get_parameter("out_dir").value
        self.device = self.get_parameter("device").value

        self.pose_topic = self.get_parameter("pose_topic").value
        self.force_topic = self.get_parameter("force_topic").value
        self.image_topics = list(self.get_parameter("image_topics").value)
        self.camera_names = list(self.get_parameter("camera_names").value)

        self.do_rollout = bool(self.get_parameter("do_rollout").value)
        self.rollout_horizon = int(self.get_parameter("rollout_horizon").value)
        self.action_is_delta = bool(self.get_parameter("action_is_delta").value)
        self.sleep_between_steps_sec = float(self.get_parameter("sleep_between_steps_sec").value)

        self.wait_timeout_sec = float(self.get_parameter("wait_timeout_sec").value)
        self.seed = int(self.get_parameter("seed").value)

        self.do_min_dataset_check = bool(self.get_parameter("do_min_dataset_check").value)
        self.dataset_check_samples = int(self.get_parameter("dataset_check_samples").value)

        if self.ckpt_dir == "" or not Path(self.ckpt_dir).is_dir():
            raise FileNotFoundError(f"ckpt_dir not found: {self.ckpt_dir}")
        if self.act_root == "" or not Path(self.act_root).is_dir():
            raise FileNotFoundError(f"act_root not found: {self.act_root}")

        _safe_mkdir(self.out_dir)

        random.seed(self.seed)
        np.random.seed(self.seed)
        if HAVE_TORCH:
            torch.manual_seed(self.seed)

        self.get_logger().info(f"[INFO] ckpt_dir={self.ckpt_dir}")
        self.get_logger().info(f"[INFO] act_root={self.act_root}")
        self.get_logger().info(f"[INFO] out_dir={self.out_dir}")
        self.get_logger().info(f"[INFO] device={self.device}")
        self.get_logger().info(f"[INFO] do_rollout={self.do_rollout} horizon={self.rollout_horizon} action_is_delta={self.action_is_delta}")

        # ---- load stats ----
        self.qpos_stats, self.act_stats, self.stats_obj = load_dataset_stats_pkl(self.ckpt_dir)
        self.state_dim = int(self.qpos_stats.mean.shape[0])
        self.action_dim = int(self.act_stats.mean.shape[0])

        self.get_logger().info(f"[STATS] state_dim={self.state_dim}, action_dim={self.action_dim}")
        self.get_logger().info(f"[STATS] qpos_mean[:5]={self.qpos_stats.mean[:5]}  qpos_std[:5]={self.qpos_stats.std[:5]}")
        self.get_logger().info(f"[STATS] act_mean[:5]={self.act_stats.mean[:5]}    act_std[:5]={self.act_stats.std[:5]}")
        self.get_logger().info(f"[STATS] loaded from ckpt_dir/dataset_stats.pkl")

        # ---- load policy ----
        if not HAVE_TORCH:
            raise RuntimeError("PyTorch is required.")

        ckpt_file = _select_ckpt_file(self.ckpt_dir)
        self.get_logger().info(f"[INFO] ckpt_file={ckpt_file}")
        self.get_logger().info(f"[INFO] ACTPolicy file={Path(self.act_root)/'policy.py'}")

        self.policy = instantiate_act_policy(
            act_root=self.act_root,
            ckpt_dir=self.ckpt_dir,
            device=self.device,
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            camera_names=self.camera_names,
        )

        missing, unexpected = load_policy_weights(self.policy, ckpt_file, self.device)
        self.get_logger().info(f"[INFO] Loaded policy. missing={len(missing)}, unexpected={len(unexpected)}")
        if len(missing) > 0:
            self.get_logger().warn("[WARN] Non-empty missing keys. If behavior is odd, checkpoint/config may not match the model definition.")

        # ---- subscribers (online check) ----
        self.latest_pose = None  # np.ndarray
        self.latest_force = None # np.ndarray
        self.latest_images: Dict[str, Any] = {}

        self.pose_sub = self.create_subscription(Float64MultiArray, self.pose_topic, self._cb_pose, 10)
        self.force_sub = self.create_subscription(Float64MultiArray, self.force_topic, self._cb_force, 10)

        self.img_subs = []
        if HAVE_IMAGE_MSG:
            for t in self.image_topics:
                sub = self.create_subscription(Image, t, lambda msg, topic=t: self._cb_image(msg, topic), 5)
                self.img_subs.append(sub)
        else:
            self.get_logger().warn("[WARN] sensor_msgs/Image not available. Image topics will be ignored.")

        # do minimal dataset check once (optional)
        if self.do_min_dataset_check:
            try:
                self._run_min_dataset_check()
            except Exception as e:
                self.get_logger().warn(f"[WARN] Minimal dataset check skipped/failed: {e}")

        # start main timer
        self.timer = self.create_timer(0.2, self._tick)
        self.did_run = False

        self.get_logger().info("✅ Node ready. Waiting for topics (pose/force/images)...")


    # -----------------
    # Callbacks
    # -----------------
    def _cb_pose(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float64)
        self.latest_pose = arr

    def _cb_force(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float64)
        self.latest_force = arr

    def _cb_image(self, msg: "Image", topic: str):
        # best-effort: decode rgb8/bgr8
        # We'll avoid cv_bridge dependency and decode raw bytes if possible.
        try:
            h = int(msg.height)
            w = int(msg.width)
            enc = str(msg.encoding).lower()
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if "rgb8" in enc or "bgr8" in enc:
                im = data.reshape((h, w, 3))
                if "bgr8" in enc:
                    # convert to rgb for consistency
                    im = im[..., ::-1]
            elif "rgba8" in enc or "bgra8" in enc:
                im = data.reshape((h, w, 4))
                if "bgra8" in enc:
                    im = im[..., [2, 1, 0, 3]]
            else:
                # fallback: try 3-channel
                im = data.reshape((h, w, -1))
                if im.shape[-1] >= 3:
                    im = im[..., :3]
            self.latest_images[topic] = im
        except Exception:
            # ignore
            pass


    # -----------------
    # Minimal dataset sanity check (best-effort)
    # -----------------
    def _run_min_dataset_check(self):
        """
        Best-effort sanity check:
        If dataset_stats.pkl includes small arrays or references, try a tiny eval.
        This is not guaranteed; we keep it minimal and never crash the node.
        """
        out_dir = Path(self.out_dir) / f"min_dataset_check_{_now_string()}"
        _safe_mkdir(str(out_dir))

        obj = self.stats_obj

        # Heuristic: if stats pkl contains sample arrays
        # e.g., "qpos_samples", "act_samples"
        qpos_samples = None
        act_samples = None

        for k in ["qpos_samples", "state_samples", "obs_samples"]:
            if k in obj:
                qpos_samples = _as_np(obj[k])
                break
        for k in ["act_samples", "action_samples", "target_samples"]:
            if k in obj:
                act_samples = _as_np(obj[k])
                break

        if qpos_samples is None or act_samples is None:
            # If it contains paths, we could load them, but format is unknown.
            # Keep it minimal: skip if samples aren't directly in the pkl.
            raise RuntimeError("No direct sample arrays found in dataset_stats.pkl (qpos_samples/act_samples).")

        n = min(self.dataset_check_samples, qpos_samples.shape[0], act_samples.shape[0])
        idx = np.random.choice(qpos_samples.shape[0], size=n, replace=False)

        pred_list = []
        gt_list = []
        for i in idx:
            qpos = qpos_samples[i].astype(np.float64)
            gt_act = act_samples[i].astype(np.float64)

            qn = self.qpos_stats.normalize(qpos)
            # no images in this mini-check
            pred_act_norm = run_policy_once(self.policy, qn, None, self.device)
            pred_act = self.act_stats.denormalize(pred_act_norm)

            pred_list.append(pred_act)
            gt_list.append(gt_act)

        pred = np.stack(pred_list, axis=0)
        gt = np.stack(gt_list, axis=0)

        mse = np.mean((pred - gt) ** 2, axis=0)
        rmse = np.sqrt(mse)

        report = {
            "n": int(n),
            "mse_per_dim": mse.tolist(),
            "rmse_per_dim": rmse.tolist(),
        }
        with open(out_dir / "min_dataset_check.json", "w") as f:
            json.dump(report, f, indent=2)

        # plot rmse
        plt.figure()
        plt.plot(rmse)
        plt.title("RMSE per dim (min dataset check)")
        plt.xlabel("dim")
        plt.ylabel("rmse")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(out_dir / "rmse_per_dim.png", dpi=160)
        plt.close()

        self.get_logger().info(f"[MIN_CHECK] saved: {out_dir}")


    # -----------------
    # Main tick
    # -----------------
    def _tick(self):
        if self.did_run:
            return

        t0 = time.time()
        ok = self._wait_for_first_msgs(timeout=self.wait_timeout_sec)
        if not ok:
            self.get_logger().warn("[WARN] Timeout waiting for pose/force/images. Still trying...")
            return

        # Run once
        try:
            self._run_online_rollout()
            self.did_run = True
            self.get_logger().info("✅ Done. (node will keep alive; Ctrl+C to exit)")
        except Exception as e:
            self.get_logger().error(f"[ERROR] rollout failed: {e}")


    def _wait_for_first_msgs(self, timeout: float) -> bool:
        """
        Wait for at least pose+force.
        Images are optional (but recommended).
        """
        start = time.time()
        while time.time() - start < timeout:
            if self.latest_pose is not None and self.latest_force is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False


    def _compose_qpos(self) -> np.ndarray:
        """
        Compose qpos vector expected by model (len=state_dim).
        Default assumption:
          qpos = [x,y,z, rx,ry,rz, fx,fy,fz]  (len=9)
        """
        pose = np.asarray(self.latest_pose, dtype=np.float64).copy()
        force = np.asarray(self.latest_force, dtype=np.float64).copy()

        # Best-effort: if pose has >=6 and force has >=3
        if pose.shape[0] < 6 or force.shape[0] < 3:
            raise RuntimeError(f"pose/force size too small. pose_len={pose.shape[0]}, force_len={force.shape[0]}")

        x, y, z, rx, ry, rz = pose[:6]
        fx, fy, fz = force[:3]

        qpos = np.array([x, y, z, rx, ry, rz, fx, fy, fz], dtype=np.float64)

        if qpos.shape[0] != self.state_dim:
            # If your model uses different dim, pad/trim best-effort
            if qpos.shape[0] > self.state_dim:
                qpos = qpos[: self.state_dim]
            else:
                qpos = np.pad(qpos, (0, self.state_dim - qpos.shape[0]), mode="constant", constant_values=0.0)
        return qpos


    def _get_images_ordered(self) -> List[np.ndarray]:
        """
        Return images in the order of self.image_topics (best-effort).
        """
        imgs = []
        for t in self.image_topics:
            imgs.append(self.latest_images.get(t, None))
        return imgs


    def _run_online_rollout(self):
        """
        Main debug: take current obs, run rollout_horizon steps.
        Save:
          - rollout.csv (qpos, pred_action, denorm_action, etc.)
          - plots (per-dim trajectories)
        """
        run_dir = Path(self.out_dir) / f"rollout_{_now_string()}"
        _safe_mkdir(str(run_dir))

        qpos0 = self._compose_qpos()
        imgs = self._get_images_ordered()
        img_tensor = _prep_images(imgs, self.device)

        # Normalize initial
        qpos_curr = qpos0.copy()
        qpos_traj = [qpos_curr.copy()]

        act_norm_traj = []
        act_denorm_traj = []

        # For delta mode: interpret denorm action as delta or absolute?
        # We provide both interpretations in outputs.
        qpos_acc_delta = qpos_curr.copy()
        qpos_traj_delta = [qpos_acc_delta.copy()]

        for t in range(self.rollout_horizon):
            qn = self.qpos_stats.normalize(qpos_curr)
            act_norm = run_policy_once(self.policy, qn, img_tensor, self.device)
            act_denorm = self.act_stats.denormalize(act_norm)

            act_norm_traj.append(act_norm.copy())
            act_denorm_traj.append(act_denorm.copy())

            # Interpretation A: action is absolute "next qpos"
            qpos_next_abs = act_denorm.copy()
            if qpos_next_abs.shape[0] != self.state_dim:
                if qpos_next_abs.shape[0] > self.state_dim:
                    qpos_next_abs = qpos_next_abs[: self.state_dim]
                else:
                    qpos_next_abs = np.pad(qpos_next_abs, (0, self.state_dim - qpos_next_abs.shape[0]), mode="constant")

            # Interpretation B: action is delta in qpos space
            qpos_next_delta = qpos_acc_delta + act_denorm
            qpos_acc_delta = qpos_next_delta
            qpos_traj_delta.append(qpos_acc_delta.copy())

            # Choose rollout update based on param
            if self.action_is_delta:
                qpos_curr = qpos_next_delta
            else:
                qpos_curr = qpos_next_abs

            qpos_traj.append(qpos_curr.copy())

            if self.sleep_between_steps_sec > 0.0:
                time.sleep(self.sleep_between_steps_sec)

        qpos_traj = np.stack(qpos_traj, axis=0)                       # [T+1, D]
        qpos_traj_delta = np.stack(qpos_traj_delta, axis=0)           # [T+1, D]
        act_norm_traj = np.stack(act_norm_traj, axis=0)               # [T, A]
        act_denorm_traj = np.stack(act_denorm_traj, axis=0)           # [T, A]

        # Save csv
        csv_path = run_dir / "rollout.csv"
        with open(csv_path, "w") as f:
            header = []
            for i in range(self.state_dim):
                header.append(f"qpos_{i}")
            for i in range(self.state_dim):
                header.append(f"qpos_delta_interp_{i}")
            for i in range(self.action_dim):
                header.append(f"act_norm_{i}")
            for i in range(self.action_dim):
                header.append(f"act_denorm_{i}")
            f.write(",".join(header) + "\n")

            T = self.rollout_horizon
            for t in range(T):
                row = []
                row += qpos_traj[t].tolist()
                row += qpos_traj_delta[t].tolist()
                row += act_norm_traj[t].tolist()
                row += act_denorm_traj[t].tolist()
                f.write(",".join([f"{x:.8f}" for x in row]) + "\n")

        # Save summary json
        summary = {
            "ckpt_dir": self.ckpt_dir,
            "act_root": self.act_root,
            "device": self.device,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "action_is_delta_param": bool(self.action_is_delta),
            "rollout_horizon": int(self.rollout_horizon),
            "pose_topic": self.pose_topic,
            "force_topic": self.force_topic,
            "image_topics": self.image_topics,
            "camera_names": self.camera_names,
            "qpos0": qpos0.tolist(),
        }
        with open(run_dir / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)

        # Plots
        self._plot_trajectories(run_dir, qpos_traj, qpos_traj_delta, act_denorm_traj)

        self.get_logger().info(f"[SAVE] {run_dir}")
        self.get_logger().info(f"[SAVE] csv={csv_path}")


    def _plot_trajectories(self, run_dir: Path, qpos_traj: np.ndarray, qpos_traj_delta: np.ndarray, act_denorm_traj: np.ndarray):
        """
        Create plots:
          - qpos over time (using chosen action_is_delta)
          - also plot delta-interp trajectory for comparison
          - plot per-dim action (denorm)
        """
        T1 = qpos_traj.shape[0]   # T+1
        T = act_denorm_traj.shape[0]

        t_q = np.arange(T1)
        t_a = np.arange(T)

        # If typical 9D: label
        labels = [f"dim{i}" for i in range(self.state_dim)]
        if self.state_dim == 9:
            labels = ["x", "y", "z", "rx", "ry", "rz", "fx", "fy", "fz"]

        # qpos
        for i in range(self.state_dim):
            plt.figure()
            plt.plot(t_q, qpos_traj[:, i], label="rollout(qpos)")
            plt.plot(t_q, qpos_traj_delta[:, i], linestyle="--", label="delta-interp(qpos)")
            plt.title(f"qpos trajectory: {labels[i]}")
            plt.xlabel("step")
            plt.ylabel(labels[i])
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(run_dir / f"qpos_{i:02d}_{labels[i]}.png", dpi=160)
            plt.close()

        # action denorm
        for i in range(min(self.action_dim, self.state_dim)):
            plt.figure()
            plt.plot(t_a, act_denorm_traj[:, i], label="act_denorm")
            plt.title(f"action (denorm): {labels[i]}")
            plt.xlabel("step")
            plt.ylabel(labels[i])
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(run_dir / f"act_denorm_{i:02d}_{labels[i]}.png", dpi=160)
            plt.close()

        # magnitude summaries (helps detect "collapsed" outputs)
        act_std = np.std(act_denorm_traj, axis=0)
        qpos_delta = np.diff(qpos_traj, axis=0)
        qpos_step_std = np.std(qpos_delta, axis=0)

        plt.figure()
        plt.plot(act_std, label="std(act_denorm)")
        plt.plot(qpos_step_std, label="std(step_delta_qpos)")
        plt.title("Per-dim std: action vs step-delta(qpos)")
        plt.xlabel("dim")
        plt.ylabel("std")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(run_dir / "std_summary.png", dpi=160)
        plt.close()


# -----------------------------
# main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ActCheckInferenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
