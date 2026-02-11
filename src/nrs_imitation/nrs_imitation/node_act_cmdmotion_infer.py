#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import inspect
from types import SimpleNamespace

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image


def pretty_vec(v, fmt="{:+.3f}"):
    v = np.asarray(v).reshape(-1,)
    return "[" + ",".join(fmt.format(float(x)) for x in v.tolist()) + "]"


class NodeCheckInference(Node):
    """
    요구사항:
      - 현재 state (x y z rx ry rz fx fy fz + rgb image) 단 1번 스냅샷
      - ckpt로부터 100-step action(absolute) trajectory를 한 번에 출력
      - Fx Fy Fz만 사용 (원래 6D force여도 3D만)
      - 실행이 최우선 (불필요 기능 제거)
    """

    def __init__(self):
        super().__init__("node_check_inference")

        # ---------------- params ----------------
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("act_root", "")
        self.declare_parameter("chunk_size", 100)

        self.declare_parameter("kl_weight", 10.0)
        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("no_pretrained", False)

        self.declare_parameter("pred_step_offset", 1)  # 참고용
        self.declare_parameter("contact_fz_threshold", 5.0)

        self.declare_parameter("dump_full", False)
        self.declare_parameter("dump_head_n", 10)
        self.declare_parameter("dump_tail_n", 10)

        self.declare_parameter("pose_topic", "/calibrated_pose")
        self.declare_parameter("force_topic", "/ftsensor/measured_Cvalue")
        self.declare_parameter("image_topic", "/camera/color/image_raw")

        # ---------------- load params ----------------
        self.ckpt_dir = self.get_parameter("ckpt_dir").value
        self.act_root = self.get_parameter("act_root").value
        self.chunk_size = int(self.get_parameter("chunk_size").value)

        self.kl_weight = float(self.get_parameter("kl_weight").value)
        self.hidden_dim = int(self.get_parameter("hidden_dim").value)
        self.dim_feedforward = int(self.get_parameter("dim_feedforward").value)
        self.no_pretrained = bool(self.get_parameter("no_pretrained").value)

        self.pred_step_offset = int(self.get_parameter("pred_step_offset").value)
        self.contact_fz_thr = float(self.get_parameter("contact_fz_threshold").value)

        self.dump_full = bool(self.get_parameter("dump_full").value)
        self.dump_head_n = int(self.get_parameter("dump_head_n").value)
        self.dump_tail_n = int(self.get_parameter("dump_tail_n").value)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.force_topic = self.get_parameter("force_topic").value
        self.image_topic = self.get_parameter("image_topic").value

        # ---------------- checks ----------------
        if not self.ckpt_dir or not os.path.isdir(self.ckpt_dir):
            raise RuntimeError(f"ckpt_dir invalid: {self.ckpt_dir}")
        self.ckpt_path = os.path.join(self.ckpt_dir, "policy_best.ckpt")
        if not os.path.isfile(self.ckpt_path):
            raise RuntimeError(f"Checkpoint not found: {self.ckpt_path}")

        # ---------------- device ----------------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"[INFO] Using device: {self.device}")

        # ---------------- sys.path (ACT import path) ----------------
        self._append_act_paths()
        self.get_logger().info(f"[ACT] sys.path[0:5]={sys.path[0:5]}")

        # ---------------- load policy (핵심: args 1개로만 생성) ----------------
        self.policy = self._load_policy_args_style()

        self.get_logger().info("[INFO] ✅ Model ready. Waiting for first state+image... (dump once)")

        # ---------------- buffers ----------------
        self.pose6 = None   # xyz rpy (6)
        self.frc3 = None    # Fx Fy Fz (3)
        self.img = None     # H W 3 uint8
        self.dumped_once = False

        # ---------------- subs ----------------
        self.create_subscription(Float64MultiArray, self.pose_topic, self._cb_pose, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, self.force_topic, self._cb_force, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._cb_image, qos_profile_sensor_data)

        # ---------------- loop ----------------
        self.timer = self.create_timer(0.02, self._loop)  # 50 Hz

    def _append_act_paths(self):
        if not self.act_root:
            return
        paths = [
            os.path.join(self.act_root, "custom"),
            self.act_root,
            os.path.join(self.act_root, "act"),
            os.path.join(self.act_root, "act", "detr"),
        ]
        for p in paths[::-1]:
            if p and os.path.isdir(p) and (p not in sys.path):
                sys.path.insert(0, p)

    def _cb_pose(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32).reshape(-1,)
        if arr.size >= 6:
            self.pose6 = arr[:6].copy()

    def _cb_force(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32).reshape(-1,)
        if arr.size >= 3:
            # IMPORTANT: Fx Fy Fz만 사용
            self.frc3 = arr[:3].copy()

    def _cb_image(self, msg: Image):
        try:
            h, w = msg.height, msg.width
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if data.size < h * w * 3:
                return
            self.img = data[: h * w * 3].reshape(h, w, 3).copy()
        except Exception:
            return

    def _ready(self):
        return (self.pose6 is not None) and (self.frc3 is not None) and (self.img is not None)

    def _load_policy_args_style(self):
        """
        ACTPolicy.__init__(self, args) 형태에 맞춰서 무조건 args 1개로 생성.
        """
        self.get_logger().info("[INFO] Loading policy (training-time custom policy.py)...")

        from policy import ACTPolicy  # act_root/custom/policy.py 를 타게 됨

        sig = inspect.signature(ACTPolicy.__init__)
        params = list(sig.parameters.values())

        # self 제외하면 1개여야 함 (지금 에러가 이 케이스)
        if len(params) != 2:
            raise RuntimeError(f"ACTPolicy.__init__ signature unexpected: {sig}")

        # args/config 객체를 만들어서 전달
        # (policy.py에서 필요한 필드가 더 있어도, getattr로 읽는 형태면 이걸로 충분)
        args = SimpleNamespace()
        args.kl_weight = self.kl_weight
        args.hidden_dim = self.hidden_dim
        args.dim_feedforward = self.dim_feedforward
        args.no_pretrained = self.no_pretrained

        # 혹시 policy가 기대할 만한 흔한 필드들(있어도 해 없고, 없어서 터지는 경우 방지)
        args.backbone = "resnet18"
        args.lr_backbone = 0.0
        args.num_queries = 100
        args.camera_names = ["rgb"]
        args.state_dim = 9
        args.action_dim = 9

        policy = ACTPolicy(args)
        policy.to(self.device)
        policy.eval()

        ckpt = torch.load(self.ckpt_path, map_location=self.device)
        state_dict = ckpt.get("state_dict", ckpt)

        missing, unexpected = policy.load_state_dict(state_dict, strict=False)
        self.get_logger().info(
            f"[INFO] Loaded policy from {self.ckpt_path}. missing={len(missing)}, unexpected={len(unexpected)}"
        )
        return policy

    @torch.no_grad()
    def _infer_trajectory(self, qpos0: np.ndarray, img0: np.ndarray, T: int) -> np.ndarray:
        """
        qpos0: (9,)  [xyz rpy fx fy fz]
        img0:  (H,W,3) uint8
        return: (T,9) predicted action trajectory
        """
        qpos_seq = np.tile(qpos0.reshape(1, 9), (T, 1)).astype(np.float32)  # (T,9)
        img_seq = np.tile(img0.reshape(1, *img0.shape), (T, 1, 1, 1)).astype(np.float32) / 255.0  # (T,H,W,3)

        # to tensor
        q = torch.from_numpy(qpos_seq).to(self.device).unsqueeze(0)  # (1,T,9)
        img = np.transpose(img_seq, (0, 3, 1, 2))                   # (T,3,H,W)
        img = torch.from_numpy(img).to(self.device).unsqueeze(0)    # (1,T,3,H,W)

        out = self.policy(q, img)

        # unwrap common outputs
        if isinstance(out, dict):
            for k in ["a_hat", "actions", "pred", "action", "out"]:
                if k in out:
                    out = out[k]
                    break
        elif isinstance(out, (tuple, list)):
            out = out[0]

        if isinstance(out, torch.Tensor) and out.dim() == 3:
            out = out[0]  # (T,9)

        if not isinstance(out, torch.Tensor):
            raise RuntimeError(f"Policy output is not Tensor. type={type(out)}")
        pred = out.detach().float().cpu().numpy()

        if pred.shape != (T, 9):
            raise RuntimeError(f"Unexpected output shape: {pred.shape}, expected {(T,9)}")
        return pred

    def _dump(self, traj: np.ndarray, qpos0: np.ndarray, meas_fz: float):
        T = traj.shape[0]
        contact = 1 if (meas_fz > self.contact_fz_thr) else 0
        off = max(0, min(T - 1, int(self.pred_step_offset)))

        self.get_logger().info(
            f"[DUMP] contact={contact} meas_fz={meas_fz:+.3f} | qpos0={pretty_vec(qpos0)} | T={T} | next_idx={off}"
        )
        self.get_logger().info(
            f"[DUMP] pred_fz: min={float(np.min(traj[:,8])):+.3f} max={float(np.max(traj[:,8])):+.3f} mean={float(np.mean(traj[:,8])):+.3f}"
        )
        self.get_logger().info(f"[DUMP] pred_next(step={off}) = {pretty_vec(traj[off])}")

        if self.dump_full:
            lines = []
            for i in range(T):
                v = traj[i]
                lines.append(
                    f"{i:03d}: xyz={pretty_vec(v[0:3])} rpy={pretty_vec(v[3:6])} f={pretty_vec(v[6:9])}"
                )
            # log truncation 방지: 25줄씩
            for k in range(0, len(lines), 25):
                self.get_logger().info("[TRAJ]\n" + "\n".join(lines[k:k+25]))
        else:
            hn = max(0, min(T, self.dump_head_n))
            tn = max(0, min(T - hn, self.dump_tail_n))

            if hn > 0:
                lines = []
                for i in range(hn):
                    v = traj[i]
                    lines.append(
                        f"{i:03d}: xyz={pretty_vec(v[0:3])} rpy={pretty_vec(v[3:6])} f={pretty_vec(v[6:9])}"
                    )
                self.get_logger().info("[TRAJ:HEAD]\n" + "\n".join(lines))

            if tn > 0:
                lines = []
                for i in range(T - tn, T):
                    v = traj[i]
                    lines.append(
                        f"{i:03d}: xyz={pretty_vec(v[0:3])} rpy={pretty_vec(v[3:6])} f={pretty_vec(v[6:9])}"
                    )
                self.get_logger().info("[TRAJ:TAIL]\n" + "\n".join(lines))

    def _loop(self):
        if self.dumped_once:
            return
        if not self._ready():
            return

        # snapshot once
        qpos0 = np.concatenate([self.pose6, self.frc3], axis=0).astype(np.float32)  # (9,)
        meas_fz = float(self.frc3[2])
        img0 = self.img

        try:
            traj = self._infer_trajectory(qpos0, img0, int(self.chunk_size))
            self._dump(traj, qpos0, meas_fz)
            self.dumped_once = True
            self.get_logger().info("[DONE] dumped once. (node keeps running)")
        except Exception as e:
            self.get_logger().error(f"[FAIL] inference failed: {e}")
            self.dumped_once = True


def main():
    rclpy.init()
    node = NodeCheckInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
