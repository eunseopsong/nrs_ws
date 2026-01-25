#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import uuid
from types import SimpleNamespace
from typing import Optional, Dict, List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge


# ----------------------------
# Utils
# ----------------------------
def _ensure_uint8_3ch(img: np.ndarray) -> np.ndarray:
    """(H,W,3) uint8로 정규화. bgr8 기준."""
    if img is None:
        return img
    if img.dtype != np.uint8:
        # scale/clip
        img = np.clip(img, 0, 255).astype(np.uint8)

    if img.ndim == 2:
        img = np.repeat(img[..., None], 3, axis=2)
    elif img.ndim == 3 and img.shape[2] == 1:
        img = np.repeat(img, 3, axis=2)
    elif img.ndim == 3 and img.shape[2] > 3:
        img = img[:, :, :3]
    return img


def _resize_to(img: np.ndarray, h: int, w: int) -> np.ndarray:
    import cv2
    if img.shape[0] == h and img.shape[1] == w:
        return img
    return cv2.resize(img, (w, h))


# ----------------------------
# ACT Backend (build_act_model 기반)
# ----------------------------
class ActBackend:
    """
    - nrs_act repo의 detr_vae build()를 이용해 모델 생성
    - ckpt_dir/policy_best.ckpt 로드
    - ckpt_dir/dataset_stats.pkl 로드 (action_mean/std)
    """
    def __init__(
        self,
        ckpt_dir: str,
        device: str,
        nrs_act_root: str,
        camera_names: List[str],
        # model hyperparams (학습과 맞춰야 함)
        backbone: str = "resnet18",
        hidden_dim: int = 512,
        dim_feedforward: int = 3200,
        num_queries: int = 100,   # chunk_size(H)
        enc_layers: int = 4,
        dec_layers: int = 7,
        nheads: int = 8,
        kl_weight: int = 10,
    ):
        self.ckpt_dir = ckpt_dir
        self.device = device
        self.nrs_act_root = nrs_act_root
        self.camera_names = camera_names

        self.backbone = backbone
        self.hidden_dim = hidden_dim
        self.dim_feedforward = dim_feedforward
        self.num_queries = num_queries
        self.enc_layers = enc_layers
        self.dec_layers = dec_layers
        self.nheads = nheads
        self.kl_weight = kl_weight

        self.policy = None
        self.action_mean = None
        self.action_std = None

    def _prepare_syspath(self):
        root = self.nrs_act_root
        sys.path.extend(
            [
                root,
                os.path.join(root, "act"),
                os.path.join(root, "act", "model"),
                os.path.join(root, "act", "detr"),
                os.path.join(root, "act", "detr", "util"),
                os.path.join(root, "custom"),
            ]
        )

    def load(self):
        if not self.ckpt_dir:
            raise RuntimeError("ckpt_dir is empty")

        ckpt_path = os.path.join(self.ckpt_dir, "policy_best.ckpt")
        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"policy_best.ckpt not found: {ckpt_path}")

        stats_path = os.path.join(self.ckpt_dir, "dataset_stats.pkl")
        if not os.path.exists(stats_path):
            raise FileNotFoundError(f"dataset_stats.pkl not found: {stats_path}")

        # imports from nrs_act
        self._prepare_syspath()
        import torch
        import pickle
        from act.detr.models.detr_vae import build as build_act_model  # noqa

        # build args (학습 설정과 맞춰야 함)
        args = SimpleNamespace(
            lr=1e-5,
            lr_backbone=1e-5,
            batch_size=1,
            weight_decay=1e-4,
            epochs=1,
            lr_drop=200,
            clip_max_norm=0.1,

            hidden_dim=self.hidden_dim,
            dim_feedforward=self.dim_feedforward,
            num_queries=self.num_queries,
            backbone=self.backbone,

            position_embedding="sine",
            dilation=False,
            masks=False,

            enc_layers=self.enc_layers,
            dec_layers=self.dec_layers,
            nheads=self.nheads,
            dropout=0.1,
            pre_norm=False,

            eval=True,
            camera_names=self.camera_names,
            kl_weight=self.kl_weight,
        )

        device = self.device
        if device == "cuda":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        dev = torch.device(device)

        policy = build_act_model(args)
        ckpt = torch.load(ckpt_path, map_location=dev)
        state_dict = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt
        policy.load_state_dict(state_dict, strict=False)
        policy.to(dev).eval()

        with open(stats_path, "rb") as f:
            stats = pickle.load(f)

        # action mean/std (있으면 denorm)
        if "action_mean" in stats and "action_std" in stats:
            self.action_mean = np.array(stats["action_mean"], dtype=np.float32)
            self.action_std = np.array(stats["action_std"], dtype=np.float32)

        self.policy = policy
        return ckpt_path

    def infer_chunk(self, pose6: np.ndarray, imgs_bgr: List[np.ndarray]) -> np.ndarray:
        """
        입력:
          pose6: (6,) float32/64  [x_mm,y_mm,z_mm, wx_rad,wy_rad,wz_rad]
          imgs_bgr: [img0(H,W,3), img1(H,W,3)]  uint8 bgr

        출력:
          chunk: (H, D) in REAL space (denorm applied if stats present)
        """
        if self.policy is None:
            raise RuntimeError("policy not loaded")

        import torch

        # 이미지 전처리
        img0 = _ensure_uint8_3ch(imgs_bgr[0])
        img1 = _ensure_uint8_3ch(imgs_bgr[1])
        if img0 is None or img1 is None:
            raise RuntimeError("images are not ready")

        # 해상도 맞추기
        h, w = img0.shape[:2]
        if img1.shape[:2] != (h, w):
            img1 = _resize_to(img1, h, w)

        # (1,2,3,H,W)
        t0 = torch.from_numpy(img0).permute(2, 0, 1).float() / 255.0
        t1 = torch.from_numpy(img1).permute(2, 0, 1).float() / 255.0
        cams = torch.stack([t0, t1], dim=0).unsqueeze(0)

        pose_t = torch.tensor(pose6, dtype=torch.float32).unsqueeze(0)

        dev = next(self.policy.parameters()).device
        cams = cams.to(dev)
        pose_t = pose_t.to(dev)

        with torch.no_grad():
            out = self.policy(pose_t, cams)
            action_t = out[0] if isinstance(out, tuple) else out

        action_np = action_t.detach().cpu().numpy()  # (1, H*D) or (1,H,D) 등
        action_np = action_np.reshape(action_np.shape[0], -1)  # (1, *)
        # D는 checkpoint/모델에 따라 달라서, H는 num_queries로 고정되어 있다고 가정
        H = self.num_queries
        D = action_np.shape[1] // H
        action_np = action_np.reshape(H, D).astype(np.float32)  # (H,D) in norm space

        # denorm
        if self.action_mean is not None and self.action_std is not None:
            if len(self.action_mean) == D:
                action_np = action_np * self.action_std[None, :] + self.action_mean[None, :]
            # 차원이 다르면 그냥 norm 그대로 사용(로그로 확인 권장)

        return action_np  # (H,D) real-ish


# ----------------------------
# ROS2 Node (auto-run)
# ----------------------------
class ActKeyboardInferNode(Node):
    def __init__(self):
        super().__init__("act_keyboard_infer_node")

        # -------- params
        self.declare_parameter("ckpt_dir", "")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("nrs_act_root", "/home/eunseop/nrs_lab2/nrs_act")

        self.declare_parameter("hz", 125.0)
        self.declare_parameter("max_steps", 4000)

        self.declare_parameter("pose_topic", "/ur10skku/currentP")   # Float64MultiArray (len>=6)
        self.declare_parameter("force_topic", "/ur10skku/currentF")  # Float64MultiArray (freshness만)
        self.declare_parameter("image_topics", ["/realsense/top/color/image_raw", "/realsense/ee/color/image_raw"])

        self.declare_parameter("traj_state_topic", "/act_infer/traj_state")
        self.declare_parameter("traj_point_topic", "/act_infer/traj_point")

        self.declare_parameter("currentP_angle_deg", True)

        # model hyperparams (학습과 맞추기)
        self.declare_parameter("camera_names", ["cam_front", "cam_head"])
        self.declare_parameter("backbone", "resnet18")
        self.declare_parameter("hidden_dim", 512)
        self.declare_parameter("dim_feedforward", 3200)
        self.declare_parameter("num_queries", 100)
        self.declare_parameter("enc_layers", 4)
        self.declare_parameter("dec_layers", 7)
        self.declare_parameter("nheads", 8)
        self.declare_parameter("kl_weight", 10)

        # 실행 모드
        self.declare_parameter("auto_start", True)
        self.declare_parameter("run_once_and_exit", True)
        self.declare_parameter("start_delay_sec", 0.5)
        self.declare_parameter("require_fresh_sec", 0.5)

        # publish 신뢰성(4000개 drop 방지)
        self.declare_parameter("point_qos_depth", 20000)
        self.declare_parameter("publish_sleep", False)        # True면 125Hz로 실제 sleep
        self.declare_parameter("exit_delay_sec", 2.0)         # publish 후 DDS flush 대기

        # -------- read params
        self.ckpt_dir = str(self.get_parameter("ckpt_dir").value).strip()
        self.device = str(self.get_parameter("device").value)
        self.nrs_act_root = str(self.get_parameter("nrs_act_root").value)

        self.hz = float(self.get_parameter("hz").value)
        self.max_steps = int(self.get_parameter("max_steps").value)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)
        self.image_topics = list(self.get_parameter("image_topics").value)

        self.traj_state_topic = str(self.get_parameter("traj_state_topic").value)
        self.traj_point_topic = str(self.get_parameter("traj_point_topic").value)

        self.currentP_angle_deg = bool(self.get_parameter("currentP_angle_deg").value)

        self.camera_names = list(self.get_parameter("camera_names").value)
        self.backbone = str(self.get_parameter("backbone").value)
        self.hidden_dim = int(self.get_parameter("hidden_dim").value)
        self.dim_feedforward = int(self.get_parameter("dim_feedforward").value)
        self.num_queries = int(self.get_parameter("num_queries").value)
        self.enc_layers = int(self.get_parameter("enc_layers").value)
        self.dec_layers = int(self.get_parameter("dec_layers").value)
        self.nheads = int(self.get_parameter("nheads").value)
        self.kl_weight = int(self.get_parameter("kl_weight").value)

        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.run_once_and_exit = bool(self.get_parameter("run_once_and_exit").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.require_fresh_sec = float(self.get_parameter("require_fresh_sec").value)

        self.point_qos_depth = int(self.get_parameter("point_qos_depth").value)
        # max_steps보다 작으면 강제로 올림
        if self.point_qos_depth < self.max_steps + 100:
            self.point_qos_depth = self.max_steps + 100

        self.publish_sleep = bool(self.get_parameter("publish_sleep").value)
        self.exit_delay_sec = float(self.get_parameter("exit_delay_sec").value)

        if len(self.image_topics) != 2:
            raise RuntimeError("image_topics는 반드시 2개여야 함 (cam0, cam1)")
        if len(self.camera_names) != 2:
            raise RuntimeError("camera_names는 반드시 2개여야 함 (학습 camera_names 대응용)")

        # -------- QoS
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_point = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.point_qos_depth,
        )

        # -------- pubs
        self.pub_state = self.create_publisher(String, self.traj_state_topic, qos_state)
        self.pub_point = self.create_publisher(Float64MultiArray, self.traj_point_topic, qos_point)

        # -------- subs
        self._bridge = CvBridge()
        self._latest_pose6: Optional[np.ndarray] = None
        self._latest_force3: Optional[np.ndarray] = None
        self._latest_imgs: Dict[str, np.ndarray] = {}
        self._t_pose = 0.0
        self._t_force = 0.0
        self._t_img: Dict[str, float] = {}

        self.create_subscription(Float64MultiArray, self.pose_topic, self.cb_pose, 50)
        self.create_subscription(Float64MultiArray, self.force_topic, self.cb_force, 50)

        for topic in self.image_topics:
            self.create_subscription(Image, topic, lambda m, t=topic: self.cb_img(m, t), 10)

        # -------- backend
        self.backend = ActBackend(
            ckpt_dir=self.ckpt_dir,
            device=self.device,
            nrs_act_root=self.nrs_act_root,
            camera_names=self.camera_names,
            backbone=self.backbone,
            hidden_dim=self.hidden_dim,
            dim_feedforward=self.dim_feedforward,
            num_queries=self.num_queries,
            enc_layers=self.enc_layers,
            dec_layers=self.dec_layers,
            nheads=self.nheads,
            kl_weight=self.kl_weight,
        )

        if not self.ckpt_dir:
            self.get_logger().error("ckpt_dir 파라미터가 비어있음. 예: -p ckpt_dir:=/home/eunseop/.../0124_1904")
        else:
            try:
                ckpt_path = self.backend.load()
                self.get_logger().info(f"[ACT] loaded: {ckpt_path}")
            except Exception as e:
                self.get_logger().error(f"[ACT] backend.load() failed: {e}")

        self.get_logger().info(f"pose_topic={self.pose_topic}, force_topic={self.force_topic}")
        self.get_logger().info(f"image_topics={self.image_topics}")
        self.get_logger().info(f"publish: state={self.traj_state_topic}, point={self.traj_point_topic}")
        self.get_logger().info(f"QoS: point_depth={self.point_qos_depth}, publish_sleep={self.publish_sleep}, exit_delay={self.exit_delay_sec}s")

        # auto-start timer
        if self.auto_start:
            self.create_timer(self.start_delay_sec, self._auto_start_once)

        self._started = False

    # -------- callbacks
    def cb_pose(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        x, y, z, wx, wy, wz = msg.data[:6]
        if self.currentP_angle_deg:
            wx = np.deg2rad(wx)
            wy = np.deg2rad(wy)
            wz = np.deg2rad(wz)
        self._latest_pose6 = np.array([x, y, z, wx, wy, wz], dtype=np.float32)
        self._t_pose = time.time()

    def cb_force(self, msg: Float64MultiArray):
        # currentF가 [Fx,Fy,Fz,Mx,My,Mz]이든 [Fx,Fy,Fz]이든 앞 3개만 사용
        if len(msg.data) < 3:
            return
        fx, fy, fz = msg.data[:3]
        self._latest_force3 = np.array([fx, fy, fz], dtype=np.float32)
        self._t_force = time.time()

    def cb_img(self, msg: Image, topic_name: str):
        try:
            cv = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._latest_imgs[topic_name] = cv
            self._t_img[topic_name] = time.time()
        except Exception:
            return

    # -------- auto start
    def _auto_start_once(self):
        if self._started:
            return
        self._started = True

        try:
            self._do_infer_and_publish()
        except Exception as e:
            self.get_logger().error(f"[INFER] failed: {e}")

        if self.run_once_and_exit:
            self.get_logger().info(f"[EXIT] run_once_and_exit=true -> sleep {self.exit_delay_sec:.2f}s then shutdown")
            time.sleep(max(0.0, self.exit_delay_sec))
            rclpy.shutdown()

    # -------- inference core
    def _wait_inputs_ready(self, timeout_sec: float = 10.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec and rclpy.ok():
            now = time.time()

            pose_ok = (self._latest_pose6 is not None) and ((now - self._t_pose) <= self.require_fresh_sec)
            # force는 freshness만 (없어도 진행은 가능하게)
            force_ok = (self._latest_force3 is not None) and ((now - self._t_force) <= self.require_fresh_sec)

            img_ok = True
            for t in self.image_topics:
                if t not in self._latest_imgs:
                    img_ok = False
                    break
                if (now - self._t_img.get(t, 0.0)) > self.require_fresh_sec:
                    img_ok = False
                    break

            if pose_ok and img_ok:
                return True

            time.sleep(0.02)

        raise TimeoutError("inputs not ready/fresh (pose/images). timeout.")

    def _infer_traj_9d(self) -> np.ndarray:
        """
        rollout 방식:
          - 모델 1회 -> (H,D) chunk
          - chunk를 누적해서 max_steps까지 채움
          - 다음 chunk 입력 pose는 직전 chunk의 마지막 pose(앞 6개)로 업데이트
        최종:
          traj9d[k] = [x_mm,y_mm,z_mm, wx_rad,wy_rad,wz_rad, fx,fy,fz]
        """
        if self.backend.policy is None:
            raise RuntimeError("ActBackend: policy not loaded")

        # 최신 스냅샷
        pose_in = self._latest_pose6.copy()  # (6,)
        img0 = self._latest_imgs[self.image_topics[0]]
        img1 = self._latest_imgs[self.image_topics[1]]

        traj_rows: List[np.ndarray] = []
        remain = self.max_steps

        while remain > 0:
            chunk = self.backend.infer_chunk(pose_in, [img0, img1])  # (H,D)
            if chunk.ndim != 2:
                raise RuntimeError(f"chunk shape invalid: {chunk.shape}")

            H, D = chunk.shape

            # D < 9 이면 padding (postprocessor는 9D를 기대)
            if D < 9:
                pad = np.zeros((H, 9), dtype=np.float32)
                pad[:, :D] = chunk
                chunk9 = pad
            else:
                chunk9 = chunk[:, :9].astype(np.float32)

            take = min(remain, H)
            traj_rows.append(chunk9[:take])
            remain -= take

            # 다음 입력 pose는 chunk의 마지막 pose
            pose_in = chunk9[take - 1, :6].copy()

        traj = np.vstack(traj_rows)  # (max_steps,9)
        return traj

    def _do_infer_and_publish(self):
        self._wait_inputs_ready(timeout_sec=10.0)

        traj_id = uuid.uuid4().hex[:8]
        self.get_logger().info(f"[INFER] start id={traj_id} hz={self.hz} max_steps={self.max_steps}")

        traj9d = self._infer_traj_9d()
        if traj9d.shape != (self.max_steps, 9):
            raise RuntimeError(f"traj9d shape mismatch: {traj9d.shape} vs ({self.max_steps},9)")

        # START
        st = String()
        st.data = f"START id={traj_id} hz={self.hz:.6f} N={traj9d.shape[0]}"
        self.pub_state.publish(st)

        # points
        msg = Float64MultiArray()
        sleep_dt = 1.0 / self.hz if (self.publish_sleep and self.hz > 1e-6) else 0.0

        for k in range(traj9d.shape[0]):
            msg.data = traj9d[k].tolist()
            self.pub_point.publish(msg)
            if sleep_dt > 0.0:
                time.sleep(sleep_dt)

        # END
        ed = String()
        ed.data = f"END id={traj_id} N={traj9d.shape[0]}"
        self.pub_state.publish(ed)

        self.get_logger().info(f"[INFER] done id={traj_id} published {traj9d.shape[0]} points")


def main(args=None):
    rclpy.init(args=args)
    node = ActKeyboardInferNode()
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
