#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading
from datetime import datetime
import pickle

import numpy as np
import torch
import cv2
import h5py
from cv_bridge import CvBridge

# ======================
# 경로 / 설정
# ======================
ROOT_DIR = "/home/eunseop/nrs_lab2/nrs_act"
CKPT_ROOT = "/home/eunseop/nrs_lab2/checkpoints/ur10e_swing"
DATASET_EP_DIR = "/home/eunseop/nrs_lab2/datasets/ACT/1114_1643/episodes"
LOG_DIR = "/home/eunseop/nrs_lab2/analysis_logs"

# 비교할 데모 에피소드 인덱스 (episode_0.hdf5, episode_10.hdf5 등)
DEMO_EP_IDX = 0  # 필요하면 나중에 바꿔서 사용

sys.path.extend(
    [
        ROOT_DIR,
        os.path.join(ROOT_DIR, "act"),
        os.path.join(ROOT_DIR, "act", "model"),
        os.path.join(ROOT_DIR, "act", "detr"),
        os.path.join(ROOT_DIR, "act", "detr", "util"),
        os.path.join(ROOT_DIR, "custom"),
    ]
)

from act.detr.models.detr_vae import build as build_act_model  # noqa: E402
from custom_constants import TASK_CONFIGS  # noqa: E402

# ======================
# ROS2 관련 임포트
# ======================
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, JointState


# ==============================================================
# 유틸: 최신 체크포인트 폴더 찾기 (MMDD_HHMM 형식)
# ==============================================================
def find_latest_ckpt_dir(root_dir: str) -> str:
    if not os.path.isdir(root_dir):
        return root_dir

    candidates = []
    for name in os.listdir(root_dir):
        full = os.path.join(root_dir, name)
        if not os.path.isdir(full):
            continue
        # "MMDD_HHMM" 형식만 후보
        if len(name) == 9 and name[4] == "_":
            mmdd = name[:4]
            hhmm = name[5:]
            if mmdd.isdigit() and hhmm.isdigit():
                candidates.append(name)

    if not candidates:
        return root_dir

    candidates.sort()
    latest_name = candidates[-1]
    return os.path.join(root_dir, latest_name)


# ==============================================================
# 📸 Isaac Sim 카메라 수신용 클래스 (front + top)
# ==============================================================
class ImageRecorder:
    def __init__(
        self,
        front_topic="/front_camera/rgb",
        top_topic="/top_camera/rgb",
        node_name="ur10e_image_recorder",
    ):
        self._lock = threading.Lock()
        self._front_image = None
        self._top_image = None
        self._stop_evt = threading.Event()
        self.bridge = CvBridge()

        # ROS2 init (이미 되어 있으면 예외 무시)
        try:
            rclpy.init(args=None)
        except RuntimeError as e:
            if "must only be called once" not in str(e):
                raise

        self.node = Node(node_name)

        # 두 카메라 구독
        self.node.create_subscription(Image, front_topic, self._front_cb, 10)
        self.node.create_subscription(Image, top_topic, self._top_cb, 10)

        # executor + spin thread
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        print("[INFO] ImageRecorder initialized:")
        print(f"  - Front camera: {front_topic}")
        print(f"  - Top camera:   {top_topic}")

    def _spin(self):
        while not self._stop_evt.is_set():
            self._exec.spin_once(timeout_sec=0.05)

    def _normalize_img(self, img: np.ndarray) -> np.ndarray:
        """이미지를 (H, W, 3), uint8 로 정규화"""
        if img.dtype != np.uint8:
            img = cv2.convertScaleAbs(img)

        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif img.shape[2] == 1:
            img = np.repeat(img, 3, axis=2)
        elif img.shape[2] > 3:
            img = img[:, :, :3]
        return img

    def _front_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = self._normalize_img(img)
            with self._lock:
                self._front_image = img.copy()
        except Exception as e:
            self.node.get_logger().error(f"Front camera error: {e}")

    def _top_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = self._normalize_img(img)
            with self._lock:
                self._top_image = img.copy()
        except Exception as e:
            self.node.get_logger().error(f"Top camera error: {e}")

    def get_images(self):
        with self._lock:
            front = self._front_image.copy() if self._front_image is not None else None
            top = self._top_image.copy() if self._top_image is not None else None
        return {"front": front, "top": top}

    def wait_for_images(self, timeout: float = 5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            imgs = self.get_images()
            if imgs["front"] is not None and imgs["top"] is not None:
                return imgs
            time.sleep(0.1)
        raise TimeoutError("Timeout waiting for both camera images")

    def shutdown(self):
        try:
            self._stop_evt.set()
            if self._spin_thread.is_alive():
                self._spin_thread.join(timeout=2.0)
            self._exec.shutdown()
            self.node.destroy_node()
        except Exception as e:
            print(f"ImageRecorder shutdown error: {e}")


# ==============================================================
# 🤖 ACT Policy Inference (Closed-loop + Temporal Aggregation)
#    + Demo joints vs Pred joints CSV 로깅
# ==============================================================
class ActPolicyInfer:
    def __init__(self):
        # ROS2 init (이미 되어 있으면 예외 무시)
        try:
            rclpy.init(args=None)
        except RuntimeError as e:
            if "must only be called once" not in str(e):
                raise

        self.node = Node("act_policy_infer")
        self.curr_joint = np.zeros(6, dtype=np.float32)

        # publisher / subscriber
        self.joint_pub = self.node.create_publisher(
            JointState, "/isaac_joint_commands", 10
        )
        self.node.create_subscription(
            JointState, "/isaac_joint_states", self._joint_cb, 10
        )

        # 카메라 수신 (front + top)
        self.img_recorder = ImageRecorder(
            front_topic="/front_camera/rgb",
            top_topic="/top_camera/rgb",
            node_name="ur10e_image_recorder",
        )

        # ======================
        # episode_len (TASK_CONFIGS) 기본값
        # ======================
        task_cfg = TASK_CONFIGS.get("ur10e_swing", {})
        self.episode_len = int(task_cfg.get("episode_len", 600))

        # ======================
        # 모델 초기화 (가장 최신 ckpt 폴더 자동 선택)
        # ======================
        latest_ckpt_dir = find_latest_ckpt_dir(CKPT_ROOT)
        ckpt_path = os.path.join(latest_ckpt_dir, "policy_best.ckpt")
        self.ckpt_dir = latest_ckpt_dir

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.node.get_logger().info(f"[INFO] Loading checkpoint from {ckpt_path}")

        import argparse

        # training command:
        # python3 imitate_episodes.py ... --policy_class ACT --kl_weight 10
        #   --chunk_size 100 --hidden_dim 512 --batch_size 8 ...
        args = argparse.Namespace(
            lr=1e-5,
            lr_backbone=1e-5,
            batch_size=1,
            weight_decay=1e-4,
            epochs=1,
            lr_drop=200,
            clip_max_norm=0.1,
            hidden_dim=512,
            dim_feedforward=3200,
            num_queries=100,  # == chunk_size
            backbone="resnet18",
            position_embedding="sine",
            dilation=False,
            masks=False,
            enc_layers=6,
            dec_layers=6,
            nheads=8,
            dropout=0.1,
            pre_norm=False,
            eval=True,
            camera_names=["cam_front", "cam_head"],
            kl_weight=10,
        )

        self.policy = build_act_model(args)
        num_params = sum(p.numel() for p in self.policy.parameters() if p.requires_grad)
        self.node.get_logger().info(
            f"[INFO] DETR-VAE parameters: {num_params/1e6:.2f}M"
        )

        ckpt = torch.load(ckpt_path, map_location=device)
        state_dict = ckpt["state_dict"] if "state_dict" in ckpt else ckpt
        missing, unexpected = self.policy.load_state_dict(state_dict, strict=False)
        self.node.get_logger().info(
            f"✅ Loaded weights (missing={len(missing)}, unexpected={len(unexpected)})"
        )

        self.policy.to(device).eval()
        self.node.get_logger().info("✅ ACT model ready for inference")

        # temporal aggregation 설정 (normalized space에서 동작)
        self.chunk_size = args.num_queries
        self.max_hist = 5         # 각 time-offset 별로 최근 몇 개까지만 사용
        self.alpha = 1.0          # 최근 예측에 더 큰 가중치 (지수 감쇠 계수)
        self.pred_queue = [list() for _ in range(self.chunk_size)]

        # ======================
        # dataset_stats.pkl 로부터 action mean/std 로드 (denorm 용)
        # ======================
        self.action_mean = None
        self.action_std = None
        stats_path = os.path.join(latest_ckpt_dir, "dataset_stats.pkl")
        if os.path.exists(stats_path):
            try:
                with open(stats_path, "rb") as f:
                    stats = pickle.load(f)
                if "action_mean" in stats and "action_std" in stats:
                    self.action_mean = np.array(stats["action_mean"], dtype=np.float32)
                    self.action_std = np.array(stats["action_std"], dtype=np.float32)
                    self.node.get_logger().info(
                        "[INFO] Loaded action_mean/std from dataset_stats.pkl "
                        f"(mean={self.action_mean}, std={self.action_std})"
                    )
                else:
                    self.node.get_logger().warn(
                        "[WARN] dataset_stats.pkl 에 'action_mean', 'action_std' 키가 없습니다. "
                        "denormalization 은 비활성화됩니다."
                    )
            except Exception as e:
                self.node.get_logger().warn(
                    f"[WARN] Failed to load dataset_stats.pkl: {e}"
                )
        else:
            self.node.get_logger().warn(
                f"[WARN] dataset_stats.pkl not found at {stats_path}. "
                "denormalization 은 비활성화됩니다."
            )

        # ======================
        # Demo episode joints 로드 (episode_DEMO_EP_IDX.hdf5)
        # ======================
        self.demo_joints = None
        self.demo_len = None
        self._load_demo_episode(episode_idx=DEMO_EP_IDX)

        # demo 길이에 맞춰 episode_len 조정
        if self.demo_len is not None:
            old_len = self.episode_len
            self.episode_len = min(self.episode_len, self.demo_len)
            self.node.get_logger().info(
                f"[INFO] episode_len adjusted {old_len} -> {self.episode_len} "
                f"based on demo_len={self.demo_len}"
            )

        # ======================
        # Inference & 비교 로그 파일 준비
        # ======================
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime("%m%d_%H%M%S")
        self.log_path = os.path.join(LOG_DIR, f"act_infer_{ts}.csv")
        self.log_file = open(self.log_path, "w", buffering=1)

        # CSV 헤더: step, ros_time, demo_j0..5, pred_j0..5
        header = "step,ros_time,d0,d1,d2,d3,d4,d5,p0,p1,p2,p3,p4,p5\n"
        self.log_file.write(header)
        self.node.get_logger().info(f"[INFO] Inference log -> {self.log_path}")

    # ----------------------------------------------------------
    # ROS 콜백
    # ----------------------------------------------------------
    def _joint_cb(self, msg):
        if len(msg.position) >= 6:
            self.curr_joint = np.array(msg.position[:6], dtype=np.float32)

    # ----------------------------------------------------------
    # HDF5 안에서 (T, 6) 모양의 dataset 자동 탐색
    # ----------------------------------------------------------
    def _find_episode_dataset(self, f: h5py.File):
        candidates = []

        def visitor(name, obj):
            if isinstance(obj, h5py.Dataset):
                if obj.ndim == 2 and obj.shape[1] == 6:
                    candidates.append(name)

        f.visititems(visitor)

        if not candidates:
            return None, None

        # 이름에 따라 약간의 우선순위
        def score(name: str):
            s = 0
            lname = name.lower()
            if "action" in lname:
                s -= 3
            if "joint" in lname:
                s -= 2
            if "qpos" in lname:
                s -= 1
            return s

        candidates.sort(key=score)
        ds_name = candidates[0]
        data = f[ds_name][()]  # (T,6)
        return ds_name, data

    # ----------------------------------------------------------
    # demo episode 로드
    # ----------------------------------------------------------
    def _load_demo_episode(self, episode_idx: int):
        ep_path = os.path.join(DATASET_EP_DIR, f"episode_{episode_idx}.hdf5")
        if not os.path.exists(ep_path):
            self.node.get_logger().warn(
                f"[WARN] Demo episode file not found: {ep_path}. "
                "d0~d5는 NaN으로 채워집니다."
            )
            return

        try:
            with h5py.File(ep_path, "r") as f:
                ds_name, data = self._find_episode_dataset(f)
            if ds_name is None or data is None:
                self.node.get_logger().warn(
                    f"[WARN] {ep_path} 안에서 (T,6) dataset 을 찾지 못했습니다. "
                    "d0~d5는 NaN으로 채워집니다."
                )
                return

            self.demo_joints = np.asarray(data, dtype=np.float32)
            self.demo_len = self.demo_joints.shape[0]
            self.node.get_logger().info(
                f"[INFO] Loaded demo joints from {ep_path}, "
                f"dataset='{ds_name}', shape={self.demo_joints.shape}"
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"[WARN] Failed to load demo episode from {ep_path}: {e}"
            )

    # ----------------------------------------------------------
    # action denormalization helpers
    # ----------------------------------------------------------
    def _denorm_single(self, action_norm: np.ndarray) -> np.ndarray:
        if self.action_mean is None or self.action_std is None:
            return action_norm
        return action_norm * self.action_std + self.action_mean

    # ----------------------------------------------------------
    # temporal queue 업데이트 + aggregation
    # ----------------------------------------------------------
    def _shift_queue(self):
        # time step 진행에 따라 offset 0 -> 과거, 1 -> 0, ... 식으로 이동
        self.pred_queue = self.pred_queue[1:] + [list()]

    def _update_queue_with_chunk(self, chunk_norm: np.ndarray):
        """
        chunk_norm: (H,6) normalized actions (policy output)
        pred_queue[k] 에 offset k에 해당하는 예측을 push
        """
        H = min(self.chunk_size, chunk_norm.shape[0])
        for k in range(H):
            lst = self.pred_queue[k]
            lst.append(chunk_norm[k].copy())
            # 너무 오래된 건 버리기
            if len(lst) > self.max_hist:
                lst.pop(0)

    def _aggregate_current(self) -> np.ndarray:
        """
        pred_queue[0] 에 쌓여 있는 여러 예측을 지수 가중 평균으로 합침.
        결과는 normalized action (6,)
        """
        candidates = self.pred_queue[0]
        if not candidates:
            return None

        cand = np.stack(candidates, axis=0)  # (M,6)
        M = cand.shape[0]
        # 가장 최근 것이 가장 큰 weight
        idx = np.arange(M)[::-1]  # 0(가장 최근),1,... 로 쓰고 싶으면 반대로 해도 됨
        weights = np.exp(-self.alpha * idx)
        weights = weights / np.sum(weights)
        agg = (weights[:, None] * cand).sum(axis=0)
        return agg

    # ==========================================================
    # 메인 루프 (closed-loop + temporal agg, episode_len 동안 1회 실행)
    # ==========================================================
    def run(self):
        rate_hz = 20.0
        period = 1.0 / rate_hz

        try:
            # 최초 양 카메라 준비될 때까지 대기
            _ = self.img_recorder.wait_for_images(timeout=10.0)
            self.node.get_logger().info("Camera images ready!")

            step = 0
            last_time = self.node.get_clock().now()

            while rclpy.ok() and step < self.episode_len:
                imgs = self.img_recorder.get_images()
                front_np = imgs["front"]
                top_np = imgs["top"]

                if front_np is None or top_np is None:
                    time.sleep(0.01)
                    continue

                # --------- 3채널 uint8 맞추기 --------- #
                def norm_rgb(img):
                    if len(img.shape) == 2:
                        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    elif img.shape[2] == 1:
                        img = np.repeat(img, 3, axis=2)
                    elif img.shape[2] > 3:
                        img = img[:, :, :3]
                    if img.dtype != np.uint8:
                        img = cv2.convertScaleAbs(img)
                    return img

                front_np = norm_rgb(front_np)
                top_np = norm_rgb(top_np)

                # 해상도 안 맞으면 top을 front 사이즈로 리사이즈
                if front_np.shape[:2] != top_np.shape[:2]:
                    h, w = front_np.shape[:2]
                    top_np = cv2.resize(top_np, (w, h))

                # --------- (B, num_cams, 3, H, W) 텐서 --------- #
                front_t = (
                    torch.from_numpy(front_np).permute(2, 0, 1).float() / 255.0
                )
                top_t = (
                    torch.from_numpy(top_np).permute(2, 0, 1).float() / 255.0
                )

                cams = torch.stack([front_t, top_t], dim=0)  # (2,3,H,W)
                imgs_tensor = cams.unsqueeze(0).to(self.device)  # (1,2,3,H,W)
                qpos_tensor = torch.tensor(
                    self.curr_joint, dtype=torch.float32
                ).unsqueeze(0).to(self.device)

                # --------- time step 진행에 따라 queue shift --------- #
                if step > 0:
                    self._shift_queue()

                # --------- policy 한 번 호출 (closed-loop) --------- #
                with torch.no_grad():
                    out = self.policy(qpos_tensor, imgs_tensor)
                    if isinstance(out, tuple):
                        action_tensor = out[0]
                    else:
                        action_tensor = out

                    action_np_norm = action_tensor.detach().cpu().numpy()
                    action_np_norm = action_np_norm.reshape(-1, 6)  # (H,6)

                if step == 0:
                    self.node.get_logger().info(
                        f"[INFO] First policy output shape: {action_np_norm.shape}"
                    )

                # --------- temporal queue 업데이트 --------- #
                self._update_queue_with_chunk(action_np_norm)

                # --------- 현재 step 에 대한 temporal agg --------- #
                action_norm = self._aggregate_current()
                if action_norm is None:
                    # fallback: chunk 첫 번째 값 사용
                    action_norm = action_np_norm[0]

                # denorm → 실제 joint
                action = self._denorm_single(action_norm)  # (6,)

                # --------- JointState publish --------- #
                msg = JointState()
                msg.name = [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ]
                msg.position = action.tolist()
                self.joint_pub.publish(msg)

                self.node.get_logger().info(
                    f"[{step:03d}] Published(temporal_agg): {np.round(action, 3)}"
                )

                # --------- CSV 로깅 (demo vs pred) --------- #
                ros_time_sec = self.node.get_clock().now().nanoseconds / 1e9
                if self.demo_joints is not None and step < self.demo_len:
                    demo_vals = self.demo_joints[step]
                else:
                    demo_vals = np.full(6, np.nan, dtype=np.float32)

                line = "{:d},{:.6f},".format(step, ros_time_sec)
                line += ",".join(f"{v:.6f}" for v in demo_vals) + ","
                line += ",".join(f"{v:.6f}" for v in action) + "\n"
                self.log_file.write(line)

                step += 1

                # --------- 20 Hz 맞추기 --------- #
                now = self.node.get_clock().now()
                elapsed = (now - last_time).nanoseconds / 1e9
                if elapsed < period:
                    time.sleep(period - elapsed)
                last_time = now

            self.node.get_logger().info(
                f"[DONE] Published {step} steps with temporal agg. Shutting down episode."
            )

        except KeyboardInterrupt:
            self.node.get_logger().warn("Shutting down ACT Policy Infer...")
        finally:
            try:
                self.log_file.close()
            except Exception:
                pass

            self.img_recorder.shutdown()
            self.node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass


# ==============================================================
# Main
# ==============================================================
def main(args=None):
    infer = ActPolicyInfer()
    infer.run()


if __name__ == "__main__":
    main()
