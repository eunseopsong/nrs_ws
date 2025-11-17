#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading
import torch
import numpy as np
import cv2
from cv_bridge import CvBridge

from pathlib import Path
from datetime import datetime

# ======================
# 경로 설정
# ======================
ROOT_DIR = "/home/eunseop/nrs_lab2/nrs_act"
sys.path.extend([
    ROOT_DIR,
    os.path.join(ROOT_DIR, "act"),
    os.path.join(ROOT_DIR, "act", "model"),
    os.path.join(ROOT_DIR, "act", "detr"),
    os.path.join(ROOT_DIR, "act", "detr", "util"),
])

from act.detr.models.detr_vae import build as build_act_model

# ======================
# ROS2 관련 임포트
# ======================
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from rclpy.executors import SingleThreadedExecutor


# ==============================================================
# 🔎 가장 최신 ckpt 폴더 찾기
# ==============================================================
def find_latest_ckpt(base_dir: str) -> str:
    """
    base_dir 아래에 MMDD_HHMM 형식 서브폴더들 중
    가장 최신 폴더를 찾아 그 안의 policy_best.ckpt 경로를 리턴.
    서브폴더가 하나도 없으면 base_dir/policy_best.ckpt 로 fallback.
    """
    root = Path(base_dir)
    if not root.exists():
        raise FileNotFoundError(f"Checkpoint base dir not found: {base_dir}")

    candidates = []
    for child in root.iterdir():
        if child.is_dir():
            try:
                # 폴더 이름이 "MMDD_HHMM" 형식인지 검사
                datetime.strptime(child.name, "%m%d_%H%M")
                candidates.append(child)
            except ValueError:
                # 다른 이름 폴더는 무시
                pass

    if not candidates:
        # 예전 방식: 바로 밑에 policy_best.ckpt 가 있다고 가정
        ckpt_path = root / "policy_best.ckpt"
        return str(ckpt_path)

    # 이름 기준 내림차순 정렬 → 가장 최근(MMDD_HHMM)이 맨 앞
    candidates.sort(key=lambda p: p.name, reverse=True)
    latest_dir = candidates[0]
    ckpt_path = latest_dir / "policy_best.ckpt"
    return str(ckpt_path)


# ==============================================================
# 📸 Isaac Sim 카메라 수신용 클래스 (front + top)
# ==============================================================
class ImageRecorder:
    def __init__(self,
                 front_topic="/front_camera/rgb",
                 top_topic="/top_camera/rgb",
                 node_name="ur10e_image_recorder"):
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

        print(f"[INFO] ImageRecorder initialized:")
        print(f"  - Front camera: {front_topic}")
        print(f"  - Top camera:   {top_topic}")

    def _spin(self):
        while not self._stop_evt.is_set():
            self._exec.spin_once(timeout_sec=0.05)

    # --------- 콜백들: 항상 BGR 3채널 uint8로 맞춤 --------- #
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

    # --------- 이미지 가져오기 --------- #
    def get_images(self):
        with self._lock:
            front = self._front_image.copy() if self._front_image is not None else None
            top   = self._top_image.copy()   if self._top_image   is not None else None
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
# 🤖 ACT Policy Inference 클래스 (2 카메라 + 6개씩 20Hz publish)
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
        self.curr_joint = np.zeros(6)

        # publisher / subscriber
        self.joint_pub = self.node.create_publisher(JointState, "/isaac_joint_commands", 10)
        self.node.create_subscription(JointState, "/isaac_joint_states", self._joint_cb, 10)

        # 카메라 수신 (front + top)
        self.img_recorder = ImageRecorder(
            front_topic="/front_camera/rgb",
            top_topic="/top_camera/rgb",
            node_name="ur10e_image_recorder"
        )

        # ======================
        # 모델 초기화
        # ======================
        base_ckpt_dir = "/home/eunseop/nrs_lab2/checkpoints/ur10e_swing"
        ckpt_path = find_latest_ckpt(base_ckpt_dir)

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.node.get_logger().info(f"[INFO] Loading checkpoint from {ckpt_path}")

        import argparse
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
            num_queries=100,
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
            # ✅ 두 카메라 이름
            camera_names=["cam_front", "cam_head"],
            kl_weight=10
        )

        self.policy = build_act_model(args)
        ckpt = torch.load(ckpt_path, map_location=device)
        state_dict = ckpt["state_dict"] if "state_dict" in ckpt else ckpt
        missing, unexpected = self.policy.load_state_dict(state_dict, strict=False)
        self.node.get_logger().info(f"✅ Loaded weights (missing={len(missing)}, unexpected={len(unexpected)})")

        self.policy.to(device).eval()
        self.node.get_logger().info("✅ ACT model ready for inference")

    def _joint_cb(self, msg):
        if len(msg.position) >= 6:
            self.curr_joint = np.array(msg.position[:6])

    def run(self):
        rate_hz = 20.0
        period = 1.0 / rate_hz
        try:
            # 최초 양 카메라 준비될 때까지 대기
            imgs = self.img_recorder.wait_for_images(timeout=10.0)
            self.node.get_logger().info("Camera images ready!")

            step = 0
            action_seq = None
            last_time = self.node.get_clock().now()

            while rclpy.ok():
                imgs = self.img_recorder.get_images()
                front_np = imgs["front"]
                top_np   = imgs["top"]

                if front_np is None or top_np is None:
                    continue

                # --------- 안전하게 3채널 맞추기 --------- #
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
                top_np   = norm_rgb(top_np)

                # 해상도 안 맞으면 top을 front 사이즈로 리사이즈
                if front_np.shape[:2] != top_np.shape[:2]:
                    h, w = front_np.shape[:2]
                    top_np = cv2.resize(top_np, (w, h))

                # --------- (B, num_cams, 3, H, W) 텐서 만들기 --------- #
                front_t = torch.from_numpy(front_np).permute(2, 0, 1).float() / 255.0  # (3,H,W)
                top_t   = torch.from_numpy(top_np).permute(2, 0, 1).float() / 255.0    # (3,H,W)

                cams = torch.stack([front_t, top_t], dim=0)   # (2,3,H,W)
                imgs_tensor = cams.unsqueeze(0).to(self.device)  # (1,2,3,H,W)

                qpos_tensor = torch.tensor(self.curr_joint, dtype=torch.float32).unsqueeze(0).to(self.device)

                # --------- 한 번만 inference 해서 시퀀스 저장 --------- #
                if action_seq is None:
                    with torch.no_grad():
                        out = self.policy(qpos_tensor, imgs_tensor)
                        if isinstance(out, tuple):
                            action_tensor = out[0]
                        else:
                            action_tensor = out
                        action_seq = action_tensor.cpu().numpy().flatten()
                        total_steps = len(action_seq) // 6
                        self.node.get_logger().info(f"Total predicted steps: {total_steps}")

                # --------- 6개씩 잘라서 20Hz로 publish --------- #
                if action_seq is not None:
                    start = step * 6
                    end = start + 6
                    if end > len(action_seq):
                        # 다 썼으면 다시 처음부터
                        step = 0
                        continue

                    action = action_seq[start:end]
                    msg = JointState()
                    msg.name = [
                        "shoulder_pan_joint", "shoulder_lift_joint",
                        "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                    ]
                    msg.position = action.tolist()
                    self.joint_pub.publish(msg)
                    self.node.get_logger().info(f"[{step:03d}] Published: {np.round(action, 3)}")

                    step += 1

                # --------- 20 Hz 맞추기 --------- #
                now = self.node.get_clock().now()
                elapsed = (now - last_time).nanoseconds / 1e9
                if elapsed < period:
                    time.sleep(period - elapsed)
                last_time = now

        except KeyboardInterrupt:
            self.node.get_logger().warn("Shutting down ACT Policy Infer...")
        finally:
            self.img_recorder.shutdown()
            self.node.destroy_node()
            rclpy.shutdown()


# ==============================================================
# Main
# ==============================================================
def main(args=None):
    infer = ActPolicyInfer()
    infer.run()

if __name__ == "__main__":
    main()
