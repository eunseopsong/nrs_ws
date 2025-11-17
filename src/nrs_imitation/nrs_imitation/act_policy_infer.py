#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading
from datetime import datetime

import numpy as np
import torch
import cv2
import h5py
from cv_bridge import CvBridge

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
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, JointState


# ==============================================================
# 유틸: 최신 체크포인트 폴더 찾기 (MMDD_HHMM 형식)
# ==============================================================
def find_latest_ckpt_dir(root_dir: str) -> str:
    """
    root_dir 아래의 하위 디렉토리 중 이름이 'MMDD_HHMM' 형식인 것들 중
    가장 최신(사전순으로 가장 뒤)을 반환. 없으면 root_dir 자체 반환.
    """
    if not os.path.isdir(root_dir):
        return root_dir

    candidates = []
    for name in os.listdir(root_dir):
        full = os.path.join(root_dir, name)
        if not os.path.isdir(full):
            continue
        # 4자리_4자리 형식만 후보
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
# 🤖 ACT Policy Inference + Demo 비교
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
        # 모델 초기화 (가장 최신 ckpt 폴더 자동 선택)
        # ======================
        ckpt_root = "/home/eunseop/nrs_lab2/checkpoints/ur10e_swing"
        latest_ckpt_dir = find_latest_ckpt_dir(ckpt_root)
        ckpt_path = os.path.join(latest_ckpt_dir, "policy_best.ckpt")
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
            camera_names=["cam_front", "cam_head"],
            kl_weight=10
        )

        self.policy = build_act_model(args)
        num_params = sum(p.numel() for p in self.policy.parameters() if p.requires_grad)
        self.node.get_logger().info(f"[INFO] DETR-VAE parameters: {num_params/1e6:.2f}M")

        ckpt = torch.load(ckpt_path, map_location=device)
        state_dict = ckpt["state_dict"] if "state_dict" in ckpt else ckpt
        missing, unexpected = self.policy.load_state_dict(state_dict, strict=False)
        self.node.get_logger().info(f"✅ Loaded weights (missing={len(missing)}, unexpected={len(unexpected)})")

        self.policy.to(device).eval()
        self.node.get_logger().info("✅ ACT model ready for inference")

        # ======================
        # 데모 episode_10 joints 불러오기 (rad)
        # ======================
        demo_ep_path = "/home/eunseop/nrs_lab2/datasets/ACT/1114_1643/episodes/episode_10.hdf5"
        self.demo_joints = None
        if os.path.exists(demo_ep_path):
            try:
                with h5py.File(demo_ep_path, "r") as f:
                    # /action: (T, 6), /observations/qpos 도 동일
                    self.demo_joints = f["/action"][()]
                self.node.get_logger().info(
                    f"[INFO] Loaded demo joints from {demo_ep_path}, shape = {self.demo_joints.shape}"
                )
            except Exception as e:
                self.node.get_logger().warn(f"[WARN] Failed to load demo joints: {e}")
        else:
            self.node.get_logger().warn(f"[WARN] Demo episode file not found: {demo_ep_path}")

        # ======================
        # Inference & 비교 로그 파일 준비
        # ======================
        log_dir = "/home/eunseop/nrs_lab2/analysis_logs"
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"act_infer_{ts}.csv")
        self.log_file = open(self.log_path, "w", buffering=1)

        # CSV 헤더: step, ros_time, demo_j0..5, pred_j0..5
        header = "step,ros_time,d0,d1,d2,d3,d4,d5,p0,p1,p2,p3,p4,p5\n"
        self.log_file.write(header)
        self.node.get_logger().info(f"[INFO] Inference log -> {self.log_path}")

    def _joint_cb(self, msg):
        if len(msg.position) >= 6:
            self.curr_joint = np.array(msg.position[:6])

    # ==========================================================
    # 메인 루프
    # ==========================================================
    def run(self):
        rate_hz = 20.0
        period = 1.0 / rate_hz
        try:
            # 최초 양 카메라 준비될 때까지 대기
            _ = self.img_recorder.wait_for_images(timeout=10.0)
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
                front_t = torch.from_numpy(front_np).permute(2, 0, 1).float() / 255.0
                top_t   = torch.from_numpy(top_np).permute(2, 0, 1).float() / 255.0

                cams = torch.stack([front_t, top_t], dim=0)       # (2,3,H,W)
                imgs_tensor = cams.unsqueeze(0).to(self.device)   # (1,2,3,H,W)
                qpos_tensor = torch.tensor(
                    self.curr_joint, dtype=torch.float32
                ).unsqueeze(0).to(self.device)

                # --------- 한 번만 inference 해서 시퀀스 저장 --------- #
                if action_seq is None:
                    with torch.no_grad():
                        out = self.policy(qpos_tensor, imgs_tensor)
                        # ACT 구현에 따라 tuple or tensor
                        if isinstance(out, tuple):
                            action_tensor = out[0]
                        else:
                            action_tensor = out
                        # (N*6,) 또는 (N,6) 형태라고 가정
                        action_np = action_tensor.cpu().numpy()
                        if action_np.ndim == 1:
                            action_seq = action_np.reshape(-1, 6)  # (N,6)
                        else:
                            # (B, T, 6) 또는 (T, 6) 등 -> 전부 (N,6)으로 flatten
                            action_seq = action_np.reshape(-1, 6)

                    total_steps = action_seq.shape[0]
                    self.node.get_logger().info(
                        f"Total predicted steps: {total_steps} (action_seq shape = {action_seq.shape})"
                    )
                    if total_steps >= 3:
                        self.node.get_logger().info(
                            f"[DEBUG] First 3 predicted steps (rad):\n{np.round(action_seq[:3], 3)}"
                        )

                    # --------- 데모와의 차이 한 번 계산해서 로그 --------- #
                    if self.demo_joints is not None:
                        T = min(self.demo_joints.shape[0], total_steps)
                        demo_cut = self.demo_joints[:T]
                        pred_cut = action_seq[:T]
                        mae = np.mean(np.abs(pred_cut - demo_cut), axis=0)
                        self.node.get_logger().info(
                            f"[COMPARE] Using first {T} steps, MAE per joint (rad): {mae}"
                        )

                # --------- 6개씩 잘라서 20Hz로 publish --------- #
                if action_seq is not None:
                    if step >= action_seq.shape[0]:
                        # 다 썼으면 다시 처음부터
                        step = 0
                        continue

                    action = action_seq[step]  # shape (6,)
                    msg = JointState()
                    msg.name = [
                        "shoulder_pan_joint", "shoulder_lift_joint",
                        "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                    ]
                    msg.position = action.tolist()
                    self.joint_pub.publish(msg)
                    self.node.get_logger().info(f"[{step:03d}] Published: {np.round(action, 3)}")

                    # ----- CSV 로깅: demo vs pred ----- #
                    ros_time_sec = self.node.get_clock().now().nanoseconds / 1e9
                    if self.demo_joints is not None and step < self.demo_joints.shape[0]:
                        demo_vals = self.demo_joints[step]
                    else:
                        demo_vals = np.full(6, np.nan)

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

        except KeyboardInterrupt:
            self.node.get_logger().warn("Shutting down ACT Policy Infer...")
        finally:
            # 로그 파일 닫기
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
