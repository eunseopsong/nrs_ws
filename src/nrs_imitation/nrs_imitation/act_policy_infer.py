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
# 📸 Isaac Sim 카메라 수신용 클래스
# ==============================================================
class ImageRecorder:
    def __init__(self, front_topic="/front_camera/rgb", node_name="ur10e_image_recorder"):
        self._lock = threading.Lock()
        self._front_image = None
        self._stop_evt = threading.Event()
        self.bridge = CvBridge()

        try:
            rclpy.init(args=None)
        except RuntimeError as e:
            if "must only be called once" not in str(e):
                raise

        self.node = Node(node_name)
        self.node.create_subscription(Image, front_topic, self._front_cb, 10)

        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        print(f"[INFO] ImageRecorder initialized: {front_topic}")

    def _spin(self):
        while not self._stop_evt.is_set():
            self._exec.spin_once(timeout_sec=0.05)

    def _front_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if img.dtype != np.uint8:
                img = cv2.convertScaleAbs(img)
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif img.shape[2] == 1:
                img = np.repeat(img, 3, axis=2)
            elif img.shape[2] > 3:
                img = img[:, :, :3]
            with self._lock:
                self._front_image = img.copy()
        except Exception as e:
            self.node.get_logger().error(f"Front camera error: {e}")

    def get_front(self):
        with self._lock:
            return self._front_image.copy() if self._front_image is not None else None

    def wait_for_image(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            img = self.get_front()
            if img is not None:
                return img
            time.sleep(0.1)
        raise TimeoutError("Timeout waiting for camera image")

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
# 🤖 ACT Policy Inference 클래스
# ==============================================================
class ActPolicyInfer:
    def __init__(self):
        try:
            rclpy.init(args=None)
        except RuntimeError as e:
            if "must only be called once" not in str(e):
                raise

        self.node = Node("act_policy_infer")
        self.curr_joint = np.zeros(6)
        self.joint_pub = self.node.create_publisher(JointState, "/isaac_joint_commands", 10)
        self.node.create_subscription(JointState, "/isaac_joint_states", self._joint_cb, 10)

        self.img_recorder = ImageRecorder()

        ckpt_dir = "/home/eunseop/nrs_lab2/checkpoints/ur10e_swing"
        ckpt_path = os.path.join(ckpt_dir, "policy_best.ckpt")
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
            camera_names=["cam_front"],
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
        rate = self.node.create_rate(20)
        try:
            front_np = self.img_recorder.wait_for_image(timeout=10.0)
            self.node.get_logger().info("Camera image ready!")

            while rclpy.ok():
                front_np = self.img_recorder.get_front()
                if front_np is None:
                    continue

                if len(front_np.shape) == 2:
                    front_np = cv2.cvtColor(front_np, cv2.COLOR_GRAY2BGR)
                elif front_np.shape[2] == 1:
                    front_np = np.repeat(front_np, 3, axis=2)
                elif front_np.shape[2] > 3:
                    front_np = front_np[:, :, :3]

                if front_np.dtype != np.uint8:
                    front_np = cv2.convertScaleAbs(front_np)

                print("[DEBUG] front shape:", front_np.shape, "dtype:", front_np.dtype)
                front = torch.from_numpy(front_np).permute(2, 0, 1).unsqueeze(0).float() / 255.0
                print("[DEBUG] front tensor shape:", front.shape)

                imgs_tensor = front.to(self.device)
                qpos_tensor = torch.tensor(self.curr_joint, dtype=torch.float32).unsqueeze(0).to(self.device)

                with torch.no_grad():
                    action_tensor = self.policy(qpos_tensor, imgs_tensor)

                    # ✅ tuple 반환 시 첫 번째 값만 사용
                    if isinstance(action_tensor, tuple):
                        action_tensor = action_tensor[0]

                    action = action_tensor.cpu().numpy().flatten()

                msg = JointState()
                msg.name = [
                    "shoulder_pan_joint", "shoulder_lift_joint",
                    "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                ]
                msg.position = action.tolist()
                self.joint_pub.publish(msg)
                self.node.get_logger().info(f"Published inferred joint command: {np.round(action, 3)}")

                rate.sleep()

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
