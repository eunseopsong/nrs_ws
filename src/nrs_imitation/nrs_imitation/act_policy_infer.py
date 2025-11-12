#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import torch
import numpy as np
import argparse
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState

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

from utils import set_seed
from act.detr.models.detr_vae import build as build_act_model

# ======================
# ROS2 관련 임포트
# ======================
import rclpy
from rclpy.node import Node


class ActPolicyInfer(Node):
    def __init__(self):
        super().__init__("act_policy_infer")

        # ======================
        # ROS2 설정
        # ======================
        self.bridge = CvBridge()
        self.front_image = None
        self.top_image = None
        self.curr_joint_state = np.zeros(6)

        # 카메라 구독
        self.create_subscription(Image, "/front_camera/rgb", self.front_callback, 10)
        self.create_subscription(Image, "/top_camera/rgb", self.top_callback, 10)

        # 조인트 구독 및 퍼블리셔
        self.create_subscription(JointState, "/isaac_joint_states", self.joint_callback, 10)
        self.joint_pub = self.create_publisher(JointState, "/isaac_joint_command", 10)

        # 타이머 콜백 (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # ======================
        # 모델 로드
        # ======================
        ckpt_dir = "/home/eunseop/nrs_lab2/checkpoints/ur10e_swing"
        ckpt_path = os.path.join(ckpt_dir, "policy_best.ckpt")
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device

        self.get_logger().info(f"[INFO] Loading checkpoint from {ckpt_path}")

        # 모델 인자 설정
        model_args = argparse.Namespace(
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
            kl_weight=10,
        )

        # ACT 모델 생성 및 weight 로드
        self.policy = build_act_model(model_args)
        ckpt = torch.load(ckpt_path, map_location=device)
        state_dict = ckpt["state_dict"] if "state_dict" in ckpt else ckpt
        missing, unexpected = self.policy.load_state_dict(state_dict, strict=False)
        self.get_logger().info(f"✅ Loaded weights (missing={len(missing)}, unexpected={len(unexpected)})")

        self.policy.to(device)
        self.policy.eval()
        self.get_logger().info("✅ ACT model ready for inference")

    # ======================
    # 콜백 함수들
    # ======================
    def front_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # grayscale일 경우 (H, W) → (H, W, 3)
            if len(cv_image.shape) == 2:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            self.front_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Front cam error: {e}")


    def top_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # grayscale일 경우 (H, W) → (H, W, 3)
            if len(cv_image.shape) == 2:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            self.top_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Top cam error: {e}")

    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            self.curr_joint_state = np.array(msg.position[:6])

    def timer_callback(self):
        if self.front_image is None or self.top_image is None:
            self.get_logger().warn("Waiting for camera images...")
            return

        qpos_tensor = torch.tensor(self.curr_joint_state, dtype=torch.float32).unsqueeze(0).to(self.device)

        # --- 수정된 부분 ---
        front = torch.from_numpy(self.front_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        top   = torch.from_numpy(self.top_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        imgs = torch.cat([front, top], dim=1).to(self.device)  # (1, 6, H, W)

        with torch.no_grad():
            action_tensor = self.policy(qpos_tensor, imgs)
            action = action_tensor.cpu().numpy().flatten()

        joint_msg = JointState()
        joint_msg.name = [
            "shoulder_pan_joint", "shoulder_lift_joint",
            "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        joint_msg.position = action.tolist()
        self.joint_pub.publish(joint_msg)

        self.get_logger().info(f"Published inferred joint command: {np.round(action, 3)}")



def main(args=None):
    rclpy.init(args=args)
    node = ActPolicyInfer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
