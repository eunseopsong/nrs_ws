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

# 🔹 ACT 데이터 루트 (최신 episodes_ft 자동 탐색용)
ACT_DATA_ROOT = "/home/eunseop/nrs_lab2/datasets/ACT"

# 비교할 데모 에피소드 인덱스 (episode_0.hdf5, episode_10.hdf5 등)
DEMO_EP_IDX = 0  # 필요하면 나중에 바꿔서 사용

# publish/데모 기준 action 차원: 6 joint + 3 force
ACTION_DIM = 9

# 로그 저장 디렉토리
LOG_DIR = "/home/eunseop/nrs_lab2/analysis_logs"

# sys.path 설정
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
from std_msgs.msg import Float64MultiArray   # 🔹 ACT → JointControl 전달용


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
# 유틸: 최신 episode 디렉토리 찾기 (ACT_DATA_ROOT 아래 MMDD_HHMM/episodes_ft)
# ==============================================================
def find_latest_episode_dir(root_dir: str) -> str:
    """
    root_dir (예: /home/.../datasets/ACT) 아래에서
    이름이 'MMDD_HHMM' 형식인 폴더 중 가장 최신을 고른 뒤,
    그 안의 'episodes_ft' 또는 'episodes' 디렉토리 경로를 반환.
    """
    if not os.path.isdir(root_dir):
        return root_dir

    candidates = []
    for name in os.listdir(root_dir):
        sub = os.path.join(root_dir, name)
        if not os.path.isdir(sub):
            continue
        # "MMDD_HHMM" 형식 필터링
        if len(name) == 9 and name[4] == "_":
            mmdd = name[:4]
            hhmm = name[5:]
            if mmdd.isdigit() and hhmm.isdigit():
                candidates.append(name)

    if not candidates:
        return root_dir

    candidates.sort()
    latest_name = candidates[-1]
    base_dir = os.path.join(root_dir, latest_name)

    ep_ft = os.path.join(base_dir, "episodes_ft")
    ep    = os.path.join(base_dir, "episodes")

    if os.path.isdir(ep_ft):
        return ep_ft
    if os.path.isdir(ep):
        return ep

    # 디렉토리가 아직 없어도 일단 episodes_ft 경로를 리턴
    return ep_ft


# 🔹 이제 DATASET_EP_DIR을 자동으로 결정
DATASET_EP_DIR = find_latest_episode_dir(ACT_DATA_ROOT)


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
# 🤖 ACT Policy Inference (Closed-loop, 1-step action + smoothing)
#    + Demo action(6 joint + 3 force) vs Pred action CSV 로깅
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

        self.node.get_logger().info(
            f"[INFO] Using DATASET_EP_DIR = {DATASET_EP_DIR}"
        )

        # ======================
        # publisher / subscriber
        # ======================

        # 🔹 JointControl의 control_mode == 7 이 subscribe 하는 토픽으로 publish
        #   - joints 6개  → /action_joints
        #   - force 3개   → /action_force
        self.act_joints_pub = self.node.create_publisher(
            Float64MultiArray, "/action_joints", 10
        )
        self.act_force_pub = self.node.create_publisher(
            Float64MultiArray, "/action_force", 10
        )

        # JointState 구독 (현재 관절 상태)
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
            num_queries=100,  # == chunk_size (H)
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

        # ======================
        # 1-step smoothing (temporal agg 대체)
        # ======================
        self.smooth_alpha = 0.4  # 0.0 ~ 1.0 (1.0이면 smoothing 없음)
        self.prev_action_norm = None  # 정규화 공간에서 smoothing

        # 모델이 실제로 출력하는 action 차원 (처음 forward할 때 결정)
        self.model_action_dim = None

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
                        f"(len={len(self.action_mean)})"
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
        # Demo episode actions 로드 (episode_DEMO_EP_IDX.hdf5)
        #   - shape: (T, 9) [joints(6) + force(3)] 가정
        # ======================
        self.demo_actions = None
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

        # CSV 헤더: step, ros_time,
        #   demo_j0..5, demo_fx, demo_fy, demo_fz,
        #   pred_j0..5, pred_fx, pred_fy, pred_fz
        header = (
            "step,ros_time,"
            "dj0,dj1,dj2,dj3,dj4,dj5,dfx,dfy,dfz,"
            "pj0,pj1,pj2,pj3,pj4,pj5,pfx,pfy,pfz\n"
        )
        self.log_file.write(header)
        self.node.get_logger().info(f"[INFO] Inference log -> {self.log_path}")

        # --------------------------------------------------
        # 🔹 Interpolation (upsampling) 설정
        #   - policy: 20 Hz
        #   - publish: 20 * interp_factor Hz
        # --------------------------------------------------
        self.interp_factor = 5          # 예: 5면 20Hz → 100Hz
        self.prev_joints_cmd = None    # 이전 step joint action (6,)
        self.prev_force_cmd = None     # 이전 step force action (3,)

    # ----------------------------------------------------------
    # ROS 콜백
    # ----------------------------------------------------------
    def _joint_cb(self, msg):
        if len(msg.position) >= 6:
            self.curr_joint = np.array(msg.position[:6], dtype=np.float32)

    # ----------------------------------------------------------
    # HDF5 안에서 (T, ACTION_DIM) 또는 (T,6) 모양의 dataset 자동 탐색
    # ----------------------------------------------------------
    def _find_episode_dataset(self, f: h5py.File):
        candidates = []

        def visitor(name, obj):
            if isinstance(obj, h5py.Dataset):
                if obj.ndim == 2 and obj.shape[1] in (6, ACTION_DIM):
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
        data = f[ds_name][()]  # (T,6) 또는 (T,9)
        return ds_name, data

    # ----------------------------------------------------------
    # demo episode 로드
    #   - demo_actions: (T, ACTION_DIM) 혹은 (T,6) → 내부에서 분리해서 사용
    # ----------------------------------------------------------
    def _load_demo_episode(self, episode_idx: int):
        ep_path = os.path.join(DATASET_EP_DIR, f"episode_{episode_idx}.hdf5")
        if not os.path.exists(ep_path):
            self.node.get_logger().warn(
                f"[WARN] Demo episode file not found: {ep_path}. "
                "demo 값은 NaN으로 채워집니다."
            )
            return

        try:
            with h5py.File(ep_path, "r") as f:
                ds_name, data = self._find_episode_dataset(f)
            if ds_name is None or data is None:
                self.node.get_logger().warn(
                    f"[WARN] {ep_path} 안에서 (T,6) 또는 (T,{ACTION_DIM}) dataset 을 찾지 못했습니다. "
                    "demo 값은 NaN으로 채워집니다."
                )
                return

            data = np.asarray(data, dtype=np.float32)
            if data.shape[1] == ACTION_DIM:
                # joints(6) + force(3)가 그대로 들어있는 경우
                self.demo_actions = data
            elif data.shape[1] == 6:
                # joints만 있는 경우 → force는 NaN으로 채움
                T = data.shape[0]
                demo = np.full((T, ACTION_DIM), np.nan, dtype=np.float32)
                demo[:, :6] = data
                self.demo_actions = demo
            else:
                # 이 경우는 위에서 shape 필터링했으므로 거의 안 옴
                self.demo_actions = None
                self.node.get_logger().warn(
                    f"[WARN] Unexpected demo data shape: {data.shape}"
                )
                return

            self.demo_len = self.demo_actions.shape[0]
            self.node.get_logger().info(
                f"[INFO] Loaded demo actions from {ep_path}, "
                f"dataset='{ds_name}', shape={self.demo_actions.shape}"
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

    # ==========================================================
    # 메인 루프 (closed-loop, 1-step action + smoothing + interpolation)
    #   - policy 호출: episode_len 번 (예: 400)
    #   - publish: episode_len * interp_factor 번 (예: 400 * 100 = 40000)
    #   - publish 주파수는 20 Hz 유지 → 전체 시간은 interp_factor 배로 늘어남
    # ==========================================================
    def run(self):
        policy_rate_hz = 20.0      # policy step 기준 속도 (여기서는 의미상만, 실제는 sub-step 기준)
        period = 1.0 / policy_rate_hz

        try:
            # 최초 양 카메라 준비될 때까지 대기
            _ = self.img_recorder.wait_for_images(timeout=10.0)
            self.node.get_logger().info("Camera images ready!")

            policy_step = 0          # ACT policy 호출 횟수 (0..episode_len-1)
            global_step = 0          # 실제 publish step (0..episode_len*interp_factor-1)

            while rclpy.ok() and policy_step < self.episode_len:
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

                # --------- policy 한 번 호출 --------- #
                with torch.no_grad():
                    out = self.policy(qpos_tensor, imgs_tensor)
                    if isinstance(out, tuple):
                        action_tensor = out[0]
                    else:
                        action_tensor = out

                    action_np_norm = action_tensor.detach().cpu().numpy()

                # 모델 action 차원 자동 결정 (첫 step에서)
                if self.model_action_dim is None:
                    self.model_action_dim = int(action_tensor.shape[-1])
                    self.node.get_logger().info(
                        f"[INFO] Detected model_action_dim = {self.model_action_dim}"
                    )

                    # dataset_stats와 차원 안 맞으면 denorm 비활성화
                    if (
                        self.action_mean is not None
                        and len(self.action_mean) != self.model_action_dim
                    ):
                        self.node.get_logger().warn(
                            "[WARN] model_action_dim과 dataset_stats action_dim 불일치. "
                            "denormalization 비활성화."
                        )
                        self.action_mean = None
                        self.action_std = None

                # (H, model_action_dim)로 reshape
                action_np_norm = action_np_norm.reshape(-1, self.model_action_dim)

                if policy_step == 0:
                    self.node.get_logger().info(
                        f"[INFO] First policy output shape (H, D): "
                        f"{action_np_norm.shape}"
                    )

                # --------- 이번 policy step에서 사용할 action (index 0) --------- #
                curr_action_norm = action_np_norm[0]  # (D,)

                # --------- smoothing (정규화 공간에서) --------- #
                if self.prev_action_norm is None:
                    smoothed_action_norm = curr_action_norm
                else:
                    a = self.smooth_alpha
                    smoothed_action_norm = (
                        a * curr_action_norm + (1.0 - a) * self.prev_action_norm
                    )
                self.prev_action_norm = smoothed_action_norm

                # denorm → 실제 joint+force
                action_full = self._denorm_single(smoothed_action_norm)  # (D,)
                action_full = np.asarray(action_full, dtype=np.float32)

                # 최소 6차원 확보
                if action_full.shape[0] < 6:
                    pad = np.zeros(6, dtype=np.float32)
                    pad[: action_full.shape[0]] = action_full
                    action_full = pad

                # 🔹 joints: 항상 앞 6개 사용
                joints_cmd = action_full[:6]

                # 🔹 force: model이 9차원 이상이면 6:9 사용, 아니면 0
                if action_full.shape[0] >= 9:
                    force_cmd = action_full[6:9]
                else:
                    force_cmd = np.zeros(3, dtype=np.float32)

                # 첫 policy step이면 prev를 현재와 동일하게 세팅
                if self.prev_joints_cmd is None:
                    self.prev_joints_cmd = joints_cmd.copy()
                    self.prev_force_cmd = force_cmd.copy()

                # --------------------------------------------------
                # 🔹 prev → current 를 interp_factor 개로 선형보간해서
                #    20 Hz 로 publish → 한 policy step당 interp_factor 개의 action
                # --------------------------------------------------
                for k in range(self.interp_factor):
                    alpha = float(k + 1) / float(self.interp_factor)  # 0~1

                    interp_joints = (
                        (1.0 - alpha) * self.prev_joints_cmd
                        + alpha * joints_cmd
                    )
                    interp_force = (
                        (1.0 - alpha) * self.prev_force_cmd
                        + alpha * force_cmd
                    )

                    # publish
                    j_msg = Float64MultiArray()
                    j_msg.data = interp_joints.tolist()
                    self.act_joints_pub.publish(j_msg)

                    f_msg = Float64MultiArray()
                    f_msg.data = interp_force.tolist()
                    self.act_force_pub.publish(f_msg)

                    # 로그 (global_step 기준)
                    self.node.get_logger().info(
                        f"[g{global_step:05d} | p{policy_step:03d}] "
                        f"joints={np.round(interp_joints, 3)}, "
                        f"force={np.round(interp_force, 3)}"
                    )

                    # CSV 로깅: demo 는 해당 policy_step 기준 값 반복
                    ros_time_sec = self.node.get_clock().now().nanoseconds / 1e9
                    if self.demo_actions is not None and policy_step < self.demo_len:
                        demo_vals = self.demo_actions[policy_step]
                    else:
                        demo_vals = np.full(ACTION_DIM, np.nan, dtype=np.float32)

                    demo_j = demo_vals[:6]
                    demo_f = demo_vals[6:9]

                    line = "{:d},{:.6f},".format(global_step, ros_time_sec)
                    line += ",".join(f"{v:.6f}" for v in demo_j) + ","
                    line += ",".join(f"{v:.6f}" for v in demo_f) + ","
                    line += ",".join(f"{v:.6f}" for v in interp_joints) + ","
                    line += ",".join(f"{v:.6f}" for v in interp_force) + "\n"
                    self.log_file.write(line)

                    global_step += 1
                    time.sleep(period)  # 항상 20 Hz 유지

                # 다음 구간 보간을 위해 prev <- current
                self.prev_joints_cmd = joints_cmd.copy()
                self.prev_force_cmd = force_cmd.copy()

                policy_step += 1

            self.node.get_logger().info(
                f"[DONE] policy_step={policy_step}, global_step={global_step} "
                f"(interp_factor={self.interp_factor})"
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
