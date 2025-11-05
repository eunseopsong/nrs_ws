#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import re
import json
import yaml
import openvr
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from vive_tracker_interfaces.srv import ViveCalibration

# 기존 utils
from vive_tracker_ros2.utils import (
    calculate_calibration_matrix,
    transform_HmdMatrix2npmatrix,
    matrix_to_pose,
    matrix_to_twist,
    pose_to_matrix,
)

import ament_index_python.packages

# json 저장 경로 (기존 코드 유지)
CONFIG_PATH = os.path.join(
    ament_index_python.packages.get_package_share_directory("vive_tracker_ros2"),
    "config",
)


class ViveTracker(Node):
    def __init__(self):
        super().__init__("vive_tracker")
        self.vr_system = None
        self.trackers = {}

        # ROS, VR 초기화
        self.initial_ros()
        self.initial_vr()

        # json 캘리브레이션 (서비스로 저장되는 것) – 그냥 읽어두기만
        if os.path.exists(os.path.join(CONFIG_PATH, "calibration_matrix.json")):
            with open(os.path.join(CONFIG_PATH, "calibration_matrix.json"), "r") as f:
                data = json.load(f)
                self.T_tool_opt = np.array(data["T_tool_opt"])
                self.T_trans_opt = np.array(data["T_trans_opt"])
        else:
            self.T_tool_opt = np.eye(4)
            self.T_trans_opt = np.eye(4)

        # 🔴 ROS1 스타일 yaml 로드 (네가 src/yaml 에 넣은 거)
        self.T_AD, self.T_BC, self.T_CE, self.R_Adj = self.load_ros1_style_calib()

        # 이전 포즈 저장
        self.raw_pose_matrix = np.eye(4)
        self.prev_raw_pose_matrix = np.eye(4)
        self.prev_calibrated_pose_matrix = np.eye(4)
        self.prev_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def load_ros1_style_calib(self):
        """
        1순위: ~/nrs_ws/src/vive_tracker_ros2/yaml/calibration_matrix.yaml
        2순위: install/share/vive_tracker_ros2/yaml/calibration_matrix.yaml
        없으면 전부 identity
        """
        src_path = os.path.expanduser(
            "~/nrs_ws/src/vive_tracker_ros2/yaml/calibration_matrix.yaml"
        )
        install_path = os.path.join(
            ament_index_python.packages.get_package_share_directory("vive_tracker_ros2"),
            "yaml",
            "calibration_matrix.yaml",
        )

        if os.path.exists(src_path):
            yaml_path = src_path
            self.get_logger().info(f"[vive_tracker_node] load yaml from src: {yaml_path}")
        elif os.path.exists(install_path):
            yaml_path = install_path
            self.get_logger().info(f"[vive_tracker_node] load yaml from install: {yaml_path}")
        else:
            self.get_logger().error(
                "[vive_tracker_node] no calibration_matrix.yaml found in src or install – use identity."
            )
            return np.eye(4), np.eye(4), np.eye(4), np.eye(3)

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        T_AD = np.array(data.get("T_AD", np.eye(4)))
        T_BC = np.array(data.get("T_BC", np.eye(4)))
        T_CE = np.array(data.get("T_CE", np.eye(4)))
        R_Adj_3 = np.array(data.get("R_Adj", np.eye(3)))

        # R_Adj 3x3 -> 4x4 로 올리기
        R_Adj = np.eye(4)
        R_Adj[:3, :3] = R_Adj_3

        return T_AD, T_BC, T_CE, R_Adj

    # ------------------------------------------------------------------
    def initial_ros(self):
        # 기본 퍼블리셔
        self.raw_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/raw_pose", 10
        )
        self.calibrated_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/calibrated_pose", 10
        )

        # 서비스 – 기존 기능 그대로
        self.calibrate_srv = self.create_service(
            ViveCalibration, "vive_tracker_ros/calibrate", self.cb_calibrate
        )

        # 파라미터
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("child_frame", "vive_tracker")

        self.publish_tf = self.get_parameter("publish_tf").value
        self.base_frame = self.get_parameter("base_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        # 기타
        self.logger = self.get_logger()
        self.clock = self.get_clock()
        self.timer = self.create_timer(0.01, self.cb_vive_timer)
        self.tf_broadcaster = TransformBroadcaster(self)

    # ------------------------------------------------------------------
    def initial_vr(self):
        try:
            self.logger.info("Initializing VR system... This may take a few moments.")
            self.vr_system = openvr.init(openvr.VRApplication_Other)
            self.logger.info("VR system initialized successfully!")

            # 디바이스 검사
            poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )
            device_found = False
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                cls = self.vr_system.getTrackedDeviceClass(i)
                if cls != openvr.TrackedDeviceClass_Invalid:
                    device_found = True
                    self.logger.info(f"[vive_tracker_node] Found VR device: (index {i})")

            if not device_found:
                self.logger.warn("No VR devices found. Check SteamVR / tracker connection.")

            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize VR system: {e}")
            return False

    # ------------------------------------------------------------------
    def cb_calibrate(self, request, response):
        # 원래 있던 서비스 – 놔둠
        self.logger.info("[Debug] ViveTracker calibration service called.")
        robot_mats = [pose_to_matrix(pose) for pose in request.robot_poses]
        tracker_mats = [pose_to_matrix(pose) for pose in request.tracker_poses]

        if len(robot_mats) != len(tracker_mats):
            self.logger.error("robot_poses and tracker_poses must have same length")
            response.success = False
            return response

        try:
            self.T_tool_opt, self.T_trans_opt = calculate_calibration_matrix(
                robot_mats, tracker_mats
            )
        except Exception as e:
            self.logger.error(f"calibration failed: {e}")
            response.success = False
            return response

        os.makedirs(CONFIG_PATH, exist_ok=True)
        with open(os.path.join(CONFIG_PATH, "calibration_matrix.json"), "w") as f:
            json.dump(
                {
                    "T_tool_opt": self.T_tool_opt.tolist(),
                    "T_trans_opt": self.T_trans_opt.tolist(),
                },
                f,
                indent=4,
            )

        response.success = True
        return response

    # ------------------------------------------------------------------
    def get_tracker_poses(self):
        """모든 tracker 장치의 raw pose 를 가져와 딕셔너리에 넣어두고 key 목록을 반환"""
        try:
            if self.vr_system is None:
                self.logger.error("VR system not initialized.")
                return {}

            poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )

            tracker_data = {}
            tracker_found = False

            for i in range(openvr.k_unMaxTrackedDeviceCount):
                try:
                    dev_class = self.vr_system.getTrackedDeviceClass(i)
                    if dev_class == openvr.TrackedDeviceClass_GenericTracker:
                        pose = poses[i]
                        if pose.bDeviceIsConnected and pose.bPoseIsValid:
                            tracker_found = True
                            try:
                                serial = self.vr_system.getStringTrackedDeviceProperty(
                                    i, openvr.Prop_SerialNumber_String
                                )
                            except Exception as e:
                                self.logger.warn(f"can't get serial for tracker {i}: {e}")
                                serial = f"tracker_{i}"

                            safe_serial = re.sub(r"[^a-zA-Z0-9_]", "_", serial)

                            # 새 트래커면 publisher 만들어줌
                            if serial not in self.trackers:
                                raw_topic = f"vive_tracker_ros/{safe_serial}/raw_pose"
                                cali_topic = f"vive_tracker_ros/{safe_serial}/calibrated_pose"
                                child_frame = f"{self.child_frame}_{safe_serial}"
                                self.trackers[serial] = {
                                    "device_index": i,
                                    "prev_raw_matrix": np.eye(4),
                                    "prev_calibrated_matrix": np.eye(4),
                                    "child_frame": child_frame,
                                    "publisher_raw": self.create_publisher(Odometry, raw_topic, 10),
                                    "publisher_calibrated": self.create_publisher(Odometry, cali_topic, 10),
                                }
                                self.logger.info(f"새 트래커 발견: {serial} (index {i})")
                                self.logger.info(f"토픽: {raw_topic}, {cali_topic}")

                            # utils 로 raw matrix 변환
                            raw_pose_m = transform_HmdMatrix2npmatrix(pose.mDeviceToAbsoluteTracking)
                            self.trackers[serial]["raw_pose_matrix"] = raw_pose_m

                            tracker_data[serial] = self.trackers[serial]
                except Exception as e:
                    self.logger.warn(f"error reading device {i}: {e}")

            if not tracker_found and not hasattr(self, "_tracker_warning_shown"):
                self.logger.warn("No valid tracker pose. Check connection.")
                self._tracker_warning_shown = True
            elif tracker_found and hasattr(self, "_tracker_warning_shown"):
                self.logger.info("Tracker connection restored.")
                delattr(self, "_tracker_warning_shown")

            return tracker_data.keys()

        except Exception as e:
            self.logger.error(f"get_tracker_poses error: {e}")
            return []

    # ------------------------------------------------------------------
    def create_vive_msg(self, pose: Pose, twist: Twist, frame_id="world"):
        msg = Odometry()
        msg.header.stamp = self.clock.now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose = pose
        msg.twist.twist = twist
        return msg

    # ------------------------------------------------------------------
    def cb_vive_timer(self):
        current_time = self.clock.now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        current_tracker_ids = self.get_tracker_poses()
        if not current_tracker_ids:
            return

        tracker_list = list(current_tracker_ids)

        for tracker_id in tracker_list:
            tracker_data = self.trackers[tracker_id]

            # 0) utils 로부터 온 raw (SteamVR 기준)
            raw_M = tracker_data["raw_pose_matrix"]

            # 1) utils 가 y,z를 한 번 바꿔놨다고 가정하고 ROS1이 쓰던 형태로 되돌리기
            #    (필요 없으면 이 4줄만 주석처리해서 테스트하면 돼)
            raw_M_ros1 = raw_M.copy()
            # 행 스왑 (y,z)
            raw_M_ros1[[1, 2], :] = raw_M_ros1[[2, 1], :]
            # 열 스왑 (y,z)
            raw_M_ros1[:, [1, 2]] = raw_M_ros1[:, [2, 1]]

            # 2) ROS1 처럼 먼저 R_Adj^T 곱해서 자세 보정
            adj_M = np.eye(4)
            adj_M[:3, :3] = self.R_Adj[:3, :3].T
            vr_pose_adj = adj_M @ raw_M_ros1

            # 3) 그리고 나서 T_AD * (...) * T_CE
            calibrated_M = self.T_AD @ vr_pose_adj @ self.T_CE

            tracker_data["calibrated_pose_matrix"] = calibrated_M

            # ---- 퍼블리시 파트는 원래 코드 그대로 ----
            raw_pose = matrix_to_pose(raw_M)              # raw 그대로 보여주고
            calibrated_pose = matrix_to_pose(calibrated_M)  # 변환 결과

            raw_twist = matrix_to_twist(
                raw_M,
                tracker_data["prev_raw_matrix"],
                dt,
            )
            calibrated_twist = matrix_to_twist(
                calibrated_M,
                tracker_data["prev_calibrated_matrix"],
                dt,
            )

            tracker_data["publisher_raw"].publish(
                self.create_vive_msg(raw_pose, raw_twist)
            )
            tracker_data["publisher_calibrated"].publish(
                self.create_vive_msg(calibrated_pose, calibrated_twist)
            )

            # 이전 값 저장
            tracker_data["prev_raw_matrix"] = raw_M.copy()
            tracker_data["prev_calibrated_matrix"] = calibrated_M.copy()

            # TF 브로드캐스트
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = self.clock.now().to_msg()
                t.header.frame_id = self.base_frame
                t.child_frame_id = tracker_data["child_frame"]
                t.transform.translation.x = calibrated_pose.position.x
                t.transform.translation.y = calibrated_pose.position.y
                t.transform.translation.z = calibrated_pose.position.z
                t.transform.rotation.x = calibrated_pose.orientation.x
                t.transform.rotation.y = calibrated_pose.orientation.y
                t.transform.rotation.z = calibrated_pose.orientation.z
                t.transform.rotation.w = calibrated_pose.orientation.w
                self.tf_broadcaster.sendTransform(t)

        # 첫 번째 트래커는 공통 토픽에도 발행
        if tracker_list:
            first_id = tracker_list[0]
            first_tracker = self.trackers[first_id]

            self.raw_pose_matrix = first_tracker["raw_pose_matrix"].copy()
            self.calibrated_pose_matrix = first_tracker["calibrated_pose_matrix"].copy()

            self.raw_pose = matrix_to_pose(self.raw_pose_matrix)
            self.calibrated_pose = matrix_to_pose(self.calibrated_pose_matrix)

            self.raw_twist = matrix_to_twist(
                self.raw_pose_matrix, self.prev_raw_pose_matrix, dt
            )
            self.calibrated_twist = matrix_to_twist(
                self.calibrated_pose_matrix, self.prev_calibrated_pose_matrix, dt
            )

            self.raw_pose_pub.publish(
                self.create_vive_msg(self.raw_pose, self.raw_twist)
            )
            self.calibrated_pose_pub.publish(
                self.create_vive_msg(self.calibrated_pose, self.calibrated_twist)
            )

            self.prev_raw_pose_matrix = self.raw_pose_matrix.copy()
            self.prev_calibrated_pose_matrix = self.calibrated_pose_matrix.copy()

        self.prev_time = current_time


def main(args=None):
    rclpy_initialized = False
    try:
        rclpy.init(args=args)
        rclpy_initialized = True

        tracker = ViveTracker()
        if tracker.vr_system is None:
            print("오류: VR 시스템을 초기화할 수 없습니다. SteamVR이 실행 중인지 확인하세요.")
            return 1

        print("ViveTracker 노드가 시작되었습니다. 종료하려면 Ctrl+C")
        rclpy.spin(tracker)

    except KeyboardInterrupt:
        print("ViveTracker 노드를 종료합니다...")
    finally:
        # VR shutdown
        try:
            openvr.shutdown()
        except Exception as e:
            print(f"VR 시스템 종료 중 오류: {e}")

        # node destroy
        try:
            tracker.destroy_node()
        except Exception:
            pass

        if rclpy_initialized:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    main()
