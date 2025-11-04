#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import openvr
import numpy as np
import os
import json
from scipy.spatial.transform import Rotation as R
import ament_index_python.packages
import re  # 파일 상단에 추가
# ROS msg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry
from vive_tracker_interfaces.srv import ViveCalibration
from tf2_ros import TransformBroadcaster

# Utils
from vive_tracker_ros2.utils import (
    calculate_calibration_matrix,
    transform_HmdMatrix2npmatrix,
    matrix_to_pose,
    matrix_to_twist,
    pose_to_matrix,
)


# 생성되는 Json 파일 경로: 
# /home/vision/ros2_ws/install/vive_tracker_ros2/share/vive_tracker_ros2/config


CONFIG_PATH = os.path.join(
    ament_index_python.packages.get_package_share_directory("vive_tracker_ros2"),
    "config"
)








class ViveTracker(Node):
    def __init__(self):
        super().__init__("vive_tracker")
        self.vr_system = None
        
        # 트래커 관련 데이터 저장을 위한 딕셔너리 (tracker_id를 키로 사용)
        self.trackers = {}
        self.tracker_ids = []  # 발견된 트래커의 ID 목록

        # Initialize
        self.initial_ros()
        self.initial_vr()

        self.raw_pose_matrix = np.eye(4)

        if os.path.exists(os.path.join(CONFIG_PATH, "calibration_matrix.json")):
            with open(
                os.path.join(CONFIG_PATH, "calibration_matrix.json"),
                "r",
            ) as file:
                data = json.load(file)
                self.T_tool_opt = np.array(data["T_tool_opt"])
                self.T_trans_opt = np.array(data["T_trans_opt"])
        else:
            self.T_tool_opt = np.eye(4)
            self.T_trans_opt = np.eye(4)

        self.prev_raw_pose_matrix = np.eye(4)
        self.prev_calibrated_pose_matrix = np.eye(4)
        self.prev_time = self.get_clock().now()

    def initial_ros(self):
        # 기본 퍼블리셔 (기존 코드와의 호환성 유지)
        self.raw_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/raw_pose", 10
        )
        self.calibrated_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/calibrated_pose", 10
        )

        # 서비스
        self.calibrate_srv = self.create_service(
            ViveCalibration, "vive_tracker_ros/calibrate", self.cb_calibrate
        )

        # ros2 파라미터
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("child_frame", "vive_tracker")

        self.publish_tf = self.get_parameter("publish_tf").value
        self.base_frame = self.get_parameter("base_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        # 로거, 클락, 타이머
        self.logger = self.get_logger()
        self.clock = self.get_clock()
        self.timer = self.create_timer(0.01, self.cb_vive_timer)

        # TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)

    def initial_vr(self):
        try:
            self.logger.info("Initializing VR system... This may take a few moments.")
            self.vr_system = openvr.init(openvr.VRApplication_Other)
            self.logger.info("VR system initialized successfully!")
            
            # Check if any VR devices are connected
            device_found = False
            poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )
            
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                device_class = self.vr_system.getTrackedDeviceClass(i)
                if device_class != openvr.TrackedDeviceClass_Invalid:
                    device_found = True
                    device_type = "Unknown"
                    if device_class == openvr.TrackedDeviceClass_HMD:
                        device_type = "HMD"
                    elif device_class == openvr.TrackedDeviceClass_Controller:
                        device_type = "Controller"
                    elif device_class == openvr.TrackedDeviceClass_GenericTracker:
                        device_type = "Tracker"
                    self.logger.info(f"Found VR device: {device_type} (index {i})")
            
            if not device_found:
                self.logger.warn("No VR devices found. Is your tracker connected and SteamVR running?")
                
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize VR system: {e}")
            self.logger.error("Make sure SteamVR is installed and running, and your tracker is connected.")
            return False

    def cb_calibrate(self, request, response):
        # print("[Debug] ViveTracker 서비스가 시작되었습니다.")
        self.logger.info("[Debug] ViveTracker 서비스가 시작되었습니다.")
        robot_matrices = [pose_to_matrix(pose) for pose in request.robot_poses]
        tracker_matrices = [pose_to_matrix(pose) for pose in request.tracker_poses]

        # 데이터 개수 확인
        if len(robot_matrices) != len(tracker_matrices):
            self.logger.error(
                "The number of robot poses and tracker poses must be the same"
            )
            response.success = False
            return response
        else:
            self.logger.info(
                f"Received {len(robot_matrices)} robot poses and {len(tracker_matrices)} tracker poses"
            )

        try:
            self.logger.info("[Debug] Starting calibration matrix calculation...")
            self.T_tool_opt, self.T_trans_opt = calculate_calibration_matrix(
                robot_matrices, tracker_matrices
            )
        except Exception as e:
            self.logger.error(f"Failed to calculate calibration matrix: {e}")
            response.success = False
            return response
        
        os.makedirs(CONFIG_PATH, exist_ok=True)
        self.logger.info("[Debug] Making sure the config directory exists...")
        with open(os.path.join(CONFIG_PATH, "calibration_matrix.json"), "w") as file:
            self.logger.info("[Debug] Saving calibration matrix to json file...")
            json.dump(
                {
                    "T_tool_opt": self.T_tool_opt.tolist(),
                    "T_trans_opt": self.T_trans_opt.tolist(),
                },
                file,
                indent=4,
            )

        response.success = True
        return response

    def get_tracker_poses(self):
        """Get poses of all available trackers"""
        try:
            if self.vr_system is None:
                self.logger.error("VR 시스템이 초기화되지 않았습니다.")
                return {}

            poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )

            tracker_data = {}
            tracker_found = False

            for i in range(openvr.k_unMaxTrackedDeviceCount):
                try:
                    device_class = self.vr_system.getTrackedDeviceClass(i)

                    if device_class == openvr.TrackedDeviceClass_GenericTracker:
                        pose = poses[i]

                        if pose.bDeviceIsConnected and pose.bPoseIsValid:
                            tracker_found = True

                            # 시리얼 번호 가져오기
                            try:
                                serial = self.vr_system.getStringTrackedDeviceProperty(
                                    i, openvr.Prop_SerialNumber_String
                                )
                            except Exception as e:
                                self.logger.warn(f"트래커 {i}의 시리얼 번호를 가져오는 중 오류 발생: {e}")
                                serial = f"tracker_{i}"

                            # 안전한 토픽 이름용 시리얼
                            safe_serial = re.sub(r"[^a-zA-Z0-9_]", "_", serial)

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
                                    "publisher_calibrated": self.create_publisher(Odometry, cali_topic, 10)
                                }
                                self.logger.info(f"새 트래커 발견: {serial} (index {i})")
                                self.logger.info(f"토픽 발행 시작: {raw_topic}, {cali_topic}")

                            raw_pose_matrix = transform_HmdMatrix2npmatrix(pose.mDeviceToAbsoluteTracking)
                            self.trackers[serial]["raw_pose_matrix"] = raw_pose_matrix

                            tracker_data[serial] = self.trackers[serial]
                except Exception as e:
                    self.logger.warn(f"장치 {i} 정보 가져오는 중 오류 발생: {e}")

            if not tracker_found and not hasattr(self, "_tracker_warning_shown"):
                self.logger.warn("유효한 트래커 포즈를 찾을 수 없습니다. 트래커가 올바르게 설정되어 있는지 확인하세요.")
                self._tracker_warning_shown = True
            elif tracker_found and hasattr(self, "_tracker_warning_shown"):
                self.logger.info("트래커 연결이 복원되었습니다!")
                delattr(self, "_tracker_warning_shown")

            return tracker_data.keys()

        except Exception as e:
            self.logger.error(f"트래커 포즈를 가져오는 중 오류 발생: {e}")
            return []

            
    # 기존 get_tracker_pose 함수는 호환성을 위해 남겨두지만 첫 번째 트래커만 반환
    def get_tracker_pose(self):
        """Get the pose of the first found tracker (legacy function)"""
        try:
            poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )
            
            tracker_found = False
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                pose = poses[i]
                if (
                    pose.bDeviceIsConnected
                    and pose.bPoseIsValid
                    and self.vr_system.getTrackedDeviceClass(i)
                    == openvr.TrackedDeviceClass_GenericTracker
                ):
                    tracker_found = True
                    return transform_HmdMatrix2npmatrix(pose.mDeviceToAbsoluteTracking)
                    
            if not tracker_found and not hasattr(self, "_tracker_warning_shown"):
                self.logger.warn("유효한 트래커 포즈를 찾을 수 없습니다. 트래커가 올바르게 설정되어 있는지 확인하세요.")
                self._tracker_warning_shown = True
            elif tracker_found and hasattr(self, "_tracker_warning_shown"):
                self.logger.info("트래커 연결이 복원되었습니다!")
                delattr(self, "_tracker_warning_shown")
                
            return None
        except Exception as e:
            self.logger.error(f"트래커 포즈를 가져오는 중 오류 발생: {e}")
            return None

    def create_vive_msg(self, pose: Pose, twist: Twist, frame_id="world"):
        pose_msg = Odometry()
        pose_msg.header.stamp = self.clock.now().to_msg()
        pose_msg.header.frame_id = frame_id

        pose_msg.pose.pose = pose
        pose_msg.twist.twist = twist

        return pose_msg

    def cb_vive_timer(self):
        # 시간 초기화
        current_time = self.clock.now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        # 트래커 포즈 업데이트 - 활성 트래커 ID 목록 가져오기
        current_tracker_ids = self.get_tracker_poses()
        
        # 트래커가 없으면 리턴
        if not current_tracker_ids:
            return
        
        # 회전 변환 행렬 (모든 트래커에 적용)
        rotation_transform = np.eye(4)
        rotation_transform[:3, :3] = R.from_euler("y", 180, degrees=True).as_matrix()
        
        # 발견된 각 트래커에 대해 처리
        tracker_list = list(current_tracker_ids)  # ID 목록을 리스트로 변환
        for tracker_id in tracker_list:
            tracker_data = self.trackers[tracker_id]
            
            # 캘리브레이션된 포즈 계산
            tracker_data["calibrated_pose_matrix"] = (
                self.T_trans_opt @ tracker_data["raw_pose_matrix"] @ rotation_transform
            )
            
            # 행렬을 포즈와 트위스트로 변환
            raw_pose = matrix_to_pose(tracker_data["raw_pose_matrix"])
            calibrated_pose = matrix_to_pose(tracker_data["calibrated_pose_matrix"])
            
            # 트위스트 계산 (속도)
            raw_twist = matrix_to_twist(
                tracker_data["raw_pose_matrix"], 
                tracker_data["prev_raw_matrix"], 
                dt
            )
            calibrated_twist = matrix_to_twist(
                tracker_data["calibrated_pose_matrix"], 
                tracker_data["prev_calibrated_matrix"], 
                dt
            )
            
            # 메시지 생성
            raw_msg = self.create_vive_msg(raw_pose, raw_twist)
            cali_msg = self.create_vive_msg(calibrated_pose, calibrated_twist)
            
            # 메시지 발행 (트래커별)
            tracker_data["publisher_raw"].publish(raw_msg)
            tracker_data["publisher_calibrated"].publish(cali_msg)
            
            # 이전 포즈 업데이트
            tracker_data["prev_raw_matrix"] = tracker_data["raw_pose_matrix"].copy()
            tracker_data["prev_calibrated_matrix"] = tracker_data["calibrated_pose_matrix"].copy()
            
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
        
        # 기존 코드와의 호환성을 위해 첫 번째 트래커 정보 유지
        if tracker_list:
            first_tracker_id = tracker_list[0]
            first_tracker = self.trackers[first_tracker_id]
            
            self.raw_pose_matrix = first_tracker["raw_pose_matrix"].copy()
            self.calibrated_pose_matrix = first_tracker["calibrated_pose_matrix"].copy()
            
            # 기존 퍼블리셔에도 발행
            self.raw_pose = matrix_to_pose(self.raw_pose_matrix)
            self.calibrated_pose = matrix_to_pose(self.calibrated_pose_matrix)
            self.raw_twist = matrix_to_twist(
                self.raw_pose_matrix, self.prev_raw_pose_matrix, dt
            )
            self.calibrated_twist = matrix_to_twist(
                self.calibrated_pose_matrix, self.prev_calibrated_pose_matrix, dt
            )
            
            raw_msg = self.create_vive_msg(self.raw_pose, self.raw_twist)
            cali_msg = self.create_vive_msg(self.calibrated_pose, self.calibrated_twist)
            self.raw_pose_pub.publish(raw_msg)
            self.calibrated_pose_pub.publish(cali_msg)
            
            self.prev_raw_pose_matrix = self.raw_pose_matrix.copy()
            self.prev_calibrated_pose_matrix = self.calibrated_pose_matrix.copy()
            
        # 시간 업데이트
        self.prev_time = current_time

def main(args=None):
    # rclpy 초기화 여부 추적
    rclpy_initialized = False
    
    try:
        rclpy.init(args=args)
        rclpy_initialized = True
        
        tracker = ViveTracker()
        if tracker.vr_system is None:
            print("오류: VR 시스템을 초기화할 수 없습니다. SteamVR이 실행 중인지 확인하세요.")
            return 1
            
        print("ViveTracker 노드가 시작되었습니다. 종료하려면 Ctrl+C를 누르세요.")
        
        try:
            rclpy.spin(tracker)
        except KeyboardInterrupt:
            print("ViveTracker 노드를 종료합니다...")
        except Exception as e:
            print(f"예상치 못한 오류: {e}")
            
    finally:
        # VR 시스템 리소스 정리
        try:
            print("VR 시스템 리소스를 정리합니다...")
            openvr.shutdown()
        except Exception as e:
            print(f"VR 시스템 종료 중 오류 발생: {e}")
        
        # 노드 정리
        if "tracker" in locals():
            try:
                tracker.destroy_node()
            except Exception as e:
                print(f"노드 종료 중 오류 발생: {e}")
        
        # rclpy 종료 (초기화된 경우에만)
        if rclpy_initialized:
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"ROS 종료 중 오류 발생: {e}")
        
    return 0

if __name__ == "__main__":
    main()
