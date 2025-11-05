#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import re

import openvr
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from vive_tracker_interfaces.srv import ViveCalibration

# 기존 utils 그대로 가져와도 되고, calibrate 서비스에서만 씀
from vive_tracker_ros2.utils import (
    calculate_calibration_matrix,
    matrix_to_pose,
    matrix_to_twist,
    pose_to_matrix,
)

import ament_index_python.packages


def rot_x(th_rad: float) -> np.ndarray:
    c = np.cos(th_rad)
    s = np.sin(th_rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=np.float64,
    )


def rot_z(th_rad: float) -> np.ndarray:
    c = np.cos(th_rad)
    s = np.sin(th_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def openvr_pose_to_np44(pose) -> np.ndarray:
    """
    ROS1 C++에서 하던 것처럼 openvr의 3x4를 그대로 4x4로 올린다.
    pose.mDeviceToAbsoluteTracking 은 3행 4열.
    """
    m = pose.mDeviceToAbsoluteTracking
    M = np.eye(4, dtype=np.float64)
    # rotation
    M[0, 0] = m[0][0]
    M[0, 1] = m[0][1]
    M[0, 2] = m[0][2]

    M[1, 0] = m[1][0]
    M[1, 1] = m[1][1]
    M[1, 2] = m[1][2]

    M[2, 0] = m[2][0]
    M[2, 1] = m[2][1]
    M[2, 2] = m[2][2]

    # translation
    M[0, 3] = m[0][3]
    M[1, 3] = m[1][3]
    M[2, 3] = m[2][3]

    return M


class ViveTracker(Node):
    def __init__(self):
        super().__init__("vive_tracker")

        self.vr_system = None
        self.trackers = {}

        # ---- ROS init ----
        self._init_ros()
        # ---- VR init ----
        self._init_vr()
        # ---- calibration (json) for service result, not the ros1-style yaml ----
        self._init_json_calib()
        # ---- ros1-style yaml load ----
        self._init_yaml_calib()

        self.prev_time = self.get_clock().now()

    # ----------------------------------------------------------------------
    # init blocks
    # ----------------------------------------------------------------------
    def _init_ros(self):
        self.raw_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/raw_pose", 10
        )
        self.calibrated_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/calibrated_pose", 10
        )

        self.calibrate_srv = self.create_service(
            ViveCalibration, "vive_tracker_ros/calibrate", self.cb_calibrate
        )

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("child_frame", "vive_tracker")

        self.publish_tf = self.get_parameter("publish_tf").value
        self.base_frame = self.get_parameter("base_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.01, self.cb_vive_timer)  # 100 Hz

    def _init_vr(self):
        try:
            self.get_logger().info("Initializing VR system...")
            self.vr_system = openvr.init(openvr.VRApplication_Other)
            self.get_logger().info("VR system initialized successfully!")

            poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding,
                0,
                openvr.k_unMaxTrackedDeviceCount,
            )

            for i in range(openvr.k_unMaxTrackedDeviceCount):
                device_class = self.vr_system.getTrackedDeviceClass(i)
                if device_class != openvr.TrackedDeviceClass_Invalid:
                    self.get_logger().info(
                        f"[vive_tracker_node] Found VR device: (index {i})"
                    )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize VR system: {e}")
            self.vr_system = None

    def _init_json_calib(self):
        # 이건 원래 코드가 쓰던 json (calibrate 서비스 결과 저장용)
        share_dir = ament_index_python.packages.get_package_share_directory(
            "vive_tracker_ros2"
        )
        self.config_dir_install = os.path.join(share_dir, "config")

        if os.path.exists(os.path.join(self.config_dir_install, "calibration_matrix.json")):
            with open(
                os.path.join(self.config_dir_install, "calibration_matrix.json"), "r"
            ) as f:
                data = json.load(f)
                self.T_tool_opt = np.array(data["T_tool_opt"], dtype=np.float64)
                self.T_trans_opt = np.array(data["T_trans_opt"], dtype=np.float64)
        else:
            self.T_tool_opt = np.eye(4, dtype=np.float64)
            self.T_trans_opt = np.eye(4, dtype=np.float64)

    def _init_yaml_calib(self):
        """
        ros1에서 하던 T_AD, T_BC, T_CE, R_Adj를 yaml에서 읽어온다.
        src 경로 먼저, 안 되면 install/share 경로
        """
        # 1) src 기준
        src_yaml = os.path.join(
            os.path.expanduser("~/nrs_ws/src/vive_tracker_ros2/yaml"),
            "calibration_matrix.yaml",
        )
        # 2) install/share 기준
        share_dir = ament_index_python.packages.get_package_share_directory(
            "vive_tracker_ros2"
        )
        install_yaml = os.path.join(share_dir, "yaml", "calibration_matrix.yaml")

        yaml_path = src_yaml if os.path.exists(src_yaml) else install_yaml

        self.get_logger().info(f"[vive_tracker_node] load yaml from src: {yaml_path}")

        # 진짜로 읽기
        import yaml

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        # 없으면 단위행렬
        self.T_AD = np.array(data.get("T_AD", np.eye(4)), dtype=np.float64)
        self.T_BC = np.array(data.get("T_BC", np.eye(4)), dtype=np.float64)
        self.T_CE = np.array(data.get("T_CE", np.eye(4)), dtype=np.float64)
        self.R_Adj = np.array(data.get("R_Adj", np.eye(3)), dtype=np.float64)

        # ros1에서 하던 heuristic TAdj 만들기
        # VR_Cali_TAdj.block(0,0,3,3) = RotZ(0)*RotX(0)*VR_Cali_RAdj.transpose();
        # 나머지는 단위
        self.T_Adj = np.eye(4, dtype=np.float64)
        R_adj_full = rot_z(0.0) @ rot_x(0.0) @ self.R_Adj.T
        self.T_Adj[:3, :3] = R_adj_full

    # ----------------------------------------------------------------------
    # services
    # ----------------------------------------------------------------------
    def cb_calibrate(self, request, response):
        robot_matrices = [pose_to_matrix(pose) for pose in request.robot_poses]
        tracker_matrices = [pose_to_matrix(pose) for pose in request.tracker_poses]

        if len(robot_matrices) != len(tracker_matrices):
            self.get_logger().error(
                "The number of robot poses and tracker poses must be the same"
            )
            response.success = False
            return response

        try:
            self.T_tool_opt, self.T_trans_opt = calculate_calibration_matrix(
                robot_matrices, tracker_matrices
            )
        except Exception as e:
            self.get_logger().error(f"Failed to calculate calibration matrix: {e}")
            response.success = False
            return response

        os.makedirs(self.config_dir_install, exist_ok=True)
        with open(
            os.path.join(self.config_dir_install, "calibration_matrix.json"), "w"
        ) as f:
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

    # ----------------------------------------------------------------------
    # tracker pose
    # ----------------------------------------------------------------------
    def _update_trackers_from_vr(self):
        if self.vr_system is None:
            return []

        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            openvr.k_unMaxTrackedDeviceCount,
        )

        current_ids = []

        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_GenericTracker:
                pose = poses[i]
                if not (pose.bDeviceIsConnected and pose.bPoseIsValid):
                    continue

                try:
                    serial = self.vr_system.getStringTrackedDeviceProperty(
                        i, openvr.Prop_SerialNumber_String
                    )
                except Exception:
                    serial = f"tracker_{i}"

                safe_serial = re.sub(r"[^a-zA-Z0-9_]", "_", serial)

                if serial not in self.trackers:
                    raw_topic = f"vive_tracker_ros/{safe_serial}/raw_pose"
                    cali_topic = f"vive_tracker_ros/{safe_serial}/calibrated_pose"
                    child_frame = f"{self.child_frame}_{safe_serial}"

                    self.trackers[serial] = {
                        "device_index": i,
                        "child_frame": child_frame,
                        "publisher_raw": self.create_publisher(
                            Odometry, raw_topic, 10
                        ),
                        "publisher_calibrated": self.create_publisher(
                            Odometry, cali_topic, 10
                        ),
                        "prev_raw_matrix": np.eye(4, dtype=np.float64),
                        "prev_calibrated_matrix": np.eye(4, dtype=np.float64),
                    }
                    self.get_logger().info(f"새 트래커 발견: {serial} (index {i})")

                # 여기서 ROS1처럼 openvr → 4x4
                raw_pose_matrix = openvr_pose_to_np44(pose)
                self.trackers[serial]["raw_pose_matrix"] = raw_pose_matrix
                current_ids.append(serial)

        return current_ids

    # ----------------------------------------------------------------------
    # main timer
    # ----------------------------------------------------------------------
    def create_vive_msg(self, pose: Pose, twist: Twist, frame_id="world"):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose = pose
        msg.twist.twist = twist
        return msg

    def cb_vive_timer(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9

        current_ids = self._update_trackers_from_vr()
        if not current_ids:
            self.prev_time = now
            return

        for serial in current_ids:
            tdata = self.trackers[serial]
            raw_M = tdata["raw_pose_matrix"]

            # ====== ROS1 방식 그대로 ======
            # 1) heuristic T_Adj를 앞에 곱한다
            M_adj = self.T_Adj @ raw_M
            # 2) T_AD * (...) * T_CE
            M_cal = self.T_AD @ M_adj @ self.T_CE
            # (참고: ROS1은 T_BC는 지금 안 씀)

            # pose/twist로 변환
            raw_pose = matrix_to_pose(raw_M)
            cal_pose = matrix_to_pose(M_cal)

            raw_twist = matrix_to_twist(
                raw_M,
                tdata["prev_raw_matrix"],
                dt,
            )
            cal_twist = matrix_to_twist(
                M_cal,
                tdata["prev_calibrated_matrix"],
                dt,
            )

            # publish (tracker별)
            raw_msg = self.create_vive_msg(raw_pose, raw_twist)
            cal_msg = self.create_vive_msg(cal_pose, cal_twist)

            tdata["publisher_raw"].publish(raw_msg)
            tdata["publisher_calibrated"].publish(cal_msg)

            # TF
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = self.base_frame
                t.child_frame_id = tdata["child_frame"]
                t.transform.translation.x = cal_pose.position.x
                t.transform.translation.y = cal_pose.position.y
                t.transform.translation.z = cal_pose.position.z
                t.transform.rotation = cal_pose.orientation
                self.tf_broadcaster.sendTransform(t)

            # backup
            tdata["prev_raw_matrix"] = raw_M.copy()
            tdata["prev_calibrated_matrix"] = M_cal.copy()

        # 첫 번째 트래커는 예전 토픽에도 발행해서 기존 코드와 호환
        first_id = current_ids[0]
        first = self.trackers[first_id]
        raw_pose = matrix_to_pose(first["raw_pose_matrix"])
        cal_pose = matrix_to_pose(first["prev_calibrated_matrix"])
        raw_twist = matrix_to_twist(
            first["raw_pose_matrix"], first["prev_raw_matrix"], dt
        )
        cal_twist = matrix_to_twist(
            first["prev_calibrated_matrix"], first["prev_calibrated_matrix"], dt
        )

        self.raw_pose_pub.publish(self.create_vive_msg(raw_pose, raw_twist))
        self.calibrated_pose_pub.publish(self.create_vive_msg(cal_pose, cal_twist))

        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ViveTracker()

    if node.vr_system is None:
        node.get_logger().error("VR system is not initialized. Exiting.")
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            openvr.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
