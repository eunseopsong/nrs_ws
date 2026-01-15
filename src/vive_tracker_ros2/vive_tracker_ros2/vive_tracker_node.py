#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import re
import math

import openvr
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from vive_tracker_interfaces.srv import ViveCalibration

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
    m = pose.mDeviceToAbsoluteTracking
    M = np.eye(4, dtype=np.float64)
    M[0, 0] = m[0][0]; M[0, 1] = m[0][1]; M[0, 2] = m[0][2]
    M[1, 0] = m[1][0]; M[1, 1] = m[1][1]; M[1, 2] = m[1][2]
    M[2, 0] = m[2][0]; M[2, 1] = m[2][1]; M[2, 2] = m[2][2]
    M[0, 3] = m[0][3]
    M[1, 3] = m[1][3]
    M[2, 3] = m[2][3]
    return M


def rotmat_to_rotvec(R: np.ndarray) -> np.ndarray:
    """
    Rotation matrix (3x3) -> rotation vector w (3,)
    w = axis * angle  (angle in [0, pi], rad)
    """
    tr = float(np.trace(R))
    cos_th = (tr - 1.0) * 0.5
    cos_th = max(-1.0, min(1.0, cos_th))
    th = math.acos(cos_th)

    if th < 1e-12:
        return np.zeros(3, dtype=np.float64)

    # near pi: sin(th) is small -> use special handling
    if abs(math.pi - th) < 1e-5:
        axis = np.zeros(3, dtype=np.float64)
        axis[0] = math.sqrt(max(0.0, (R[0, 0] + 1.0) * 0.5))
        axis[1] = math.sqrt(max(0.0, (R[1, 1] + 1.0) * 0.5))
        axis[2] = math.sqrt(max(0.0, (R[2, 2] + 1.0) * 0.5))

        if R[0, 1] < 0.0:
            axis[1] = -axis[1]
        if R[0, 2] < 0.0:
            axis[2] = -axis[2]

        n = float(np.linalg.norm(axis))
        if n < 1e-10:
            v = np.array([R[2, 1] - R[1, 2],
                          R[0, 2] - R[2, 0],
                          R[1, 0] - R[0, 1]], dtype=np.float64)
            nv = float(np.linalg.norm(v))
            if nv < 1e-12:
                return np.zeros(3, dtype=np.float64)
            axis = v / nv
        else:
            axis = axis / n

        return axis * th

    v = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ], dtype=np.float64)

    s = math.sin(th)
    scale = th / (2.0 * s)
    return scale * v


# ✅ 네가 준 측정치 기반 "기본" 보정 (YAML에 T_SA 있으면 그걸 우선)
DEFAULT_T_SA = np.array(
    [
        [-0.999987700,  0.000392998,  0.004944318,  0.0],
        [ 0.000416257,  0.999988849,  0.004704043,  0.0],
        [-0.004942414,  0.004706043, -0.999976713,  0.0],
        [ 0.0,          0.0,          0.0,          1.0],
    ],
    dtype=np.float64,
)


class ViveTracker(Node):
    def __init__(self):
        super().__init__("vive_tracker")

        self.vr_system = None
        self.trackers = {}

        self._init_ros()
        self._init_vr()
        self._init_json_calib()
        self._init_yaml_calib()

        self.prev_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # ROS init
    # ------------------------------------------------------------------
    def _init_ros(self):
        self.raw_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/raw_pose", 10
        )
        self.calibrated_pose_pub_odom = self.create_publisher(
            Odometry, "vive_tracker_ros/calibrated_pose", 10
        )

        # PoseStamped raw (/raw_pose)
        self.raw_pose_pub_pose = self.create_publisher(
            PoseStamped, "/raw_pose", 10
        )

        # ✅ /calibrated_pose : [x y z wx wy wz] (m, rad)
        self.calibrated_pose_pub = self.create_publisher(
            Float64MultiArray, "/calibrated_pose", 10
        )

        self.calibrate_srv = self.create_service(
            ViveCalibration, "vive_tracker_ros/calibrate", self.cb_calibrate
        )

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("child_frame", "vive_tracker")

        # ✅ T_SA 적용 방식 선택 (기본: 좌곱)
        # - left:  M_cal = T_SA @ M_cal  (프레임 자체를 맞추는 용도에 보통 더 자연스러움)
        # - right: M_cal = M_cal @ T_SA  (센서 바디 축 오프셋 같은 "로컬" 보정에 가까움)
        self.declare_parameter("apply_T_SA", True)
        self.declare_parameter("T_SA_side", "left")  # "left" or "right"
        self.declare_parameter("debug_print_T_SA", False)

        self.publish_tf = self.get_parameter("publish_tf").value
        self.base_frame = self.get_parameter("base_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.apply_T_SA = bool(self.get_parameter("apply_T_SA").value)
        self.T_SA_side = str(self.get_parameter("T_SA_side").value).lower()
        self.debug_print_T_SA = bool(self.get_parameter("debug_print_T_SA").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.cb_vive_timer)

    # ------------------------------------------------------------------
    # VR init
    # ------------------------------------------------------------------
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
                    self.get_logger().info(f"[vive_tracker_node] Found VR device: (index {i})")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize VR system: {e}")
            self.vr_system = None

    # ------------------------------------------------------------------
    # json calib (service 결과)
    # ------------------------------------------------------------------
    def _init_json_calib(self):
        share_dir = ament_index_python.packages.get_package_share_directory(
            "vive_tracker_ros2"
        )
        self.config_dir_install = os.path.join(share_dir, "config")

        json_path = os.path.join(self.config_dir_install, "calibration_matrix.json")
        if os.path.exists(json_path):
            with open(json_path, "r") as f:
                data = json.load(f)
            self.T_tool_opt = np.array(data["T_tool_opt"], dtype=np.float64)
            self.T_trans_opt = np.array(data["T_trans_opt"], dtype=np.float64)
        else:
            self.T_tool_opt = np.eye(4, dtype=np.float64)
            self.T_trans_opt = np.eye(4, dtype=np.float64)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _to_T44(mat_like):
        """Accept 4x4 or 3x3. Convert to 4x4 homogeneous. Return None if invalid."""
        if mat_like is None:
            return None
        M = np.array(mat_like, dtype=np.float64)
        if M.shape == (4, 4):
            return M
        if M.shape == (3, 3):
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = M
            return T
        return None

    @staticmethod
    def _is_valid_T(T: np.ndarray) -> bool:
        if T is None or T.shape != (4, 4):
            return False
        if not np.all(np.isfinite(T)):
            return False
        # 마지막 행은 [0 0 0 1] 근처인지 체크(너무 엄격하게 하진 않음)
        if np.linalg.norm(T[3, :] - np.array([0, 0, 0, 1], dtype=np.float64)) > 1e-3:
            return False
        return True

    # ------------------------------------------------------------------
    # ros1-style yaml calib
    # ------------------------------------------------------------------
    def _init_yaml_calib(self):
        src_yaml = os.path.join(
            os.path.expanduser("~/nrs_ws/src/vive_tracker_ros2/yaml"),
            "calibration_matrix.yaml",
        )
        share_dir = ament_index_python.packages.get_package_share_directory(
            "vive_tracker_ros2"
        )
        install_yaml = os.path.join(share_dir, "yaml", "calibration_matrix.yaml")

        yaml_path = src_yaml if os.path.exists(src_yaml) else install_yaml
        self.get_logger().info(f"[vive_tracker_node] load yaml: {yaml_path}")

        import yaml
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        self.T_AD = np.array(data.get("T_AD", np.eye(4)), dtype=np.float64)
        self.T_BC = np.array(data.get("T_BC", np.eye(4)), dtype=np.float64)
        self.T_CE = np.array(data.get("T_CE", np.eye(4)), dtype=np.float64)
        self.R_Adj = np.array(data.get("R_Adj", np.eye(3)), dtype=np.float64)

        # ROS1 순서: T_Adj = R_Adj.T
        self.T_Adj = np.eye(4, dtype=np.float64)
        R_adj_full = rot_z(0.0) @ rot_x(0.0) @ self.R_Adj.T
        self.T_Adj[:3, :3] = R_adj_full

        # ✅ T_SA: YAML에 있으면 우선, 없으면 DEFAULT
        T_SA_loaded = self._to_T44(data.get("T_SA", None))
        if self._is_valid_T(T_SA_loaded):
            self.T_SA = T_SA_loaded
            self.get_logger().info("[vive_tracker_node] T_SA loaded from yaml.")
        else:
            self.T_SA = DEFAULT_T_SA.copy()
            self.get_logger().warn("[vive_tracker_node] T_SA not found/invalid in yaml. Using DEFAULT_T_SA.")

        if self.debug_print_T_SA:
            self.get_logger().info(f"T_SA_side={self.T_SA_side}, apply_T_SA={self.apply_T_SA}")
            self.get_logger().info("T_SA=\n" + np.array2string(self.T_SA, precision=6, suppress_small=True))

    # ------------------------------------------------------------------
    # service
    # ------------------------------------------------------------------
    def cb_calibrate(self, request, response):
        robot_matrices = [pose_to_matrix(pose) for pose in request.robot_poses]
        tracker_matrices = [pose_to_matrix(pose) for pose in request.tracker_poses]

        if len(robot_matrices) != len(tracker_matrices):
            self.get_logger().error("robot pose count != tracker pose count")
            response.success = False
            return response

        try:
            self.T_tool_opt, self.T_trans_opt = calculate_calibration_matrix(
                robot_matrices, tracker_matrices
            )
        except Exception as e:
            self.get_logger().error(f"calibration failed: {e}")
            response.success = False
            return response

        os.makedirs(self.config_dir_install, exist_ok=True)
        with open(os.path.join(self.config_dir_install, "calibration_matrix.json"), "w") as f:
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
    # tracker pose update
    # ------------------------------------------------------------------
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
                        "publisher_raw": self.create_publisher(Odometry, raw_topic, 10),
                        "publisher_calibrated": self.create_publisher(Odometry, cali_topic, 10),
                        "prev_raw_matrix": np.eye(4, dtype=np.float64),
                        "prev_calibrated_matrix": np.eye(4, dtype=np.float64),
                    }
                    self.get_logger().info(f"새 트래커 발견: {serial}")

                raw_pose_matrix = openvr_pose_to_np44(pose)
                self.trackers[serial]["raw_pose_matrix"] = raw_pose_matrix
                current_ids.append(serial)

        return current_ids

    # ------------------------------------------------------------------
    # timer
    # ------------------------------------------------------------------
    def create_vive_msg(self, pose: Pose, twist: Twist, frame_id="world"):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose = pose
        msg.twist.twist = twist
        return msg

    def create_pose_stamped(self, pose: Pose, frame_id="world") -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose = pose
        return msg

    def _apply_T_SA_to_M_cal(self, M_cal: np.ndarray) -> np.ndarray:
        if not self.apply_T_SA:
            return M_cal

        if self.T_SA_side == "right":
            # 로컬(바디) 축 오프셋 느낌
            return M_cal @ self.T_SA

        # 기본: left
        # 프레임 align 목적(네가 말한 "spatial angle frame 맞추기")에는 보통 이게 더 자연스러움
        return self.T_SA @ M_cal

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

            # base chain (ROS1 순서)
            M_adj = self.T_Adj @ raw_M
            M_cal = self.T_AD @ M_adj @ self.T_CE

            # ✅ spatial-angle alignment
            M_cal = self._apply_T_SA_to_M_cal(M_cal)

            # pose로
            raw_pose = matrix_to_pose(raw_M)
            cal_pose = matrix_to_pose(M_cal)

            raw_twist = matrix_to_twist(raw_M, tdata["prev_raw_matrix"], dt)
            cal_twist = matrix_to_twist(M_cal, tdata["prev_calibrated_matrix"], dt)

            # Odometry publish (tracker별)
            tdata["publisher_raw"].publish(self.create_vive_msg(raw_pose, raw_twist))
            tdata["publisher_calibrated"].publish(self.create_vive_msg(cal_pose, cal_twist))

            # /calibrated_pose publish: [x y z wx wy wz]
            px = float(M_cal[0, 3])
            py = float(M_cal[1, 3])
            pz = float(M_cal[2, 3])

            Rm = M_cal[:3, :3]
            wvec = rotmat_to_rotvec(Rm)  # (rad) rotation vector

            arr = Float64MultiArray()
            arr.data = [px, py, pz, float(wvec[0]), float(wvec[1]), float(wvec[2])]
            self.calibrated_pose_pub.publish(arr)

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

            tdata["prev_raw_matrix"] = raw_M.copy()
            tdata["prev_calibrated_matrix"] = M_cal.copy()

        # 호환용 퍼블리시 (첫 트래커)
        first_id = current_ids[0]
        first = self.trackers[first_id]

        raw_pose = matrix_to_pose(first["raw_pose_matrix"])
        raw_twist = matrix_to_twist(first["raw_pose_matrix"], first["prev_raw_matrix"], dt)

        # /raw_pose (PoseStamped)
        self.raw_pose_pub_pose.publish(self.create_pose_stamped(raw_pose, frame_id="world"))

        # 기존 Odometry 호환 publish 유지
        self.raw_pose_pub.publish(self.create_vive_msg(raw_pose, raw_twist))

        # calibrated odom 호환 publish 유지
        cal_pose = matrix_to_pose(first["prev_calibrated_matrix"])
        cal_twist = matrix_to_twist(first["prev_calibrated_matrix"], first["prev_calibrated_matrix"], dt)
        self.calibrated_pose_pub_odom.publish(self.create_vive_msg(cal_pose, cal_twist))

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
