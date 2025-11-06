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
from geometry_msgs.msg import Pose, Twist, TransformStamped
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


class ViveTracker(Node):
    def __init__(self):
        super().__init__("vive_tracker")

        self.vr_system = None
        self.trackers = {}

        # rpy 연속성용
        self.r2e_init_flag = False
        self.r2e_pre_rpy = np.zeros(3, dtype=float)

        self._init_ros()
        self._init_vr()
        self._init_json_calib()
        self._init_yaml_calib()

        self.prev_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # ROS init
    # ------------------------------------------------------------------
    def _init_ros(self):
        # 원래 있던 것들
        self.raw_pose_pub = self.create_publisher(
            Odometry, "vive_tracker_ros/raw_pose", 10
        )
        self.calibrated_pose_pub_odom = self.create_publisher(
            Odometry, "vive_tracker_ros/calibrated_pose", 10
        )
        # 여기만 변경: rpy 토픽을 /calibrated_pose 로 통합
        self.calibrated_pose_pub = self.create_publisher(
            Float64MultiArray, "/calibrated_pose", 10
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

        self.T_Adj = np.eye(4, dtype=np.float64)
        R_adj_full = rot_z(0.0) @ rot_x(0.0) @ self.R_Adj.T
        self.T_Adj[:3, :3] = R_adj_full

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
    # quaternion -> rot
    # ------------------------------------------------------------------
    @staticmethod
    def quat_to_rot(w, x, y, z):
        Rm = np.zeros((3, 3), dtype=float)
        Rm[0, 0] = 1 - 2*y*y - 2*z*z
        Rm[0, 1] = 2*x*y - 2*z*w
        Rm[0, 2] = 2*x*z + 2*y*w

        Rm[1, 0] = 2*x*y + 2*z*w
        Rm[1, 1] = 1 - 2*x*x - 2*z*z
        Rm[1, 2] = 2*y*z - 2*x*w

        Rm[2, 0] = 2*x*z - 2*y*w
        Rm[2, 1] = 2*y*z + 2*x*w
        Rm[2, 2] = 1 - 2*x*x - 2*y*y
        return Rm

    def rot_to_rpy_ros1(self, Rm: np.ndarray) -> np.ndarray:
        rpy = np.zeros(3, dtype=float)

        if Rm[2, 0] > 0.998:   # north pole
            rpy[0] = math.atan2(Rm[0, 1], Rm[1, 1])
            rpy[1] = math.pi / 2.0
            rpy[2] = 0.0
        elif Rm[2, 0] < -0.998:  # south pole
            rpy[0] = math.atan2(Rm[0, 1], Rm[1, 1])
            rpy[1] = -math.pi / 2.0
            rpy[2] = 0.0
        else:
            rpy[0] = math.atan2(-Rm[1, 0], Rm[0, 0])
            rpy[1] = math.asin(Rm[2, 0])
            rpy[2] = math.atan2(-Rm[2, 1], Rm[2, 2])

        if rpy[0] < 0:
            rpy[0] += 2 * math.pi

        if not self.r2e_init_flag:
            self.r2e_pre_rpy = rpy.copy()
            self.r2e_init_flag = True
            return rpy

        out = np.zeros(3, dtype=float)
        for i in range(3):
            orig = abs(rpy[i] - self.r2e_pre_rpy[i])
            orig_pl = abs(rpy[i] + 2 * math.pi - self.r2e_pre_rpy[i])
            orig_mi = abs(rpy[i] - 2 * math.pi - self.r2e_pre_rpy[i])

            if orig <= orig_pl and orig <= orig_mi:
                out[i] = rpy[i]
            elif orig_pl <= orig and orig_pl <= orig_mi:
                out[i] = rpy[i] + 2 * math.pi
            else:
                out[i] = rpy[i] - 2 * math.pi

        self.r2e_pre_rpy = out.copy()
        return out

    @staticmethod
    def wrap_pi(a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

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

            # ROS1 순서
            M_adj = self.T_Adj @ raw_M
            M_cal = self.T_AD @ M_adj @ self.T_CE

            # pose로
            raw_pose = matrix_to_pose(raw_M)
            cal_pose = matrix_to_pose(M_cal)

            raw_twist = matrix_to_twist(raw_M, tdata["prev_raw_matrix"], dt)
            cal_twist = matrix_to_twist(M_cal, tdata["prev_calibrated_matrix"], dt)

            # Odometry publish (tracker별)
            tdata["publisher_raw"].publish(self.create_vive_msg(raw_pose, raw_twist))
            tdata["publisher_calibrated"].publish(self.create_vive_msg(cal_pose, cal_twist))

            # 여기서 rpy로도 publish → 통합 토픽 /calibrated_pose
            px = float(M_cal[0, 3])
            py = float(M_cal[1, 3])
            pz = float(M_cal[2, 3])

            Rm = M_cal[:3, :3]
            rpy_ros1 = self.rot_to_rpy_ros1(Rm)

            # 네가 쓰던 스왑/부호 그대로
            roll_raw = rpy_ros1[2]   # 원래 yaw
            pitch_raw = rpy_ros1[1]
            yaw_raw = rpy_ros1[0]    # 원래 roll

            roll_raw *= -1.0
            pitch_raw *= -1.0
            yaw_raw *= -1.0

            roll = self.wrap_pi(roll_raw)
            pitch = self.wrap_pi(pitch_raw)
            yaw = self.wrap_pi(yaw_raw)

            arr = Float64MultiArray()
            arr.data = [px, py, pz, roll, pitch, yaw]
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
        cal_pose = matrix_to_pose(first["prev_calibrated_matrix"])
        raw_twist = matrix_to_twist(first["raw_pose_matrix"], first["prev_raw_matrix"], dt)
        cal_twist = matrix_to_twist(first["prev_calibrated_matrix"], first["prev_calibrated_matrix"], dt)
        self.raw_pose_pub.publish(self.create_vive_msg(raw_pose, raw_twist))
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
