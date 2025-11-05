#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
# pybind .so 가 있는 위치 추가
sys.path.append('/home/eunseop/nrs_ws/src/nrs_ik_py_bind')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
import math

import nrs_ik_core  # pybind로 만든 IK 모듈

VIVE_TOPIC = '/vive_tracker_ros/LHR_B3BA320E/calibrated_pose'


def quat_to_rot(w, x, y, z):
    """ROS1 VIVEnode::Qua2Rot과 동일"""
    R = np.zeros((3, 3), dtype=float)
    R[0, 0] = 1 - 2*y*y - 2*z*z
    R[0, 1] = 2*x*y - 2*z*w
    R[0, 2] = 2*x*z + 2*y*w

    R[1, 0] = 2*x*y + 2*z*w
    R[1, 1] = 1 - 2*x*x - 2*z*z
    R[1, 2] = 2*y*z - 2*x*w

    R[2, 0] = 2*x*z - 2*y*w
    R[2, 1] = 2*y*z + 2*x*w
    R[2, 2] = 1 - 2*x*x - 2*y*y
    return R


def wrap_pi(a: float) -> float:
    """(-pi, pi] 로 접기"""
    return (a + math.pi) % (2 * math.pi) - math.pi


class ServoUR10e(Node):
    def __init__(self):
        super().__init__('servo_ur10e')

        qos_profile = QoSProfile(depth=10)
        # Isaac Sim 쪽이 듣는 토픽 이름
        self.joint_pub = self.create_publisher(
            JointState,
            '/isaac_joint_commands',
            qos_profile
        )

        # UR10e 조인트 이름 (네 FK 노드와 동일)
        self.joint_state = JointState()
        self.joint_state.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        self.num_joints = len(self.joint_state.name)
        self.joint_state.position = [0.0] * self.num_joints
        self.joint_state.velocity = [0.0] * self.num_joints
        self.joint_state.effort = [0.0] * self.num_joints

        # 주기적으로 마지막 조인트값 계속 publish
        self.timer = self.create_timer(0.001, self.publish_joint_state)

        # IK solver 준비 (tool_z 필요하면 수정)
        self.ik_solver = nrs_ik_core.IKSolver(tool_z=0.0, use_degrees=False)

        # Vive tracker 구독
        self.vive_sub = self.create_subscription(
            Odometry,
            VIVE_TOPIC,
            self.vive_callback,
            10
        )

        # ROS1식 RPY 연속성용
        self.r2e_init_flag = False
        self.r2e_pre_rpy = np.zeros(3, dtype=float)

    def publish_joint_state(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)

    def vive_callback(self, msg: Odometry):
        # 1) 위치는 그대로 사용
        p = msg.pose.pose.position
        x = float(p.x)
        y = float(p.y)
        z = float(p.z)
        z = z + 0.285  # Vive tracker 높이 오프셋 보정

        # 2) quat -> rot
        o = msg.pose.pose.orientation
        Rm = quat_to_rot(o.w, o.x, o.y, o.z)

        # 3) ROS1 방식으로 rpy 뽑기
        rpy_ros1 = self.rot_to_rpy_ros1(Rm)
        # rpy_ros1 = [roll_ros1, pitch_ros1, yaw_ros1]

        # 4) roll <-> yaw 스왑
        roll_raw = rpy_ros1[2]   # 원래 yaw
        pitch_raw = rpy_ros1[1]
        yaw_raw = rpy_ros1[0]    # 원래 roll

        # 5) 여기서 부호 반전 (네가 관측한 대로 모두 반대로 동작하므로)
        roll_raw *= -1.0
        pitch_raw *= -1.0
        yaw_raw *= -1.0

        # 6) (-pi, pi] 로 래핑
        r = wrap_pi(roll_raw)
        p_ = wrap_pi(pitch_raw)
        yaw = wrap_pi(yaw_raw)

        # 디버깅
        self.get_logger().info(
            f"pose → x:{x:.6f}, y:{y:.6f}, z:{z:.6f}, r:{r:.6f}, p:{p_:.6f}, yaw:{yaw:.6f}"
        )

        # 7) IK에 넣을 PoseRPY 구성
        pose = nrs_ik_core.PoseRPY()
        pose.line_no = 0
        pose.x = x
        pose.y = y
        pose.z = z
        pose.r = r
        pose.p = p_
        pose.yaw = yaw

        # 8) IK 계산
        ok, q = self.ik_solver.compute(pose)

        if not ok:
            self.get_logger().warn("servo_ur10e: IK failed for current pose")
            return

        q_list = list(q)
        if len(q_list) != self.num_joints:
            self.get_logger().error("servo_ur10e: IK returned wrong joint size")
            return

        # 9) 조인트 업데이트 → timer가 계속 publish
        self.joint_state.position = q_list

    def rot_to_rpy_ros1(self, Rm: np.ndarray) -> np.ndarray:
        """
        ROS1 VIVEnode::VR_Rot2RPY 그대로 옮긴 것
        """
        rpy = np.zeros(3, dtype=float)

        if Rm[2, 0] > 0.998:   # north pole
            rpy[0] = math.atan2(Rm[0, 1], Rm[1, 1])  # roll
            rpy[1] = math.pi / 2.0
            rpy[2] = 0.0
        elif Rm[2, 0] < -0.998:  # south pole
            rpy[0] = math.atan2(Rm[0, 1], Rm[1, 1])
            rpy[1] = -math.pi / 2.0
            rpy[2] = 0.0
        else:
            rpy[0] = math.atan2(-Rm[1, 0], Rm[0, 0])   # roll
            rpy[1] = math.asin(Rm[2, 0])               # pitch
            rpy[2] = math.atan2(-Rm[2, 1], Rm[2, 2])   # yaw

        # 연속성 보정
        if rpy[0] < 0:
            rpy[0] += 2 * math.pi

        if not self.r2e_init_flag:
            self.r2e_pre_rpy = rpy.copy()
            self.r2e_init_flag = True
            return rpy

        out = np.zeros(3, dtype=float)
        for i in range(3):
            orig = abs(rpy[i] - self.r2e_pre_rpy[i])
            orig_pl = abs(rpy[i] + 2*math.pi - self.r2e_pre_rpy[i])
            orig_mi = abs(rpy[i] - 2*math.pi - self.r2e_pre_rpy[i])

            if orig <= orig_pl and orig <= orig_mi:
                out[i] = rpy[i]
            elif orig_pl <= orig and orig_pl <= orig_mi:
                out[i] = rpy[i] + 2*math.pi
            else:
                out[i] = rpy[i] - 2*math.pi

        self.r2e_pre_rpy = out.copy()
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ServoUR10e()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
