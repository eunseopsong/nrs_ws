#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append('/home/eunseop/nrs_ws/src/nrs_ik_py_bind')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

import nrs_ik_core  # pybind IK 모듈

POSE_TOPIC = '/calibrated_pose'   # 여기서 [x, y, z, r, p, yaw] 가 옴


def wrap_pi(a: float) -> float:
    """(-pi, pi] 로 접기"""
    return (a + math.pi) % (2 * math.pi) - math.pi


class ServoUR10e(Node):
    def __init__(self):
        super().__init__('servo_ur10e')

        qos_profile = QoSProfile(depth=10)

        # Isaac Sim 쪽으로 보낼 조인트 커맨드 퍼블리셔
        self.joint_pub = self.create_publisher(
            JointState,
            '/isaac_joint_commands',
            qos_profile
        )

        # UR10e 조인트 이름
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

        # 주기적으로 마지막 joint_state 계속 뿌리기
        self.timer = self.create_timer(0.001, self.publish_joint_state)

        # IK solver
        self.ik_solver = nrs_ik_core.IKSolver(tool_z=0.0, use_degrees=False)

        # 실제 pose 받아오는 구독자
        self.pose_sub = self.create_subscription(
            Float64MultiArray,
            POSE_TOPIC,
            self.pose_callback,
            10
        )

    def publish_joint_state(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)

    def pose_callback(self, msg: Float64MultiArray):
        # 기대하는 형식: [x, y, z, r, p, yaw]
        if len(msg.data) < 6:
            self.get_logger().warn("calibrated_pose length < 6, ignore")
            return

        x = float(msg.data[0])
        y = float(msg.data[1])
        z = float(msg.data[2])
        r = wrap_pi(float(msg.data[3]))
        p_ = wrap_pi(float(msg.data[4]))
        yaw = wrap_pi(float(msg.data[5]))

        # 필요하면 여기서 z 오프셋
        z = z + 0.285

        self.get_logger().info(
            f"pose → x:{x:.6f}, y:{y:.6f}, z:{z:.6f}, r:{r:.6f}, p:{p_:.6f}, yaw:{yaw:.6f}"
        )

        # IK에 넣을 PoseRPY 구성
        pose = nrs_ik_core.PoseRPY()
        pose.line_no = 0
        pose.x = x
        pose.y = y
        pose.z = z
        pose.r = r
        pose.p = p_
        pose.yaw = yaw

        # IK 수행
        ok, q = self.ik_solver.compute(pose)
        if not ok:
            self.get_logger().warn("servo_ur10e: IK failed for current pose")
            return

        q_list = list(q)
        if len(q_list) != self.num_joints:
            self.get_logger().error("servo_ur10e: IK returned wrong joint size")
            return

        # 결과를 저장해두면 timer가 계속 publish
        self.joint_state.position = q_list

    def destroy_node(self):
        # 필요하면 여기서 정리
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoUR10e()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
