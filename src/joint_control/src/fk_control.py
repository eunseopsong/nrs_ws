#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
from scipy.spatial.transform import Rotation as R  # scipy 사용

class EndEffectorController(Node):
    def __init__(self):
        super().__init__('fk_controller')
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # Create a JointState message
        self.joint_state = JointState()
        self.joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.num_joints = len(self.joint_state.name)
        self.joint_state.position = [0.0] * self.num_joints
        self.joint_state.velocity = [0.0] * self.num_joints
        self.joint_state.effort = [0.0] * self.num_joints

        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # 주기적으로 JointState 퍼블리시
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        # """ Euler Angles -> Rotation Matrix 변환 """
        return R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()

    def move_to_pose(self, target_joints):
        # """ 입력된 조인트 목표 값으로 이동 """
        if len(target_joints) != self.num_joints:
            self.get_logger().error("입력된 조인트 개수가 올바르지 않습니다!")
            return

        self.joint_state.position = target_joints
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f"Moving to joints: {target_joints}")


def main(args=None):
    rclpy.init(args=args)
    controller = EndEffectorController()

    while rclpy.ok():
        try:
            # 사용자 입력 받기 (6개 조인트 각도를 직접 입력)
            user_input = input("Enter 6 joint angles (deg) separated by spaces: ")
            joint_angles = np.radians(list(map(float, user_input.split())))  # deg -> rad 변환
            controller.move_to_pose(joint_angles.tolist())
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Invalid input: {e}")

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()