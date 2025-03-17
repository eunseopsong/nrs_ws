#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R  # scipy 사용

# backup 2025.03.11

class EndEffectorController(Node):

    def __init__(self):
        super().__init__('pub_effort')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', qos_profile)

        # Create a JointState message
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

        # 초기값
        self.joint_state.position = [0.0] * self.num_joints
        self.joint_state.velocity = [0.0] * self.num_joints
        self.joint_state.effort   = [0.0] * self.num_joints

        # 100Hz 타이머 (주기적으로 메시지 발행)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.publish_command_msg)

    def publish_command_msg(self):
        """
        주기적으로 JointState 퍼블리시.
        -> 사용자가 마지막으로 설정한 position이 계속 송신됨.
        """
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """ Euler Angles -> Rotation Matrix 변환 (필요하면 사용) """
        return R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()

    def move_to_pose(self, target_joints):
        """
        입력된 관절 목표 값(라디안)을 저장.
        timer_callback()에서 계속 publish될 것임.
        """
        if len(target_joints) != self.num_joints:
            self.get_logger().error("입력된 조인트 개수가 올바르지 않습니다!")
            return

        self.joint_state.position = target_joints
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

        self.get_logger().info(f"Moving to joints (rad): {target_joints}")


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorController()

    # rclpy.spin을 별도 쓰레드로 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            user_input = input("Enter 6 joint angles (deg) separated by spaces (Ctrl-C to quit): ")
            try:
                joint_angles_deg = list(map(float, user_input.split()))
                if len(joint_angles_deg) != 6:
                    print("6개의 실수를 입력하세요.")
                    continue

                # deg -> rad & 소수점 4자리까지 반올림
                joint_angles_rad = np.round(np.radians(joint_angles_deg), 4)

                # 설정
                node.move_to_pose(joint_angles_rad.tolist())

            except ValueError:
                print("유효하지 않은 입력입니다. (예: 30 45 0 -90 0 45)")
            except KeyboardInterrupt:
                break

    except KeyboardInterrupt:
        # print("\nExiting...")
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    # 정리
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # spin_thread.join() # spin_thread가 daemon이므로 join()은 필요 없음 (Is is right??)


if __name__ == '__main__':
    main()
