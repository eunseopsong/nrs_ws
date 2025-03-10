#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
from numpy.linalg import norm, solve
import threading

class FKEffortController(Node):
    def __init__(self):
        super().__init__('fk_effort_controller')
        
        # 퍼블리셔: PD 제어로 계산된 토크 명령 발행
        self.effort_pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # 구독: 현재 로봇의 관절상태 (/joint_states)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # UR10 기준 6개 관절 이름 (필요에 따라 수정)
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        self.num_joints = len(self.joint_names)

        # 현재 관절각(q)와 속도(dq)
        self.q_current = np.zeros(self.num_joints)
        self.dq_current = np.zeros(self.num_joints)

        # 목표 관절각(q_des), 초기값 0
        self.q_des = np.zeros(self.num_joints)

        # PD 게인 (예시값, 환경에 맞게 조정 필요)
        self.Kp = np.array([100.0]*self.num_joints)
        self.Kd = np.array([10.0]*self.num_joints)

        # 100Hz 주기로 제어 실행 (타이머 콜백)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.control_loop_timer)

        self.get_logger().info("FK Effort Controller initialized.")

    def joint_state_callback(self, msg: JointState):
        """
        /joint_states 콜백: 현재 관절 각도와 속도를 받아 저장
        """
        # 여기서는 msg.name의 순서가 self.joint_names와 동일하다고 가정
        if len(msg.position) < self.num_joints or len(msg.velocity) < self.num_joints:
            return
        self.q_current = np.array(msg.position[:self.num_joints])
        self.dq_current = np.array(msg.velocity[:self.num_joints])

    def control_loop_timer(self):
        """
        100Hz로 PD 제어를 수행하여 토크(effort)를 계산 후 퍼블리시
        """
        e = self.q_des - self.q_current           # 위치 오차
        de = - self.dq_current                    # 속도 오차 (목표 속도 0 가정)
        tau = self.Kp * e + self.Kd * de          # 토크 계산 (element-wise)

        # JointState 메시지 구성
        effort_msg = JointState()
        effort_msg.header.stamp = self.get_clock().now().to_msg()
        effort_msg.name = self.joint_names
        effort_msg.position = [0.0] * self.num_joints  # 사용하지 않음
        effort_msg.velocity = [0.0] * self.num_joints  # 사용하지 않음
        effort_msg.effort = tau.tolist()

        self.effort_pub.publish(effort_msg)

    def set_target_joints(self, target_joints_deg):
        """
        사용자 입력한 6개 관절 각도 (deg)를 라디안으로 변환 후 q_des 업데이트
        """
        if len(target_joints_deg) != self.num_joints:
            self.get_logger().error("입력된 관절 개수가 6개가 아닙니다!")
            return
        self.q_des = np.radians(target_joints_deg)
        self.get_logger().info(f"Set target joints (deg): {target_joints_deg}")


def main(args=None):
    rclpy.init(args=args)
    controller = FKEffortController()

    # 별도 스레드에서 spin() 실행하여 타이머 콜백 지속 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        while True:
            user_input = input("Enter 6 joint angles (deg) separated by spaces (Ctrl-C to quit): ")
            try:
                joint_angles_deg = list(map(float, user_input.split()))
                if len(joint_angles_deg) != 6:
                    print("6개의 실수를 입력하세요.")
                    continue

                controller.set_target_joints(joint_angles_deg)
            except ValueError:
                print("유효한 실수 6개를 입력하세요. 예: 30 45 0 -90 0 45")
            except Exception as e:
                print(f"Invalid input: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")

    controller.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
