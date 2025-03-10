#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class FKEffortController(Node):
    def __init__(self):
        super().__init__('fk_effort_controller')
        
        # 퍼블리셔: PD 제어로 계산된 토크 명령을 발행
        self.effort_pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # 구독: 현재 로봇의 관절상태 (/joint_states)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 우리 로봇(UR10) 기준 6개 관절 이름 (주문형으로 원하는 이름으로 수정 가능)
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        self.num_joints = len(self.joint_names)

        # 현재 관절각(q), 속도(dq)
        self.q_current = np.zeros(self.num_joints)
        self.dq_current = np.zeros(self.num_joints)

        # 목표 관절각(q_des), 기본 0으로 초기화
        self.q_des = np.zeros(self.num_joints)

        # PD 게인 (예시값)
        # 환경/시뮬에 맞춰 적절히 조정 필요!
        self.Kp = np.array([100.0]*self.num_joints)
        self.Kd = np.array([10.0]*self.num_joints)

        # 100Hz 주기로 제어 실행
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.control_loop_timer)

        self.get_logger().info("FK Effort Controller initialized.")

    def joint_state_callback(self, msg: JointState):
        """
        /joint_states 콜백: 실제 로봇(또는 시뮬)에서 퍼블리시되는 현재 관절 값 받아 저장
        """
        # msg.name 안에 조인트 이름이 ["shoulder_pan_joint", "shoulder_lift_joint", ...] 순서로 들어올 수도 있고,
        # 순서가 다를 수도 있으니, 인덱스 맞춰서 가져오기
        # 여기서는 가정: msg.name 순서가 self.joint_names와 동일하다고 가정(필요시 매핑 로직 구현)
        if len(msg.position) < self.num_joints or len(msg.velocity) < self.num_joints:
            return  # 메시지가 부족한 경우
        # 간단히 인덱스대로 저장
        self.q_current = np.array(msg.position[:self.num_joints])
        self.dq_current = np.array(msg.velocity[:self.num_joints])

    def control_loop_timer(self):
        """
        주기적(100Hz)으로 PD 제어하여 effort(토크) 계산 후 퍼블리시
        """
        # PD 제어 계산
        e = self.q_des - self.q_current           # 위치 오차
        de = - self.dq_current                    # 속도 오차(목표 속도=0 가정)
        tau = self.Kp * e + self.Kd * de          # 토크 계산 (element-wise)

        # JointState 메시지 구성
        effort_msg = JointState()
        effort_msg.header.stamp = self.get_clock().now().to_msg()
        effort_msg.name = self.joint_names
        # position/velocity는 사용하지 않을 것이므로 빈 배열 or 0
        effort_msg.position = [0.0]*self.num_joints
        effort_msg.velocity = [0.0]*self.num_joints
        effort_msg.effort = tau.tolist()

        # 퍼블리시
        self.effort_pub.publish(effort_msg)

    def set_target_joints(self, target_joints_deg):
        """
        사용자가 입력한 6개 관절 (deg 단위) 값을 라디안으로 변환 후 q_des에 저장
        """
        if len(target_joints_deg) != self.num_joints:
            self.get_logger().error("입력된 조인트 개수가 6개가 아님!")
            return

        # deg -> rad
        target_joints_rad = np.radians(target_joints_deg)
        self.q_des = target_joints_rad
        self.get_logger().info(f"Set target joints (deg): {target_joints_deg}")


def main(args=None):
    rclpy.init(args=args)
    controller = FKEffortController()

    try:
        while rclpy.ok():
            # 사용자 입력 받기 (6개 조인트 각도 deg)
            user_input = input("Enter 6 joint angles (deg) separated by spaces (Ctrl-C to quit): ")
            try:
                joint_angles_deg = list(map(float, user_input.split()))
                if len(joint_angles_deg) != 6:
                    print("6개의 실수를 입력하세요.")
                    continue

                # 목표 조인트 각도 설정
                controller.set_target_joints(joint_angles_deg)

            except Exception as e:
                print(f"Invalid input: {e}")
                continue

            rclpy.spin_once(controller, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\nExiting...")

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
