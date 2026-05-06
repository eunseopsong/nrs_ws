#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class ContactAdaptationNode(Node):
    def __init__(self):
        super().__init__('contact_adaptation_node')
        
        # 1. 퍼블리셔 (명령 하달)
        # 로봇의 목표 위치 및 자세 (x, y, z, wx, wy, wz)
        self.target_p_pub = self.create_publisher(Float64MultiArray, '/ur10skku/targetP', 10)
        # 로봇의 목표 인가 힘 (fx, fy, fz)
        self.target_f_pub = self.create_publisher(Float64MultiArray, '/ur10skku/targetF', 10)
        
        # 2. 서브스크라이버 (센서 값 수신)
        # 로봇의 현재 위치/자세
        self.current_p_sub = self.create_subscription(
            Float64MultiArray, '/ur10skku/currentP', self.current_p_callback, 10)
        # 6축 힘/모멘트 센서 데이터 (fx, fy, fz, mx, my, mz)
        self.ft_data_sub = self.create_subscription(
            Float64MultiArray, '/ur10skku/ftdata', self.ft_data_callback, 10)
            
        # 내부 상태 저장 변수 초기화
        self.current_pos = None  # [x, y, z, wx, wy, wz]
        self.current_ft = None   # [fx, fy, fz, mx, my, mz]
        
        # 3. 메인 제어 루프 타이머 (100Hz = 0.01초 주기)
        self.timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info("Contact Adaptation Node가 성공적으로 실행되었습니다. 데이터 수신 대기 중...")

    def current_p_callback(self, msg):
        """현재 로봇의 위치/자세를 지속적으로 업데이트"""
        self.current_pos = msg.data

    def ft_data_callback(self, msg):
        """F/T 센서 데이터를 지속적으로 업데이트"""
        self.current_ft = msg.data

    def control_loop(self):
        """0.01초마다 실행되는 메인 두뇌 역할"""
        # 데이터가 모두 들어올 때까지 대기
        if self.current_pos is None or self.current_ft is None:
            return

        # 현재 F/T 데이터 파싱 (단위 및 좌표계는 시뮬레이션 환경에 맞게 조정 필요)
        fx, fy, fz, mx, my, mz = self.current_ft
        
        # 현재 위치 데이터 파싱
        x, y, z, wx, wy, wz = self.current_pos

        # [Step 1] 접촉 판단 (예: Z축 방향으로 누르는 힘이 2N 이상 발생했을 때)
        # ※ 방향(+/-)은 아이작 심 센서 세팅에 따라 다를 수 있으니 확인 후 수정하세요.
        contact_threshold = 2.0 
        is_contact = abs(fz) > contact_threshold

        target_p_msg = Float64MultiArray()
        target_f_msg = Float64MultiArray()

        if is_contact:
            # =================================================================
            # [Step 2 & 3] Intrinsic Contact Sensing 알고리즘 적용 및 보정값 계산
            # =================================================================
            # TODO: 여기에 연구원님이 작성해두신 접촉점 탐지 알고리즘을 연결하세요!
            # 예시: contact_x, contact_y = my_contact_algorithm(fx, fy, fz, mx, my, mz)
            
            # 아래는 가상의 보정 로직입니다. (중앙으로 맞추기 위해 손목을 살짝 틂)
            # 모멘트가 발생했다는 것은 중앙이 아니라는 뜻이므로, 이를 줄이는 방향으로 자세 이동
            adaptation_gain = 0.001 
            target_wx = wx + (my * adaptation_gain) # Y축 모멘트를 상쇄하도록 X축 회전
            target_wy = wy - (mx * adaptation_gain) # X축 모멘트를 상쇄하도록 Y축 회전
            
            # [Step 4] 명령 퍼블리시 (표면 밀착)
            # 위치는 현재를 유지(또는 표면 방향으로 살짝 이동)하고 자세만 비틂
            target_p_msg.data = [x, y, z, target_wx, target_wy, wz]
            
            # Z축으로 10N의 힘을 지속적으로 주어 밀착을 유지하라는 명령
            target_f_msg.data = [0.0, 0.0, -10.0] 
            
            # (선택) 디버깅용 로그 출력
            # self.get_logger().info(f"접촉 감지됨! 오리엔테이션 보정 중... Fz: {fz:.2f}")

        else:
            # 접촉 전 상태: 현재 위치 유지, 인가 힘 없음 (또는 탐색을 위해 아래로 이동하는 로직 추가 가능)
            target_p_msg.data = self.current_pos
            target_f_msg.data = [0.0, 0.0, 0.0]

        # 최종적으로 토픽에 데이터 쏘기
        self.target_p_pub.publish(target_p_msg)
        self.target_f_pub.publish(target_f_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ContactAdaptationNode()
    
    try:
        rclpy.spin(node) # 노드 무한 실행
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 노드가 종료되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()