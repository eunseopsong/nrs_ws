#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# [수정 1] WrenchStamped 메시지 타입 임포트 추가
from geometry_msgs.msg import WrenchStamped

class FeasibilityTestNode(Node):
    def __init__(self):
        super().__init__('feasibility_test_node')
        
        # Publisher: /ur10skku/cmdMotion (9차원 데이터 송신)
        self.cmd_motion_pub = self.create_publisher(Float64MultiArray, '/ur10skku/cmdMotion', 10)
        
        # Subscribers
        self.current_p_sub = self.create_subscription(
            Float64MultiArray, '/ur10skku/currentP', self.current_p_callback, 10)
            
        # [수정 2] ftdata의 구독 타입을 WrenchStamped로 변경
        self.ft_data_sub = self.create_subscription(
            WrenchStamped, '/ur10skku/ftdata', self.ft_data_callback, 10)
            
        self.current_pos = None  # [x, y, z, wx, wy, wz]
        self.current_ft = None   # [fx, fy, fz, mx, my, mz]
        
        # [데이터 고정] 하위 6개 값 세팅
        self.fixed_wx = 0.0
        self.fixed_wy = 0.0
        self.fixed_wz = 1.57
        self.fixed_fx = 0.0
        self.fixed_fy = 0.0
        self.fixed_fz = 0.0

        # 제어 주기 및 하강 설정
        self.timer_period = 0.01  # 100Hz (0.01초)
        self.z_step_mm = 0.5      # Z축 하강 속도 (1틱당 0.5mm 하강)
        
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.is_contacted = False
        
        self.get_logger().info("피지빌리티 테스트 노드 실행: 수직 강하 대기 중...")

    def current_p_callback(self, msg):
        self.current_pos = msg.data

    # [수정 3] WrenchStamped 구조에 맞게 데이터 파싱 로직 변경
    def ft_data_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        mx = msg.wrench.torque.x
        my = msg.wrench.torque.y
        mz = msg.wrench.torque.z
        self.current_ft = [fx, fy, fz, mx, my, mz]

    def contact_point_detection_algorithm(self, pos, ft):
        self.get_logger().info("접촉 유지 중: 접촉점 탐지 알고리즘 구동...", throttle_duration_sec=1.0)
        x, y, z = pos[0], pos[1], pos[2]
        return x, y, z 

    def control_loop(self):
        # 센서 데이터가 모두 수신될 때까지 대기
        if self.current_pos is None or self.current_ft is None:
            return

        x, y, z, _, _, _ = self.current_pos
        _, _, fz, _, _, _ = self.current_ft

        cmd_msg = Float64MultiArray()

        # 접촉 임계값 설정
        contact_threshold = 2.0 
        
        if abs(fz) > contact_threshold:
            if not self.is_contacted:
                self.get_logger().info(f"표면 접촉 감지! (Fz: {fz:.2f}N). 탐지 알고리즘으로 전환합니다.")
                self.is_contacted = True
            
            target_x, target_y, target_z = self.contact_point_detection_algorithm(self.current_pos, self.current_ft)
            
            cmd_msg.data = [
                target_x, target_y, target_z, 
                self.fixed_wx, self.fixed_wy, self.fixed_wz, 
                self.fixed_fx, self.fixed_fy, self.fixed_fz
            ]
        else:
            self.is_contacted = False
            target_z = z - self.z_step_mm

            cmd_msg.data = [
                x, y, target_z, 
                self.fixed_wx, self.fixed_wy, self.fixed_wz, 
                self.fixed_fx, self.fixed_fy, self.fixed_fz
            ]

        self.cmd_motion_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FeasibilityTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드가 종료되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()