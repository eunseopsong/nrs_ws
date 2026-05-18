#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped

class FeasibilityTestNode2(Node):
    def __init__(self):
        super().__init__('feasibility_test_node2') 
        
        self.cmd_motion_pub = self.create_publisher(Float64MultiArray, '/ur10skku/cmdMotion', 10)
        
        self.current_p_sub = self.create_subscription(
            Float64MultiArray, '/ur10skku/currentP', self.current_p_callback, 10)
        self.ft_data_sub = self.create_subscription(
            WrenchStamped, '/ur10skku/ftdata', self.ft_data_callback, 10)
            
        self.current_pos = None  
        self.current_ft = None   
        self.home_pos = None     # [추가] 초기 위치를 저장할 변수
        
        self.fixed_wx = 0.0
        self.fixed_wy = 0.0
        self.fixed_wz = 1.57
        self.fixed_fx = 0.0
        self.fixed_fy = 0.0
        self.fixed_fz = 0.0

        self.timer_period = 0.01  
        self.z_step_mm = 0.5      
        
        # 0: 대기, 1: 목표 이동, 2: 수직 하강, 3: 접촉 알고리즘, 4: 원위치 복귀
        self.state = 0 
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_wx = 0.0
        self.target_wy = 0.0
        self.target_wz = 1.57
        self.start_z = 0.0 
        # [핵심 추가] 닿는 순간의 위치와 자세를 영구 고정할 변수
        self.locked_pos = None
        # 무한 루프로 입력을 받는 스레드
        self.input_thread = threading.Thread(target=self.wait_for_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("피지빌리티 테스트 노드 V2 (원위치 복귀 기능 탑재) 실행 완료.")

    def current_p_callback(self, msg):
        self.current_pos = msg.data

    def ft_data_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        mx = msg.wrench.torque.x
        my = msg.wrench.torque.y
        mz = msg.wrench.torque.z
        self.current_ft = [fx, fy, fz, mx, my, mz]

    def wait_for_user_input(self):
        import time
        while self.current_pos is None:
            time.sleep(0.1)

        # [핵심] 최초 1회만 현재 위치를 Home으로 굳혀서 저장합니다.
        self.home_pos = self.current_pos[:]

        # 노드가 살아있는 동안 계속 메뉴를 띄웁니다.
        while rclpy.ok():
            print("\n" + "="*50)
            print(" [로봇 제어 커맨드 라인]")
            print("  s : 새로운 테스트 시작 (목표 위치 지정 후 강하)")
            print("  h : 원위치(Home)로 강제 복귀")
            print("="*50)
            
            cmd = input("명령어 입력 (s 또는 h): ").strip().lower()

            if cmd == 's':
                try:
                    in_x = input("이동할 목표 X 위치 (mm, 그대로 두려면 엔터): ")
                    in_y = input("이동할 목표 Y 위치 (mm, 그대로 두려면 엔터): ")
                    print("\n자세(Orientation) 설정도 변경하시겠습니까? (rad 단위)")
                    in_wx = input(f"목표 WX (기본값 {self.fixed_wx}): ")
                    in_wy = input(f"목표 WY (기본값 {self.fixed_wy}): ")
                    in_wz = input(f"목표 WZ (기본값 {self.fixed_wz}): ")

                    self.target_x = float(in_x) if in_x.strip() != "" else self.current_pos[0]
                    self.target_y = float(in_y) if in_y.strip() != "" else self.current_pos[1]
                    
                    self.target_wx = float(in_wx) if in_wx.strip() != "" else self.fixed_wx
                    self.target_wy = float(in_wy) if in_wy.strip() != "" else self.fixed_wy
                    self.target_wz = float(in_wz) if in_wz.strip() != "" else self.fixed_wz
                    
                    self.start_z = self.current_pos[2] 
                    
                    self.fixed_wx = self.target_wx
                    self.fixed_wy = self.target_wy
                    self.fixed_wz = self.target_wz
                    self.locked_pos = None

                    print(f"\n[명령 접수] X:{self.target_x}, Y:{self.target_y} 로 이동 후 하강합니다.")
                    self.state = 1 # 테스트 시작

                except ValueError:
                    print("입력 오류: 숫자를 정확히 입력해주세요.")
            
            elif cmd == 'h':
                print("\n[인터럽트] 로봇을 최초 초기 위치(Home)로 강제 복귀시킵니다.")
                self.state = 4 # 원위치 복귀 상태로 즉시 전환
            
            else:
                print("알 수 없는 명령어입니다. 's' 또는 'h'를 입력해주세요.")

    def contact_point_detection_algorithm(self, pos, ft):
        self.get_logger().info("접촉 알고리즘 구동 중... (중지하려면 터미널에 h 입력)", throttle_duration_sec=2.0)
        x, y, z = pos[0], pos[1], pos[2]
        return x, y, z 

    def control_loop(self):
        if self.current_pos is None or self.current_ft is None:
            return

        x, y, z, wx, wy, wz = self.current_pos
        _, _, fz, _, _, _ = self.current_ft

        cmd_msg = Float64MultiArray()
        contact_threshold = 2.0 

        if self.state == 0:
            return

        elif self.state == 1:
            error_x = abs(self.target_x - x)
            error_y = abs(self.target_y - y)
            error_wx = abs(self.target_wx - wx)
            error_wy = abs(self.target_wy - wy)
            error_wz = abs(self.target_wz - wz)
            
            if error_x < 1.0 and error_y < 1.0 and error_wx < 0.05 and error_wy < 0.05 and error_wz < 0.05:
                self.get_logger().info("목표 위치 도달 완료! Z축 하강을 시작합니다.")
                self.state = 2 
            else:
                cmd_msg.data = [
                    self.target_x, self.target_y, self.start_z, 
                    self.target_wx, self.target_wy, self.target_wz, 
                    self.fixed_fx, self.fixed_fy, self.fixed_fz
                ]
                self.cmd_motion_pub.publish(cmd_msg)

        elif self.state == 2:
            if abs(fz) > contact_threshold:
                self.get_logger().info(f"표면 접촉 감지! (Fz: {fz:.2f}N). 로봇 위치를 고정합니다.")
                
                # [핵심 변경] 닿는 순간의 6D 포즈(위치+자세) 전체 배열을 '박제'합니다.
                self.locked_pos = self.current_pos[:]
                self.state = 3 
            else:
                target_z = z - self.z_step_mm
                cmd_msg.data = [
                    self.target_x, self.target_y, target_z, 
                    self.fixed_wx, self.fixed_wy, self.fixed_wz, 
                    self.fixed_fx, self.fixed_fy, self.fixed_fz
                ]
                self.cmd_motion_pub.publish(cmd_msg)

        elif self.state == 3:
            # [핵심 변경] 로봇이 떠오르거나 미끄러지지 못하도록, 
            # 어드미턴스 제어기가 밀어내든 말든 무조건 닿았던 순간의 위치(locked_pos)만을 목표치로 계속 쏩니다.
            cmd_msg.data = [
                self.locked_pos[0], self.locked_pos[1], self.locked_pos[2], 
                self.locked_pos[3], self.locked_pos[4], self.locked_pos[5], 
                self.fixed_fx, self.fixed_fy, self.fixed_fz
            ]
            self.cmd_motion_pub.publish(cmd_msg)
            
            # (옵션) 2초마다 정지 중임을 알림
            self.get_logger().info("로봇 정지 및 접촉 유지 중... (원위치 복귀는 터미널에 h 입력)", throttle_duration_sec=2.0)

        # [추가] 상태 4: 저장해둔 Home 위치로 복귀
        elif self.state == 4:
            home_x, home_y, home_z, home_wx, home_wy, home_wz = self.home_pos
            
            error_x = abs(home_x - x)
            error_y = abs(home_y - y)
            error_z = abs(home_z - z)
            
            # 초기 위치 오차가 1mm 미만이면 도착으로 간주
            if error_x < 1.0 and error_y < 1.0 and error_z < 1.0:
                if getattr(self, '_home_reached', False) == False:
                    self.get_logger().info("원위치 복귀 완료! 터미널에서 새 명령(s)을 입력하세요.")
                    self._home_reached = True
                self.state = 0 # 도착 후 다시 대기 상태로 전환
            else:
                self._home_reached = False
                # Home 좌표로 명령 지속 발행
                cmd_msg.data = [
                    home_x, home_y, home_z, 
                    home_wx, home_wy, home_wz, 
                    0.0, 0.0, 0.0
                ]
                self.cmd_motion_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FeasibilityTestNode2()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드가 종료되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()