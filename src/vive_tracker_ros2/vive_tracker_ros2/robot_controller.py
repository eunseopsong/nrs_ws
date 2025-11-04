#!/usr/bin/env python3
"""
Robot Controller Node for TCP Command Interface
Subscribes to calibrated VR tracker pose and controls a robot using TCP commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

# 로봇 제어 인터페이스를 위한 메시지 유형은 사용 중인 로봇에 따라 변경하세요
# 다음은 두산로봇을 위한 예시입니다
# from doosan_msgs.srv import RobotMove  # 두산로봇 메시지 예시
# from std_srvs.srv import Trigger  # 일반적인 서비스 메시지

class RobotTCPController(Node):
    def __init__(self):
        super().__init__("robot_tcp_controller")
        
        # 파라미터 선언
        self.declare_parameter('max_linear_velocity', 0.1)  # m/s
        self.declare_parameter('max_angular_velocity', 0.5)  # rad/s
        self.declare_parameter('enable_robot_control', False)  # 안전 파라미터
        self.declare_parameter('robot_base_frame', 'base')  # 로봇의 베이스 프레임
        self.declare_parameter('robot_tcp_frame', 'tool0')  # 로봇의 TCP 프레임
        
        # 파라미터 가져오기
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.robot_control_enabled = self.get_parameter('enable_robot_control').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_tcp_frame = self.get_parameter('robot_tcp_frame').value
        
        # VR 트래커의 캘리브레이션된 포즈 구독
        self.vr_pose_sub = self.create_subscription(
            Odometry,
            "vive_tracker_ros/calibrated_pose",
            self.vr_pose_callback,
            10
        )
        
        # 로봇 제어를 위한 발행자 (로봇 드라이버에 맞게 수정 필요)
        self.robot_cmd_pub = self.create_publisher(
            PoseStamped,  # 로봇 드라이버에 맞는 메시지 타입으로 변경하세요
            "robot_tcp_command",  # 로봇 드라이버에 맞는 토픽으로 변경하세요
            10
        )
        
        # 로봇 상태 모니터링을 위한 구독 (선택사항)
        # self.robot_state_sub = self.create_subscription(
        #     YourRobotStateMsg,  # 로봇 상태 메시지 타입으로 변경하세요
        #     "robot_state_topic",  # 로봇 상태 토픽으로 변경하세요
        #     self.robot_state_callback,
        #     10
        # )
        
        # 이전 상태 저장 변수
        self.prev_pose = None
        self.prev_time = self.get_clock().now()
        
        # 초기화 메시지
        self.get_logger().info("로봇 TCP 컨트롤러 초기화 완료")
        if not self.robot_control_enabled:
            self.get_logger().warn("로봇 제어가 비활성화되어 있습니다. 활성화하려면 'enable_robot_control' 파라미터를 true로 설정하세요.")
    
    def vr_pose_callback(self, msg: Odometry):
        """VR 트래커 포즈 데이터를 처리하는 콜백 함수"""
        if not self.robot_control_enabled:
            return
            
        # 현재 시간 및 경과 시간 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        # Odometry 메시지에서 포즈 추출
        current_pose = msg.pose.pose
        
        # 안전 검사
        if not self.is_pose_safe(current_pose):
            self.get_logger().warn("안전하지 않은 포즈가 감지되었습니다. 로봇 이동을 중지합니다.")
            return
        
        # 속도 제한 검사
        if self.prev_pose is not None and dt > 0:
            if not self.is_velocity_safe(current_pose, self.prev_pose, dt):
                self.get_logger().warn("속도가 너무 높습니다. 명령을 건너뜁니다.")
                return
        
        # 로봇 명령 생성
        robot_cmd = self.create_robot_command(current_pose)
        
        # 로봇에 명령 전송
        self.robot_cmd_pub.publish(robot_cmd)
        
        # 이전 값 업데이트
        self.prev_pose = current_pose
        self.prev_time = current_time
    
    def is_pose_safe(self, pose: Pose) -> bool:
        """포즈가 안전한 작업 영역 내에 있는지 검사"""
        # 로봇의 안전 작업 영역 한계 설정 (미터 단위)
        # 이 값은 로봇의 모델과 설치 환경에 맞게 조정해야 합니다
        x_limits = [-1.0, 1.0]  # x축 한계
        y_limits = [-1.0, 1.0]  # y축 한계
        z_limits = [0.1, 2.0]   # z축 한계 (바닥 위)
        
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        # 좌표가 한계 내에 있는지 확인
        if not (x_limits[0] <= x <= x_limits[1]):
            return False
        if not (y_limits[0] <= y <= y_limits[1]):
            return False
        if not (z_limits[0] <= z <= z_limits[1]):
            return False
            
        return True
    
    def is_velocity_safe(self, current_pose: Pose, prev_pose: Pose, dt: float) -> bool:
        """속도가 안전 한계 내에 있는지 검사"""
        if dt <= 0:
            return True
            
        # 선형 속도 계산
        dx = current_pose.position.x - prev_pose.position.x
        dy = current_pose.position.y - prev_pose.position.y
        dz = current_pose.position.z - prev_pose.position.z
        
        linear_vel = np.sqrt(dx**2 + dy**2 + dz**2) / dt
        
        # 선형 속도 확인
        if linear_vel > self.max_linear_vel:
            self.get_logger().warn(f"선형 속도 {linear_vel:.3f}m/s가 제한 {self.max_linear_vel}m/s을 초과합니다.")
            return False
            
        # 각속도 검사도 추가할 수 있음
        # 현재와 이전 쿼터니언에서 회전 변화 계산
        
        return True
    
    def create_robot_command(self, pose: Pose) -> PoseStamped:
        """VR 트래커 포즈에서 로봇 명령 메시지 생성"""
        cmd = PoseStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.robot_base_frame
        
        # 포즈 데이터 복사
        cmd.pose = pose
        
        return cmd
    
    # 로봇 상태 콜백 (선택사항)
    # def robot_state_callback(self, msg):
    #     """로봇 상태 모니터링을 위한 콜백 함수"""
    #     # 로봇 상태 처리 (예: 위치, 속도, 오류 등)
    #     pass

def main(args=None):
    rclpy.init(args=args)
    
    controller = RobotTCPController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
