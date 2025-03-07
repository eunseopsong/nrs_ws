#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.spatial.transform import Rotation as R
import os
import threading

class IKController(Node):
    def __init__(self):
        super().__init__('ik_controller')
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', 100)

        # JointState 메시지 정의
        self.joint_state = JointState()
        self.joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.num_joints = len(self.joint_state.name)
        self.joint_state.position = [0.0] * self.num_joints

        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # UR10 모델 로드 (URDF 경로 설정 필요!)
        urdf_path = "/home/eunseop/nrs_ws/src/urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur10.urdf"
        urdf_path = os.path.expanduser(urdf_path)
        
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF 파일을 찾을 수 없습니다: {urdf_path}")
            return

        self.robot = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir=None, root_joint=None)
        self.model = self.robot.model
        self.data = self.robot.data

    def timer_callback(self):
        """ 주기적으로 JointState 퍼블리시 """
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """ Euler Angles -> Quaternion 변환 """
        return R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()

    def inverse_kinematics(self, x, y, z, roll, pitch, yaw):
        """ Pinocchio를 이용한 역기구학 계산 (Levenberg-Marquardt 방식 적용) """
        ee_frame_id = self.model.getFrameId("wrist_3_link")  # UR10 EE 프레임 ID
        q_guess = np.zeros(self.model.nq)  # 초기 Guess 값 (6Dof)
        target_pos = np.array([x, y, z])
        target_quat = self.euler_to_quaternion(roll, pitch, yaw)

        max_iters = 100  # 최대 반복 횟수
        tol = 1e-4  # 오차 허용치
        alpha = 0.1  # 업데이트 비율

        for i in range(max_iters):
            pin.forwardKinematics(self.model, self.data, q_guess)
            pin.updateFramePlacement(self.model, self.data, ee_frame_id)
            
            ee_pos = self.data.oMf[ee_frame_id].translation
            ee_quat = pin.Quaternion(self.data.oMf[ee_frame_id].rotation)
            
            pos_err = target_pos - ee_pos
            quat_err = target_quat - ee_quat.coeffs()

            # 종료 조건 확인
            if np.linalg.norm(pos_err) < tol and np.linalg.norm(quat_err) < tol:
                return q_guess.tolist()

            J = pin.computeFrameJacobian(self.model, self.data, q_guess, ee_frame_id)
            dq = np.linalg.pinv(J) @ np.hstack([pos_err, quat_err[:3]])  # Pseudo-Inverse 사용
            q_guess += alpha * dq  # 업데이트

        self.get_logger().error("IK 솔루션을 찾을 수 없습니다!")
        return None

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        """ IK 계산 후 해당 조인트 값으로 이동 """
        joints = self.inverse_kinematics(x, y, z, roll, pitch, yaw)
        
        if joints is None:
            self.get_logger().error("유효한 IK 솔루션이 없습니다!")
            return

        self.joint_state.position = joints
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f"Moved to EE position: {[x, y, z]}, orientation: {[roll, pitch, yaw]}")

def user_input_thread(controller):
    while rclpy.ok():
        try:
            user_input = input("Enter x y z roll pitch yaw (meters & degrees): ")
            x, y, z, roll, pitch, yaw = map(float, user_input.split())
            roll, pitch, yaw = np.radians([roll, pitch, yaw])  # Deg -> Rad 변환
            controller.move_to_pose(x, y, z, roll, pitch, yaw)
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Invalid input: {e}")

def main(args=None):
    rclpy.init(args=args)
    controller = IKController()
    
    if controller.model is None:
        print("로봇 모델이 올바르게 로드되지 않았습니다. 종료합니다.")
        return

    thread = threading.Thread(target=user_input_thread, args=(controller,))
    thread.start()

    rclpy.spin(controller)

    thread.join()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
