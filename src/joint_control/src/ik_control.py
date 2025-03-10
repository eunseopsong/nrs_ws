#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
from numpy.linalg import norm, solve

import pinocchio as pin
from scipy.spatial.transform import Rotation as R
import threading

class IKController(Node):
    def __init__(self, urdf_path: str, end_effector_name: str = "tool0"):
        """
        Pinocchio를 이용한 UR10 역운동학 + JointState 퍼블리시 노드
        (URDF 수정 없이, '사용자 입력 좌표 -> URDF base_link 좌표' 변환)
        """
        super().__init__('ik_controller')

        # (1) Pinocchio 모델 로드
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # 말단(엔드 이펙터) 프레임 ID
        self.ee_id = self.model.getFrameId(end_effector_name)
        if self.ee_id < 0:
            raise ValueError(f"프레임 '{end_effector_name}'을(를) URDF에서 찾을 수 없습니다.")

        # (2) ROS2 퍼블리셔: JointState
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # 퍼블리시할 JointState 기본값
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
        self.joint_state.position = [0.0]*self.num_joints
        self.joint_state.velocity = [0.0]*self.num_joints
        self.joint_state.effort   = [0.0]*self.num_joints

        # (3) 주기적 퍼블리시(100Hz) 타이머
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # (4) IK 파라미터
        self.max_iter = 1000
        self.eps = 1e-4
        self.dt = 0.1
        self.damp = 1e-12

        self.get_logger().info("IKController 노드가 초기화되었습니다.")

    def timer_callback(self):
        """
        주기적으로 JointState 메시지 퍼블리시
        """
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

    def move_to_pose(self, joint_angles_rad):
        """
        IK 결과로 나온 관절각(라디안)을 joint_state에 반영.
        다음 timer_callback부터 계속 이 값이 퍼블리시됨.
        """
        if len(joint_angles_rad) != self.num_joints:
            self.get_logger().error("6개의 관절값이 필요합니다.")
            return

        self.joint_state.position = joint_angles_rad
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f"Moving to joints (rad): {joint_angles_rad}")

    def transform_user_input_to_base(self, x_in, y_in, z_in, roll_in, pitch_in, yaw_in, degrees=True):
        """
        Isaac Sim에서 보이는 좌표(x,y,z, roll,pitch,yaw)를 
        UR10 URDF base_link 좌표계로 변환(Z축 180도 회전 보정).
        """
        if degrees:
            roll_in  = np.deg2rad(roll_in)
            pitch_in = np.deg2rad(pitch_in)
            yaw_in   = np.deg2rad(yaw_in)

        R_user = R.from_euler('xyz', [roll_in, pitch_in, yaw_in], degrees=False).as_matrix()

        # Rz(pi): x->-x, y->-y, z->z
        Rz_pi = np.array([
            [-1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0],
            [ 0.0,  0.0,  1.0]
        ])

        # 위치 변환
        xyz_in = np.array([x_in, y_in, z_in], dtype=float)
        xyz_out = Rz_pi @ xyz_in
        x_out, y_out, z_out = xyz_out

        # 회전행렬 변환
        R_out = Rz_pi @ R_user
        roll_out, pitch_out, yaw_out = R.from_matrix(R_out).as_euler('xyz')

        return x_out, y_out, z_out, roll_out, pitch_out, yaw_out

    def compute_ik(self, x, y, z, roll, pitch, yaw, degrees=True):
        """
        사용자가 준 (x,y,z,roll,pitch,yaw)를 Pinocchio IK로 변환해 관절각(q)를 구함.
        """
        # (A) 좌표 변환
        x_base, y_base, z_base, roll_base, pitch_base, yaw_base = \
            self.transform_user_input_to_base(x, y, z, roll, pitch, yaw, degrees=degrees)

        # (B) 목표 SE(3)
        Rmat = R.from_euler('xyz', [roll_base, pitch_base, yaw_base], degrees=False).as_matrix()
        tvec = np.array([x_base, y_base, z_base], dtype=float)
        oMdes = pin.SE3(Rmat, tvec)

        # (C) 초기 관절값
        q = pin.neutral(self.model)

        # (D) 반복 알고리즘
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            oM_ee = self.data.oMf[self.ee_id]
            eeM_des = oM_ee.actInv(oMdes)
            err = pin.log(eeM_des).vector

            if norm(err) < self.eps:
                return q, True

            # 자코비안
            J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_id, pin.ReferenceFrame.LOCAL)
            Jlog = pin.Jlog6(eeM_des.inverse())
            J = -Jlog.dot(J)

            lhs = J.dot(J.T) + self.damp * np.eye(6)
            v = -J.T.dot(solve(lhs, err))

            q = pin.integrate(self.model, q, v*self.dt)

        return q, False


def main(args=None):
    rclpy.init(args=args)

    urdf_path = "/home/eunseop/nrs_ws/src/urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur10.urdf"
    end_effector_frame = "tool0"

    controller = IKController(urdf_path, end_effector_frame)

    # spin을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        while True:
            user_input = input("\nEnter [x y z roll pitch yaw] in deg (Ctrl-C to exit): ")
            try:
                vals = list(map(float, user_input.split()))
                if len(vals) != 6:
                    print("잘못된 입력: 6개의 실수가 필요합니다! (x y z roll pitch yaw)")
                    continue

                x, y, z, roll, pitch, yaw = vals
                # IK 계산
                q_sol, success = controller.compute_ik(x, y, z, roll, pitch, yaw, degrees=True)

                if success:
                    print(f"IK 성공! 조인트(rad] = {q_sol}")
                    controller.move_to_pose(q_sol.tolist())
                else:
                    print("IK 실패: 최대 반복 횟수 내 수렴하지 못함.")

            except ValueError:
                print("유효한 실수 6개를 입력하세요. 예: 0.5 0.0 0.5 0 90 0")
            except KeyboardInterrupt:
                break

    except KeyboardInterrupt:
        print("종료합니다...")

    # 종료 처리
    controller.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
