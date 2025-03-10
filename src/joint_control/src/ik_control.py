#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
from numpy.linalg import norm, solve

import pinocchio as pin
from scipy.spatial.transform import Rotation as R


class IKController(Node):
    def __init__(self, urdf_path: str, end_effector_name: str = "tool0"):
        """
        Pinocchio를 이용한 UR10 역기구학 + JointState 퍼블리시 노드
        (URDF는 수정 없이, 코드에서 사용자 입력 좌표계만 뒤집어 주어 사용)
        """
        super().__init__('ik_controller')

        # 1) Pinocchio 모델 로드
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # 말단 프레임 ID
        self.ee_id = self.model.getFrameId(end_effector_name)
        if self.ee_id < 0:
            raise ValueError(f"프레임 '{end_effector_name}'을(를) URDF에서 찾을 수 없습니다.")

        # 2) ROS2 퍼블리셔 및 JointState 초기화
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', 10)

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
        self.joint_state.position = [0.0] * self.num_joints
        self.joint_state.velocity = [0.0] * self.num_joints
        self.joint_state.effort = [0.0] * self.num_joints

        # 3) 주기적 퍼블리시를 위한 타이머
        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 4) IK 파라미터 설정
        self.max_iter = 1000
        self.eps = 1e-4
        self.dt = 0.1
        self.damp = 1e-12

        self.get_logger().info("IKController 노드가 초기화되었습니다.")

    def timer_callback(self):
        """
        일정 주기로 JointState를 퍼블리시.
        """
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

    def move_to_pose(self, joint_angles_rad):
        """
        계산된 관절각(라디안)을 현재 joint_state에 반영하고 즉시 퍼블리시.
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
        URDF 수정 없이, '사용자가 보기에 직관적인 좌표계' → '실제 URDF base_link' 좌표계 변환.

        URDF 상 base_link는 Z축으로 pi(180도)만큼 돌아가 있으므로,
        Rz(pi)로 위치와 자세를 뒤집어 줌.

        - (x_in, y_in, z_in)는 사용자가 보는 좌표 (예: 시뮬레이터에서 +X 전방, +Y 좌측, +Z 위)
        - 반환값 (x_out, y_out, z_out, roll_out, pitch_out, yaw_out)는 URDF base_link 기준
        """
        # 1) roll_in, pitch_in, yaw_in을 라디안으로 (degrees=True 시)
        if degrees:
            roll_in = np.deg2rad(roll_in)
            pitch_in = np.deg2rad(pitch_in)
            yaw_in = np.deg2rad(yaw_in)

        # 2) 사용자 입력의 회전행렬
        R_user = R.from_euler('xyz', [roll_in, pitch_in, yaw_in], degrees=False).as_matrix()

        # 3) Rz(pi) 정의 (x->-x, y->-y, z->z)
        Rz_pi = np.array([
            [-1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0],
            [ 0.0,  0.0,  1.0]
        ])

        # 4) 위치 변환
        xyz_in = np.array([x_in, y_in, z_in], dtype=float)
        xyz_out = Rz_pi @ xyz_in  # 행렬 곱
        x_out, y_out, z_out = xyz_out

        # 5) 회전행렬 변환
        R_out = Rz_pi @ R_user
        # 다시 roll, pitch, yaw (XYZ 오일러)로 되돌림
        roll_out, pitch_out, yaw_out = R.from_matrix(R_out).as_euler('xyz')

        # 6) 반환
        return x_out, y_out, z_out, roll_out, pitch_out, yaw_out

    def compute_ik(self, x, y, z, roll, pitch, yaw, degrees=True):
        """
        (x, y, z, roll, pitch, yaw)를 입력받아 Pinocchio로 Iterative IK 수행.
        여기서 '사용자 입장에서 직관적'인 좌표계를 → 'URDF base_link' 좌표계로 변환 후 IK.
        """
        # (A) 입력 좌표계를 URDF base_link 기준으로 뒤집어 줌
        x_base, y_base, z_base, roll_base, pitch_base, yaw_base = \
            self.transform_user_input_to_base(x, y, z, roll, pitch, yaw, degrees=degrees)

        # (B) 회전행렬 + 평행이동 → SE(3)
        Rmat = R.from_euler('xyz', [roll_base, pitch_base, yaw_base], degrees=False).as_matrix()
        tvec = np.array([x_base, y_base, z_base], dtype=float)
        oMdes = pin.SE3(Rmat, tvec)

        # (C) 초기 관절값 (neutral)
        q = pin.neutral(self.model)

        # (D) 반복 알고리즘 (Jacobian 기반 IK)
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            oM_ee = self.data.oMf[self.ee_id]  # 현재 말단 포즈
            eeM_des = oM_ee.actInv(oMdes)
            err = pin.log(eeM_des).vector

            if norm(err) < self.eps:
                return q, True

            # 자코비안 (말단 로컬 프레임)
            J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_id, pin.ReferenceFrame.LOCAL)
            # log(iMd) 보정
            Jlog = pin.Jlog6(eeM_des.inverse())
            J = -Jlog.dot(J)

            # 댐핑 least squares
            lhs = J.dot(J.T) + self.damp * np.eye(6)
            v = -J.T.dot(solve(lhs, err))

            # integrate 로 q 갱신
            q = pin.integrate(self.model, q, v * self.dt)

        return q, False


def main(args=None):
    rclpy.init(args=args)

    # (1) URDF 경로를 사용자 환경에 맞게 설정
    # 예: "/home/eunseop/nrs_ws/src/urdf_files_dataset/urdf_files/ros-industrial/..."
    urdf_path = "/home/eunseop/nrs_ws/src/urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur10.urdf"
    end_effector_frame = "tool0"

    controller = IKController(urdf_path, end_effector_frame)

    try:
        while rclpy.ok():
            # (2) 터미널에서 (x y z roll pitch yaw) 입력 (deg)
            user_input = input("\nEnter [x y z roll pitch yaw] in deg (Ctrl-C to exit): ")

            vals = list(map(float, user_input.split()))
            if len(vals) != 6:
                print("잘못된 입력: 6개의 실수가 필요합니다! (x y z roll pitch yaw)")
                continue

            x, y, z, roll, pitch, yaw = vals

            # (3) IK 수행
            q_sol, success = controller.compute_ik(x, y, z, roll, pitch, yaw, degrees=True)

            if success:
                print("IK 성공! 조인트(rad] =", q_sol)
                # (4) 로봇 움직이기
                controller.move_to_pose(q_sol.tolist())
            else:
                print("IK 실패: 최대 반복 횟수 내 수렴하지 못함.")

            # spin_some() 등으로 콜백 처리
            rclpy.spin_once(controller, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("종료합니다...")

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
