import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

from geometry_msgs.msg import Pose, Twist


def transform_HmdMatrix2npmatrix(hmd_matrix):
    raw = np.array(
        [
            [hmd_matrix[0][0], hmd_matrix[0][1], hmd_matrix[0][2], hmd_matrix[0][3]],
            [hmd_matrix[1][0], hmd_matrix[1][1], hmd_matrix[1][2], hmd_matrix[1][3]],
            [hmd_matrix[2][0], hmd_matrix[2][1], hmd_matrix[2][2], hmd_matrix[2][3]],
            [0, 0, 0, 1],
        ]
    )

    # y <-> z 스왑 행렬
    swap_yz = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],  # y <- z
        [0, 1, 0, 0],  # z <- y
        [0, 0, 0, 1],
    ])

    # ✨ 여기! 앞에 곱해준다
    np_matrix = swap_yz @ raw
    return np_matrix



def matrix_to_pose(matrix: np.ndarray):
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    quat = R.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def pose_to_matrix(pose: Pose):
    matrix = np.eye(4)
    matrix[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
    matrix[:3, :3] = R.from_quat(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    ).as_matrix()
    return matrix


def matrix_to_twist(prev_matrix, current_matrix, dt):
    if dt == 0:
        return Twist()

    # Linear velocity (position difference)
    linear_vel = [
        (current_matrix[0, 3] - prev_matrix[0, 3]) / dt,
        (current_matrix[1, 3] - prev_matrix[1, 3]) / dt,
        (current_matrix[2, 3] - prev_matrix[2, 3]) / dt,
    ]

    # Angular velocity (rotation matrix difference)
    # R1^T * R2 = exp(w^)
    relative_rotation = np.dot(prev_matrix[:3, :3].T, current_matrix[:3, :3])
    
    # 수치적 오류 방지를 위한 처리
    trace_value = (np.trace(relative_rotation) - 1) / 2
    # -1.0과 1.0 사이의 값으로 제한 (arccos의 정의역)
    trace_value = np.clip(trace_value, -1.0, 1.0)
    angle = np.arccos(trace_value)
    
    if abs(angle) < 1e-6:
        angular_vel = [0.0, 0.0, 0.0]
    else:
        axis = np.array(
            [
                relative_rotation[2, 1] - relative_rotation[1, 2],
                relative_rotation[0, 2] - relative_rotation[2, 0],
                relative_rotation[1, 0] - relative_rotation[0, 1],
            ]
        )
        # sin(angle)이 0에 가까워지는 경우를 방지
        sin_angle = np.sin(angle)
        if abs(sin_angle) < 1e-6:
            angular_vel = [0.0, 0.0, 0.0]
        else:
            axis = axis / (2 * sin_angle)
            angular_vel = (axis * angle / dt).tolist()

    # Create Twist message
    twist = Twist()
    twist.linear.x = linear_vel[0]
    twist.linear.y = linear_vel[1]
    twist.linear.z = linear_vel[2]

    twist.angular.x = angular_vel[0]
    twist.angular.y = angular_vel[1]
    twist.angular.z = angular_vel[2]

    return twist


def calculate_calibration_matrix(robot_matrices, tracker_matrices):
    x0 = np.zeros(12)
    res = least_squares(residuals, x0, args=(robot_matrices, tracker_matrices))
    opt_params = res.x

    # T_tool 행렬 구성
    R_tool_opt = R.from_rotvec(opt_params[:3]).as_matrix()
    t_tool_opt = opt_params[3:6]
    T_tool_opt = np.eye(4)
    T_tool_opt[:3, :3] = R_tool_opt
    T_tool_opt[:3, 3] = t_tool_opt

    # T_trans 행렬 구성
    R_trans_opt = R.from_rotvec(opt_params[6:9]).as_matrix()
    t_trans_opt = opt_params[9:12]
    T_trans_opt = np.eye(4)
    T_trans_opt[:3, :3] = R_trans_opt
    T_trans_opt[:3, 3] = t_trans_opt

    return T_tool_opt, T_trans_opt


def residuals(param_vec, robot_matrices, tracker_matrices):
    # param_vec[:3] = T_tool의 회전 (rotvec), param_vec[3:6] = T_tool의 병진
    # param_vec[6:9] = T_trans의 회전 (rotvec), param_vec[9:12] = T_trans의 병진
    # Rodrigues 회전 벡터를 회전행렬로 변환
    R_tool = R.from_rotvec(param_vec[:3]).as_matrix()
    t_tool = param_vec[3:6]
    R_trans = R.from_rotvec(param_vec[6:9]).as_matrix()
    t_trans = param_vec[9:12]

    # 4x4 형태의 변환 행렬 구성
    T_tool = np.eye(4)
    T_tool[:3, :3] = R_tool
    T_tool[:3, 3] = t_tool
    T_trans = np.eye(4)
    T_trans[:3, :3] = R_trans
    T_trans[:3, 3] = t_trans

    # 모든 데이터에 대한 residual 계산
    res = []
    for i in range(len(robot_matrices)):
        TR = robot_matrices[i]
        TV = tracker_matrices[i]
        # 좌변: TR * T_tool, 우변: T_trans * TV
        left_mat = TR.dot(T_tool)
        right_mat = T_trans.dot(TV)
        # 병진 차이 (3차원)
        trans_diff = left_mat[:3, 3] - right_mat[:3, 3]
        # 회전 차이: left와 right의 상대 회전을 회전벡터로 추출
        R_left = left_mat[:3, :3]
        R_right = right_mat[:3, :3]
        # 상대 회전행렬: R_err = R_left * R_right^T
        R_err = R_left.dot(R_right.T)
        # 회전 오차를 회전벡터로 (norm = 각도, direction = 회전축)
        rotvec_err = R.from_matrix(R_err).as_rotvec()
        # residual 벡터에 추가 (길이 6)
        res.extend(trans_diff)
        res.extend(rotvec_err)
    return np.array(res)
