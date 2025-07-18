import os
import numpy as np
import json
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

def rowwise_cross(a, b):
    return np.cross(a, b)

# %% Data Loading
def load_and_preprocess_tic_data(data_num=2, base_folder="data/TIC/training"):
    """
    TIC 데이터를 불러오고 전처리를 수행합니다.
    Returns:
        raw_vel:       선형/각속도 벡터 (Nx6)
        TWC_comp_FT:   보정된 Force/Torque (Nx6)
        ori_FT:        원본 측정된 Force/Torque (Nx6)
    """
    # 현재 스크립트 위치 기준으로 절대경로 계산
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(current_dir, '..', base_folder, str(data_num))

    # 파일 경로 구성
    file_vel = os.path.join(data_path, 'TIC_input_vel.txt')
    file_TWC_comp = os.path.join(data_path, 'TWC_compft.txt')
    file_ori_ft = os.path.join(data_path, 'TWC_target_ori_ft.txt')

    try:
        # 앞 28줄 제거 (MATLAB의 raw_xxx(1:28,:) = []와 동일)
        raw_vel = np.loadtxt(file_vel, dtype=float, ndmin=2)[28:, :]
        TWC_comp_FT = np.loadtxt(file_TWC_comp, dtype=float, ndmin=2)[28:, :]
        ori_FT = np.loadtxt(file_ori_ft, dtype=float, ndmin=2)[28:, :]
    except Exception as e:
        raise RuntimeError(f"❌ 파일 읽기 실패: {e}")

    print("✅ TIC 데이터 로드 및 전처리 완료")
    print(f" - raw_vel shape: {raw_vel.shape}")
    print(f" - TWC_comp_FT shape: {TWC_comp_FT.shape}")
    print(f" - raw_FT shape: {ori_FT.shape}")

    return raw_vel, TWC_comp_FT, ori_FT

# %% Data Preprocessing
# % Extract RPY directly and force/moment data
def split_tic_components(raw_vel, TWC_comp_FT):
    """
    raw_vel과 TWC_comp_FT 데이터를 분리하여 구성 요소를 반환합니다.

    Returns:
        lin_vel_data:     (Nx3) 선형 속도
        ang_vel_data:     (Nx3) 각속도
        TWC_force_data:   (Nx3) 보상된 힘
        TWC_moment_data:  (Nx3) 보상된 모멘트
    """
    lin_vel_data = raw_vel[:, 0:3]
    ang_vel_data = raw_vel[:, 3:6]
    TWC_force_data = TWC_comp_FT[:, 0:3]
    TWC_moment_data = TWC_comp_FT[:, 3:6]

    print("✅ TIC 구성 요소 분리 완료")
    print(f" - lin_vel_data shape: {lin_vel_data.shape}")
    print(f" - ang_vel_data shape: {ang_vel_data.shape}")
    print(f" - TWC_force_data shape: {TWC_force_data.shape}")
    print(f" - TWC_moment_data shape: {TWC_moment_data.shape}")

    return lin_vel_data, ang_vel_data, TWC_force_data, TWC_moment_data


def run_kalman_and_convert(lin_vel_data, ang_vel_data, dt=0.01):
    """
    Kalman 필터로 선형/각속도 미분 → 가속도 추정 후,
    힘센서 기준으로 변환된 가속도를 계산합니다.
    Returns:
        lin_acc_data, ang_acc_data, converted_acc
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))
    current_basename = os.path.splitext(os.path.basename(__file__))[0]
    lin_acc_tune_file = os.path.join(current_dir, f"{current_basename}_lin_em_gain.json")
    ang_acc_tune_file = os.path.join(current_dir, f"{current_basename}_ang_em_gain.json")

    # Kalman 필터 적용
    ang_acc_data = kalman_em_diff_estimate(ang_vel_data, dt, ang_acc_tune_file)
    lin_acc_data = kalman_em_diff_estimate(lin_vel_data, dt, lin_acc_tune_file)

    # 기준 변환
    r = np.array([0, 0, -0.185])
    r_mat = np.tile(r, (lin_acc_data.shape[0], 1))

    omega_cross_r = rowwise_cross(ang_vel_data, r_mat)
    omega_cross_omega_cross_r = rowwise_cross(ang_vel_data, omega_cross_r)
    alpha_cross_r = rowwise_cross(ang_acc_data, r_mat)

    converted_acc = lin_acc_data - alpha_cross_r - omega_cross_omega_cross_r

    print("✅ Kalman 추정 및 기준 변환 완료")
    print(f" - lin_acc_data shape: {lin_acc_data.shape}")
    print(f" - ang_acc_data shape: {ang_acc_data.shape}")
    print(f" - converted_acc shape: {converted_acc.shape}")

    return lin_acc_data, ang_acc_data, converted_acc

def kalman_em_diff_estimate(vel_data, dt, filename, max_iter=20):
    """
    EM 기반 Kalman 필터를 이용해 1차 미분값(가속도 등)을 추정한다.
    Q, R을 지정된 경로에서 불러오거나 EM 알고리즘으로 학습하여 저장함.
    최종적으로 필터링된 acc_out (Nx3)을 반환한다.
    """
    import os
    import json
    import numpy as np

    A = np.eye(3)
    H = np.eye(3)
    n = vel_data.shape[0]
    acc_out = np.zeros((n, 3))

    json_path = filename.replace('.mat', '.json')

    if os.path.exists(json_path):
        with open(json_path, 'r') as f:
            json_data = json.load(f)
            Q = np.array(json_data["Q"])
            R = np.array(json_data["R"])
        print(f"[Kalman EM] Loaded Q, R from {json_path}")
    else:
        print(f"[Kalman EM] No Q/R found. Start tuning...")
        Q = np.eye(3) * 1e-4
        R = np.eye(3) * 1e-3
        x = np.zeros((3, 1))
        P = np.eye(3)

        for _ in range(max_iter):
            xs = np.zeros((3, n))
            Ps = np.zeros((3, 3, n))
            Pcs = np.zeros((3, 3, n - 1))
            xf = np.zeros((3, n))
            Pf = np.zeros((3, 3, n))

            x_pred = x.copy()
            P_pred = P.copy()

            # Forward Kalman Filtering
            for k in range(1, n):
                z = (vel_data[k] - vel_data[k - 1]).reshape(3, 1) / dt
                K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
                x = x_pred + K @ (z - H @ x_pred)
                P = (np.eye(3) - K @ H) @ P_pred
                xf[:, k] = x.flatten()
                Pf[:, :, k] = P
                x_pred = A @ x
                P_pred = A @ P @ A.T + Q

            # Backward Smoothing
            xs[:, -1] = xf[:, -1]
            Ps[:, :, -1] = Pf[:, :, -1]
            for k in range(n - 2, -1, -1):
                Ck = Pf[:, :, k] @ A.T @ np.linalg.inv(A @ Pf[:, :, k] @ A.T + Q)
                xs[:, k] = xf[:, k] + Ck @ (xs[:, k + 1] - A @ xf[:, k])
                Ps[:, :, k] = Pf[:, :, k] + Ck @ (Ps[:, :, k + 1] - A @ Pf[:, :, k] @ A.T - Q) @ Ck.T
                Pcs[:, :, k] = Ck @ Ps[:, :, k + 1]

            # M-step: update Q, R
            Q_sum = np.zeros((3, 3))
            R_sum = np.zeros((3, 3))
            for k in range(1, n):
                dz = (vel_data[k] - vel_data[k - 1]).reshape(3, 1) / dt - H @ xs[:, k].reshape(3, 1)
                dx = xs[:, k].reshape(3, 1) - A @ xs[:, k - 1].reshape(3, 1)
                Q_sum += dx @ dx.T + Ps[:, :, k] + A @ Ps[:, :, k - 1] @ A.T \
                         - A @ Pcs[:, :, k - 1].T - Pcs[:, :, k - 1] @ A.T
                R_sum += dz @ dz.T + H @ Ps[:, :, k] @ H.T

            Q = Q_sum / (n - 1)
            R = R_sum / (n - 1)

        # Save Q, R to JSON
        with open(json_path, 'w') as f:
            json.dump({"Q": Q.tolist(), "R": R.tolist()}, f, indent=4)
        print(f"[Kalman EM] Tuned and saved Q, R to {json_path}")

    # Final Kalman Filtering
    x = np.zeros((3, 1))
    P = np.eye(3)
    for k in range(1, n):
        z = (vel_data[k] - vel_data[k - 1]).reshape(3, 1) / dt
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
        x = x + K @ (z - H @ x)
        P = (np.eye(3) - K @ H) @ P
        acc_out[k] = x.flatten()
        x = A @ x
        P = A @ P @ A.T + Q

    return acc_out

def build_input_target_data(converted_acc, ang_acc_data, ang_vel_data,
                             TWC_force_data, TWC_moment_data):
    """
    입력 벡터와 목표 벡터를 구성하는 함수.
    Returns:
        input_data: (N, 9)
        target_data: (N, 6)
    """
    input_data = np.hstack([converted_acc, ang_acc_data, ang_vel_data])
    target_data = np.hstack([TWC_force_data, TWC_moment_data])

    print("✅ 입력 및 목표 데이터 구성 완료")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - target_data shape: {target_data.shape}")

    return input_data, target_data


def normalize_input_target_data(input_data, target_data):
    """
    입력/목표 데이터를 [-1, 1] 범위로 정규화하며, min/max 값을 함께 반환합니다.
    데이터는 saturation 처리 후 정규화됩니다.
    
    Returns:
        input_data_norm, target_data_norm,
        input_min, input_max, target_min, target_max
    """
    # 각 열별 min/max 계산
    input_min = np.min(input_data, axis=0)
    input_max = np.max(input_data, axis=0)
    target_min = np.min(target_data, axis=0)
    target_max = np.max(target_data, axis=0)

    # Saturation 처리
    input_data_saturated = np.clip(input_data, input_min, input_max)
    target_data_saturated = np.clip(target_data, target_min, target_max)

    # 정규화 함수 정의
    def normalize(value, min_val, max_val):
        return 2 * ((value - min_val) / (max_val - min_val)) - 1

    # 정규화 수행
    input_data_norm = normalize(input_data_saturated, input_min, input_max)
    target_data_norm = normalize(target_data_saturated, target_min, target_max)

    print("✅ 입력 및 목표 데이터 정규화 완료")
    print(f" - input_data_norm shape: {input_data_norm.shape}")
    print(f" - target_data_norm shape: {target_data_norm.shape}")

    return input_data_norm, target_data_norm, input_min, input_max, target_min, target_max






def main():
    print("✅ TIC KalmanEM Node 시작")

    try:
        raw_vel, TWC_comp_FT, ori_FT = load_and_preprocess_tic_data(data_num=2)
    except RuntimeError as e:
        print(e)
        return

    lin_vel_data, ang_vel_data, TWC_force_data, TWC_moment_data = \
        split_tic_components(raw_vel, TWC_comp_FT)

    lin_acc_data, ang_acc_data, converted_acc = \
        run_kalman_and_convert(lin_vel_data, ang_vel_data)

    input_data, target_data = build_input_target_data(
        converted_acc, ang_acc_data, ang_vel_data,
        TWC_force_data, TWC_moment_data
    )

    # ⬇️ 정규화 단계
    input_data_norm, target_data_norm, input_min, input_max, target_min, target_max = \
        normalize_input_target_data(input_data, target_data)




if __name__ == '__main__':
    main()