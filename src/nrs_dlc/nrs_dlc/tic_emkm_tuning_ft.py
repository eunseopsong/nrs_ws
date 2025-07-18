import os
import numpy as np
import json
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

def load_and_preprocess_tic_data(data_num=1, base_folder="data/TIC/training"):
    """
    TIC 데이터를 불러오고 전처리를 수행합니다.
    Returns:
        raw_vel:       선형/각속도 벡터 (Nx6)
        TWC_comp_FT:   보정된 Force/Torque (Nx6)
        raw_FT:        원본 측정된 Force/Torque (Nx6)
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
        raw_FT = np.loadtxt(file_ori_ft, dtype=float, ndmin=2)[28:, :]
    except Exception as e:
        raise RuntimeError(f"❌ 파일 읽기 실패: {e}")

    print("✅ TIC 데이터 로드 및 전처리 완료")
    print(f" - raw_vel shape: {raw_vel.shape}")
    print(f" - TWC_comp_FT shape: {TWC_comp_FT.shape}")
    print(f" - raw_FT shape: {raw_FT.shape}")

    return raw_vel, TWC_comp_FT, raw_FT

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

def run_kalman_em_filter(force_data, moment_data, dt=0.01):
    """
    force와 moment 데이터를 Kalman EM 필터에 통과시켜 추정된 가속도를 반환한다.
    Q, R이 저장된 JSON 파일이 존재하지 않으면 EM 알고리즘으로 자동 튜닝 후 저장함.
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))
    current_basename = os.path.splitext(os.path.basename(__file__))[0]

    force_json_file = os.path.join(current_dir, f"{current_basename}_force_em_gain.json")
    moment_json_file = os.path.join(current_dir, f"{current_basename}_moment_em_gain.json")

    print("🧠 Kalman EM 추정 시작...")
    force_em_result = kalman_em_diff_estimate(force_data, dt, force_json_file)
    moment_em_result = kalman_em_diff_estimate(moment_data, dt, moment_json_file)
    print("✅ Kalman EM 추정 완료")

    return force_em_result, moment_em_result

def kalman_em_diff_estimate(ft_data, dt, json_filename, max_iter=20):
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
    n = ft_data.shape[0]
    acc_out = np.zeros((n, 3))

    # ✅ 사용자 지정 저장 디렉토리
    home_dir = os.path.expanduser("~")
    save_dir = os.path.join(home_dir, "nrs_ws", "src", "nrs_dlc", "json", "TIC")
    os.makedirs(save_dir, exist_ok=True)

    # 파일 이름만 유지한 채 경로 붙이기
    base_filename = os.path.basename(json_filename)
    json_path = os.path.join(save_dir, base_filename)

    # 📂 기존 json이 있으면 불러오기
    if os.path.exists(json_path):
        with open(json_path, 'r') as f:
            gain = json.load(f)
            Q = np.array(gain['Q'])
            R = np.array(gain['R'])
        print(f"[Kalman EM] Loaded Q, R from {json_path} (Load complete.)")
    else:
        print("[Kalman EM] No gain found. Start tuning...")

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

            # Forward filtering
            for k in range(1, n):
                z = (ft_data[k] - ft_data[k - 1]).reshape(3, 1) / dt
                K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
                x = x_pred + K @ (z - H @ x_pred)
                P = (np.eye(3) - K @ H) @ P_pred
                xf[:, k] = x.flatten()
                Pf[:, :, k] = P
                x_pred = A @ x
                P_pred = A @ P @ A.T + Q

            # Backward smoothing
            xs[:, -1] = xf[:, -1]
            Ps[:, :, -1] = Pf[:, :, -1]
            for k in range(n - 2, -1, -1):
                Ck = Pf[:, :, k] @ A.T @ np.linalg.inv(A @ Pf[:, :, k] @ A.T + Q)
                xs[:, k] = xf[:, k] + Ck @ (xs[:, k + 1] - A @ xf[:, k])
                Ps[:, :, k] = Pf[:, :, k] + Ck @ (Ps[:, :, k + 1] - A @ Pf[:, :, k] @ A.T - Q) @ Ck.T
                Pcs[:, :, k] = Ck @ Ps[:, :, k + 1]

            Q_sum = np.zeros((3, 3))
            R_sum = np.zeros((3, 3))
            for k in range(1, n):
                dz = (ft_data[k] - ft_data[k - 1]).reshape(3, 1) / dt - H @ xs[:, k].reshape(3, 1)
                dx = xs[:, k].reshape(3, 1) - A @ xs[:, k - 1].reshape(3, 1)
                Q_sum += dx @ dx.T + Ps[:, :, k] + A @ Ps[:, :, k - 1] @ A.T - A @ Pcs[:, :, k - 1].T - Pcs[:, :, k - 1] @ A.T
                R_sum += dz @ dz.T + H @ Ps[:, :, k] @ H.T

            Q = Q_sum / (n - 1)
            R = R_sum / (n - 1)

        # 🔽 Q, R 저장
        with open(json_path, 'w') as f:
            json.dump({'Q': Q.tolist(), 'R': R.tolist()}, f, indent=4)
        print(f"[Kalman EM] Tuning complete. Gains saved to {json_path}")

    # Final Kalman filtering
    x = np.zeros((3, 1))
    P = np.eye(3)
    for k in range(1, n):
        z = (ft_data[k] - ft_data[k - 1]).reshape(3, 1) / dt
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
        x = x + K @ (z - H @ x)
        P = (np.eye(3) - K @ H) @ P
        acc_out[k] = x.flatten()
        x = A @ x
        P = A @ P @ A.T + Q

    return acc_out

def validate_and_plot_derivatives(original_data, kalman_data, dt, title='Force', unit='N/s'):
    """
    Kalman 필터 결과와 직접 미분, Gaussian smoothing 결과를 비교 시각화합니다.
    Args:
        original_data: Nx3 원본 입력값 (ex. 보정된 힘/모멘트)
        kalman_data:   Nx3 Kalman 필터 결과
        dt:            샘플링 간격 (초)
        title:         제목 (Force / Moment 등)
        unit:          y축 단위 (예: N/s, Nm/s)
    """

    time_profile = np.arange(original_data.shape[0]) * dt
    deri = np.zeros_like(original_data)
    smooth = np.zeros_like(original_data)

    # 직접 미분
    prev = np.zeros(3)
    for i in range(original_data.shape[0]):
        deri[i] = (original_data[i] - prev) / dt
        prev = original_data[i]

    # Gaussian smoothing
    for i in range(3):
        smooth[:, i] = gaussian_filter1d(deri[:, i], sigma=5)

    # Plot
    component_names = ['x', 'y', 'z']
    plt.figure(figsize=(8, 6))
    plt.suptitle(f'{title} Derivative Comparison', fontsize=14)

    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_profile, kalman_data[:, i], 'r-', linewidth=2, label='Kalman Filter')
        plt.plot(time_profile, deri[:, i], 'k-', linewidth=0.8, label='Direct Derivative')
        plt.plot(time_profile, smooth[:, i], 'b-', linewidth=2, label='Gaussian Smoothing')
        plt.xlabel('Time (s)')
        plt.ylabel(f'{title[0]}{component_names[i]} ({unit})')
        plt.grid(True)
        plt.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def main():
    print("✅ TIC KalmanEM Node 시작")

    try:
        raw_vel, TWC_comp_FT, raw_FT = load_and_preprocess_tic_data(data_num=1)
    except RuntimeError as e:
        print(e)
        return

    lin_vel_data, ang_vel_data, TWC_force_data, TWC_moment_data = \
        split_tic_components(raw_vel, TWC_comp_FT)

    # 🧠 칼만 필터 적용 (force, moment 모두)
    force_EM_data, moment_EM_data = run_kalman_em_filter(TWC_force_data, TWC_moment_data)
    # ✅ 시각화: Kalman vs 미분 vs smoothing
    validate_and_plot_derivatives(TWC_force_data, force_EM_data, dt=0.01, title='Force', unit='N/s')
    validate_and_plot_derivatives(TWC_moment_data, moment_EM_data, dt=0.01, title='Moment', unit='Nm/s')

if __name__ == '__main__':
    main()