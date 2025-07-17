import os
import numpy as np

def load_and_preprocess_tic_data(data_num=1, base_folder="data/TIC/training"):
    """
    TIC 데이터를 불러오고 전처리를 수행합니다.
    Returns:
        lin_vel, ang_vel          (Nx3, Nx3)
        TWC_force, TWC_moment     (Nx3, Nx3)
        raw_vel, TWC_comp_FT, raw_FT (Nx6, Nx6, Nx6)
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(current_dir, '..', base_folder, str(data_num))

    file_vel = os.path.join(data_path, 'TIC_input_vel.txt')
    file_TWC_comp = os.path.join(data_path, 'TWC_compft.txt')
    file_ori_ft = os.path.join(data_path, 'TWC_target_ori_ft.txt')

    try:
        raw_vel = np.loadtxt(file_vel, dtype=float, ndmin=2)[28:, :]
        TWC_comp_FT = np.loadtxt(file_TWC_comp, dtype=float, ndmin=2)[28:, :]
        raw_FT = np.loadtxt(file_ori_ft, dtype=float, ndmin=2)[28:, :]
    except Exception as e:
        print(f"❌ 파일 읽기 실패: {e}")
        return None, None, None, None, None, None, None

    lin_vel = raw_vel[:, 0:3]
    ang_vel = raw_vel[:, 3:6]

    TWC_force = TWC_comp_FT[:, 0:3]
    TWC_moment = TWC_comp_FT[:, 3:6]

    print("✅ TIC 데이터 로드 및 전처리 완료")
    print(f" - lin_vel shape: {lin_vel.shape}")
    print(f" - ang_vel shape: {ang_vel.shape}")
    print(f" - TWC_force shape: {TWC_force.shape}")
    print(f" - TWC_moment shape: {TWC_moment.shape}")

    return lin_vel, ang_vel, TWC_force, TWC_moment, raw_vel, TWC_comp_FT, raw_FT

def main():
    print("✅ TIC KalmanEM Node 시작")
    
    lin_vel, ang_vel, TWC_force, TWC_moment, _, _, _ = \
        load_and_preprocess_tic_data(data_num=1)

    if lin_vel is None:
        return  # 에러 발생 시 종료

    # 이후: kalman_em_diff_estimate(TWC_force), 시각화 등


if __name__ == '__main__':
    main()