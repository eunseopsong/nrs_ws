import os
import numpy as np

import os
import numpy as np

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

def main():
    print("✅ TIC KalmanEM Node 시작")

    try:
        raw_vel, TWC_comp_FT, raw_FT = load_and_preprocess_tic_data(data_num=1)
    except RuntimeError as e:
        print(e)
        return

    lin_vel_data, ang_vel_data, TWC_force_data, TWC_moment_data = \
        split_tic_components(raw_vel, TWC_comp_FT)

    # 다음: 칼만 필터 적용 및 시각화




if __name__ == '__main__':
    main()