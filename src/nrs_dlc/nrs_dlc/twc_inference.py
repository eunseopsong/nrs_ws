import os
import numpy as np

# %% Data Loading
def load_and_preprocess_data(data_num=2, base_folder="data/training"):
    # 현재 실행 파일 기준 상대 경로 설정
    current_dir = os.path.dirname(__file__)
    data_path = os.path.join(current_dir, '..', base_folder, str(data_num))

    file_rot = os.path.join(data_path, 'Ori_rot.txt')
    file_ft = os.path.join(data_path, 'Ori_ft.txt')

    # 파일 불러오기
    try:
        raw_rot = np.loadtxt(file_rot)
        raw_ft = np.loadtxt(file_ft)
    except Exception as e:
        print(f"❌ 파일 읽기 실패: {e}")
        return None, None, None, None

    # 첫 번째 행 제거
    raw_rot = raw_rot[1:, :]
    raw_ft = raw_ft[1:, :]

    # %% Data Preprocessing
    # 입력: 회전행렬 9개 요소
    input_data = raw_rot[:, 0:9]  # Nx9

    # 출력: 힘 (Fx, Fy, Fz) 및 모멘트 (Mx, My, Mz)
    force_data = raw_ft[:, 0:3]    # Nx3
    moment_data = raw_ft[:, 3:6]   # Nx3

    # 출력 통합 (Fx, Fy, Fz, Mx, My, Mz)
    target_data = np.hstack([force_data, moment_data])  # Nx6

    print("✅ 데이터 로드 및 전처리 완료")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - force_data shape: {force_data.shape}")
    print(f" - moment_data shape: {moment_data.shape}")
    print(f" - target_data shape: {target_data.shape}")

    return input_data, force_data, moment_data, target_data

# %% Main Entry Point for ROS 2
def main():
    print("✅ TWC Inference Node 시작")
    input_data, force_data, moment_data, target_data = load_and_preprocess_data(data_num=2)

if __name__ == '__main__':
    main()

# from this