import os
import numpy as np

# %% Data Loading
def load_and_preprocess_data(data_num=2, base_folder="data/training"):
    # 현재 실행 파일 기준 상대 경로 설정
    current_dir = os.path.dirname(__file__)
    data_path = os.path.join(current_dir, '..', base_folder, str(data_num))

    file_ft = os.path.join(data_path, 'Ori_ft.txt')
    file_rot = os.path.join(data_path, 'Ori_rot.txt')

    # 파일 불러오기
    try:
        raw_ft = np.loadtxt(file_ft)
        raw_rot = np.loadtxt(file_rot)
    except Exception as e:
        print(f"❌ 파일 읽기 실패: {e}")
        return None, None, None, None

    # remove the first row (line 15 in matlab)
    raw_ft = raw_ft[1:, :]
    raw_rot = raw_rot[1:, :]

    # %% Data Preprocessing
    # % Extract RPY directly and force/moment data
    input_data = raw_rot[:, 0:9] # Rot elements (1*9 matrix))
    force_data = raw_ft[:, 0:3]  # Fx, Fy, Fz (Nx3 matrix)
    moment_data = raw_ft[:, 3:6] # Mx, My, Mz (Nx3 matrix)

    # Combine force and moment data as target
    # Fx, Fy, Fz, Mx, My, Mz (N*6 matrix)
    target_data = np.hstack([force_data, moment_data])

    print("✅ 데이터 로드 및 전처리 완료")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - force_data shape: {force_data.shape}")
    print(f" - moment_data shape: {moment_data.shape}")
    print(f" - target_data shape: {target_data.shape}")

    return input_data, force_data, moment_data, target_data

# %% Normalize Utility
def normalize_data(input_data, target_data):
    input_min = np.min(input_data, axis=0)
    input_max = np.max(input_data, axis=0)
    input_data_saturated = np.clip(input_data, input_min, input_max)

    target_min = np.min(target_data, axis=0)
    target_max = np.max(target_data, axis=0)
    target_data_saturated = np.clip(target_data, target_min, target_max)

    def normalize(val, min_val, max_val):
        return 2 * (val - min_val) / (max_val - min_val) - 1

    def denormalize(val, min_val, max_val):
        return (val + 1) / 2 * (max_val - min_val) + min_val

    input_norm = normalize(input_data_saturated, input_min, input_max)
    target_norm = normalize(target_data_saturated, target_min, target_max)

    return input_norm, target_norm, input_min, input_max, target_min, target_max, denormalize
# %% Main Entry Point for ROS 2
def main():
    print("✅ TWC Inference Node 시작")
    input_data, _, _, target_data = load_and_preprocess_data(data_num=2)
    input_norm, target_norm, _, _, _, _, _ = normalize_data(input_data, target_data)

    print(" - input_data_norm shape:", input_norm.shape)
    print(" - target_data_norm shape:", target_norm.shape)


if __name__ == '__main__':
    main()

# from this