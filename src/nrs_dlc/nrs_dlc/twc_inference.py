import numpy as np
import os

def main():
    print("✅ TWC inference node started.")

    # 파일 경로 설정
    data_num = 2
    folder_name = "Data/training"
    base_path = os.path.join(folder_name, str(data_num))

    file_rot = os.path.join(base_path, "Ori_rot.txt")
    file_ft = os.path.join(base_path, "Ori_ft.txt")

    # 파일 읽기
    try:
        raw_rot = np.loadtxt(file_rot)
        raw_ft = np.loadtxt(file_ft)
    except Exception as e:
        print(f"❌ 파일 읽기 실패: {e}")
        return

    # 첫 행 제거
    raw_rot = raw_rot[1:, :]
    raw_ft = raw_ft[1:, :]

    # 회전행렬 (9개 요소), 힘/모멘트 분리
    input_data = raw_rot[:, 0:9]       # Nx9
    force_data = raw_ft[:, 0:3]        # Nx3
    moment_data = raw_ft[:, 3:6]       # Nx3

    print("📥 데이터 로드 완료:")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - force_data shape: {force_data.shape}")
    print(f" - moment_data shape: {moment_data.shape}")

if __name__ == '__main__':
    main()
