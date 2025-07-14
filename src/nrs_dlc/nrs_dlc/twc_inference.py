import os
import numpy as np

def main():
    print("✅ TWC inference node started.")

    # 현재 파일 기준 경로 계산
    current_dir = os.path.dirname(__file__)
    data_num = 2
    data_path = os.path.join(current_dir, '..', 'data', 'training', str(data_num))

    file_rot = os.path.join(data_path, 'Ori_rot.txt')
    file_ft  = os.path.join(data_path, 'Ori_ft.txt')

    try:
        raw_rot = np.loadtxt(file_rot)
        raw_ft = np.loadtxt(file_ft)
    except Exception as e:
        print(f"❌ 파일 읽기 실패: {file_rot} 또는 {file_ft} → {e}")
        return

    # 첫 행 제거
    raw_rot = raw_rot[1:, :]
    raw_ft = raw_ft[1:, :]

    input_data = raw_rot[:, 0:9]
    force_data = raw_ft[:, 0:3]
    moment_data = raw_ft[:, 3:6]

    print("📥 데이터 로드 완료:")
    print(f" - input_data shape: {input_data.shape}")
    print(f" - force_data shape: {force_data.shape}")
    print(f" - moment_data shape: {moment_data.shape}")

if __name__ == '__main__':
    main()
