import os
import numpy as np

def load_tic_data(data_num=1, base_path="data/training"):
    """
    TIC 데이터를 로드하고 전처리합니다.
    Returns:
        raw_vel:     선형/각속도 벡터 (Nx6)
        TWC_comp_FT: 보상된 Force/Torque (Nx6)
        raw_FT:      원본 측정된 Force/Torque (Nx6)
    """
    folder_path = os.path.join(base_path, str(data_num))

    file_vel = os.path.join(folder_path, "TIC_input_vel.txt")
    file_TWC_comp = os.path.join(folder_path, "TWC_compft.txt")
    file_ori_ft = os.path.join(folder_path, "TWC_target_ori_ft.txt")

    try:
        raw_vel = np.loadtxt(file_vel, dtype=float, ndmin=2)[28:, :]
        TWC_comp_FT = np.loadtxt(file_TWC_comp, dtype=float, ndmin=2)[28:, :]
        raw_FT = np.loadtxt(file_ori_ft, dtype=float, ndmin=2)[28:, :]
    except Exception as e:
        raise RuntimeError(f"파일 로드 중 오류 발생: {e}")

    return raw_vel, TWC_comp_FT, raw_FT

def main():
    print("✅ TIC KalmanEM Node 시작")
    
    # 데이터 로드
    raw_vel, TWC_comp_FT, raw_FT = load_tic_data(data_num=1)
    
    # 이후 연산: 별도 함수로 분리 예정 (e.g., run_kalman_em_process)

if __name__ == '__main__':
    main()