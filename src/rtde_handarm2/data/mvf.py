import numpy as np

# ================== 설정 ==================
window_size = 50               # 이동평균 필터 윈도우 크기
target_rows = 10000             # 보간 후 원하는 총 행 수
input_path  = "DLC_robot_transfer.txt"               # 입력 파일
output_path = "DLC_robot_transfer_filtered_interp.txt"  # 결과 저장 파일
# ==========================================

def moving_average_filter(data, window_size):
    """이동평균 필터"""
    kernel = np.ones(window_size) / window_size
    filtered = np.apply_along_axis(lambda m: np.convolve(m, kernel, mode='same'), axis=0, arr=data)
    return filtered

def linear_interpolation(data, target_rows):
    """행 수를 target_rows까지 선형보간"""
    n_rows, n_cols = data.shape
    if target_rows <= n_rows:
        print("[경고] target_rows가 현재 행보다 작거나 같아 보간 생략.")
        return data

    old_idx = np.linspace(0, 1, n_rows)
    new_idx = np.linspace(0, 1, target_rows)
    interp_data = np.zeros((target_rows, n_cols))

    for i in range(n_cols):
        interp_data[:, i] = np.interp(new_idx, old_idx, data[:, i])

    return interp_data

def main():
    # 데이터 읽기
    data = np.loadtxt(input_path)
    print(f"원본 shape: {data.shape}")

    # 이동평균 필터 적용
    filtered = moving_average_filter(data, window_size)

    # 앞뒤 window_size만큼 제거
    trimmed = filtered[window_size:-window_size, :]
    print(f"필터링 후 shape: {trimmed.shape}")

    # 선형보간 적용
    interpolated = linear_interpolation(trimmed, target_rows)
    print(f"보간 후 shape: {interpolated.shape}")

    # 7번째(인덱스6), 8번째(인덱스7) 열을 0으로 설정
    interpolated[:, 6:8] = 0.0

    # 결과 저장
    np.savetxt(output_path, interpolated, fmt="%.6f")
    print(f"완료: '{output_path}' 저장됨 (7,8열은 0으로 설정됨)")

if __name__ == "__main__":
    main()

