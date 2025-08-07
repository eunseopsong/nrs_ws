import numpy as np

# 데이터 로드
data = np.loadtxt('Descre_P_recording.txt')

# 5번째 열 (index 4)에서 값이 pi보다 크면 -2pi, -pi보다 작으면 +2pi 처리
pi = np.pi
data[:, 4] = np.where(data[:, 4] > pi, data[:, 4] - 2*pi, data[:, 4])
data[:, 4] = np.where(data[:, 4] < -pi, data[:, 4] + 2*pi, data[:, 4])

data[:, 5] = np.where(data[:, 5] > pi/2, data[:, 5] - 2*pi, data[:, 5])
data[:, 5] = np.where(data[:, 5] < -pi/2, data[:, 5] + 2*pi, data[:, 5])

# 수정된 데이터를 동일한 이름으로 저장 (원하는 포맷으로 출력)
np.savetxt('Descre_P_recording.txt', data, fmt='%.6f', delimiter=' ', header='', comments='')

# 결과 확인 (변경된 5번째 열)
print(data[:10, 4])  # 변경된 5번째 열을 출력해봄
print(data[:10, 5])
