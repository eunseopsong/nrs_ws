import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3D 플롯용 (일부 환경에선 필요)

# 1. txt 파일에서 데이터 읽기
#    -> 각 줄: x y z roll pitch yaw
data = np.loadtxt("urp_waypoints.txt")   # 파일 이름/경로 맞게 수정

# 2. 앞 3열만 사용 (x, y, z)
xyz = data[:, :3]   # (N, 3)
x = xyz[:, 0]
y = xyz[:, 1]
z = xyz[:, 2]

# ========================
# (A) 3D 산점도 시각화
# ========================
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.scatter(x, y, z, s=8)  # s는 점 크기

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Waypoints in 3D")

ax.view_init(elev=30, azim=45)  # 보기 각도(원하면 조절)

plt.tight_layout()
plt.show()

# ================================
# (B) x, y, z를 행 순서대로 시각화
# ================================
idx = np.arange(len(x))  # 행 인덱스 (0, 1, 2, ...)

fig2, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

axes[0].plot(idx, x)
axes[0].set_ylabel("X")
axes[0].set_title("X / Y / Z vs Index")

axes[1].plot(idx, y)
axes[1].set_ylabel("Y")

axes[2].plot(idx, z)
axes[2].set_ylabel("Z")
axes[2].set_xlabel("Index (row number)")

plt.tight_layout()
plt.show()
