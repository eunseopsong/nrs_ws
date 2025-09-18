import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# HDF5 파일 읽기
file_path = "trajectory.h5"
with h5py.File(file_path, "r") as f:
    x = f["x"][:]
    y = f["y"][:]
    z = f["z"][:]
    Fx = f["Fx"][:]
    Fy = f["Fy"][:]
    Fz = f["Fz"][:]

# --- 3D Trajectory + Force vectors ---
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Trajectory plot
ax.plot(x, y, z, color="blue", label="Trajectory")

# Force vectors (샘플링해서 표시: 너무 많으면 복잡해짐)
step = max(1, len(x)//50)   # 점 50개 정도만 표시
ax.quiver(
    x[::step], y[::step], z[::step],   # 위치
    Fx[::step], Fy[::step], Fz[::step],# 힘 벡터
    length=0.05, normalize=True, color="red", label="Force vectors"
)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Trajectory with Force Vectors")
ax.legend()
plt.show()

