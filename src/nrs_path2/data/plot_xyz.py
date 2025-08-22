#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

def load_xyz(path: str):
    if not os.path.exists(path):
        raise FileNotFoundError(f"파일을 찾을 수 없습니다: {path}")
    data = np.loadtxt(path, dtype=float, ndmin=2)
    if data.shape[1] < 3:
        raise ValueError("txt 파일에 최소 3열(x,y,z)이 필요합니다.")
    return data[:, 0], data[:, 1], data[:, 2]  # x, y, z

def main():
    path_a = "/home/eunseop/nrs_ws/src/nrs_path2/data/visual_final_waypoints.txt"
    path_b = "/home/eunseop/nrs_ws/src/nrs_path2/data/Ori_path_transformed.txt"

    # 데이터 로드
    x_a, y_a, z_a = load_xyz(path_a)
    x_b, y_b, z_b = load_xyz(path_b)

    # Figure 생성 (2개 서브플롯: XY 비교 / Z 비교)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # ---- (x,y) 2D 비교 ----
    axes[0].plot(x_a, y_a, linestyle='-',  marker='o',  markersize=2, label=os.path.basename(path_a))
    axes[0].plot(x_b, y_b, linestyle='--', marker='s',  markersize=2, label=os.path.basename(path_b))
    axes[0].set_title("XY Trajectory (Overlay)")
    axes[0].set_xlabel("X")
    axes[0].set_ylabel("Y")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend(loc="best")

    # ---- z 단독 비교 ----
    axes[1].plot(z_a, linestyle='-',  marker='o', markersize=2, label=os.path.basename(path_a))
    axes[1].plot(z_b, linestyle='--', marker='s', markersize=2, label=os.path.basename(path_b))
    axes[1].set_title("Z values (Overlay)")
    axes[1].set_xlabel("Index")
    axes[1].set_ylabel("Z")
    axes[1].grid(True)
    axes[1].legend(loc="best")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

