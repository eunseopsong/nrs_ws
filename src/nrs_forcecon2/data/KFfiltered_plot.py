import numpy as np
import matplotlib.pyplot as plt

def load_data(filename):
    """텍스트 파일에서 데이터를 읽어들여 반환하는 함수"""
    data = np.loadtxt(filename)
    raw_pos = data[:, 0]  # 첫 번째 열 (Position)
    position = data[:, 1]  # 첫 번째 열 (Position)
    velocity = data[:, 2]  # 두 번째 열 (Velocity)
    acceleration = data[:, 3]  # 세 번째 열 (Acceleration)
    return raw_pos, position, velocity, acceleration

def plot_data(raw_pos ,position, velocity, acceleration, time_interval=0.002):
    """시간에 따른 Position, Velocity, Acceleration을 그리는 함수"""
    # 시간 값 생성
    time = np.arange(0, len(position) * time_interval, time_interval)

    # 위치, 속도, 가속도의 그래프를 생성
    plt.figure(figsize=(10, 6))

    # Position 그래프
    plt.subplot(4, 1, 1)
    plt.plot(time, raw_pos, label="Position", color='b')
    plt.xlabel("Time (seconds)")
    plt.ylabel("Position")
    plt.title("Position vs Time")
    plt.grid(True)

    # Position 그래프
    plt.subplot(4, 1, 2)
    plt.plot(time, position, label="Position", color='b')
    plt.xlabel("Time (seconds)")
    plt.ylabel("Position")
    plt.title("Position vs Time")
    plt.grid(True)

    # Velocity 그래프
    plt.subplot(4, 1, 3)
    plt.plot(time, velocity, label="Velocity", color='g')
    plt.xlabel("Time (seconds)")
    plt.ylabel("Velocity")
    plt.title("Velocity vs Time")
    plt.grid(True)

    # Acceleration 그래프
    plt.subplot(4, 1, 4)
    plt.plot(time, acceleration, label="Acceleration", color='r')
    plt.xlabel("Time (seconds)")
    plt.ylabel("Acceleration")
    plt.title("Acceleration vs Time")
    plt.grid(True)

    # 그래프 표시
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 데이터 파일 경로
    input_file = "KFfiltered_results.txt"  # 데이터 파일 경로를 여기에 입력하세요.

    # 데이터 로드
    raw_pos, position, velocity, acceleration = load_data(input_file)

    # 데이터 플로팅
    plot_data(raw_pos, position, velocity, acceleration)
