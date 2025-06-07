#!/usr/bin/env python3

import rospy
import rospkg
import matplotlib.pyplot as plt
import numpy as np
import os

def main():
    rospy.init_node('plot_record_node', anonymous=True)

    # 설정값
    sampling_time = 0.002  # 2ms

    file_name = 'FAAC3step_msg/mon1.txt'  # 파일명에 맞게 수정

    # 파일 경로 설정
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('nrs_msgmonitoring')  # 패키지명에 맞게 수정
    file_path = os.path.join(pkg_path, 'recording', file_name)

    rospy.loginfo(f"Loading data from: {file_path}")

    try:
        data = np.loadtxt(file_path, delimiter=None)  # 자동 구분자 인식
    except Exception as e:
        rospy.logerr(f"Error reading file: {e}")
        return

    # 샘플 수 및 시각 생성
    num_samples = data.shape[0]
    num_signals = data.shape[1]
    time = np.arange(num_samples) * sampling_time

    # 서브플롯 생성
    fig, axes = plt.subplots(num_signals, 1, sharex=True, figsize=(10, 2*num_signals))
    if num_signals == 1:
        axes = [axes]  # subplot이 1개일 경우 리스트로 변환

    for i in range(num_signals):
        axes[i].plot(time, data[:, i])
        axes[i].set_ylabel(f'Signal {i}')
        axes[i].grid(True)

    axes[-1].set_xlabel('Time [s]')
    plt.suptitle('Recorded Signal Plot')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
