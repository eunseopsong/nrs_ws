from setuptools import find_packages, setup
import os
from glob import glob

package_name = "vive_tracker_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        # launch 설치 유지
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),

        # ✅ yaml 설치 (네가 쓰는 calibration_matrix.yaml 포함)
        (os.path.join("share", package_name, "yaml"), glob("yaml/*.yaml")),

        # ✅ config 설치 (json, 기타 설정파일 있으면 설치)
        (os.path.join("share", package_name, "config"), glob("config/*")),

        # ✅ rviz 같은 파일 있으면(없으면 빈 glob라서 문제 없음)
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hwpark",
    maintainer_email="hw.park@aidinrobotics.co.kr",
    description="vive tracker ros2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 기존 유지
            "vive_tracker_node = vive_tracker_ros2.vive_tracker_node:main",
            "test_calibration = vive_tracker_ros2.test_calibration:main",
            "robot_controller = vive_tracker_ros2.robot_controller:main",
            "vr_diagnostic_tool = vive_tracker_ros2.vr_diagnostic_tool:main",

            # ✅ 내가 준 수정본(파일명이 vive_tracker.py 일 때)
            # ros2 run vive_tracker_ros2 vive_tracker
            "vive_tracker = vive_tracker_ros2.vive_tracker:main",
        ],
    },
)
