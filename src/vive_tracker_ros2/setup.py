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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
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
            "vive_tracker_node = vive_tracker_ros2.vive_tracker_node:main",
            "test_calibration = vive_tracker_ros2.test_calibration:main",
            "robot_controller = vive_tracker_ros2.robot_controller:main",
            "vr_diagnostic_tool = vive_tracker_ros2.vr_diagnostic_tool:main",
        ],
    },
)
