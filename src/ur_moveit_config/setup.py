from setuptools import setup
import os
from glob import glob

package_name = "ur_moveit_config"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="MoveIt configuration for UR10",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    data_files=[
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),  # ✅ launch 파일 추가
    ],
)
