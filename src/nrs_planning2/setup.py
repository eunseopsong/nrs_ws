from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nrs_planning2'

setup(
    name=package_name,
    version='0.0.0',
    # 1) src/ 아래에서 파이썬 패키지를 찾고,
    packages=find_packages(where='src', exclude=['test']),
    # 2) '' 루트를 src 폴더로 매핑
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eunseop',
    maintainer_email='lexondms1@g.skku.edu',
    description='Linear interpolation planning node for nrs_planning2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # src 폴더 안의 planning_node.py 의 main() 을 가리킵니다
            'nrs_planning_node = src.planning_node:main',
        ],
    },
)
