from setuptools import setup

package_name = 'nrs_planning2'

setup(
    name=package_name,
    version='0.0.0',
    # 1) 'nrs_planning2' 패키지는 실제로 src/ 폴더에 들어 있습니다
    packages=[package_name],
    # 2) package_name → 'src' 디렉터리 매핑
    package_dir={package_name: 'src'},
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
    entry_points={
        'console_scripts': [
            # 이제 import 경로는 nrs_planning2.planning_node:main
            'interpolation_publisher = src.planning_node:main',
        ],
    },
)
