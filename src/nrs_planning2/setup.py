from setuptools import find_packages, setup

package_name = 'nrs_planning2'

setup(
    name=package_name,
    version='0.0.0',
    # test 폴더만 제외하고, 최상위에 있는 src 패키지를 찾아 설치
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eunseop',
    maintainer_email='lexondms1@g.skku.edu',
    description='Interpolation node for nrs_planning2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 모듈 이름을 'src'로 바꿔야, src 폴더가 설치된 패키지로 인식됨
            'nrs_planning_node = src.planning_node:main',
        ],
    },
)
