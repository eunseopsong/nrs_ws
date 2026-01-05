from setuptools import find_packages, setup

package_name = 'nrs_imitation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'torch',
        'opencv-python',
        'numpy',
        'cv_bridge',
    ],
    zip_safe=True,
    maintainer='eunseop',
    maintainer_email='lexondms1@g.skku.edu',
    description='ROS2 node for ACT policy inference with Isaac Sim UR10e',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'act_policy_infer = nrs_imitation.act_policy_infer:main',
            'vr_demo_recorder = nrs_imitation.vr_demo_recorder:main',
        ],
    },
)
