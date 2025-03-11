from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'joint_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eunseop',
    maintainer_email='lexondms1@g.skku.edu',
    description='6-DOF manipulator joint control using ROS 2 and Isaac Sim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_control = src.fk_control:main',
            'ik_control = src.ik_control:main',
            'pub_effort = src.pub_effort:main',
            'sub_joint_states = src.sub_joint_states:main'
        ],
    },
)
