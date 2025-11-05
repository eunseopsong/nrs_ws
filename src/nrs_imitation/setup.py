from setuptools import find_packages, setup

package_name = 'nrs_imitation'

setup(
    name=package_name,
    version='0.0.0',
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
    description='ROS2 nodes for imitation / VR tracking → robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ros2 run nrs_imitation vive_to_ee_pose
            'servo_ur10e = nrs_imitation.servo_ur10e:main',
        ],
    },
)
