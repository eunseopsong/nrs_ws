import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

def generate_launch_description():

    # 기본 설정
    pkg_name = 'nrs_path2'
    robot_file_name = 'ur10e.xacro'
    pkg_path = get_package_share_directory(pkg_name)

    urdf_path = os.path.join(pkg_path, 'urdf', robot_file_name)

    # robot_description parameter 세팅
    robot_description = {
        'robot_description': Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_path
        ])
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
