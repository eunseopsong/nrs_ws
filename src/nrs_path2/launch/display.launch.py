from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # URDF 파일 경로
    urdf_path = os.path.join(
        os.environ['HOME'],
        'nrs_ws', 'src', 'urdf_files_dataset', 'urdf_files',
        'ros-industrial', 'xacro_generated', 'universal_robots',
        'ur_description', 'urdf', 'ur10e.urdf'
    )

    return LaunchDescription([
        # robot_state_publisher 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # rviz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[]
        ),
    ])
