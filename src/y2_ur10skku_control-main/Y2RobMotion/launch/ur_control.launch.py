from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ur_control_path = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'),
        'launch',
        'ur_control.launch.py',
    ])

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_control_path),
        launch_arguments={
            'ur_type': 'ur10',   
            'robot_ip': '192.168.0.47',
            'initial_joint_controller': 'forward_position_controller',
            'activate_joint_controller': 'true',
        }.items()
    )

    return LaunchDescription([ur_launch])
