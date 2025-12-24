from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    singleArm_node = Node(
        package='Y2RobMotion',
        executable='singleArm_motion',
        name='singleArm_motion',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        singleArm_node
    ])
