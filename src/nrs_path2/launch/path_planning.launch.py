from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_nrs_path2 = get_package_share_directory('nrs_path2')

    rviz_config_path = os.path.join(pkg_nrs_path2, 'rviz', 'rviz_config.rviz')

    return LaunchDescription([
        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # 주요 노드 실행
        Node(
            package='nrs_path2',
            executable='nrs_node_path_generation',
            name='nrs_node_path_generation',
            output='screen'
        ),
        Node(
            package='nrs_path2',
            executable='nrs_node_visualization',
            name='nrs_node_visualization',
            output='screen'
        ),

        # static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),

        # joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),
    ])
