from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 주요 노드 실행
        Node(
            package='nrs_path2',
            executable='nrs_node_path_generation',
            name='nrs_node_path_generation',
            output='screen'
        ),
        # Node(
        #     package='nrs_path2',
        #     executable='nrs_node_visualization',
        #     name='nrs_node_visualization',
        #     output='screen'
        # )
    ])
