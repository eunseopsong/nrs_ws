from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', '/path/to/your/rviz_config.rviz'],  # 실제 경로로 수정
            output='screen'
        ),

        # Path Generation Node
        Node(
            package='nrs_path2',
            executable='nrs_node_path_generation',
            name='nrs_node_path_generation',
            output='screen'
        ),

        # Simulation Node (필요 시)
        Node(
            package='nrs_path2',
            executable='nrs_node_simulation',  # 존재하는 경우만 사용
            name='nrs_node_simulation',
            output='screen'
        ),

        # Static Transform Publisher: map → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ['move_group/fake_controller_joint_states']
            }],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),
    ])
