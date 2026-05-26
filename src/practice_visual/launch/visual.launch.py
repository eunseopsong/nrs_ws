import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('practice_visual')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'practice.urdf')
    # 저장한 rviz 설정 파일 경로 추가!
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'practice.rviz') 
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='practice_visual',
            executable='ifs_node',
            name='nonlinear_surface_contactsensing',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'practice_config.yaml')]
        ),
        # 로봇 뼈대 정보를 퍼블리시하는 노드
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # RViz2 실행 노드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments에 rviz 설정 파일 넘겨주기!
            arguments=['-d', rviz_config_file] 
        )
    ])

