import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('practice_visual')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'practice.urdf')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'practice.rviz') 
    
    # [수정 완료] 이제 올바른 뼈대 파일인 spindle_surface.stl을 정확히 가리킵니다.
    stl_file_path = os.path.join(pkg_dir, 'meshes', 'toolsf.stl')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='practice_visual',
            executable='ifs_node',
            name='nonlinear_surface_contactsensing',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, 'config', 'practice_config.yaml'),
                {'mesh_directory': stl_file_path} 
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] 
        )
    ])