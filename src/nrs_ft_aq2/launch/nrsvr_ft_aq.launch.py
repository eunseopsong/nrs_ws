from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('nrs_ft_aq2')
    config_file = os.path.join(pkg_share, 'config', 'VR_config.yaml')

    ft_node = Node(
        package='nrs_ft_aq2',
        executable='ft_aq_main_node',
        name='ft_aq',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        ft_node
    ])
