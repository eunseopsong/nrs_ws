import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='True',
        description='Use Joint State Publisher'
    ),
    DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Use RViz'
    ),
    ]	

def generate_launch_description():
    description_package = FindPackageShare('aidin_rby1_description')
    
    xacro_path = os.path.join( get_package_share_directory('aidin_rby1_description'), 'model', 'urdf')

    # RViz
    rviz_config_file = get_package_share_directory('aidin_rby1_description') + "/rviz/default.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     condition=IfCondition(LaunchConfiguration('use_rviz')))
    
    # Robot Description
    robot_description_content = ParameterValue(
        Command([
            'xacro ', os.path.join(xacro_path, 'aidin_rby1.urdf.xacro'),
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher GUI
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Robot Joint State Publisher GUI
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher'))
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(ARGUMENTS + nodes)
