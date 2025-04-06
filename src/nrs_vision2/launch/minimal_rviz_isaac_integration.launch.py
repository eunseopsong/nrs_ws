import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # RViz 설정 파일 경로 (패키지 내 config 폴더에 rviz_config.rviz 파일이 있다고 가정)
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            get_package_share_directory("nrs_vision2"),
            "config",
            "rviz_config.rviz"
        ]),
        description="RViz configuration file"
    )

    # RViz 노드
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )

    # UR-10 URDF 파일 경로 (패키지 내 urdf 폴더에 ur10.urdf.xacro 파일이 있다고 가정)
    urdf_file = PathJoinSubstitution([
        get_package_share_directory("nrs_vision2"),
        "urdf",
        "ur10.urdf.xacro"
    ])

    # robot_state_publisher 노드: URDF를 읽어 로봇 상태를 TF로 발행
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_file}]
    )

    # Static TF 발행 노드: world -> ur10_base_link (URDF에 맞게 베이스 링크 이름 조정)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "ur10_base_link"]
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
        robot_state_publisher_node,
        static_tf_node,
    ])
