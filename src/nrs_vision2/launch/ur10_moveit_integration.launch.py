import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # UR10 URDF 파일 절대 경로 (패키지 외부에 있음)
    urdf_path = "/home/eunseop/nrs_ws/src/urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur10.urdf"

    # URDF 파일 내용을 읽어 문자열로 저장
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # RViz 설정 파일 경로 (launch arg로 전달, 기본값은 nrs_vision2 패키지 내 config 폴더에 위치)
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("nrs_vision2"),
            "config",
            "rviz_config.rviz"
        ),
        description="Absolute path to RViz config file"
    )

    # robot_state_publisher: URDF를 파라미터로 전달하여 TF 트리 생성
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # Static TF publisher: world → ur10_base_link (URDF에 맞게 베이스 링크 이름 조정)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "ur10_base_link"],
    )

    # RViz2 노드: RViz 설정 파일에 RobotModel 디스플레이가 포함되어 있으면 UR10이 자동으로 표시됨
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription([
        rviz_config_arg,
        robot_state_publisher_node,
        static_tf_node,
        rviz_node,
    ])
