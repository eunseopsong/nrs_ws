import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type -- [mock_components, isaac]",
    )

    # UR10 MoveIt 설정
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit_config")  # 'ur' 패키지 사용
        .robot_description(
            # file_path=PathJoinSubstitution([
            #     FindPackageShare("ur_description"), "urdf", "ur10.urdf"
            # ]),
            file_path="/home/eunseop/nrs_ws/src/urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur10.urdf",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path=PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "config", "ur10.srdf"
        ]))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path=PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "config", "ur_controllers.yaml"
        ]))
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Move Group 실행 (MoveIt 핵심 기능)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz 실행
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "launch", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF (UR10 기본 좌표계 설정)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # UR10 State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Warehouse mongodb server (MoveIt의 데이터 저장)
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            db_arg,
            ros2_control_hardware_type,
            rviz_node,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            mongodb_server_node,
        ]
    )