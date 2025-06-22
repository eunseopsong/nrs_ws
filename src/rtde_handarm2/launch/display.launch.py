from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # RViz 설정 파일 경로 인자
        # DeclareLaunchArgument(
        #     name='rvizconfig',
        #     default_value=PathJoinSubstitution([
        #         FindPackageShare('rtde_handarm2'),  # ROS 2 패키지명으로 수정
        #         'rviz',
        #         'config.rviz'
        #     ]),
        #     description='Path to the RViz config file'
        # ),

        # 로봇 설명 파라미터 (Xacro → URDF 변환)
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{
        #         'robot_description': Command([
        #             'xacro ',
        #             PathJoinSubstitution([
        #                 FindPackageShare('rtde_handarm2'),
        #                 'urdf',
        #                 'robot.xacro'
        #             ])
        #         ])
        #     }]
        # ),

        # RViz 실행
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', LaunchConfiguration('rvizconfig')],
        #     output='screen'
        # ),

        # VR 데이터를 구독하는 노드 실행
        Node(
            package='rtde_handarm2',
            executable='Yoon_UR10e_main',
            name='Yoon_UR10e_main',
            output='screen'
        ),

        # 정적 변환: world → robot_base_joint
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_world_to_robot_base',
        #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot_base_joint']
        # ),

        # 정적 변환: robot_base_joint → workpiece
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_world_to_workpiece',
        #     arguments=['0', '0', '0', '0', '0', '0', 'robot_base_joint', 'workpiece']
        # )
    ])
