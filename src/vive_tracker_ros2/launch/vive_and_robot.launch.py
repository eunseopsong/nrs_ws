from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vive_tracker_ros2",
                executable="vive_tracker_node",
                name="vive_tracker_node",
                output="screen",
                parameters=[
                    {
                        "base_frame": "base_link",
                        "child_frame": "vive_tracker",
                        "publish_tf": True
                    },
                ],
            ),
            # 로봇 컨트롤러 노드 추가
            Node(
                package="vive_tracker_ros2",
                executable="robot_controller",  # 실행 파일 이름 (엔트리 포인트로 등록해야 함)
                name="robot_tcp_controller",
                output="screen",
                parameters=[
                    {
                        "max_linear_velocity": 0.1,  # m/s
                        "max_angular_velocity": 0.5,  # rad/s
                        "enable_robot_control": False,  # 기본값은 안전을 위해 false
                        "robot_base_frame": "base",
                        "robot_tcp_frame": "tool0"
                    },
                ],
            ),
        ]
    )
