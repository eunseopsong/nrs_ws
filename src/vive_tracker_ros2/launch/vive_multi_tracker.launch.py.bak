from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 런치 파라미터 선언
    publish_tf = LaunchConfiguration('publish_tf', default='true')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    child_frame = LaunchConfiguration('child_frame', default='vive_tracker')
    
    return LaunchDescription([
        # 런치 파라미터 정의
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='TF 발행 여부'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='기준 프레임'
        ),
        DeclareLaunchArgument(
            'child_frame',
            default_value='vive_tracker',
            description='트래커 프레임 이름 (기본값, 각 트래커는 시리얼에 따라 다른 이름 사용)'
        ),
        
        # 바이브 트래커 노드
        Node(
            package="vive_tracker_ros2",
            executable="vive_tracker_node",
            name="vive_tracker_node",
            output="screen",
            parameters=[
                {
                    "publish_tf": publish_tf,
                    "base_frame": base_frame,
                    "child_frame": child_frame
                },
            ],
        ),
    ])
