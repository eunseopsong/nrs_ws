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
        ]
    )
