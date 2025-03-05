#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # Create a JointState message
        self.joint_state = JointState()
        self.joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        num_joints = len(self.joint_state.name)
        self.joint_state.position = [0.0] * num_joints
        self.joint_state.velocity = [0.0] * num_joints  # Ensure velocity is initialized
        self.joint_state.effort = [0.0] * num_joints    # Ensure effort is initialized

        # Position control the robot to wiggle around each joint
        self.default_joints = np.array([0.0, -1.16, 0.0, -2.3, 0.0, 1.6])
        self.motion_range = np.array([0.5] * num_joints)  # 0.5 radian
        self.time_start = time.time()

        timer_period = 0.01  # seconds (100Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        offset = np.sin(time.time() - self.time_start) * self.motion_range
        joint_position = self.default_joints + offset
        self.joint_state.position = joint_position.tolist()
        self.joint_state.velocity = [0.0] * len(self.joint_state.position)  # Ensure velocity is consistent
        self.joint_state.effort = [0.0] * len(self.joint_state.position)    # Ensure effort is consistent
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    ros2_publisher = JointCommandPublisher()
    rclpy.spin(ros2_publisher)

    # Destroy the node explicitly
    ros2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
