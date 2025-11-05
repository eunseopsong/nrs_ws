#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

VIVE_TOPIC = '/vive_tracker_ros/LHR_B3BA320E/calibrated_pose'


class ServoUR10e(Node):
    def __init__(self):
        # 여기 이름이 rqt_graph에 보이는 노드 이름
        super().__init__('servo_ur10e')

        self.joint_pub = self.create_publisher(
            JointState,
            '/isaac_joint_command',
            10
        )

        self.vive_sub = self.create_subscription(
            Odometry,
            VIVE_TOPIC,
            self.vive_callback,
            10
        )

        self.init_pos = None
        self.init_rot = None

    def vive_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=float)

        o = msg.pose.pose.orientation
        quat_xyzw = np.array([o.x, o.y, o.z, o.w], dtype=float)
        rot = R.from_quat(quat_xyzw)

        if self.init_pos is None:
            self.init_pos = pos
            self.init_rot = rot
            self.get_logger().info("servo_ur10e: initial pose stored")
            return

        delta_pos = pos - self.init_pos
        rot_delta = self.init_rot.inv() * rot
        delta_rpy = rot_delta.as_euler('xyz', degrees=False)

        self.get_logger().info(
            f"delta pos: {delta_pos}, delta rpy: {delta_rpy}"
        )

    def publish_joints(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'right_joint_1', 'right_joint_2', 'right_joint_3',
            'right_joint_4', 'right_joint_5', 'right_joint_6'
        ]
        msg.position = list(q)
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoUR10e()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
