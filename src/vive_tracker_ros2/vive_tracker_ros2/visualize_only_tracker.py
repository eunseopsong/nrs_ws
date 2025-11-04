#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import openvr
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from vive_tracker_ros2.utils import transform_HmdMatrix2npmatrix, matrix_to_pose

class ViveTFBroadcaster(Node):
    def __init__(self):
        super().__init__('vive_tf_broadcaster')
        self.vr_system = openvr.init(openvr.VRApplication_Other)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("child_frame", "vive_tracker")
        self.base_frame = self.get_parameter("base_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.timer = self.create_timer(0.01, self.broadcast_tf)

    def broadcast_tf(self):
        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_GenericTracker:
                pose = poses[i]
                if pose.bDeviceIsConnected and pose.bPoseIsValid:
                    matrix = transform_HmdMatrix2npmatrix(pose.mDeviceToAbsoluteTracking)
                    matrix[:3, :3] = matrix[:3, :3] @ R.from_euler("y", 180, degrees=True).as_matrix()
                    pose_msg = matrix_to_pose(matrix)

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.base_frame
                    t.child_frame_id = self.child_frame
                    t.transform.translation.x = pose_msg.position.x
                    t.transform.translation.y = pose_msg.position.y
                    t.transform.translation.z = pose_msg.position.z
                    t.transform.rotation = pose_msg.orientation
                    self.tf_broadcaster.sendTransform(t)
                    break  # 첫 번째 트래커만 브로드캐스트

def main(args=None):
    rclpy.init(args=args)
    node = ViveTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        openvr.shutdown()

if __name__ == '__main__':
    main()
