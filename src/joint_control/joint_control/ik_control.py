#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation as R

class IKController(Node):
    def __init__(self):
        super().__init__('ik_controller')
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # Define UR10 link lengths (meters)
        self.L1 = 0.127    # Base to Shoulder
        self.L2 = 0.612    # Shoulder to Elbow
        self.L3 = 0.5723   # Elbow to Wrist 1
        self.L4 = 0.163941 # Wrist 1 to Wrist 2
        self.L5 = 0.1157   # Wrist 2 to Wrist 3
        self.L6 = 0.0922   # Wrist 3 to End Effector

        # Define JointState message
        self.joint_state = JointState()
        self.joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.num_joints = len(self.joint_state.name)
        self.joint_state.position = [0.0] * self.num_joints
        self.joint_state.velocity = [0.0] * self.num_joints
        self.joint_state.effort = [0.0] * self.num_joints

        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        """ 주기적으로 JointState 퍼블리시 """
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """ Convert Euler angles to a rotation matrix """
        return R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()

    def inverse_kinematics(self, x, y, z, roll, pitch, yaw):
        """ Compute inverse kinematics using UR10 kinematic model """
        d = np.sqrt(x**2 + y**2) - self.L4 - self.L5 - self.L6  # Effective reach excluding wrist
        h = z - self.L1  # Height from base to EE
        
        # Law of Cosines for theta2
        c2 = (d**2 + h**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        if abs(c2) > 1.0:
            self.get_logger().error("Target out of reach!")
            return None
        s2 = np.sqrt(1 - c2**2)  # Elbow-Up Configuration
        theta2 = np.arctan2(s2, c2)

        # Shoulder and Elbow Joint Calculation
        theta1 = np.arctan2(y, x)
        theta3 = np.arctan2(h, d) - np.arctan2(self.L3 * s2, self.L2 + self.L3 * c2)

        # Compute wrist angles
        wrist_rotation = self.euler_to_rotation_matrix(roll, pitch, yaw)
        theta4 = np.arctan2(wrist_rotation[1, 0], wrist_rotation[0, 0])  # Wrist yaw
        theta5 = np.arctan2(np.sqrt(wrist_rotation[0, 0]**2 + wrist_rotation[1, 0]**2), wrist_rotation[2, 0])  # Wrist pitch
        theta6 = np.arctan2(wrist_rotation[2, 1], wrist_rotation[2, 2])  # Wrist roll

        joints = [theta1, theta3, theta2, theta4, theta5, theta6]

        # Logging IK result
        self.get_logger().info(f"Calculated Joint Angles: {joints}")

        return joints

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        """ Compute inverse kinematics and publish joint commands """
        target_position = [x, y, z]
        rotation_matrix = self.euler_to_rotation_matrix(roll, pitch, yaw)

        # Inverse Kinematics 계산
        joints = self.inverse_kinematics(x, y, z, roll, pitch, yaw)

        if joints is None:
            self.get_logger().error("IK solution is invalid!")
            return

        self.joint_state.position = joints
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f"Moved to EE position: {target_position}, orientation: {roll, pitch, yaw}")

def main(args=None):
    rclpy.init(args=args)
    controller = IKController()
    
    while rclpy.ok():
        try:
            user_input = input("Enter x y z roll pitch yaw (meters & degrees): ")
            x, y, z, roll, pitch, yaw = map(float, user_input.split())
            roll, pitch, yaw = np.radians([roll, pitch, yaw])  # Convert degrees to radians
            controller.move_to_pose(x, y, z, roll, pitch, yaw)
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Invalid input: {e}")
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
