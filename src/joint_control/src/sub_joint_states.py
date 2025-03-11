#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState
import numpy as np

class MyJointStateSubscriber(Node):
    def __init__(self):
        super().__init__('my_joint_state_subscriber')
        qos_profile = QoSProfile(depth=10)
        # Create a subscription to /isaac_joint_states
        # Adjust QoS if necessary (see QoS note below)
        self.sub = self.create_subscription(
            JointState,
            '/isaac_joint_states',
            self.joint_state_callback,
            qos_profile
        )

    def joint_state_callback(self, msg: JointState):
        # Convert each field to a separate NumPy array (6 elements each if you have 6 joints)
        positions = np.array(msg.position)
        velocities = np.array(msg.velocity)
        efforts = np.array(msg.effort)

        # Now you have three separate arrays, each length 6
        # Do whatever you need with them. For example, print them:
        self.get_logger().info(f"Positions: {positions}")
        self.get_logger().info(f"Velocities: {velocities}")
        self.get_logger().info(f"Efforts: {efforts}\n")

def main(args=None):
    rclpy.init(args=args)
    node = MyJointStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
