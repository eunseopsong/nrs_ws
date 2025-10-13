#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import csv

class UR10eGainTuner(Node):
    def __init__(self):
        super().__init__('ur10e_gain_tuner')

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', qos_profile)
        self.subscription = self.create_subscription(JointState, '/isaac_joint_states', self.joint_state_callback, qos_profile)

        # Joint info
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        self.num_joints = len(self.joint_names)

        # Parameters
        self.declare_parameter('joint_index', 0)
        self.declare_parameter('mode', 'sine')
        self.declare_parameter('amplitude', np.pi / 4)  # ±45°
        self.declare_parameter('frequency', 0.5)
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('visualize', True)

        self.joint_index = self.get_parameter('joint_index').value
        self.mode = self.get_parameter('mode').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.duration = self.get_parameter('duration').value
        self.visualize = self.get_parameter('visualize').value

        self.get_logger().info(
            f"▶ Gain tuning start: joint={self.joint_index}, mode={self.mode}, amp={self.amplitude:.3f}, freq={self.frequency}, duration={self.duration}"
        )

        # 초기 joint 위치 (deg → rad)
        self.initial_pos_rad = np.radians([0, -90, -90, -90, 90, 0])

        # Data logs
        self.ref_positions, self.ref_velocities = [], []
        self.meas_positions, self.meas_velocities = [], []
        self.time_log = []

        # JointState message
        self.joint_state = JointState()
        self.joint_state.name = self.joint_names
        self.joint_state.position = list(self.initial_pos_rad)
        self.joint_state.velocity = [0.0] * self.num_joints
        self.joint_state.effort = [0.0] * self.num_joints

        self.meas_joint_pos = [0.0] * self.num_joints
        self.meas_joint_vel = [0.0] * self.num_joints

        # Timer
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.005, self.update_command)

    def joint_state_callback(self, msg):
        """구독한 measured joint state 저장"""
        self.meas_joint_pos = list(msg.position)
        self.meas_joint_vel = list(msg.velocity)

    def update_command(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        if t > self.duration:
            self.finish_experiment()
            return

        # ===== Reference trajectory =====
        q0 = self.initial_pos_rad[self.joint_index]
        if self.mode == "sine":
            pos_ref = q0 + self.amplitude * np.sin(2 * np.pi * self.frequency * t)
        elif self.mode == "step":
            pos_ref = q0 + (self.amplitude if (int(t) % 2 == 0) else -self.amplitude)
        else:
            pos_ref = q0

        vel_ref = 0.0  # reference velocity = 0

        # ===== Publish command =====
        self.joint_state.position[self.joint_index] = pos_ref
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

        # ===== Log data =====
        self.time_log.append(t)
        self.ref_positions.append(pos_ref)
        self.ref_velocities.append(vel_ref)
        self.meas_positions.append(self.meas_joint_pos[self.joint_index])
        self.meas_velocities.append(self.meas_joint_vel[self.joint_index])

    def finish_experiment(self):
        self.destroy_timer(self.timer)
        self.get_logger().info("\n========== Gain Tuning Summary ==========")

        rms_error = np.sqrt(np.mean((np.array(self.ref_positions) - np.array(self.meas_positions)) ** 2))
        steady_state_error = abs(self.ref_positions[-1] - self.meas_positions[-1])

        self.get_logger().info(f"Joint index: {self.joint_index}")
        self.get_logger().info(f"Mode: {self.mode}")
        self.get_logger().info(f"RMS Error: {rms_error:.6f} rad")
        self.get_logger().info(f"Steady-state Error: {steady_state_error:.6f} rad")

        filename = f"gain_tuning_joint{self.joint_index}_{self.mode}.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time", "ref_pos", "ref_vel", "meas_pos", "meas_vel"])
            for i in range(len(self.time_log)):
                writer.writerow([
                    self.time_log[i],
                    self.ref_positions[i],
                    self.ref_velocities[i],
                    self.meas_positions[i],
                    self.meas_velocities[i]
                ])
        self.get_logger().info(f"Saved tuning data to {filename}")

        if self.visualize:
            self.visualize_results()

        rclpy.shutdown()

    def visualize_results(self):
        t = np.array(self.time_log)
        ref_pos = np.array(self.ref_positions)
        meas_pos = np.array(self.meas_positions)
        ref_vel = np.array(self.ref_velocities)
        meas_vel = np.array(self.meas_velocities)

        plt.figure(figsize=(8, 6))
        plt.subplot(2, 1, 1)
        plt.plot(t, ref_pos, 'b-', label='Reference Pos')
        plt.plot(t, meas_pos, 'r--', label='Measured Pos')
        plt.ylabel('Position [rad]')
        plt.legend()
        plt.title(f'Joint {self.joint_index} Tracking ({self.mode})')

        plt.subplot(2, 1, 2)
        plt.plot(t, ref_vel, 'b-', label='Reference Vel (0)')
        plt.plot(t, meas_vel, 'r--', label='Measured Vel')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [rad/s]')
        plt.legend()

        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = UR10eGainTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
