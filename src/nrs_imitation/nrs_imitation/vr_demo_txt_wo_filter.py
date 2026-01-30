#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench


class VrDemoTxtRecorder(Node):
    def __init__(self):
        super().__init__("vr_demo_txt_recorder_raw")

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter("pose_topic", "/calibrated_pose")
        self.declare_parameter("force_topic", "/ftsensor/measured_Cvalue")

        self.declare_parameter("record_hz", 125.0)
        self.declare_parameter("require_fresh_sec", 0.2)

        self.declare_parameter("start_abs_fx", 10.0)
        self.declare_parameter("stop_abs_fy", 10.0)

        self.declare_parameter(
            "save_path",
            "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/cmd_continue9D.txt"
        )

        self.declare_parameter("transfer_enable", True)
        self.declare_parameter("remote_user", "nrs_forcecon")
        self.declare_parameter("remote_ip", "192.168.0.151")
        self.declare_parameter(
            "remote_dir",
            "/home/nrs_forcecon/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/"
        )

        # ----------------------------
        # Load params
        # ----------------------------
        self.pose_topic = self.get_parameter("pose_topic").value
        self.force_topic = self.get_parameter("force_topic").value

        self.record_hz = float(self.get_parameter("record_hz").value)
        self.dt = 1.0 / self.record_hz
        self.require_fresh_sec = float(self.get_parameter("require_fresh_sec").value)

        self.start_abs_fx = float(self.get_parameter("start_abs_fx").value)
        self.stop_abs_fy = float(self.get_parameter("stop_abs_fy").value)

        self.save_path = self.get_parameter("save_path").value

        self.transfer_enable = bool(self.get_parameter("transfer_enable").value)
        self.remote_user = self.get_parameter("remote_user").value
        self.remote_ip = self.get_parameter("remote_ip").value
        self.remote_dir = self.get_parameter("remote_dir").value

        # ----------------------------
        # State
        # ----------------------------
        self.latest_pose = None   # [mm, rad]
        self.latest_force = None  # [N]
        self.latest_pose_t = 0.0
        self.latest_force_t = 0.0

        self.episode_active = False
        self.finishing = False

        self.buf_pose = []
        self.buf_force = []

        # ----------------------------
        # ROS
        # ----------------------------
        self.sub_pose = self.create_subscription(
            Float64MultiArray, self.pose_topic, self.cb_pose, 50
        )
        self.sub_force = self.create_subscription(
            Wrench, self.force_topic, self.cb_force, 10
        )
        self.timer = self.create_timer(self.dt, self.cb_timer)

        self.get_logger().info("VR RAW demo recorder initialized.")
        self.get_logger().info(f"Pose topic : {self.pose_topic}")
        self.get_logger().info(f"Force topic: {self.force_topic}")
        self.get_logger().info(
            f"Start |fx| >= {self.start_abs_fx}, Stop |fy| >= {self.stop_abs_fy}"
        )

    # ----------------------------
    # Callbacks
    # ----------------------------
    def cb_pose(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        x, y, z, wx, wy, wz = msg.data[:6]
        self.latest_pose = np.array(
            [1000.0 * x, 1000.0 * y, 1000.0 * z, wx, wy, wz],
            dtype=np.float64,
        )
        self.latest_pose_t = time.time()

    def cb_force(self, msg: Wrench):
        fx = float(msg.force.x)
        fy = float(msg.force.y)
        fz = float(msg.force.z)

        self.latest_force = np.array([fx, fy, fz], dtype=np.float64)
        self.latest_force_t = time.time()

        if self.finishing:
            return

        # Start condition
        if (not self.episode_active) and abs(fx) >= self.start_abs_fx:
            self.episode_active = True
            self.buf_pose.clear()
            self.buf_force.clear()
            self.get_logger().info("=== EPISODE STARTED ===")
            return

        # Stop condition
        if self.episode_active and abs(fy) >= self.stop_abs_fy:
            self.get_logger().info("=== EPISODE ENDED ===")
            self.finish_episode()

    def cb_timer(self):
        if not self.episode_active or self.finishing:
            return

        now = time.time()

        if self.latest_pose is None or self.latest_force is None:
            return
        if (now - self.latest_pose_t) > self.require_fresh_sec:
            return
        if (now - self.latest_force_t) > self.require_fresh_sec:
            return

        self.buf_pose.append(self.latest_pose.copy())
        self.buf_force.append(self.latest_force.copy())

    # ----------------------------
    # Finish
    # ----------------------------
    def finish_episode(self):
        if self.finishing:
            return
        self.finishing = True
        self.episode_active = False

        if len(self.buf_pose) < 5:
            self.get_logger().warn("Episode too short. Discarded.")
            rclpy.shutdown()
            return

        P = np.asarray(self.buf_pose)
        F = np.asarray(self.buf_force)
        out = np.hstack([P, F])  # (N,9)

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        with open(self.save_path, "w") as f:
            for row in out:
                f.write("\t".join(f"{v:.6f}" for v in row) + "\n")

        self.get_logger().info(
            f"Saved RAW demo: {self.save_path}  (rows={out.shape[0]})"
        )

        if self.transfer_enable:
            self.transfer_file()

        self.get_logger().info("Shutdown.")
        rclpy.shutdown()

    def transfer_file(self):
        try:
            dst = f"{self.remote_user}@{self.remote_ip}:{self.remote_dir}"
            subprocess.run(["scp", self.save_path, dst], check=True)
            self.get_logger().info("SCP transfer success.")
        except Exception as e:
            self.get_logger().error(f"SCP transfer failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VrDemoTxtRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
