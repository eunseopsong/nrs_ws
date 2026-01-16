#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench
from rclpy.qos import qos_profile_sensor_data


class Demo9DRecorder(Node):
    """
    Subscribe:
      - /calibrated_pose (Float64MultiArray): [x y z wx wy wz]  (m, rad)
      - /ftsensor/measured_Cvalue (Wrench): force.{x,y,z} (N)

    Save row:
      x y z wx wy wz fx fy fz
    Unit:
      x,y,z: mm (m->mm), w: rad, f: N

    Episode rule:
      start: |fx| >= 10 AND pose_ready
      stop : |fy| >= 10 (recording 중)

    Save path:
      ~/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/cmd_continue9D.txt

    Important fixes:
      - Do NOT start if pose not received (prevents empty file when VR/pose missing)
      - Force-append 1 line at START and STOP (prevents lines=0 even if timer didn't run yet)
      - Use sensor_data QoS for robust reception
    """

    def __init__(self):
        super().__init__("demo_9d_recorder")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("pose_topic", "/calibrated_pose")
        self.declare_parameter("ft_topic", "/ftsensor/measured_Cvalue")
        self.declare_parameter(
            "save_path",
            os.path.expanduser(
                "~/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/cmd_continue9D.txt"
            ),
        )
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("start_fx_abs_th", 10.0)
        self.declare_parameter("stop_fy_abs_th", 10.0)
        self.declare_parameter("require_fresh_sec", 0.5)  # 살짝 여유 줌(0.2->0.5)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.ft_topic = str(self.get_parameter("ft_topic").value)
        self.save_path = str(self.get_parameter("save_path").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.start_fx_abs_th = float(self.get_parameter("start_fx_abs_th").value)
        self.stop_fy_abs_th = float(self.get_parameter("stop_fy_abs_th").value)
        self.require_fresh_sec = float(self.get_parameter("require_fresh_sec").value)

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

        # ---------------------------
        # Latest data
        # ---------------------------
        self.latest_pose = None  # [x,y,z,wx,wy,wz]
        self.latest_ft = None    # [fx,fy,fz]
        self.last_pose_time = None
        self.last_ft_time = None

        # Episode state
        self.recording = False
        self.finished = False
        self.rows = []

        # ---------------------------
        # Subscribers (sensor qos)
        # ---------------------------
        self.sub_pose = self.create_subscription(
            Float64MultiArray,
            self.pose_topic,
            self.cb_pose,
            qos_profile_sensor_data,
        )
        self.sub_ft = self.create_subscription(
            Wrench,
            self.ft_topic,
            self.cb_ft,
            qos_profile_sensor_data,
        )

        # Timer
        period = 1.0 / max(1e-6, self.rate_hz)
        self.timer = self.create_timer(period, self.cb_timer)

        self.get_logger().info("9D_demonstration.py started")
        self.get_logger().info(f"pose_topic = {self.pose_topic} (Float64MultiArray len=6)")
        self.get_logger().info(f"ft_topic   = {self.ft_topic} (geometry_msgs/Wrench)")
        self.get_logger().info(f"save_path  = {self.save_path}")
        self.get_logger().info(
            f"episode rule: start=|fx|>={self.start_fx_abs_th} AND pose_ready, stop=|fy|>={self.stop_fy_abs_th}"
        )
        self.get_logger().info(f"rate_hz={self.rate_hz}, require_fresh_sec={self.require_fresh_sec}")

    # ---------------------------
    # Utils
    # ---------------------------
    def _stamp_str(self):
        t = self.get_clock().now().to_msg()
        return f"{t.sec}.{t.nanosec:09d}"

    def _age_sec(self, t):
        if t is None:
            return 1e9
        now = self.get_clock().now()
        return (now - t).nanoseconds / 1e9

    def _fresh_pose(self) -> bool:
        return (self.latest_pose is not None) and (self._age_sec(self.last_pose_time) <= self.require_fresh_sec)

    def _fresh_ft(self) -> bool:
        return (self.latest_ft is not None) and (self._age_sec(self.last_ft_time) <= self.require_fresh_sec)

    def _append_row_if_ready(self) -> bool:
        """Append one line if pose & ft are fresh. Return True if appended."""
        if not (self._fresh_pose() and self._fresh_ft()):
            return False

        x_m, y_m, z_m, wx, wy, wz = self.latest_pose
        fx, fy, fz = self.latest_ft

        # m -> mm
        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0
        z_mm = z_m * 1000.0

        line = (
            f"{x_mm:.10f} {y_mm:.10f} {z_mm:.10f} "
            f"{wx:.10f} {wy:.10f} {wz:.10f} "
            f"{fx:.10f} {fy:.10f} {fz:.10f}"
        )
        self.rows.append(line)
        return True

    # ---------------------------
    # Callbacks
    # ---------------------------
    def cb_pose(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            self.get_logger().warn("Invalid /calibrated_pose length (expected 6)")
            return
        self.latest_pose = [float(v) for v in msg.data]
        self.last_pose_time = self.get_clock().now()

    def cb_ft(self, msg: Wrench):
        fx = float(msg.force.x)
        fy = float(msg.force.y)
        fz = float(msg.force.z)

        self.latest_ft = [fx, fy, fz]
        self.last_ft_time = self.get_clock().now()

        if self.finished:
            return

        abs_fx_over_start = abs(fx) >= self.start_fx_abs_th
        abs_fy_over_stop = abs(fy) >= self.stop_fy_abs_th

        # ✅ START: pose가 실제로 들어온 상태에서만 시작
        if abs_fx_over_start and (not self.recording) and self._fresh_pose():
            self.recording = True
            self.rows.clear()
            self.get_logger().info(
                f"[START] t={self._stamp_str()} | fx={fx:.3f} fy={fy:.3f} fz={fz:.3f} | pose_age={self._age_sec(self.last_pose_time):.3f}s"
            )
            # ✅ 시작 순간 즉시 1줄 기록(보장)
            ok = self._append_row_if_ready()
            if not ok:
                self.get_logger().warn(
                    f"[START] but append failed (pose_fresh={self._fresh_pose()}, ft_fresh={self._fresh_ft()})"
                )
            return

        # ✅ STOP: recording 중일 때만
        if abs_fy_over_stop and self.recording:
            # ✅ 종료 순간도 1줄 더 남기고 저장(보장)
            self._append_row_if_ready()
            self.get_logger().info(
                f"[STOP ] t={self._stamp_str()} | fy={fy:.3f} (|fy|th={self.stop_fy_abs_th:.3f}) | lines={len(self.rows)} | saving -> {self.save_path}"
            )
            self._save_and_finish()
            return

    # ---------------------------
    # Timer: record every step
    # ---------------------------
    def cb_timer(self):
        if self.finished or (not self.recording):
            return
        self._append_row_if_ready()

    def _save_and_finish(self):
        with open(self.save_path, "w") as f:
            for ln in self.rows:
                f.write(ln + "\n")

        self.finished = True
        self.recording = False
        self.get_logger().info(
            f"[SAVED] t={self._stamp_str()} | wrote {len(self.rows)} lines -> {self.save_path}"
        )

    def destroy_node(self):
        # safety partial save
        if (not self.finished) and self.recording and (len(self.rows) > 0):
            try:
                with open(self.save_path, "w") as f:
                    for ln in self.rows:
                        f.write(ln + "\n")
                self.get_logger().warn(
                    f"[PARTIAL-SAVE] Interrupted. Saved {len(self.rows)} lines -> {self.save_path}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to save on destroy: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Demo9DRecorder()
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
