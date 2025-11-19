#!/usr/bin/env python3
import os
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped


class WaypointClickReplay(Node):
    """
    planned_path.txt (또는 동일 형식 파일)에서
      x y z roll pitch yaw
    형식으로 저장된 데이터를 읽어,
    각 행의 (x, y, z)를 순서대로 /clicked_point로 publish 하는 노드.

    - 토픽: /clicked_point (PointStamped)
    - 파라미터:
        file_path: txt 파일 경로 (기본: ~/planned_path.txt)
        publish_rate_hz: 1초당 몇 개 step publish (기본: 5.0)
        frame_id: 헤더 frame_id (기본: "map")
        loop: true면 끝까지 간 뒤 처음부터 다시 반복 (기본: False)
    """

    def __init__(self):
        super().__init__("waypoint_click_replay")

        # QoS 설정 (기본 reliable, volatile)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        # 파라미터 선언
        self.declare_parameter("file_path", "~/waypoints_publisher.txt")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("loop", False)

        file_path_param = str(self.get_parameter("file_path").value)
        self.file_path = str(Path(os.path.expanduser(file_path_param)).resolve())
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.loop = bool(self.get_parameter("loop").value)

        # 파일 로드
        if not os.path.exists(self.file_path):
            self.get_logger().error(f"File not found: {self.file_path}")
            raise FileNotFoundError(self.file_path)

        try:
            data = np.loadtxt(self.file_path)
            if data.ndim == 1:
                data = data.reshape(1, -1)

            if data.shape[1] < 3:
                raise ValueError(
                    f"File {self.file_path} must have at least 3 columns (x y z ...), "
                    f"but got {data.shape[1]}"
                )

            self.xyz = data[:, :3]  # (N, 3)
            self.num_steps = self.xyz.shape[0]
        except Exception as e:
            self.get_logger().error(f"Failed to load file {self.file_path}: {e}")
            raise

        self.get_logger().info(
            f"Loaded {self.num_steps} waypoints from {self.file_path}"
        )

        # /clicked_point 퍼블리셔
        self.pub_clicked = self.create_publisher(PointStamped, "/clicked_point", qos)

        # 현재 step index
        self.current_idx = 0

        # 아직 subscriber가 붙었는지 여부
        self.ready_to_publish = False

        # 타이머 생성
        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.timer = self.create_timer(period, self._timer_cb)

    def _timer_cb(self):
        # 1) subscriber 붙었는지 확인
        if not self.ready_to_publish:
            n_sub = self.pub_clicked.get_subscription_count()
            if n_sub == 0:
                # 아직 /clicked_point 구독 노드 없음 → 기다림
                return
            self.ready_to_publish = True
            self.get_logger().info(
                f"Found {n_sub} subscriber(s) on /clicked_point. Start replay."
            )

        # 2) 모든 step을 보냈는지 확인
        if self.current_idx >= self.num_steps:
            if self.loop:
                # 루프 모드 → 처음으로 되돌림
                self.get_logger().info(
                    f"Reached end of waypoints ({self.num_steps}). Looping back to start."
                )
                self.current_idx = 0
            else:
                # 한 번만 재생
                if self.current_idx == self.num_steps:
                    self.get_logger().info(
                        f"All {self.num_steps} waypoints published. Stopping timer."
                    )
                self.current_idx += 1  # 로그 한 번만 찍히도록
                self.timer.cancel()
                return

        x, y, z = self.xyz[self.current_idx]

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)

        self.pub_clicked.publish(msg)

        # 디버그 출력: "step index x y z"
        step_idx = self.current_idx + 1  # 1-based index
        self.get_logger().info(
            f"step {step_idx}: {x:.6f} {y:.6f} {z:.6f}"
        )

        self.current_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = WaypointClickReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
