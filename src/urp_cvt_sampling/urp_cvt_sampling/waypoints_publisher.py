#!/usr/bin/env python3
import os
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped


class WaypointClickReplay(Node):
    """
    planned_path.txt (또는 동일 형식 파일)에서
      x y z roll pitch yaw
    형식으로 저장된 데이터를 읽어,
    꺾이는 구간(코너)의 특징점만 /clicked_point 로 publish 하는 노드.

    - 토픽: /clicked_point (PointStamped)
    - 파라미터:
        file_path: txt 파일 경로 (기본: ~/urp_waypoints.txt)
        publish_rate_hz: 1초당 몇 개 step publish (기본: 5.0)
        frame_id: 헤더 frame_id (기본: "map")
        loop: true면 끝까지 간 뒤 처음부터 다시 반복 (기본: False)
        angle_threshold_deg: 코너로 판단할 최소 방향 변화각 [deg] (기본: 45도)
    """

    def __init__(self):
        super().__init__("waypoint_click_replay")

        # QoS 설정 (기본 reliable, volatile)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        # 파라미터 선언
        self.declare_parameter("file_path", "~/urp_waypoints.txt")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("loop", False)
        self.declare_parameter("angle_threshold_deg", 80.0)

        file_path_param = str(self.get_parameter("file_path").value)
        self.file_path = str(Path(os.path.expanduser(file_path_param)).resolve())
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.angle_threshold_deg = float(
            self.get_parameter("angle_threshold_deg").value
        )

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

            all_xyz = data[:, :3]  # (N, 3)

            # 코너 특징점만 추출
            self.xyz = self._extract_corner_points(
                all_xyz, angle_threshold_deg=self.angle_threshold_deg
            )
            self.num_steps = self.xyz.shape[0]

        except Exception as e:
            self.get_logger().error(f"Failed to load file {self.file_path}: {e}")
            raise

        self.get_logger().info(
            f"Loaded {all_xyz.shape[0]} original waypoints, "
            f"selected {self.num_steps} corner waypoints "
            f"from {self.file_path}"
        )

        # /clicked_point 퍼블리셔
        self.pub_clicked = self.create_publisher(PointStamped, "/clicked_point", qos)

        # 상태 변수
        self.current_idx = 0
        self.ready_to_publish = False
        self.visualized = False

        # 타이머 생성
        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.timer = self.create_timer(period, self._timer_cb)

    # -----------------------------
    # 코너 추출 함수
    # -----------------------------
    def _extract_corner_points(self, xyz: np.ndarray, angle_threshold_deg: float):
        """
        xyz : (N,3) array
        angle_threshold_deg: 연속한 두 세그먼트 방향 차이가 이 각도보다 크면 코너로 판단

        반환: corner_xyz (M,3)
        """
        N = xyz.shape[0]
        if N <= 2:
            return xyz.copy()

        # 1) xy 평면에서 연속한 세그먼트 벡터
        xy = xyz[:, :2]              # (N,2)
        seg = np.diff(xy, axis=0)    # (N-1,2)

        angles = np.arctan2(seg[:, 1], seg[:, 0])  # (N-1,)

        # 2) 세그먼트 각도 차이
        dtheta = np.diff(angles)  # (N-2,)
        dtheta = (dtheta + np.pi) % (2.0 * np.pi) - np.pi  # -pi~pi

        th = np.deg2rad(angle_threshold_deg)
        corner_mask = np.abs(dtheta) > th
        corner_indices = np.where(corner_mask)[0] + 1  # i+1가 코너

        # 3) 시작/끝 포함
        selected = np.concatenate(([0], corner_indices, [N - 1]))
        selected = np.unique(selected)

        self.get_logger().info(
            f"Corner extraction: N={N}, corners={len(corner_indices)}, "
            f"selected={len(selected)} (including start/end)."
        )

        return xyz[selected, :]

    # -----------------------------
    # 타이머 콜백: publish
    # -----------------------------
    def _timer_cb(self):
        # subscriber 붙었는지 확인
        if not self.ready_to_publish:
            n_sub = self.pub_clicked.get_subscription_count()
            if n_sub == 0:
                return
            self.ready_to_publish = True
            self.get_logger().info(
                f"Found {n_sub} subscriber(s) on /clicked_point. Start replay."
            )

        # 모든 step을 보냈는지 확인
        if self.current_idx >= self.num_steps:
            if self.loop:
                self.get_logger().info(
                    f"Reached end of waypoints ({self.num_steps}). Looping back to start."
                )
                self.current_idx = 0
            else:
                if self.current_idx == self.num_steps:
                    self.get_logger().info(
                        f"All {self.num_steps} corner waypoints published. Stopping timer."
                    )
                    self.timer.cancel()
                    # publish가 끝난 뒤 3D 시각화 (한 번만)
                    if not self.visualized:
                        self.visualized = True
                        self.visualize_waypoints_3d()
                return

        x, y, z = self.xyz[self.current_idx]

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)

        self.pub_clicked.publish(msg)

        step_idx = self.current_idx + 1
        self.get_logger().info(
            f"step {step_idx}/{self.num_steps}: {x:.6f} {y:.6f} {z:.6f}"
        )

        self.current_idx += 1

    # -----------------------------
    # 3D 시각화 함수 (publish 완료 후 호출)
    # -----------------------------
    def visualize_waypoints_3d(self):
        """
        publish 한 코너 waypoint(self.xyz)만 3D 공간에 시각화.
        """
        if self.xyz is None or self.xyz.shape[0] == 0:
            self.get_logger().warn("No waypoints to visualize.")
            return

        xyz = self.xyz
        x = xyz[:, 0]
        y = xyz[:, 1]
        z = xyz[:, 2]

        self.get_logger().info("Visualizing corner waypoints in 3D...")

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # 궤적 라인 + 특징점
        ax.plot(x, y, z, "-o", markersize=4)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Corner Waypoints (Published Points)")

        ax.view_init(elev=30, azim=45)
        plt.tight_layout()
        plt.show()


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
