#!/usr/bin/env python3
import os
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped


def extract_corner_indices(points_xyz: np.ndarray,
                           angle_deg_threshold: float = 45.0):
    """
    3D 점열에서 '꺾이는 구간'의 인덱스만 뽑음.
      - 세 점 (P_{i-1}, P_i, P_{i+1})를 기준으로
        벡터 v1 = P_i - P_{i-1}, v2 = P_{i+1} - P_i
      - v1, v2 사이 각도 θ가 angle_deg_threshold 이상이면
        i를 코너 인덱스로 판정.

    - 연속해서 여러 i가 코너로 나올 수 있어서,
      한 '코너 구간'에서는 각도가 최대인 i만 채택.
    """
    N = points_xyz.shape[0]
    if N <= 2:
        return np.arange(N, dtype=int)

    # 1-step 벡터들
    vec = points_xyz[1:] - points_xyz[:-1]   # (N-1, 3)
    norm = np.linalg.norm(vec, axis=1)

    corner_indices = []

    in_region = False
    best_i = None
    best_theta = -1.0

    for i in range(1, N - 1):
        v1 = vec[i - 1]
        v2 = vec[i]

        n1 = norm[i - 1]
        n2 = norm[i]
        if n1 < 1e-8 or n2 < 1e-8:
            continue  # 너무 짧은 segment는 스킵

        cos_theta = np.dot(v1, v2) / (n1 * n2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        theta_deg = np.degrees(np.arccos(cos_theta))

        if theta_deg >= angle_deg_threshold:
            # 코너 구간 진입 or 유지
            if not in_region:
                in_region = True
                best_i = i
                best_theta = theta_deg
            else:
                # 같은 코너 구간에서 더 큰 각도면 갱신
                if theta_deg > best_theta:
                    best_theta = theta_deg
                    best_i = i
        else:
            # 코너 구간에서 벗어날 때, 지금까지의 best_i를 저장
            if in_region and best_i is not None:
                corner_indices.append(best_i)
            in_region = False
            best_i = None
            best_theta = -1.0

    # 루프 끝났는데 코너 구간이면 마지막 것도 추가
    if in_region and best_i is not None:
        corner_indices.append(best_i)

    # 항상 처음과 마지막은 포함
    indices = [0] + corner_indices + [N - 1]
    indices = sorted(set(indices))
    return np.array(indices, dtype=int)


class WaypointClickReplay(Node):
    """
    planned_path.txt (또는 동일 형식 파일)에서
      x y z roll pitch yaw
    형식으로 저장된 데이터를 읽어,
    (x, y, z)를 순서대로 /clicked_point 로 publish.

    - 꺾이는 구간(방향이 크게 바뀌는 지점)만 추출해서 publish.
    - publish 종료 후 (loop=False) 3D 플롯으로 시각화 가능.

    파라미터:
      file_path       (str): txt 파일 경로 (기본: ~/urp_waypoints.txt)
      publish_rate_hz (float): 1초당 몇 개 step publish (기본: 5.0)
      frame_id        (str): 헤더 frame_id (기본: "map")
      loop            (bool): True면 끝까지 간 뒤 처음부터 반복
      corner_angle_deg(float): 코너 판정 각도 임계값 [deg] (기본: 45.0)
      enable_plot     (bool): publish 끝나면 3D 플롯 띄울지 (기본: True)
    """

    def __init__(self):
        super().__init__("waypoint_click_replay")

        # QoS 설정
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        # 파라미터 선언
        self.declare_parameter("file_path", "~/urp_waypoints.txt")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("loop", False)
        self.declare_parameter("corner_angle_deg", 45.0)
        self.declare_parameter("enable_plot", True)

        file_path_param = str(self.get_parameter("file_path").value)
        self.file_path = str(Path(os.path.expanduser(file_path_param)).resolve())
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.corner_angle_deg = float(self.get_parameter("corner_angle_deg").value)
        self.enable_plot = bool(self.get_parameter("enable_plot").value)

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

            # 코너 인덱스만 추출
            corner_idxs = extract_corner_indices(
                all_xyz, angle_deg_threshold=self.corner_angle_deg
            )
            self.xyz = all_xyz[corner_idxs]
            self.num_steps = self.xyz.shape[0]

        except Exception as e:
            self.get_logger().error(f"Failed to load file {self.file_path}: {e}")
            raise

        self.get_logger().info(
            f"Loaded {all_xyz.shape[0]} waypoints, "
            f"extracted {self.num_steps} corner points (angle >= {self.corner_angle_deg} deg)."
        )

        # publisher
        self.pub_clicked = self.create_publisher(PointStamped, "/clicked_point", qos)

        self.current_idx = 0
        self.ready_to_publish = False

        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.timer = self.create_timer(period, self._timer_cb)

    def _timer_cb(self):
        # subscriber 체크
        if not self.ready_to_publish:
            n_sub = self.pub_clicked.get_subscription_count()
            if n_sub == 0:
                return
            self.ready_to_publish = True
            self.get_logger().info(
                f"Found {n_sub} subscriber(s) on /clicked_point. Start replay."
            )

        # 끝까지 보냈는지 체크
        if self.current_idx >= self.num_steps:
            if self.loop:
                self.get_logger().info(
                    f"Reached end of waypoints ({self.num_steps}). Looping back to start."
                )
                self.current_idx = 0
            else:
                if self.current_idx == self.num_steps:
                    self.get_logger().info(
                        f"All {self.num_steps} waypoints published. Stopping timer."
                    )
                    # 3D 시각화
                    if self.enable_plot:
                        self.visualize_path_3d()
                self.current_idx += 1
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

        step_idx = self.current_idx + 1
        self.get_logger().info(
            f"step {step_idx}: {x:.6f} {y:.6f} {z:.6f}"
        )

        self.current_idx += 1

    def visualize_path_3d(self):
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except ImportError as e:
            self.get_logger().error(
                f"Failed to import matplotlib for 3D visualization: {e}"
            )
            return

        if self.xyz is None or self.xyz.shape[0] == 0:
            self.get_logger().warn("No waypoints to visualize.")
            return

        xs = self.xyz[:, 0]
        ys = self.xyz[:, 1]
        zs = self.xyz[:, 2]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(xs, ys, zs, marker="o")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Corner Waypoints (3D)")

        self.get_logger().info("Showing 3D waypoint visualization window.")
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
