import math
import os
from pathlib import Path as FsPath

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import Marker


class TSPPathPlanner(Node):
    """
    CVT/PDS 등으로 생성된 waypoints(3D + orientation)를 입력으로 받아,
    거리 + 회전비용이 최소가 되도록 TSP(근사) 경로를 생성하는 노드.

    비용:
        c_ij = w_dist * ||p_j - p_i|| + w_rot * ||Δrpy_ij||

    알고리즘 개요:
        1) 시작점: XY 평면에서 '왼쪽 하단' 포인트 선택
        2) Nearest-Neighbor로 초기 경로 생성
        3) 2-opt 로컬 서치로 비용 감소가 없을 때까지 반복
        4) nav_msgs/Path + Marker 퍼블리시
        5) planned_path.txt (x y z roll pitch yaw) 저장
    """

    def __init__(self):
        super().__init__('path_planner_node')

        # QoS (RViz 나중에 켜도 보이도록)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # --- 파라미터 ---
        self.declare_parameter('waypoints_topic', '/cvt_sampler_node/waypoints')
        self.declare_parameter('path_topic',      '~/planned_path')
        self.declare_parameter('frame_id',        'map')

        self.declare_parameter('w_dist',          1.0)   # [cost / m]
        self.declare_parameter('w_rot',           0.1)   # [cost / rad]
        self.declare_parameter('max_2opt_iter',   50)
        self.declare_parameter('smooth_window',   0)     # 홀수, 3 이상이면 3D 이동평균
        self.declare_parameter('publish_marker',  True)
        self.declare_parameter('save_txt_path',   '~/planned_path.txt')

        self.waypoints_topic = str(self.get_parameter('waypoints_topic').value)
        self.path_topic      = str(self.get_parameter('path_topic').value)
        self.frame_id        = str(self.get_parameter('frame_id').value)

        self.w_dist          = float(self.get_parameter('w_dist').value)
        self.w_rot           = float(self.get_parameter('w_rot').value)
        self.max_2opt_iter   = int(self.get_parameter('max_2opt_iter').value)
        self.smooth_window   = int(self.get_parameter('smooth_window').value)
        self.publish_marker  = bool(self.get_parameter('publish_marker').value)
        self.save_txt_path   = str(self.get_parameter('save_txt_path').value)

        # --- ROS 통신 ---
        self.sub_wp   = self.create_subscription(
            PoseArray, self.waypoints_topic, self.on_waypoints, qos
        )
        self.pub_path = self.create_publisher(RosPath, self.path_topic, qos)
        self.pub_marker = (
            self.create_publisher(Marker, '~/path_marker', qos)
            if self.publish_marker else None
        )

        self.get_logger().info(
            f"TSPPathPlanner ready. listen={self.waypoints_topic}, "
            f"publish={self.path_topic}, w_dist={self.w_dist}, w_rot={self.w_rot}, "
            f"save_txt={self.save_txt_path}"
        )

    # ============================================================
    # 콜백
    # ============================================================
    def on_waypoints(self, msg: PoseArray):
        N = len(msg.poses)
        if N < 2:
            self.get_logger().warn('Not enough waypoints (<2). Skip.')
            return

        # --- PoseArray -> numpy ---
        pos = np.zeros((N, 3), dtype=float)
        quat = np.zeros((N, 4), dtype=float)  # w, x, y, z
        rpy  = np.zeros((N, 3), dtype=float)

        for i, p in enumerate(msg.poses):
            pos[i, 0] = p.position.x
            pos[i, 1] = p.position.y
            pos[i, 2] = p.position.z
            qw = p.orientation.w
            qx = p.orientation.x
            qy = p.orientation.y
            qz = p.orientation.z
            quat[i, :] = [qw, qx, qy, qz]
            roll, pitch, yaw = self._quat_to_rpy(qx, qy, qz, qw)
            rpy[i, :] = [roll, pitch, yaw]

        # --- 0) 시작점: XY 평면에서 '왼쪽 하단' 포인트 선택 ---
        xy = pos[:, :2]
        min_y = np.min(xy[:, 1])
        cand_y = np.where(np.isclose(xy[:, 1], min_y, atol=1e-6))[0]
        start_idx = int(cand_y[np.argmin(xy[cand_y, 0])])

        # --- 1) 비용 행렬 (거리 + Δrpy) ---
        cost_mat = self._build_cost_matrix(pos, rpy)

        # --- 2) Nearest-Neighbor 초기 경로 (start 고정) ---
        route = self._nearest_neighbor_route(cost_mat, start_idx)

        # --- 3) 2-opt 로컬 최적화 (open path) ---
        route = self._two_opt(route, cost_mat, max_iter=self.max_2opt_iter)

        # --- 4) 최종 경로 시퀀스 ---
        pos_opt = pos[route]   # (M,3)
        rpy_opt = rpy[route]   # (M,3)
        quat_opt = quat[route] # (M,4)

        # --- 5) 3D 이동평균 스무딩(선택, 위치만) ---
        if self.smooth_window >= 3 and self.smooth_window % 2 == 1 \
           and len(pos_opt) > self.smooth_window:
            pos_opt = self._moving_average_3d(pos_opt, self.smooth_window)

        # --- 6) 거리/회전/총 비용 계산 (경로 순서 기준) ---
        L_total, R_total, J_total = self._route_cost_from_rpy(pos_opt, rpy_opt)

        # --- 7) nav_msgs/Path 생성 ---
        ros_path = RosPath()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = self.frame_id

        for i in range(pos_opt.shape[0]):
            ps = PoseStamped()
            ps.header = ros_path.header
            ps.pose.position.x = float(pos_opt[i, 0])
            ps.pose.position.y = float(pos_opt[i, 1])
            ps.pose.position.z = float(pos_opt[i, 2])

            # orientation은 원래 waypoint의 것 사용 (route 순서대로)
            qw, qx, qy, qz = quat_opt[i]
            ps.pose.orientation.w = float(qw)
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)

            ros_path.poses.append(ps)

        self.pub_path.publish(ros_path)

        # --- 8) Marker 시각화 ---
        if self.pub_marker is not None:
            mk = Marker()
            mk.header = ros_path.header
            mk.ns = 'tsp_path'
            mk.id = 1
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.003
            mk.color.a = 1.0
            mk.color.r = 0.0
            mk.color.g = 1.0
            mk.color.b = 0.0
            mk.points = []
            for ps in ros_path.poses:
                pt = Point()
                pt.x = ps.pose.position.x
                pt.y = ps.pose.position.y
                pt.z = ps.pose.position.z
                mk.points.append(pt)
            self.pub_marker.publish(mk)

        # --- 9) planned_path.txt 저장 (x y z roll pitch yaw) ---
        self._save_path_txt(pos_opt, rpy_opt, self.save_txt_path)

        # --- 10) 로그 출력 ---
        self.get_logger().info(
            f"[TSPPathPlanner] N={len(route)}, start_idx={start_idx}, "
            f"L_total={L_total:.4f} m, R_total={R_total:.4f} rad, "
            f"J_total={J_total:.4f} "
            f"(w_dist={self.w_dist}, w_rot={self.w_rot})"
        )

    # ============================================================
    # 비용 행렬 & 경로 비용
    # ============================================================
    def _build_cost_matrix(self, pos: np.ndarray, rpy: np.ndarray) -> np.ndarray:
        """
        c_ij = w_dist * ||p_j - p_i|| + w_rot * ||Δrpy_ij||
        (yaw는 -pi~pi 래핑)
        """
        N = pos.shape[0]
        cost = np.zeros((N, N), dtype=float)

        for i in range(N):
            for j in range(N):
                if i == j:
                    cost[i, j] = math.inf
                    continue
                dp = pos[j] - pos[i]
                dist = float(np.linalg.norm(dp))

                dr = rpy[j] - rpy[i]
                dr[2] = self._wrap_angle(dr[2])  # yaw wrap
                drot = float(np.linalg.norm(dr))

                cost[i, j] = self.w_dist * dist + self.w_rot * drot

        return cost

    def _route_cost_from_rpy(self, pos: np.ndarray, rpy: np.ndarray):
        """
        open path 기준으로,
            L_total, R_total, J_total 계산
        """
        M = pos.shape[0]
        if M < 2:
            return 0.0, 0.0, 0.0

        L = 0.0
        R = 0.0
        for i in range(M - 1):
            dp = pos[i + 1] - pos[i]
            L += float(np.linalg.norm(dp))

            dr = rpy[i + 1] - rpy[i]
            dr[2] = self._wrap_angle(dr[2])
            R += float(np.linalg.norm(dr))

        J = self.w_dist * L + self.w_rot * R
        return L, R, J

    # ============================================================
    # Nearest-Neighbor (시작점 고정)
    # ============================================================
    def _nearest_neighbor_route(self, cost_mat: np.ndarray, start_idx: int):
        N = cost_mat.shape[0]
        start = int(start_idx)
        route = [start]
        used = np.zeros(N, dtype=bool)
        used[start] = True

        for _ in range(N - 1):
            i = route[-1]
            # 아직 안 간 노드들
            cand = np.where(~used)[0]
            if len(cand) == 0:
                break
            costs = cost_mat[i, cand]
            j = int(cand[np.argmin(costs)])
            route.append(j)
            used[j] = True

        return route

    # ============================================================
    # 2-opt (open path, endpoints 고정 X, 단 route[0]은 사실상 start 유지됨)
    # ============================================================
    def _two_opt(self, route, cost_mat, max_iter=50):
        N = len(route)
        if N < 4:
            return route

        route = list(route)
        count_iter = 0

        while count_iter < max_iter:
            improved = False
            count_iter += 1

            # open path 2-opt
            for i in range(0, N - 3):
                for k in range(i + 2, N - 1):
                    a, b = route[i], route[i + 1]
                    c, d = route[k], route[k + 1]
                    old_cost = cost_mat[a, b] + cost_mat[c, d]
                    new_cost = cost_mat[a, c] + cost_mat[b, d]
                    if new_cost + 1e-9 < old_cost:
                        route[i + 1 : k + 1] = reversed(route[i + 1 : k + 1])
                        improved = True
                        break
                if improved:
                    break

            if not improved:
                break

        return route

    # ============================================================
    # 공통 유틸: quat -> rpy, angle wrap, 3D 이동 평균, TXT 저장
    # ============================================================
    def _wrap_angle(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _quat_to_rpy(self, qx, qy, qz, qw):
        # roll (x)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _moving_average_3d(self, pts: np.ndarray, win: int):
        """
        pts: (N,3), win: 홀수
        """
        half = win // 2
        pad = np.pad(pts, ((half, half), (0, 0)), mode='edge')
        ker = np.ones(win, dtype=float) / float(win)

        xs = np.convolve(pad[:, 0], ker, mode='valid')
        ys = np.convolve(pad[:, 1], ker, mode='valid')
        zs = np.convolve(pad[:, 2], ker, mode='valid')

        return np.stack([xs, ys, zs], axis=1)

    def _save_path_txt(self, pos: np.ndarray, rpy: np.ndarray, path_str: str):
        """
        최종 경로를 x y z roll pitch yaw 형식으로 저장.
        analyze_path.py에서 그대로 읽어서
        L_total, R_total, J_total, coverage 등을 계산하게 됨.
        """
        path = FsPath(os.path.expanduser(path_str)).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")

        lines = []
        for i in range(pos.shape[0]):
            x, y, z = pos[i]
            roll, pitch, yaw = rpy[i]
            lines.append(
                f"{x:.6f} {y:.6f} {z:.6f} "
                f"{roll:.6f} {pitch:.6f} {yaw:.6f}\n"
            )

        with open(tmp, "w", encoding="utf-8") as f:
            f.writelines(lines)
        os.replace(tmp, path)
        self.get_logger().info(f"Saved planned path to {str(path)}")


def main():
    rclpy.init()
    node = TSPPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
