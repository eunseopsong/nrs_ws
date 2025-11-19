import os
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from urp_cvt_interfaces.srv import GenerateCVTSamples

from .utils import (
    init_sites, sample_grid, nearest_indices,
    recompute_centroids, enforce_min_spacing
)


class CVTSampler(Node):
    def __init__(self):
        super().__init__('cvt_sampler_node')

        # QoS: RViz를 나중에 켜도 마지막 메시지가 보이게
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_waypoints = self.create_publisher(PoseArray, '~/waypoints', qos)

        # 서비스 등록
        self.srv = self.create_service(
            GenerateCVTSamples,
            'generate_cvt_samples',
            self.handle_request
        )

        # 일반 파라미터
        self.declare_parameter('publish_rate_hz', 5.0)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.declare_parameter('save_txt_path', '~/waypoints.txt')
        self.save_txt_path = str(self.get_parameter('save_txt_path').value)

        # 표면 파라미터
        self.declare_parameter('surface_type', 'flat')  # 'flat' | 'mesh'
        self.declare_parameter('mesh_path', '')         # mesh일 때만 사용
        self.declare_parameter('standoff', 0.0)         # [m]
        self.declare_parameter('tool_axis', '+Z')       # '+Z','-Z','+X','-X','+Y','-Y'

        self.surface_type = str(self.get_parameter('surface_type').value)
        self.mesh_path    = str(self.get_parameter('mesh_path').value)
        self.standoff     = float(self.get_parameter('standoff').value)
        self.tool_axis    = str(self.get_parameter('tool_axis').value)

        # 주기 퍼블리시 타이머
        self.last_pa = None
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.1),
            self._timer_cb
        )

        self.get_logger().info('CVT sampler ready: /generate_cvt_samples')

    # 지연 임포트로 순환/부분 초기화 방지
    def _make_surface(self):
        if self.surface_type == 'flat':
            from urp_cvt_sampling.utils_surface import FlatSurface
            return FlatSurface(z0=0.0, standoff=self.standoff, tool_axis=self.tool_axis)
        elif self.surface_type == 'mesh':
            from urp_cvt_sampling.utils_surface import MeshSurface
            return MeshSurface(mesh_path=self.mesh_path, standoff=self.standoff, tool_axis=self.tool_axis)
        else:
            raise ValueError(f"Unknown surface_type: {self.surface_type}")

    # 주기적으로 마지막 결과 재퍼블리시
    def _timer_cb(self):
        if self.last_pa is not None:
            self.last_pa.header.stamp = self.get_clock().now().to_msg()
            self.pub_waypoints.publish(self.last_pa)

    # 서비스 콜백
    def handle_request(self, req, res):
        try:
            xmin, xmax = float(req.x_min), float(req.x_max)
            ymin, ymax = float(req.y_min), float(req.y_max)
            K          = int(req.k)
            tool_width = float(req.tool_width)
            overlap    = float(req.overlap)
            max_iters  = int(req.max_iters)
            eps        = float(req.eps)
            seed       = int(req.seed)
            grid_step  = float(req.grid_step) if req.grid_step > 0.0 else max(1e-6, tool_width * 0.5)

            # 입력 검증
            pa_empty = PoseArray(); pa_empty.header.frame_id = 'map'
            if xmin >= xmax or ymin >= ymax or K <= 0 or not (0.0 <= overlap < 1.0):
                res.success = False
                res.message = 'Invalid request'
                res.waypoints = pa_empty
                res.mean_movement = 0.0
                res.iters = 0
                return res

            # === CVT (k-means 근사) ===
            sites = init_sites(xmin, xmax, ymin, ymax, K, seed)            # (K,2)
            P = sample_grid(xmin, xmax, ymin, ymax, step=grid_step)         # (M,2)

            last_move, it = float('inf'), 0
            for it in range(1, max_iters + 1):
                labels = nearest_indices(P, sites)
                new_sites = recompute_centroids(P, labels, sites)
                movement = np.linalg.norm(new_sites - sites, axis=1)
                mean_move = float(movement.mean())
                sites = new_sites
                if mean_move <= eps:
                    last_move = mean_move
                    break
                last_move = mean_move

            # 최소 간격 보장(툴 폭 및 overlap 기반)
            d_min = tool_width * (1.0 - overlap)
            sites = enforce_min_spacing(sites, d_min)  # (K',2)

            # === 표면 매핑 (2D -> 3D + orientation) ===
            surface = self._make_surface()
            pos, quat = surface.map_points(sites)  # pos: (N,3), quat: (N,4) wxyz

            # PoseArray 구성
            pa = PoseArray()
            pa.header.frame_id = 'map'
            pa.header.stamp = self.get_clock().now().to_msg()
            pa.poses = []
            for i in range(pos.shape[0]):
                p = Pose()
                p.position.x = float(pos[i, 0])
                p.position.y = float(pos[i, 1])
                p.position.z = float(pos[i, 2])
                # quat is (w,x,y,z) -> ROS msg uses (x,y,z,w)
                p.orientation.w = float(quat[i, 0])
                p.orientation.x = float(quat[i, 1])
                p.orientation.y = float(quat[i, 2])
                p.orientation.z = float(quat[i, 3])
                pa.poses.append(p)

            # 퍼블리시 + 캐시 + 저장
            self.last_pa = pa
            self.pub_waypoints.publish(pa)

            # 디버그: z 통계 찍기 (곡면 반영 확인용)
            try:
                z_vals = [pp.position.z for pp in pa.poses]
                z_min, z_mean, z_max = min(z_vals), sum(z_vals)/len(z_vals), max(z_vals)
                self.get_logger().info(f"pos z[min/mean/max]={z_min:.3f}/{z_mean:.3f}/{z_max:.3f}")
            except Exception:
                pass

            self._save_waypoints_txt(pa, self.save_txt_path)

            # 응답
            res.waypoints = pa
            res.success = True
            res.mean_movement = last_move
            res.iters = it
            res.message = f'Generated {len(pa.poses)} waypoints (K={K}), move={last_move:.4f}'
            self.get_logger().info(res.message)
            return res

        except Exception as e:
            self.get_logger().error(f"handle_request error: {e}")
            pa_empty = PoseArray(); pa_empty.header.frame_id = 'map'
            res.success = False
            res.message = f'Exception: {e}'
            res.waypoints = pa_empty
            res.mean_movement = 0.0
            res.iters = 0
            return res

    # quat -> RPY
    def _quat_to_rpy(self, qx, qy, qz, qw):
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    # TXT 저장: x y z roll pitch yaw (공백 구분, 소수6자리)
    def _save_waypoints_txt(self, pa: PoseArray, path_str: str):
        path = Path(os.path.expanduser(path_str)).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")

        lines = []
        for p in pa.poses:
            x = p.position.x
            y = p.position.y
            z = p.position.z
            qx = p.orientation.x
            qy = p.orientation.y
            qz = p.orientation.z
            qw = p.orientation.w
            roll, pitch, yaw = self._quat_to_rpy(qx, qy, qz, qw)
            lines.append(f"{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}\n")

        with open(tmp, "w", encoding="utf-8") as f:
            f.writelines(lines)
        os.replace(tmp, path)
        self.get_logger().info(f"Saved {len(pa.poses)} waypoints to {str(path)}")


def main():
    rclpy.init()
    node = CVTSampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
