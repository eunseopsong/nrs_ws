import os
import math
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseArray, Pose
from urp_cvt_interfaces.srv import GenerateCVTSamples

# 곡면/평면 매핑 유틸 (이미 패키지에 있는 utils_surface 사용)
from .utils_surface import FlatSurface, MeshSurface


class CellDecompSampler(Node):
    """
    Cell Decomposition 기반 샘플러 노드.

    입력:
      - GenerateCVTSamples.srv
        (x_min, x_max, y_min, y_max, k, tool_width, overlap, ...)

    동작 개요:
      1) 툴 폭 & overlap으로 최소 간격 d = tool_width * (1 - overlap) 계산
      2) [x_min, x_max], [y_min, y_max] 영역을 d 간격 그리드로 분해
      3) 각 cell의 중심에 1개 waypoint 배치
      4) 필요시 K 개수 제한 (cell 수가 K보다 많으면 간격을 약간 늘려 셀 개수를 줄임)
      5) flat/mesh surface 로 매핑 → 3D pose + orientation 생성
      6) PoseArray 퍼블리시 + TXT 저장 (옵션)

    사용법:
      - 기존 CVT/PDS 버전과 동일하게
        ros2 service call /generate_cvt_samples ...
      - path_planner_node는 그대로 재사용 가능 (경로 플래닝 동일)
    """

    def __init__(self):
        super().__init__('cvt_sampler_node')  # 노드 이름은 그대로 사용

        # QoS (RViz 나중에 켜도 보이게)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # 서비스
        self.srv = self.create_service(
            GenerateCVTSamples,
            'generate_cvt_samples',
            self.handle_request
        )

        # PoseArray 퍼블리셔
        self.pub_waypoints = self.create_publisher(PoseArray, '~/waypoints', qos)

        # 주기적 republish (RViz 나중에 켜도 보이게)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.last_pa = None
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.1),
            self._timer_cb
        )

        # TXT 저장 경로
        self.declare_parameter('save_txt_path', '~/waypoints_cell.txt')
        self.save_txt_path = str(self.get_parameter('save_txt_path').value)

        # 표면 타입 파라미터 (flat / mesh)
        self.declare_parameter('surface_type', 'flat')  # 'flat' 또는 'mesh'
        self.declare_parameter('mesh_path', '')
        self.declare_parameter('standoff', 0.0)
        self.declare_parameter('tool_axis', '+Z')

        self.get_logger().info(
            "CellDecompSampler (cell decomposition) ready: /generate_cvt_samples"
        )

    # --------------------------------------------------------------
    # 타이머: 마지막 결과 재퍼블리시
    # --------------------------------------------------------------
    def _timer_cb(self):
        if self.last_pa is not None:
            self.last_pa.header.stamp = self.get_clock().now().to_msg()
            self.pub_waypoints.publish(self.last_pa)

    # --------------------------------------------------------------
    # 쿼터니언 → RPY 변환 (TXT 저장용)
    # --------------------------------------------------------------
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

    # --------------------------------------------------------------
    # TXT 저장 (x y z roll pitch yaw)
    # --------------------------------------------------------------
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
            lines.append(
                f"{x:.6f} {y:.6f} {z:.6f} "
                f"{roll:.6f} {pitch:.6f} {yaw:.6f}\n"
            )

        with open(tmp, "w", encoding="utf-8") as f:
            f.writelines(lines)
        os.replace(tmp, path)
        self.get_logger().info(
            f"Saved {len(pa.poses)} cell-decomp waypoints to {str(path)}"
        )

    # --------------------------------------------------------------
    # 셀 분해 기반 2D 포인트 생성
    # --------------------------------------------------------------
    def _cell_decomp_2d(self, xmin, xmax, ymin, ymax, tool_width, overlap, K):
        """
        직사각형 영역을 셀로 나누고, 셀 중심에 포인트 배치.

        입력:
          xmin, xmax, ymin, ymax : 영역 경계
          tool_width             : 툴 직경 [m]
          overlap                : 0~1, 인접 footprint 간 중첩 비율
          K                      : 원하는 최대 포인트 수 (<=0 이면 무시)

        출력:
          pts2: (N,2) numpy array (x,y)
        """
        # 최소 간격 (폴리싱 footprint가 겹치도록)
        d = tool_width * (1.0 - overlap)
        d = max(d, 1e-4)

        width  = xmax - xmin
        height = ymax - ymin
        if width <= 0.0 or height <= 0.0:
            return np.zeros((0, 2), dtype=float)

        # 우선 d 간격 그대로 그리드 생성
        xs = np.arange(xmin, xmax + 1e-9, d)
        ys = np.arange(ymin, ymax + 1e-9, d)

        X, Y = np.meshgrid(xs, ys)
        pts = np.stack([X.ravel(), Y.ravel()], axis=1)  # (M,2)

        # 만약 너무 많으면 spacing을 늘려서 대략 K 근처로 조정
        if K > 0 and len(pts) > K:
            # 대략 sqrt 비율로 간격 scale
            scale = math.sqrt(len(pts) / float(K))
            d_new = d * scale

            xs = np.arange(xmin, xmax + 1e-9, d_new)
            ys = np.arange(ymin, ymax + 1e-9, d_new)
            X, Y = np.meshgrid(xs, ys)
            pts = np.stack([X.ravel(), Y.ravel()], axis=1)

        return pts

    # --------------------------------------------------------------
    # 서비스 콜백
    # --------------------------------------------------------------
    def handle_request(self, req, res):
        xmin = float(req.x_min)
        xmax = float(req.x_max)
        ymin = float(req.y_min)
        ymax = float(req.y_max)
        K    = int(req.k)
        tool_width = float(req.tool_width)
        overlap    = float(req.overlap)

        # grid_step, seed, max_iters, eps는 여기서는 사용 안 함
        # (인터페이스 호환만 유지)
        # 필요하면 overlap/spacing 조정에 활용 가능.

        # 빈 PoseArray (실패시 반환용)
        pa_empty = PoseArray()
        pa_empty.header.frame_id = 'map'

        # 입력 검증
        if xmin >= xmax or ymin >= ymax or tool_width <= 0.0 or not (0.0 <= overlap < 1.0):
            res.success = False
            res.message = "Invalid request parameters"
            res.waypoints = pa_empty
            res.mean_movement = 0.0
            res.iters = 0
            return res

        # 표면 설정 (flat / mesh)
        surface_type = str(self.get_parameter('surface_type').value)
        standoff = float(self.get_parameter('standoff').value)
        tool_axis = str(self.get_parameter('tool_axis').value)
        mesh_path = str(self.get_parameter('mesh_path').value)

        if surface_type == 'flat':
            surface = FlatSurface(z0=0.0, standoff=standoff, tool_axis=tool_axis)
        else:
            surface = MeshSurface(mesh_path=mesh_path, standoff=standoff, tool_axis=tool_axis)

        # 1) 2D cell decomposition 샘플 생성
        pts2 = self._cell_decomp_2d(
            xmin, xmax, ymin, ymax,
            tool_width=tool_width,
            overlap=overlap,
            K=K
        )

        if pts2.shape[0] == 0:
            res.success = False
            res.message = "No points generated by cell decomposition"
            res.waypoints = pa_empty
            res.mean_movement = 0.0
            res.iters = 0
            return res

        # 2) 표면으로 매핑 (flat/mesh → 3D pos + quat)
        pos3, quat = surface.map_points(pts2)  # pos3: (N,3), quat: (N,4) wxyz

        # 3) PoseArray 생성
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'

        for i in range(pos3.shape[0]):
            p = Pose()
            p.position.x = float(pos3[i, 0])
            p.position.y = float(pos3[i, 1])
            p.position.z = float(pos3[i, 2])

            qw, qx, qy, qz = quat[i]
            p.orientation.w = float(qw)
            p.orientation.x = float(qx)
            p.orientation.y = float(qy)
            p.orientation.z = float(qz)

            pa.poses.append(p)

        # 4) publish + 저장 + 응답
        self.last_pa = pa
        self.pub_waypoints.publish(pa)

        # TXT 저장 (x y z roll pitch yaw)
        self._save_waypoints_txt(pa, self.save_txt_path)

        res.waypoints = pa
        res.success = True
        res.mean_movement = 0.0   # CVT처럼 이동량 개념이 없으니 0
        res.iters = 1
        res.message = f"Cell decomposition: {len(pa.poses)} waypoints"

        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = CellDecompSampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
