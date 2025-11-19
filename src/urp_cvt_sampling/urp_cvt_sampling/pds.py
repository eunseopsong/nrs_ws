import rclpy
from rclpy.node import Node
import numpy as np
import math
import os
from pathlib import Path

from geometry_msgs.msg import PoseArray
from urp_cvt_interfaces.srv import GenerateCVTSamples
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Surface utility
from .utils_surface import FlatSurface, MeshSurface


# ================================================================
# Poisson Disk Sampling (Bridson Method)
# ================================================================
def pds_sample(xmin, xmax, ymin, ymax, r_min, tries=30, seed=0):
    np.random.seed(seed)

    width  = xmax - xmin
    height = ymax - ymin

    cell = r_min / math.sqrt(2)
    grid_w = int(math.ceil(width  / cell))
    grid_h = int(math.ceil(height / cell))

    grid = -np.ones((grid_h, grid_w), dtype=int)
    points = []
    active = []

    def to_grid(p):
        gx = int((p[0] - xmin) / cell)
        gy = int((p[1] - ymin) / cell)
        return gx, gy

    # 1) random initial point
    p0 = np.array([
        np.random.uniform(xmin, xmax),
        np.random.uniform(ymin, ymax)
    ])
    points.append(p0)
    active.append(0)
    gx, gy = to_grid(p0)
    grid[gy, gx] = 0

    # 2) Bridson loop
    while active:
        idx = np.random.choice(active)
        base = points[idx]
        found = False

        for _ in range(tries):
            rad = np.random.uniform(r_min, 2*r_min)
            ang = np.random.uniform(0, 2*math.pi)
            cand = base + rad * np.array([math.cos(ang), math.sin(ang)])

            if not (xmin <= cand[0] <= xmax and ymin <= cand[1] <= ymax):
                continue

            cgx, cgy = to_grid(cand)
            if not (0 <= cgx < grid_w and 0 <= cgy < grid_h):
                continue

            ok = True
            for yy in range(max(0, cgy-2), min(grid_h, cgy+3)):
                for xx in range(max(0, cgx-2), min(grid_w, cgx+3)):
                    pid = grid[yy, xx]
                    if pid >= 0:
                        if np.linalg.norm(points[pid] - cand) < r_min:
                            ok = False
                            break
                if not ok:
                    break

            if ok:
                new_id = len(points)
                points.append(cand)
                active.append(new_id)
                grid[cgy, cgx] = new_id
                found = True
                break

        if not found:
            active.remove(idx)

    return np.array(points)



# ================================================================
# Sampler Node (name preserved as cvt_sampler_node)
# ================================================================
class CVTSampler(Node):
    def __init__(self):
        super().__init__('cvt_sampler_node')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL

        self.srv = self.create_service(
            GenerateCVTSamples, 'generate_cvt_samples', self.handle_request
        )
        self.pub_waypoints = self.create_publisher(PoseArray, '~/waypoints', qos)

        # republish timer
        self.declare_parameter('publish_rate_hz', 5.0)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.last_pa = None
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.1), self._timer_cb
        )

        # save
        self.declare_parameter('save_txt_path', '~/waypoints_pds.txt')
        self.save_txt_path = str(self.get_parameter('save_txt_path').value)

        # surface settings
        self.declare_parameter('surface_type', 'flat')
        self.declare_parameter('mesh_path', '')
        self.declare_parameter('standoff', 0.0)
        self.declare_parameter('tool_axis', '+Z')

        self.get_logger().info("PDS-based sampler active (replacing CVT).")


    def _timer_cb(self):
        if self.last_pa is not None:
            self.last_pa.header.stamp = self.get_clock().now().to_msg()
            self.pub_waypoints.publish(self.last_pa)


    # --------------------------------------------------------------
    def handle_request(self, req, res):
        xmin, xmax = float(req.x_min), float(req.x_max)
        ymin, ymax = float(req.y_min), float(req.y_max)
        K = int(req.k)
        tool_width = float(req.tool_width)
        overlap = float(req.overlap)

        r_min = tool_width * (1.0 - overlap)
        seed = int(req.seed)

        # Surface selection
        surf_type = str(self.get_parameter('surface_type').value)
        if surf_type == 'flat':
            surface = FlatSurface(
                z0=0.0,
                standoff=float(self.get_parameter('standoff').value),
                tool_axis=str(self.get_parameter('tool_axis').value)
            )
        else:
            surface = MeshSurface(
                mesh_path=str(self.get_parameter('mesh_path').value),
                standoff=float(self.get_parameter('standoff').value),
                tool_axis=str(self.get_parameter('tool_axis').value)
            )

        # PDS sampling
        pts2 = pds_sample(xmin, xmax, ymin, ymax, r_min, seed=seed)

        # enforce K cap if overshoot
        if len(pts2) > K:
            pts2 = pts2[:K]

        # project to surface
        pos3, quat = surface.map_points(pts2)

        # build PoseArray
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'

        from geometry_msgs.msg import Pose
        for i in range(len(pos3)):
            p = Pose()
            p.position.x = float(pos3[i, 0])
            p.position.y = float(pos3[i, 1])
            p.position.z = float(pos3[i, 2])
            p.orientation.w = float(quat[i, 0])
            p.orientation.x = float(quat[i, 1])
            p.orientation.y = float(quat[i, 2])
            p.orientation.z = float(quat[i, 3])
            pa.poses.append(p)

        self.last_pa = pa
        self.pub_waypoints.publish(pa)
        self._save_txt(pa, self.save_txt_path)

        res.waypoints = pa
        res.success = True
        res.message = f"PDS sampling done: {len(pa.poses)} points (r_min={r_min:.3f})"
        res.mean_movement = 0.0
        res.iters = 1

        self.get_logger().info(res.message)
        return res


    def _save_txt(self, pa, path_str):
        path = Path(os.path.expanduser(path_str)).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")

        lines = []
        for p in pa.poses:
            lines.append(f"{p.position.x:.6f} {p.position.y:.6f} {p.position.z:.6f} "
                         f"{p.orientation.x:.6f} {p.orientation.y:.6f} "
                         f"{p.orientation.z:.6f} {p.orientation.w:.6f}\n")

        with open(tmp, "w") as f:
            f.writelines(lines)

        os.replace(tmp, path)
        self.get_logger().info(f"Saved PDS waypoints to {path}")


def main():
    rclpy.init()
    node = CVTSampler()   # 이름 유지
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
