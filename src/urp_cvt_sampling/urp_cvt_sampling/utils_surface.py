import os
import numpy as np
from pathlib import Path

# trimesh는 선택 의존성 (mesh 모드에서 필요)
try:
    import trimesh
    import trimesh.proximity as tmprox  # 버전 호환을 위한 모듈 임포트
    _HAS_TRIMESH = True
except Exception:
    _HAS_TRIMESH = False
    tmprox = None


def _normalize(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True) + 1e-12
    return v / n


def quat_from_two_vectors(a, b):
    """
    단위벡터 a를 b로 회전시키는 쿼터니언 (w, x, y, z)
    a, b: (...,3)
    """
    a = _normalize(np.asarray(a))
    b = _normalize(np.asarray(b))
    c = np.cross(a, b)
    d = (a * b).sum(axis=-1, keepdims=True)
    w = 1.0 + d
    q = np.concatenate([w, c], axis=-1)  # (...,4)

    # 특수 케이스: a ≈ -b (180도 회전)
    small = (np.linalg.norm(q, axis=-1, keepdims=True) < 1e-8)
    if np.any(small):
        axis = np.zeros_like(a)
        idx = np.argmax(np.abs(a), axis=-1).reshape(-1)
        for i in range(axis.reshape(-1,3).shape[0]):
            axis.reshape(-1,3)[i, (idx[i] + 1) % 3] = 1.0
        axis = _normalize(np.cross(a, axis))
        q = np.concatenate([np.zeros_like(d), axis], axis=-1)  # w=0, 축=axis

    q = q / (np.linalg.norm(q, axis=-1, keepdims=True) + 1e-12)
    return q


class SurfaceBase:
    def __init__(self, standoff=0.0, tool_axis='+Z'):
        self.standoff = float(standoff)
        self.tool_axis = str(tool_axis)  # '+Z','-Z','+X','-X','+Y','-Y'

    def map_points(self, xy):
        """ 입력: (N,2) xy, 출력: pos(N,3), quat(N,4 wxyz) """
        raise NotImplementedError

    def _axis_vec(self):
        d = {
            '+X': np.array([ 1.0,  0.0,  0.0]),
            '-X': np.array([-1.0,  0.0,  0.0]),
            '+Y': np.array([ 0.0,  1.0,  0.0]),
            '-Y': np.array([ 0.0, -1.0,  0.0]),
            '+Z': np.array([ 0.0,  0.0,  1.0]),
            '-Z': np.array([ 0.0,  0.0, -1.0]),
        }
        return d.get(self.tool_axis, np.array([0.0, 0.0, 1.0]))


class FlatSurface(SurfaceBase):
    def __init__(self, z0=0.0, standoff=0.0, tool_axis='+Z'):
        super().__init__(standoff, tool_axis)
        self.z0 = float(z0)

    def map_points(self, xy):
        xy = np.asarray(xy, dtype=float)
        N = xy.shape[0]
        n = np.tile(np.array([[0.0, 0.0, 1.0]]), (N, 1))
        axis = np.tile(self._axis_vec(), (N, 1))
        q = quat_from_two_vectors(axis, n)  # tool_axis -> normal
        pos = np.column_stack([xy, np.full(N, self.z0)])
        pos = pos + n * self.standoff
        return pos, q


class MeshSurface(SurfaceBase):
    def __init__(self, mesh_path, standoff=0.0, tool_axis='+Z'):
        super().__init__(standoff, tool_axis)
        if not _HAS_TRIMESH:
            raise RuntimeError("trimesh가 설치되어 있지 않습니다. `pip install --user trimesh`")

        p = Path(os.path.expanduser(mesh_path)).resolve()
        if not p.exists():
            raise FileNotFoundError(f"Mesh not found: {p}")

        self.mesh = trimesh.load_mesh(str(p))
        # 가능 시 ray 엔진 핸들 (pyembree 설치 시 가속)
        try:
            self.ray = self.mesh.ray
        except Exception:
            self.ray = None

    def map_points(self, xy):
        """
        위→아래(z-) 레이캐스팅으로 '윗면'과의 첫 교차를 우선 사용.
        실패한 포인트만 최근접점으로 폴백(버전 호환).
        """
        xy = np.asarray(xy, dtype=float)
        N  = xy.shape[0]

        # 레이 시작 z: AABB z_max 위로 두께의 2배 더 올려 시작(두께가 있어도 윗면 먼저 맞게)
        z_min = float(self.mesh.bounds[0][2])
        z_max = float(self.mesh.bounds[1][2])
        dz    = max(float(self.mesh.extents[2]), 1e-6)
        z_start = z_max + max(1.0, 2.0 * dz)

        origins    = np.column_stack([xy, np.full(N, z_start)])
        directions = np.tile(np.array([0.0, 0.0, -1.0]), (N, 1))

        hit_pos = np.zeros((N, 3), dtype=float)
        hit_nrm = np.zeros((N, 3), dtype=float)
        hit_ok  = np.zeros((N,),     dtype=bool)

        # 1) 레이캐스팅
        n_hit = 0
        try:
            if self.ray is not None:
                locs, idx_ray, idx_tri = self.mesh.ray.intersects_location(
                    origins, directions, multiple_hits=False
                )
                if len(idx_ray) > 0:
                    n_hit = len(idx_ray)
                    hit_pos[idx_ray] = locs
                    fn = self.mesh.face_normals[np.asarray(idx_tri, dtype=int)]
                    # 법선이 아래(-Z)면 뒤집어 윗면 기준으로 보정
                    flip = (fn[:, 2] < 0.0)
                    if np.any(flip):
                        fn[flip] = -fn[flip]
                    hit_nrm[idx_ray] = fn
                    hit_ok[idx_ray]  = True
        except Exception:
            n_hit = 0  # 레이 실패 → 폴백으로 처리

        # 2) 레이 실패분: 최근접점 폴백 (버전 호환)
        n_fallback = 0
        if not np.all(hit_ok):
            remain = np.where(~hit_ok)[0]
            if remain.size > 0:
                n_fallback = remain.size
                start = np.column_stack([xy[remain], np.full(remain.size, (z_min + z_max) * 0.5)])

                closest = None
                tri_id  = None
                try:
                    # 신버전 우선: 모듈 함수
                    if tmprox is not None and hasattr(tmprox, 'closest_point'):
                        closest, _, tri_id = tmprox.closest_point(self.mesh, start)
                    else:
                        raise AttributeError("tmprox.closest_point not available")
                except Exception:
                    # 구버전: ProximityQuery.nearest
                    pq = trimesh.proximity.ProximityQuery(self.mesh)
                    closest, _, tri_id = pq.nearest(start)

                hit_pos[remain] = closest
                fn = self.mesh.face_normals[np.asarray(tri_id, dtype=int)]
                flip = (fn[:, 2] < 0.0)
                if np.any(flip):
                    fn[flip] = -fn[flip]
                hit_nrm[remain] = fn
                hit_ok[remain]   = True

        # 3) 정규화 + standoff 적용 + 쿼터니언
        n = _normalize(hit_nrm)
        axis = np.tile(self._axis_vec(), (N, 1))
        q = quat_from_two_vectors(axis, n)  # (w,x,y,z)
        pos = hit_pos + n * self.standoff

        # 디버그 요약 (런치 터미널에 출력)
        try:
            zvals = pos[:, 2]
            nz    = n[:, 2]
            print(
                f"[MeshSurface] rays={N}, hits={n_hit}, fallback={n_fallback}, "
                f"bounds.z=[{z_min:.3f},{z_max:.3f}], "
                f"pos.z[min/mean/max]=[{zvals.min():.3f}/{zvals.mean():.3f}/{zvals.max():.3f}], "
                f"normal.z[min/mean/max]=[{nz.min():.3f}/{nz.mean():.3f}/{nz.max():.3f}]"
            )
        except Exception:
            pass

        return pos, q
