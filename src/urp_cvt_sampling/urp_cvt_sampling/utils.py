import numpy as np
from geometry_msgs.msg import Pose, PoseArray

try:
    from scipy.spatial import cKDTree as KDTree
    _HAS_KDTREE = True
except Exception:
    _HAS_KDTREE = False

def init_sites(xmin, xmax, ymin, ymax, K, seed):
    rng = np.random.default_rng(int(seed))
    xs = rng.uniform(xmin, xmax, K)
    ys = rng.uniform(ymin, ymax, K)
    return np.stack([xs, ys], axis=1)

def sample_grid(xmin, xmax, ymin, ymax, step):
    nx = max(2, int(np.ceil((xmax - xmin) / max(step, 1e-9))))
    ny = max(2, int(np.ceil((ymax - ymin) / max(step, 1e-9))))
    xs = np.linspace(xmin, xmax, nx, endpoint=False) + 0.5*(xmax-xmin)/nx
    ys = np.linspace(ymin, ymax, ny, endpoint=False) + 0.5*(ymax-ymin)/ny
    X, Y = np.meshgrid(xs, ys)
    return np.column_stack([X.ravel(), Y.ravel()])

def nearest_indices(P, sites):
    if _HAS_KDTREE and sites.shape[0] >= 64:
        tree = KDTree(sites); _, idx = tree.query(P, k=1)
        return idx.astype(int)
    d2 = ((P[:, None, :] - sites[None, :, :])**2).sum(axis=2)
    return np.argmin(d2, axis=1)

def recompute_centroids(P, labels, sites):
    K = sites.shape[0]
    new_sites = sites.copy()
    for i in range(K):
        mask = (labels == i)
        if np.any(mask):
            new_sites[i] = P[mask].mean(axis=0)
    return new_sites

def enforce_min_spacing(points, d_min):
    kept = []
    for p in points:
        if not kept:
            kept.append(p); continue
        d2 = np.min(((np.asarray(kept) - p)**2).sum(axis=1))
        if d2 >= d_min**2:
            kept.append(p)
    return np.asarray(kept)

def to_posearray(points):
    pa = PoseArray()
    pa.poses = []
    for x, y in points:
        pose = Pose()
        pose.position.x = float(x); pose.position.y = float(y); pose.position.z = 0.0
        pose.orientation.w = 1.0
        pa.poses.append(pose)
    return pa
