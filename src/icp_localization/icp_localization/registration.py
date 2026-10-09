"""固定ENU地図への点・平面ICP。地図もGNSSも追跡中には更新しない。"""
import hashlib
import importlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .tracker import valid_pose


PROJECTION_KEYS = ('origin_latitude', 'origin_longitude', 'origin_altitude',
                   'map_yaw_offset_rad', 'map_frame_id', 'datum')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def load_manifest(filename, projection, verify_hash=True):
    path = Path(filename).expanduser().resolve()
    manifest = json.loads(path.read_text())
    if manifest.get('schema') != 1:
        raise ValueError('Unsupported ICP map manifest schema')
    for key in PROJECTION_KEYS:
        a, b = manifest['projection'][key], projection[key]
        if isinstance(a, (float, int)):
            equal = abs(a-b) <= 1e-10
        else:
            equal = a == b
        if not equal:
            raise ValueError(f'ICP map / waypoint projection mismatch: {key}')
    for key in ('points', 'base_anchors'):
        item = manifest[key]
        p = (path.parent/item['path']).resolve()
        if not p.is_file() or (verify_hash and sha256(p) != item['sha256']):
            raise ValueError(f'ICP map missing or hash mismatch: {key}')
        item['path'] = str(p)
    return manifest


class Engine:
    def __init__(self, manifest, base_imu, antenna, backend_python_path=''):
        if backend_python_path:
            path = Path(backend_python_path).expanduser().resolve()
            if not path.is_dir():
                raise ValueError('small_gicp Python directory does not exist')
            sys.path.insert(0, str(path))
        self.sg = importlib.import_module('small_gicp')
        self.base_imu = np.array(base_imu, dtype=float)
        self.imu_base = np.linalg.inv(self.base_imu)
        self.antenna = np.array(antenna, dtype=float)
        points = np.load(manifest['points']['path'], allow_pickle=False, mmap_mode='r')
        if points.ndim != 2 or len(points) < 100 or points.shape[1] < 3 or not np.isfinite(points[:, :3]).all():
            raise ValueError('Map must contain finite XYZ points')
        self.target, self.tree = self.sg.preprocess_points(
            points[:, :3].astype(float), .35, num_threads=2)
        self.xyz = self.target.points()[:, :3]
        self.kd = cKDTree(self.xyz)
        self.anchors = np.load(manifest['base_anchors']['path'], allow_pickle=False)
        if len(self.anchors) == 0 or not all(valid_pose(t) for t in self.anchors):
            raise ValueError('Invalid base height/attitude anchors')
        self.anchor_kd = cKDTree(self.anchors[:, :2, 3])

    def seed(self, fix):
        # FIXのXY・yawで初期化。高度はENU地図内の車体姿勢を参照する。
        # 元地図のZは水平GNSS拘束だけで作られておりGNSS高度ではない。
        rough_base = fix[:2] - (Rotation.from_euler('z', fix[2]).as_matrix() @ self.antenna)[:2]
        distance, idx = self.anchor_kd.query(rough_base)
        if distance > 5.:
            raise ValueError('FIX is outside mapped driving corridor (5 m)')
        nearby = self.anchor_kd.query_ball_point(rough_base, 1.)
        if len(nearby) > 1 and np.ptp(self.anchors[nearby, 2, 3]) > 1.:
            raise ValueError('Ambiguous map height at FIX; operator map review required')
        base = self.anchors[idx].copy()
        old_yaw = np.arctan2(base[1, 0], base[0, 0])
        base[:3, :3] = Rotation.from_euler('z', fix[2]-old_yaw).as_matrix() @ base[:3, :3]
        base[:2, 3] = fix[:2] - (base[:3, :3] @ self.antenna)[:2]
        return base @ self.base_imu

    def base_pose(self, imu):
        return imu @ self.imu_base

    def antenna_xy(self, imu):
        base = self.base_pose(imu)
        return (base[:3, 3] + base[:3, :3] @ self.antenna)[:2]

    def register(self, points, seed, initial=False):
        points = np.asarray(points, dtype=float)
        points = points[np.isfinite(points).all(axis=1)]
        radius = np.linalg.norm(points, axis=1)
        points = points[(radius > .8) & (radius < 65.)]
        if len(points) < 100:
            raise ValueError('Too few finite scan points')
        source, _ = self.sg.preprocess_points(points, .4, num_threads=2)
        if len(source.points()) < 100:
            raise ValueError('Too few spatially distinct scan points')
        result = self.sg.align(self.target, source, self.tree,
                               init_T_target_source=seed, registration_type='PLANE_ICP',
                               max_correspondence_distance=2. if initial else 1.5,
                               max_iterations=50 if initial else 30, num_threads=2)
        pose = result.T_target_source
        if not valid_pose(pose):
            raise ValueError('ICP returned invalid SE3')
        cloud = source.points()[:, :3] @ pose[:3, :3].T + pose[:3, 3]
        distance, _ = self.kd.query(cloud, workers=1)
        delta = np.linalg.inv(seed) @ pose
        h = np.asarray(result.H)
        # Normalize mixed metre/radian axes before testing rank/conditioning.
        scale = np.sqrt(np.maximum(np.diag(h), 1e-12))
        eig = np.linalg.eigvalsh(h / scale[:, None] / scale[None, :])
        metrics = dict(converged=bool(result.converged), inliers=int(result.num_inliers),
                       overlap=float(np.mean(distance < .5)), median_m=float(np.median(distance)),
                       information_ratio=float(max(0., eig[0])/max(eig[-1], 1e-12)),
                       correction_m=float(np.linalg.norm(delta[:3, 3])),
                       rotation_deg=float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())))
        return pose, metrics
