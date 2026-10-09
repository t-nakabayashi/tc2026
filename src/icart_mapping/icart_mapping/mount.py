"""Geometry needed by mapping, in metres and degrees from hardware.yaml."""
import numpy as np
from scipy.spatial.transform import Rotation


def geometry(hardware):
    mid, gnss = hardware['mid360'], hardware['gnss']
    vectors = [np.asarray(mid[k], dtype=float) for k in ('xyz', 'rpy_deg', 'lidar_in_imu_xyz')]
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in vectors):
        raise ValueError('LiDAR取付xyz/rpy/extrinsicは有限の3要素が必要です')
    lidar, angles, extrinsic = vectors
    rotation = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    master = lidar + [0., 0., float(gnss['master_above_lidar_m'])]
    master[0] = float(gnss.get('master_forward_m', master[0]))
    if not np.isfinite(master).all():
        raise ValueError('主アンテナの取付位置が不正です')
    return dict(imu=(lidar-rotation@extrinsic).tolist(), master=master.tolist(), rpy_deg=angles.tolist())
