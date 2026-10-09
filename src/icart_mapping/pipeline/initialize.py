"""Initial horizontal placement in the waypoint ENU projection."""
from common import *


def align_graph(frames):
    projection = CONFIG['projection']
    origin = [projection[k] for k in ('origin_latitude', 'origin_longitude', 'origin_altitude')]
    poses = np.load(ROOT/'optimized_poses.npy')
    times = np.array([f['t'] for f in frames])
    raw = np.array([f['raw'] for f in frames])
    segments = np.array([f['segment'] for f in frames])
    observations = np.load(ROOT/'obs_0.npz')
    fix, status, lio = [observations[k] for k in ('fix', 'status', 'lio')]
    si = nearest(status[:, 0], fix[:, 0])
    good = ((status[si, 1] == 4) & (status[si, 2] >= 20) & (status[si, 3] > 0)
            & (status[si, 3] <= 1.000001) & (status[si, 7] >= 0)
            & (status[si, 7] <= 1.500001) & (abs(status[si, 0]-fix[:, 0]) <= .05)
            & np.isfinite(fix[:, :4]).all(axis=1))
    gps = enu(fix[:, 1:4], origin)
    trajectory = {}
    for s in json.loads((ROOT/'segments.json').read_text()):
        trajectory[s['id']] = Trajectory(lio[(lio[:, 0] >= s['start']) & (lio[:, 0] <= s['end'])])
    source, target = [], []
    for i, tm in enumerate(times):
        tr = trajectory[segments[i]]
        candidates = np.flatnonzero(good & (abs(fix[:, 0]-tm) <= .15)
                                    & (fix[:, 0] >= tr.t[0]) & (fix[:, 0] <= tr.t[-1]))
        if not len(candidates):
            continue
        j = candidates[np.argmin(abs(fix[candidates, 0]-tm))]
        k = np.clip(np.searchsorted(tr.t, fix[j, 0]), 1, len(tr.t)-1)
        if tr.t[k]-tr.t[k-1] > .3:
            continue
        pose = poses[i] @ np.linalg.inv(raw[i]) @ tr.at(fix[j, 0])
        source.append((pose[:3, :3] @ IMU_ANT+pose[:3, 3])[:2])
        target.append(gps[j, :2])
    if len(source) < 3 or np.linalg.norm(np.ptp(source, axis=0)) < 10:
        raise ValueError('座標合わせに必要な良好FIXが不足しています（3点以上・広がり10 m以上）')
    rotation, shift = fit_xy(np.array(source), np.array(target))
    world = np.eye(4)
    world[:2, :2], world[:2, 3] = rotation, shift
    # Z is a relative LiDAR height, never an absolute GNSS height constraint.
    world[2, 3] = -(poses[0, :3, :3] @ IMU_BASE+poses[0, :3, 3])[2]
    return origin, world @ poses
