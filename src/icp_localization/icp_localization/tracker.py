"""固定地図ICPの推定処理。走行許可・停止・GNSS再捕捉を扱わない。"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class QualityConfig:
    min_overlap: float = .6
    max_median_m: float = .3
    min_inliers: int = 100
    min_information_ratio: float = .0001
    max_correction_m: float = .8
    max_rotation_deg: float = 8.
    max_result_age_s: float = .5

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('ICP thresholds must be positive and finite')
        if self.min_overlap > 1 or self.min_information_ratio >= 1:
            raise ValueError('Invalid quality ratio')


def valid_pose(p):
    return (p.shape == (4, 4) and np.isfinite(p).all()
            and np.allclose(p[3], [0, 0, 0, 1], atol=1e-6)
            and np.allclose(p[:3, :3].T @ p[:3, :3], np.eye(3), atol=1e-5)
            and abs(np.linalg.det(p[:3, :3])-1) < 1e-5)


def quality_ok(metrics, config=None, initial=False):
    c = config or QualityConfig()
    keys = ('overlap', 'median_m', 'inliers', 'information_ratio', 'correction_m', 'rotation_deg')
    return (all(math.isfinite(metrics.get(k, math.nan)) for k in keys)
            and metrics.get('converged', False)
            and metrics['overlap'] >= c.min_overlap
            and metrics['median_m'] <= c.max_median_m
            and metrics['inliers'] >= c.min_inliers
            and metrics['information_ratio'] >= c.min_information_ratio
            and metrics['correction_m'] <= (2. if initial else c.max_correction_m)
            and metrics['rotation_deg'] <= c.max_rotation_deg)


class Tracker:
    def __init__(self, config=None):
        self.config = config or QualityConfig()
        self.pose = self.raw = None
        self.localized = self.initialized_once = False
        self.generation = 0
        self.last_accepted = self.last_stamp = -math.inf
        self.reason = 'waiting_initial_pose'
        self.metrics = {}

    def clear(self, reason):
        # LIO座標系の切断や明示的initialposeに対し、古いworker結果を破棄する。
        self.generation += 1
        self.pose = self.raw = None
        self.localized = False
        self.last_accepted = self.last_stamp = -math.inf
        self.reason = reason

    def seed(self, pose, raw):
        if not valid_pose(pose) or not valid_pose(raw):
            raise ValueError('Invalid initial SE3')
        self.clear('initial_map_matching')
        self.pose, self.raw = pose.copy(), raw.copy()

    def predict(self, raw):
        if self.pose is None:
            return None
        return self.pose @ np.linalg.inv(self.raw) @ raw

    def result(self, now, received, stamp, generation, pose, raw, metrics):
        if generation != self.generation or self.pose is None or stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        self.metrics = dict(metrics)
        if now-received > self.config.max_result_age_s:
            self.reason = 'old_icp_result'
            return False
        if not valid_pose(pose) or not quality_ok(metrics, self.config, not self.localized):
            self.reason = 'map_correction_rejected'
            return False
        self.pose, self.raw = pose.copy(), raw.copy()
        self.last_accepted = received
        self.localized = self.initialized_once = True
        self.reason = 'icp_ok'
        return True

    def state(self, now):
        if self.pose is None:
            return 'WAIT_INITIAL_POSE'
        if not self.localized:
            return 'INITIALIZING'
        if self.reason != 'icp_ok' or now-self.last_accepted > self.config.max_result_age_s:
            return 'LIO_PREDICTION'
        return 'ICP_TRACKING'
