"""前方閉塞判定の境界挙動を固定する。ROS グラフも LiDAR も使わない。

`_is_front_blocked` は self を参照しない純粋な判定であり、非束縛メソッドとして呼べる。
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from obstacle_monitor.obstacle_monitor_node import ObstacleMonitorNode

ROBOT_WIDTH_M = 0.6
STOP_DIST_M = 0.75


def test_empty_points_are_reported_as_clear() -> None:
    """点が 1 つも無いとき、未観測ではなく空き領域として報告される.

    返り値は blocked=False、clearance=inf である。LiDAR が何も返さない状態と、
    前方が実際に空いている状態を区別する品質フラグは無い。入力途絶を
    閉塞なしと解釈するため、購読側は鮮度を別途監視する必要がある。
    """
    blocked, clearance = ObstacleMonitorNode._is_front_blocked(
        None, np.empty((0, 2)), ROBOT_WIDTH_M, STOP_DIST_M
    )

    assert blocked is False
    assert np.isinf(clearance)


def test_point_inside_band_within_stop_distance_blocks() -> None:
    """車幅帯の内側かつ停止距離以内に点があれば閉塞と判定する."""
    points = np.array([[0.5, 0.0]])

    blocked, clearance = ObstacleMonitorNode._is_front_blocked(
        None, points, ROBOT_WIDTH_M, STOP_DIST_M, min_points=1
    )

    assert blocked is True
    assert clearance <= STOP_DIST_M


def test_point_outside_band_does_not_block() -> None:
    """車幅帯の外側の点は、停止距離以内にあっても閉塞と判定しない."""
    points = np.array([[0.5, ROBOT_WIDTH_M]])

    blocked, _ = ObstacleMonitorNode._is_front_blocked(
        None, points, ROBOT_WIDTH_M, STOP_DIST_M, min_points=1
    )

    assert blocked is False
