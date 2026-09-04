"""距離・進捗・走行時間などの走行指標を計算するモジュール。"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Tuple

Position3D = Tuple[float, float, float]


def _utc_now() -> datetime:
    return datetime.now(tz=timezone.utc)


def euclidean_distance(a: Position3D, b: Position3D) -> float:
    """3次元座標間のユークリッド距離を算出する。"""

    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5


def compute_progress_ratio(current_index: int, total_waypoints: int) -> float:
    """現在waypoint indexとwaypoint総数から走破率 [0.0, 1.0] を算出する。"""

    if total_waypoints <= 0:
        return 0.0
    ratio = current_index / total_waypoints
    return min(max(ratio, 0.0), 1.0)


def is_within_arrival_threshold(distance_m: float, arrival_threshold_m: float) -> bool:
    """目標までの距離が到達判定しきい値以内かどうかを判定する。"""

    return distance_m <= arrival_threshold_m


@dataclass
class TripSnapshot:
    """走行距離・走行時間の現在値。"""

    elapsed_sec: float = 0.0
    traveled_distance_m: float = 0.0


class TripMetrics:
    """自己位置の更新を受けて走行距離・走行時間を積算する。"""

    def __init__(self) -> None:
        self._started_at: Optional[datetime] = None
        self._last_position: Optional[Position3D] = None
        self._traveled_distance_m: float = 0.0

    def start(self, timestamp: Optional[datetime] = None) -> None:
        """計測を開始する。既に開始済みの場合は何もしない。"""

        if self._started_at is not None:
            return
        self._started_at = timestamp or _utc_now()

    def reset(self) -> None:
        """計測状態を初期化する。"""

        self._started_at = None
        self._last_position = None
        self._traveled_distance_m = 0.0

    def update_position(
        self, position: Position3D, timestamp: Optional[datetime] = None
    ) -> None:
        """自己位置の更新を反映し、走行距離を積算する。"""

        if self._started_at is None:
            self.start(timestamp)
        if self._last_position is not None:
            self._traveled_distance_m += euclidean_distance(self._last_position, position)
        self._last_position = position

    def snapshot(self, *, now: Optional[datetime] = None) -> TripSnapshot:
        """現在の走行距離・走行時間を返す。"""

        elapsed = 0.0
        if self._started_at is not None:
            current = now or _utc_now()
            elapsed = max((current - self._started_at).total_seconds(), 0.0)
        return TripSnapshot(elapsed_sec=elapsed, traveled_distance_m=self._traveled_distance_m)
