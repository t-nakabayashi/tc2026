"""topicの受信鮮度を判定するモジュール。

`stale_sec` / `lost_sec` の閾値をキー（topic名やprofile_idなど）ごとに設定できる
ようにし、UI側は判定結果 (`FreshnessLevel`) だけを参照する。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from typing import Dict, Mapping, Optional

DEFAULT_STALE_SEC = 1.0
DEFAULT_LOST_SEC = 3.0


class FreshnessLevel(Enum):
    """topic受信鮮度の判定結果。"""

    OK = 'OK'
    STALE = 'STALE'
    LOST = 'LOST'
    UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class FreshnessThresholds:
    """STALE/LOST判定の閾値 [秒]。"""

    stale_sec: float = DEFAULT_STALE_SEC
    lost_sec: float = DEFAULT_LOST_SEC


def _utc_now() -> datetime:
    return datetime.now(tz=timezone.utc)


class FreshnessMonitor:
    """キー（topic名やprofile_idなど）ごとの最終受信時刻から鮮度を判定する。"""

    def __init__(
        self,
        *,
        default_stale_sec: float = DEFAULT_STALE_SEC,
        default_lost_sec: float = DEFAULT_LOST_SEC,
    ) -> None:
        self._default_thresholds = FreshnessThresholds(default_stale_sec, default_lost_sec)
        self._thresholds: Dict[str, FreshnessThresholds] = {}
        self._last_received: Dict[str, datetime] = {}
        self._lock = threading.Lock()

    @classmethod
    def from_config(
        cls,
        config: Mapping[str, object],
        *,
        default_prefix: str = 'freshness.default',
    ) -> 'FreshnessMonitor':
        """``freshness.<key>.stale_sec`` / ``lost_sec`` 形式の設定辞書から生成する。

        `config` はドット区切りキーのフラット辞書（例:
        ``{'freshness.default.stale_sec': 1.0, 'freshness.gps.status.stale_sec': 3.0,
        'freshness.gps.status.lost_sec': 10.0}``）を想定する。
        """

        default_stale = float(config.get(f'{default_prefix}.stale_sec', DEFAULT_STALE_SEC))
        default_lost = float(config.get(f'{default_prefix}.lost_sec', DEFAULT_LOST_SEC))
        monitor = cls(default_stale_sec=default_stale, default_lost_sec=default_lost)

        stale_suffix = '.stale_sec'
        for key, value in config.items():
            if not key.startswith('freshness.') or not key.endswith(stale_suffix):
                continue
            target_key = key[len('freshness.'):-len(stale_suffix)]
            if target_key == 'default':
                continue
            lost_key = f'freshness.{target_key}.lost_sec'
            if lost_key not in config:
                continue
            monitor.set_threshold(target_key, float(value), float(config[lost_key]))
        return monitor

    def set_threshold(self, key: str, stale_sec: float, lost_sec: float) -> None:
        """指定キーの閾値を設定する。"""

        with self._lock:
            self._thresholds[key] = FreshnessThresholds(stale_sec, lost_sec)

    def mark_received(self, key: str, timestamp: Optional[datetime] = None) -> None:
        """指定キーの最終受信時刻を更新する。"""

        with self._lock:
            self._last_received[key] = timestamp or _utc_now()

    def reset(self, key: str) -> None:
        """指定キーの最終受信時刻を破棄し UNKNOWN 判定へ戻す。"""

        with self._lock:
            self._last_received.pop(key, None)

    def elapsed_seconds(self, key: str, *, now: Optional[datetime] = None) -> Optional[float]:
        """最終受信からの経過秒数を返す。未受信の場合は None を返す。"""

        with self._lock:
            last = self._last_received.get(key)
        if last is None:
            return None
        current = now or _utc_now()
        return max((current - last).total_seconds(), 0.0)

    def evaluate(self, key: str, *, now: Optional[datetime] = None) -> FreshnessLevel:
        """指定キーの鮮度を判定する。"""

        elapsed = self.elapsed_seconds(key, now=now)
        if elapsed is None:
            return FreshnessLevel.UNKNOWN
        thresholds = self._thresholds.get(key, self._default_thresholds)
        if elapsed <= thresholds.stale_sec:
            return FreshnessLevel.OK
        if elapsed <= thresholds.lost_sec:
            return FreshnessLevel.STALE
        return FreshnessLevel.LOST
