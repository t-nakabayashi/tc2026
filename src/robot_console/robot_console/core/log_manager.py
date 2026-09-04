"""profile別ログを収集・保持するモジュール。

`LaunchManager` から通知される標準出力/標準エラーの行を profile 単位で
リングバッファへ蓄積し、WARN/ERROR件数の集計や統合ログの生成を行う。
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

from ..utils import ConsoleLogBuffer

DEFAULT_BUFFER_CAPACITY = 2000

_LEVEL_PATTERN = re.compile(r'\[(DEBUG|INFO|WARN|ERROR|FATAL)\]', re.IGNORECASE)


@dataclass
class LogLevelCounts:
    """ログ行数をログレベル別に集計した結果。"""

    debug: int = 0
    info: int = 0
    warn: int = 0
    error: int = 0
    fatal: int = 0


def detect_log_level(line: str) -> Optional[str]:
    """ログ1行からログレベル文字列 (DEBUG/INFO/WARN/ERROR/FATAL) を判定する。

    ``[ERR]`` は ``LaunchManager`` がstderr行に付与するprefixのため ERROR として扱う。
    """

    if '[ERR]' in line:
        return 'ERROR'
    match = _LEVEL_PATTERN.search(line)
    if match:
        return match.group(1).upper()
    return None


def count_levels(lines: Iterable[str]) -> 'LogLevelCounts':
    """ログ行列からログレベル別件数を集計する（`LogManager` インスタンス不要）。"""

    counts = LogLevelCounts()
    for line in lines:
        level = detect_log_level(line)
        if level == 'DEBUG':
            counts.debug += 1
        elif level == 'INFO':
            counts.info += 1
        elif level == 'WARN':
            counts.warn += 1
        elif level == 'ERROR':
            counts.error += 1
        elif level == 'FATAL':
            counts.fatal += 1
    return counts


def filter_levels(lines: Iterable[str], levels: Iterable[str]) -> List[str]:
    """指定ログレベル集合に一致する行だけを抽出する。"""

    level_set = set(levels)
    return [line for line in lines if detect_log_level(line) in level_set]


class LogManager:
    """profile別ログバッファと集計を管理する。"""

    def __init__(
        self,
        profile_ids: Iterable[str] = (),
        *,
        buffer_capacity: int = DEFAULT_BUFFER_CAPACITY,
    ) -> None:
        self._buffer_capacity = buffer_capacity
        self._buffers: Dict[str, ConsoleLogBuffer] = {}
        for profile_id in profile_ids:
            self.ensure_profile(profile_id)

    def ensure_profile(self, profile_id: str) -> None:
        """指定profile用のログバッファが無ければ作成する。"""

        if profile_id not in self._buffers:
            self._buffers[profile_id] = ConsoleLogBuffer(self._buffer_capacity)

    def append(self, profile_id: str, line: str) -> None:
        """profile宛のログ行を追加する。"""

        self.ensure_profile(profile_id)
        self._buffers[profile_id].append(line)

    def snapshot(self, profile_id: str) -> List[str]:
        """指定profileの保持中ログをリストで返す。"""

        buffer = self._buffers.get(profile_id)
        return buffer.snapshot() if buffer else []

    def snapshot_all(self) -> Dict[str, List[str]]:
        """全profile分のログをまとめて返す。"""

        return {profile_id: buffer.snapshot() for profile_id, buffer in self._buffers.items()}

    def merged_snapshot(self, profile_ids: Optional[Iterable[str]] = None) -> List[str]:
        """複数profileのログにprofile名を付けて統合したログを返す。

        各profileの保持順序はそのまま維持し、profile間の厳密な時刻順マージは
        行わない。WARN/ERROR調査時の一覧性を優先する。
        """

        target_ids = list(profile_ids) if profile_ids is not None else list(self._buffers.keys())
        merged: List[str] = []
        for profile_id in target_ids:
            for line in self.snapshot(profile_id):
                merged.append(f"[{profile_id}] {line.rstrip()}")
        return merged

    def level_counts(self, profile_id: str) -> LogLevelCounts:
        """指定profileのログレベル別件数を集計する。"""

        return count_levels(self.snapshot(profile_id))

    def warn_error_lines(self, profile_id: str) -> List[str]:
        """指定profileのWARN以上のログ行のみを抽出する。"""

        return filter_levels(self.snapshot(profile_id), ('WARN', 'ERROR', 'FATAL'))

    def clear(self, profile_id: str) -> None:
        """指定profileのログを消去する。"""

        buffer = self._buffers.get(profile_id)
        if buffer:
            buffer.clear()
