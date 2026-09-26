"""`/diagnostics` へ配信する `DiagnosticStatus` の組み立て。

ROS通信を行わない純粋な変換のみを持ち、`rclpy` なしで単体テストできる。
規約は `docs/ノード健全性監視設計.md` 5章を正とする。
"""

from __future__ import annotations

from typing import Iterable, Mapping, Optional, Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# 観点の名前（設計書 5.3節）。
ASPECT_LIVENESS = 'liveness'
ASPECT_DEVICE = 'device'
ASPECT_QUALITY = 'quality'

# `DiagnosticStatus.level` は ROS 2 の byte 型であり、Python では長さ1の bytes となる。
OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

_LEVELS = (OK, WARN, ERROR, STALE)

# 深刻度の順序。複数の観点を1つへ集約するときの比較に使う。
_SEVERITY_ORDER = {OK: 0, STALE: 1, WARN: 2, ERROR: 3}


class DiagnosticNameError(ValueError):
    """`DiagnosticStatus.name` の規約違反。"""


def compose_name(node_name: str, aspect: str) -> str:
    """`<ノード名>/<観点>` 形式の `DiagnosticStatus.name` を組み立てる。

    Args:
        node_name (str): 配信元ノード名（`Node.get_name()` の値）.
        aspect (str): 観点名. `liveness` / `device` / `quality` を基本とする.

    Returns:
        str: 組み立てた name.

    Raises:
        DiagnosticNameError: ノード名または観点名が空、もしくは `/` を含む場合.
    """

    for value, label in ((node_name, 'ノード名'), (aspect, '観点名')):
        if not value:
            raise DiagnosticNameError(f'{label} が空である')
        if '/' in value:
            raise DiagnosticNameError(f'{label} に "/" を含めることはできない: {value}')
    return f'{node_name}/{aspect}'


def build_status(
    node_name: str,
    aspect: str,
    level: bytes,
    message: str,
    *,
    values: Optional[Mapping[str, object]] = None,
    hardware_id: str = '',
) -> DiagnosticStatus:
    """1件の `DiagnosticStatus` を組み立てる。

    `values` は表示と事後解析のための補助情報であり、制御判断には使わない
    （設計書 5.4節）。値は `str()` で文字列化する。

    Args:
        node_name (str): 配信元ノード名.
        aspect (str): 観点名.
        level (bytes): `OK` / `WARN` / `ERROR` / `STALE` のいずれか.
        message (str): 日本語1行の状況説明.
        values (Optional[Mapping[str, object]]): 補助情報.
        hardware_id (str): 実デバイスを持つ場合のみ設定する識別子.

    Returns:
        DiagnosticStatus: 組み立てた診断1件.

    Raises:
        ValueError: `level` が規定値以外の場合.
    """

    if level not in _LEVELS:
        raise ValueError(f'level は OK/WARN/ERROR/STALE のいずれかである必要がある: {level}')
    status = DiagnosticStatus()
    status.name = compose_name(node_name, aspect)
    status.level = level
    status.message = message
    status.hardware_id = hardware_id
    status.values = [
        KeyValue(key=str(key), value=str(value)) for key, value in (values or {}).items()
    ]
    return status


def build_array(stamp, statuses: Sequence[DiagnosticStatus]) -> DiagnosticArray:
    """`DiagnosticArray` を組み立てる。

    Args:
        stamp: `builtin_interfaces/Time` 相当の配信時刻.
        statuses (Sequence[DiagnosticStatus]): 同時に配信する診断.

    Returns:
        DiagnosticArray: 組み立てた配列メッセージ.
    """

    array = DiagnosticArray()
    array.header.stamp = stamp
    array.status = list(statuses)
    return array


def worst_level(levels: Iterable[bytes]) -> bytes:
    """複数の `level` から最も深刻なものを返す。

    空の場合は `OK` を返す。深刻度は OK < STALE < WARN < ERROR とする。

    Args:
        levels (Iterable[bytes]): 集約対象の level.

    Returns:
        bytes: 最も深刻な level.
    """

    worst = OK
    for level in levels:
        if _SEVERITY_ORDER.get(level, 0) > _SEVERITY_ORDER[worst]:
            worst = level
    return worst
