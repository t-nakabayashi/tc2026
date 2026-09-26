"""profileごとの健全性を、起動状態・トピック鮮度・自己申告診断から合成する。

`docs/ノード健全性監視設計.md` 4章・5.5節の規則をROS非依存の純関数として実装する。
`ConsoleCore` から呼び出し、結果を `HealthSummaryView` として `ConsoleSnapshot` へ載せる。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence

from ..utils import NodeLaunchStatus
from .freshness import FreshnessLevel

# profile起動を要求してからこの秒数の間は、未受信を異常と判定しない（設計書 3.6節）。
DEFAULT_STARTUP_GRACE_SEC = 10.0

# 自己申告は 1 Hz。ROS 時刻の停止や補正に影響されない受信時刻で監視する。
DIAGNOSTIC_STALE_SEC = 3.0
DIAGNOSTIC_LOST_SEC = 10.0

# 診断の level（diagnostic_msgs/DiagnosticStatus）の値。
# robot_console は diagnostic_msgs へ依存せずに扱えるよう、int で保持する。
DIAGNOSTIC_OK = 0
DIAGNOSTIC_WARN = 1
DIAGNOSTIC_ERROR = 2
DIAGNOSTIC_STALE = 3


@dataclass(frozen=True)
class DiagnosticEntry:
    """`/diagnostics` から受け取った診断1件。"""

    node_name: str
    aspect: str
    level: int
    message: str
    received_at: float = field(default_factory=time.monotonic)


def diagnostic_freshness(
    entries: Sequence[DiagnosticEntry], node_names: Sequence[str], *, now: float
) -> List[FreshnessLevel]:
    """指定ノード各々の最終診断受信から鮮度を求める。未受信は UNKNOWN。"""

    latest: Dict[str, float] = {}
    for entry in entries:
        latest[entry.node_name] = max(
            latest.get(entry.node_name, entry.received_at), entry.received_at
        )
    levels = []
    for name in node_names:
        if name not in latest:
            levels.append(FreshnessLevel.UNKNOWN)
        elif now - latest[name] <= DIAGNOSTIC_STALE_SEC:
            levels.append(FreshnessLevel.OK)
        elif now - latest[name] <= DIAGNOSTIC_LOST_SEC:
            levels.append(FreshnessLevel.STALE)
        else:
            levels.append(FreshnessLevel.LOST)
    return levels


def aggregate_topic_freshness(
    levels: Sequence[FreshnessLevel], *, within_startup_grace: bool
) -> FreshnessLevel:
    """health_topics 各々の鮮度を1つへ集約する（設計書 4.1節）。

    部分的な途絶を `STALE` として残すのは、一部センサだけが途絶える部分故障を
    「全滅」と同じ扱いにせず、画面上で区別できるようにするためである。

    Args:
        levels (Sequence[FreshnessLevel]): 対象topicの鮮度.
        within_startup_grace (bool): 起動猶予期間内か.

    Returns:
        FreshnessLevel: 集約結果. 対象が無い場合は UNKNOWN.
    """

    if not levels:
        return FreshnessLevel.UNKNOWN
    ok_count = sum(1 for level in levels if level == FreshnessLevel.OK)
    if ok_count == len(levels):
        return FreshnessLevel.OK
    if any(level in (FreshnessLevel.OK, FreshnessLevel.STALE) for level in levels):
        return FreshnessLevel.STALE
    return FreshnessLevel.UNKNOWN if within_startup_grace else FreshnessLevel.LOST


def combine_with_launch_status(
    launch_status: NodeLaunchStatus, topic_health: FreshnessLevel
) -> tuple:
    """起動状態とトピック鮮度から表示状態を決める（設計書 4.2節）。

    Args:
        launch_status (NodeLaunchStatus): `LaunchManager` が管理する子プロセスの状態.
        topic_health (FreshnessLevel): `aggregate_topic_freshness()` の結果.

    Returns:
        tuple: (status 文字列, health, externally_started).
    """

    if launch_status == NodeLaunchStatus.ERROR:
        return ('ERROR', FreshnessLevel.LOST, False)
    if launch_status == NodeLaunchStatus.STARTING:
        return ('STARTING', FreshnessLevel.UNKNOWN, False)

    if launch_status == NodeLaunchStatus.RUNNING:
        if topic_health == FreshnessLevel.OK:
            return ('RUNNING', FreshnessLevel.OK, False)
        if topic_health == FreshnessLevel.STALE:
            return ('RUNNING', FreshnessLevel.STALE, False)
        if topic_health == FreshnessLevel.UNKNOWN:
            return ('STARTING', FreshnessLevel.UNKNOWN, False)
        # プロセスは生存しているのに出力が無い。publisher は作られたが
        # データが流れていない状態であり、外形からしか捉えられない。
        return ('ERROR', FreshnessLevel.LOST, False)

    # STOPPED / STOPPING。GUI が起動していないだけで、外部起動されている場合がある。
    if topic_health in (FreshnessLevel.OK, FreshnessLevel.STALE):
        return ('RUNNING', topic_health, True)
    return ('STOPPED', FreshnessLevel.UNKNOWN, False)


def apply_diagnostics(
    status: str, health: FreshnessLevel, entries: Sequence[DiagnosticEntry]
) -> tuple:
    """自己申告診断で合成結果を補正する（設計書 5.5節）。

    entries は受信期限内の診断のみとする。診断だけで死活を判定する profile の
    受信鮮度は、呼び出し側が起動状態との合成に使用する。

    Args:
        status (str): 補正前の status.
        health (FreshnessLevel): 補正前の health.
        entries (Sequence[DiagnosticEntry]): 対象profileに属する診断.

    Returns:
        tuple: (status, health, 最も深刻な診断の message).
    """

    if not entries:
        return (status, health, '')
    if status == 'STOPPED':
        # 停止と判定した profile には診断の残存期間中も稼働文言を表示しない。
        return (status, health, '')
    worst = max(entries, key=lambda entry: _diagnostic_severity(entry.level))
    if worst.level == DIAGNOSTIC_ERROR:
        return ('ERROR', FreshnessLevel.LOST, worst.message)
    if worst.level in (DIAGNOSTIC_WARN, DIAGNOSTIC_STALE) and health == FreshnessLevel.OK:
        return (status, FreshnessLevel.STALE, worst.message)
    return (status, health, worst.message)


def _diagnostic_severity(level: int) -> int:
    """診断levelの深刻度。OK < STALE < WARN < ERROR とする。"""

    return {DIAGNOSTIC_OK: 0, DIAGNOSTIC_STALE: 1, DIAGNOSTIC_WARN: 2, DIAGNOSTIC_ERROR: 3}.get(
        level, 0
    )


def select_diagnostics(
    diagnostics: Mapping[str, DiagnosticEntry], node_names: Sequence[str],
    *, now: Optional[float] = None,
) -> List[DiagnosticEntry]:
    """profileに属するノードの診断だけを抜き出す。

    Args:
        diagnostics (Mapping[str, DiagnosticEntry]): `<ノード名>/<観点>` をキーとする診断.
        node_names (Sequence[str]): profileの `diagnostic_nodes`.
        now (Optional[float]): monotonic 秒。省略時は現在時刻.

    Returns:
        List[DiagnosticEntry]: 該当する診断.
    """

    targets = set(node_names)
    current = time.monotonic() if now is None else now
    return [
        entry for entry in diagnostics.values()
        if entry.node_name in targets and current - entry.received_at <= DIAGNOSTIC_LOST_SEC
    ]


def build_summary(
    *,
    launch_status: NodeLaunchStatus,
    topic_levels: Sequence[FreshnessLevel],
    diagnostics: Sequence[DiagnosticEntry],
    within_startup_grace: bool,
    diagnostic_levels: Sequence[FreshnessLevel] = (),
) -> Dict[str, object]:
    """1 profile分の表示状態を合成する。

    Returns:
        Dict[str, object]: `status` / `health` / `externally_started` /
            `diagnostic_message` を持つ辞書.
    """

    levels = topic_levels or diagnostic_levels
    topic_health = aggregate_topic_freshness(
        levels, within_startup_grace=within_startup_grace
    )
    if not levels:
        # トピック・自己申告の両方が未設定の場合のみプロセス状態へ戻す。
        status, health, external = _launch_status_only(launch_status)
    else:
        status, health, external = combine_with_launch_status(launch_status, topic_health)
    status, health, message = apply_diagnostics(status, health, diagnostics)
    return {
        'status': status,
        'health': health,
        'externally_started': external,
        'diagnostic_message': message,
    }


_LAUNCH_ONLY_HEALTH: Dict[NodeLaunchStatus, FreshnessLevel] = {
    NodeLaunchStatus.RUNNING: FreshnessLevel.OK,
    NodeLaunchStatus.STARTING: FreshnessLevel.STALE,
    NodeLaunchStatus.STOPPING: FreshnessLevel.STALE,
    NodeLaunchStatus.STOPPED: FreshnessLevel.UNKNOWN,
    NodeLaunchStatus.ERROR: FreshnessLevel.LOST,
}


def _launch_status_only(launch_status: NodeLaunchStatus) -> tuple:
    """トピック・自己申告の両方が未設定の profile の表示状態。"""

    status = 'STOPPED' if launch_status == NodeLaunchStatus.STOPPING else launch_status.name
    return (status, _LAUNCH_ONLY_HEALTH.get(launch_status, FreshnessLevel.UNKNOWN), False)


def within_grace(
    started_at: Optional[float], now: float, grace_sec: float = DEFAULT_STARTUP_GRACE_SEC
) -> bool:
    """起動猶予期間内かを判定する。

    Args:
        started_at (Optional[float]): 起動要求時刻 [s]. 未起動なら None.
        now (float): 現在時刻 [s].
        grace_sec (float): 猶予秒数.

    Returns:
        bool: 猶予期間内なら True.
    """

    if started_at is None:
        return False
    return (now - started_at) < grace_sec
