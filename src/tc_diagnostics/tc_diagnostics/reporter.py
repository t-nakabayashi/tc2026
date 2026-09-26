"""ノードの自己申告診断を `/diagnostics` へ配信するヘルパ。

各ノードは `DiagnosticReporter` を生成し、観点ごとに `report()` を呼ぶだけでよい。
publisher の生成、配信周期、`DiagnosticStatus.name` の規約適用は本モジュールが
一手に引き受ける。判定ロジックは各ノードの責務であり、本モジュールは持たない。

`diagnostic_updater` を使わないのは、依存とノードごとの定型コードを増やさずに
`docs/ノード健全性監視設計.md` の name 規約を1か所で強制するためである。
"""

from __future__ import annotations

import threading
from typing import Dict, Mapping, Optional

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .diagnostic_core import ASPECT_LIVENESS, OK, build_array, build_status

DIAGNOSTICS_TOPIC = '/diagnostics'
DEFAULT_PERIOD_S = 1.0

# 配信様式 stream の既定QoS（docs/トピック通信規約.md 4章）。
_QOS_DIAGNOSTICS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DiagnosticReporter:
    """観点ごとの申告値を保持し、一定周期で `/diagnostics` へまとめて配信する。"""

    def __init__(self, node: Node, *, period_s: float = DEFAULT_PERIOD_S) -> None:
        """publisher と配信タイマーを生成する。

        Args:
            node (Node): 配信元ノード. name の先頭に `node.get_name()` を使う.
            period_s (float): 配信周期 [s]. 既定は 1.0（公称レート 1 Hz）.

        Raises:
            ValueError: `period_s` が正でない場合.
        """

        if not period_s > 0.0:
            raise ValueError(f'period_s は正である必要がある: {period_s}')
        self._node = node
        self._node_name = node.get_name()
        self._lock = threading.Lock()
        self._statuses: Dict[str, object] = {}
        self._publisher = node.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, _QOS_DIAGNOSTICS
        )
        self._timer = node.create_timer(period_s, self.publish)

    def report(
        self,
        aspect: str,
        level: bytes,
        message: str,
        *,
        values: Optional[Mapping[str, object]] = None,
        hardware_id: str = '',
    ) -> None:
        """観点1件の申告値を更新する。次の配信タイミングで反映される。

        同じ観点を再度申告した場合は上書きする。配信は周期タイマーが行うため、
        高頻度に呼んでも配信レートは変わらない。

        Args:
            aspect (str): 観点名. `diagnostic_core` の `ASPECT_*` を使う.
            level (bytes): `OK` / `WARN` / `ERROR` / `STALE`.
            message (str): 日本語1行の状況説明.
            values (Optional[Mapping[str, object]]): 補助情報.
            hardware_id (str): 実デバイスを持つ場合のみ設定する識別子.
        """

        status = build_status(
            self._node_name, aspect, level, message, values=values, hardware_id=hardware_id
        )
        with self._lock:
            self._statuses[aspect] = status

    def clear(self, aspect: str) -> None:
        """観点1件の申告を取り下げる。該当が無い場合は何もしない。"""

        with self._lock:
            self._statuses.pop(aspect, None)

    def publish(self) -> None:
        """保持中の申告値を `/diagnostics` へ配信する。

        ノードの全観点を1配列で送る。削除した観点は次の配列から除外する。
        申告が1件も無い間は配信せず、受信側の10秒の期限で失効する。
        """

        with self._lock:
            statuses = list(self._statuses.values())
        if not statuses:
            return
        self._publisher.publish(
            build_array(self._node.get_clock().now().to_msg(), statuses)
        )


def report_alive(reporter: DiagnosticReporter, message: str = '稼働中') -> None:
    """生存のみを申告する最小の呼び出し。

    主処理に固有の判定を持たないノードが `liveness` を申告するために使う。

    Args:
        reporter (DiagnosticReporter): 申告先.
        message (str): 状況説明.
    """

    reporter.report(ASPECT_LIVENESS, OK, message)
