"""ワークスペース全体の pytest 共通セットアップ。

`pytest-forked` は `pytest.mark.forked` の付いたテストを `os.fork()` した
子プロセスで実行する。`fork()` は呼び出したスレッドだけを複製するため、
親プロセスの他スレッドがロックを保持したまま複製されると、そのロックは
子プロセスで永久に解放されない。子は停止し続け、テストは失敗もせず終わらない。

危険なのは ROS だけではない。次のいずれも親プロセスにスレッドを残す。

- ROS context の初期化。`rclpy.init()` / `Context.init()` が生成する DDS の
  内部スレッドは `rclpy.shutdown()` 後も残留する
- `ThreadingHTTPServer` や `launch_manager` のリーダースレッド
- Qt のイベントループやレンダリングスレッド
- numpy (OpenBLAS) の演算スレッドプール

そのため、ROS を起動するテストや背景スレッドを作るテストは次のいずれかを使う。

- `pytest.mark.forked` で子プロセスへ隔離する
  (`src/robot_console/tools/tests/isolated/` 配下は conftest.py が付与する)
- `subprocess` で別プロセスを起動する
  (`src/gnss_lio_fusion/tests/` 配下で使用)

どちらでもスレッドは pytest の親プロセスではなく別プロセスで生成され、
親プロセスは追加スレッドを持たないまま維持される。

## 検査の仕組み

この規則が破れた場合、デッドロックの代わりにテストを失敗させる。検査は 2 つある。

1. **スレッド数の検査**（一般的な網）
   pytest 起動直後のネイティブスレッド数を基準として記録し、forked テストの
   直前に増えていれば失敗させる。`/proc/self/status` の `Threads:` を見るため、
   C ライブラリが作る非 Python スレッドも数える。`threading.enumerate()` では
   これらは見えない。

   基準値が 1 でない点に注意する。pytest は entry point 経由で
   `launch_testing_ros` を読み込み、その先で `rclpy` が import される。
   `rclpy` は `init()` を呼ばなくても import だけで 2 スレッドを立てる。
   この分は避けられないため基準へ含め、そこからの増加だけを検出する。

   計測は親プロセスで行う必要がある。`pytest_runtest_setup()` は forked テストでは
   子プロセス側で実行され、子のスレッドは fork した 1 本だけになるため、子で数えても
   親の汚染は分からない。fork 前の親で `pytest_runtest_protocol()` から記録し、
   子はその値を引き継いで判定する。

2. **ROS context 初期化の検査**（原因を名指しする）
   `Context.init()` を包んで呼び出したプロセスを記録する。fork した子は
   この記録を引き継ぐため、子プロセス側で「親が初期化済みだった」と判別できる。
   スレッド数の検査でも捕まるが、原因を特定した文言を出すために併用する。
"""

from __future__ import annotations

import os
from typing import Any

import pytest

# ROS context を初期化したプロセスの PID。未初期化なら None。
_ros_context_initialized_pid: int | None = None

# pytest 起動直後のネイティブスレッド数。避けられない分を基準として持つ。
_baseline_native_threads: int | None = None

# テスト開始時点で親プロセスが持っていたネイティブスレッド数。
# fork 前に親で記録し、子プロセスは fork によってこの値を引き継ぐ。
_parent_native_threads: int | None = None


def _native_thread_count() -> int:
    """自プロセスのネイティブスレッド数を返す。

    `threading.enumerate()` は Python が生成したスレッドしか数えない。DDS、Qt、
    OpenBLAS などが作るスレッドを含めるため `/proc/self/status` を読む。

    Returns:
        int: スレッド数。取得できない場合は -1。
    """

    try:
        with open('/proc/self/status', encoding='utf-8') as status:
            for line in status:
                if line.startswith('Threads:'):
                    return int(line.split()[1])
    except OSError:
        pass
    return -1


def _install_ros_context_probe() -> None:
    """`Context.init()` を包み、ROS context の初期化を記録する。

    ROS 環境が有効でない場合は何もしない。その場合 ROS を使うテストは
    `pytest.importorskip('rclpy')` などで skip されるため、検査も不要になる。
    """

    try:
        from rclpy.context import Context
    except ImportError:
        return

    original_init = Context.init

    def init(self: Any, *args: Any, **kwargs: Any) -> Any:
        global _ros_context_initialized_pid
        _ros_context_initialized_pid = os.getpid()
        return original_init(self, *args, **kwargs)

    Context.init = init


_install_ros_context_probe()


def pytest_configure(config: pytest.Config) -> None:
    """避けられないスレッド数を基準として記録する。

    Args:
        config (pytest.Config): pytest の設定オブジェクト.
    """

    global _baseline_native_threads
    _baseline_native_threads = _native_thread_count()


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_protocol(item: pytest.Item, nextitem: pytest.Item | None):
    """fork 前の親プロセスのスレッド数を記録する。

    `pytest_runtest_protocol` は firstresult のフックであり、pytest-forked の実装が
    forked テストを処理して結果を返した時点で以降の実装は呼ばれない。通常の
    実装として登録すると forked テストでは記録できないため、hookwrapper にする。
    hookwrapper の yield 前は、すべての実装より先に親プロセスで実行される。

    Args:
        item (pytest.Item): 実行しようとしているテスト.
        nextitem (pytest.Item | None): 次に実行するテスト.

    Yields:
        None: 以降の実装へ処理を渡す.
    """

    global _parent_native_threads
    _parent_native_threads = _native_thread_count()
    yield


def pytest_runtest_setup(item: pytest.Item) -> None:
    """親プロセスがスレッドを抱えた状態の forked テストを失敗させる。

    forked テストではこのフックも子プロセス側で実行される。子は fork 時点の
    親の状態を引き継ぐため、ここで基準からの増加を見れば親の汚染が分かる。

    Args:
        item (pytest.Item): 実行しようとしているテスト.
    """

    if item.get_closest_marker('forked') is None:
        return
    # 診断時に fork を無効化して挙動を調べる場合だけ検査を外す。通常の実行では設定しない。
    if os.environ.get('TC2026_SKIP_FORK_GUARD'):
        return

    if _ros_context_initialized_pid not in (None, os.getpid()):
        pytest.fail(
            'pytest の親プロセスで ROS context が初期化済みのため、forked テストは '
            'デッドロックする。ROS を起動するテストは pytest.mark.forked または '
            'subprocess で別プロセスへ隔離すること（conftest.py の docstring 参照）。',
            pytrace=False,
        )

    if _baseline_native_threads is None or _baseline_native_threads < 0:
        return
    if _parent_native_threads is None or _parent_native_threads <= _baseline_native_threads:
        return
    pytest.fail(
        f'fork 元の親プロセスでスレッドが増えている（起動時 {_baseline_native_threads} '
        f'→ {_parent_native_threads}）。この状態で fork するとロックを引き継いで'
        'デッドロックする。'
        '直前に実行されたテストのうち、ROS・HTTP サーバ・Qt・背景スレッドを'
        '起動するものを subprocess または pytest.mark.forked で隔離すること'
        '（conftest.py の docstring 参照）。',
        pytrace=False,
    )
