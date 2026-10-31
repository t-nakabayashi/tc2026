"""`isolated/` 配下のテストを別インタプリタで実行する。

`isolated/` には `QWebEngineView` を生成するテストを置いている。同一インタプリタ内で
複数の `QWebEngineView` を跨いでテストを実行すると QtWebEngine 内部の状態が汚染され、
実行順序次第でセグメンテーションフォルトを起こす。`os.fork()` では親のメモリと
ロック状態を複製するためこの汚染を避けられないが、`subprocess` は新しいインタプリタを
起動するため影響を受けない。

子プロセス側の pytest は `isolated/` だけを収集するため、fork 元の親プロセスは
Qt も ROS も触れていない状態に保たれる。個々のテストの粒度は子プロセス側で維持され、
失敗時は子の出力をそのまま表示する。
"""

import os
from pathlib import Path
import subprocess
import sys

ISOLATED_DIR = Path(__file__).resolve().parent / 'isolated'

# Qt の初期化とタイル読み込みを含むため、単体テストとしては長めに取る。
TIMEOUT_S = 600


def test_isolated_qtwebengine_suite_passes() -> None:
    """`isolated/` の全テストが別インタプリタで成功することを確認する。"""
    completed = subprocess.run(
        [
            sys.executable,
            '-m',
            'pytest',
            '-q',
            '-p',
            'no:cacheprovider',
            str(ISOLATED_DIR),
        ],
        capture_output=True,
        text=True,
        timeout=TIMEOUT_S,
        env=dict(os.environ, TC2026_RUN_ISOLATED='1'),
    )
    assert completed.returncode == 0, (
        'isolated/ のテストが失敗した。以下は子プロセスの出力。\n'
        f'--- stdout ---\n{completed.stdout}\n--- stderr ---\n{completed.stderr}'
    )
