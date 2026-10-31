"""tools/tests 配下の pytest 共通セットアップ。

ROS 2 環境が有効化されていない場合でも `robot_console` パッケージを
import できるよう、`sensor_msgs` / `numpy` / `PIL` の最小スタブを用意する。
ROS 2 環境が有効な場合（`sensor_msgs` などが実際に import できる場合）は
本モジュールは何もしない。

また、`PyQt5` を使うテストがディスプレイの無い環境でも実行できるよう、他の
モジュールが `PyQt5` をimportする前に `QT_QPA_PLATFORM=offscreen` を既定値
として設定する。offscreen では地図タブの表示切り替え時にOpenGLコンテキストを
作成できない環境があるため、Qt Quick はソフトウェア描画を既定値にする。

`opencv-python` は import 時に `QT_QPA_PLATFORM_PLUGIN_PATH` を自身が同梱する
Qtプラグイン（cv2/qt/plugins、PyQt5と異なるQtビルド）へ書き換える。同一プロセス
内でPyQt5のQApplicationがこのパスからプラットフォームプラグインを読み込むと、
Qtライブラリのバージョン不整合により（タイミング依存で）セグメンテーション
フォルトを起こすことを確認したため、`robot_console.ui_qt.qt_environment` の
`fix_qt_plugin_path_conflict()` でPyQt5自身のプラグインパスへ明示的に戻す
（実運用のentry point `ui_qt_main.py` でも同じ関数を使用する）。

`ui_qt.widgets.map_view.MapView`（`QWebEngineView`）はQApplication生成前に
`Qt::AA_ShareOpenGLContexts` 属性が設定されていないと動作しないため、
`enable_qtwebengine_shared_opengl_contexts()` も同様にQApplication生成前へ
配置する（`ui_qt_main.py` でも同じ関数を使用する）。

さらに、同一プロセス内で複数の `QWebEngineView` を跨いでテストを実行すると、
QtWebEngine内部の状態（プロファイル初期化まわり）が汚染され、テストの実行順序
次第でセグメンテーションフォルトを起こすことを確認した。`QWebEngineView` を
実際に生成するテストだけを `pytest.mark.forked` で個別隔離する案も試したが、
それより前に実行される他のテスト（`launch_manager` のリーダースレッドや
`ThreadingHTTPServer` などバックグラウンドスレッドを作るテストを含む）が同一
プロセス内で先に走った状態から `os.fork()` すると、フォーク時に他スレッドが
保持していたロックが子プロセスへ引き継がれずデッドロックすることを確認した。
そのため、本ディレクトリ配下の全テストを一律 `pytest.mark.forked` 対象にする
（`pytest_collection_modifyitems()` 参照、要 `pytest-forked`）。
依存はワークスペースの `requirements.txt` で導入し、`pytest.ini` の
`required_plugins` で未導入時の実行を防ぐ。
ワークスペース全体の `pytest` 実行時も、本ディレクトリ配下のテストのみが
フォーク対象になり、他パッケージのテストには影響しない。

同じ理由から、フォーク元となる pytest の親プロセスは ROS context を初期化して
いてはならない。ROS context の初期化で生じる DDS の内部スレッドは
`rclpy.shutdown()` 後も残留するため、そのまま fork すると本ディレクトリの
`test_bag_recorder.py` や `test_console_node.py` のように子プロセスで
`rclpy.init()` を呼ぶテストがデッドロックする。ROS を起動するテストを
別プロセスへ隔離する規則と、その検査はワークスペース直下の `conftest.py`
に置く。

このワークスペースはnumpy/opencv等のバージョン固定のためPython venv
（`--system-site-packages`）の使用を前提とする（CLAUDE.md「実行環境」参照）。
venvが有効化されていない場合、pytest自体は動作し得るが依存関係の
バージョンが想定と異なる可能性があるため、収集時に警告のみ出す。
"""

from __future__ import annotations

import os
import sys
import types
from pathlib import Path

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
if os.environ['QT_QPA_PLATFORM'] == 'offscreen':
    os.environ.setdefault('QT_QUICK_BACKEND', 'software')

if not os.environ.get('VIRTUAL_ENV'):
    import warnings

    warnings.warn(
        'venvが有効化されていません。'
        'このワークスペースはPython venv(--system-site-packages)の使用を'
        '前提としています。意図しない場合は有効化してから再実行してください。',
        stacklevel=1,
    )

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from robot_console.ui_qt.qt_environment import (
        enable_qtwebengine_shared_opengl_contexts,
        fix_qt_plugin_path_conflict,
    )

    fix_qt_plugin_path_conflict()
    enable_qtwebengine_shared_opengl_contexts()
except ImportError:
    pass


def _install_sensor_stubs() -> None:
    sensor_msgs = types.ModuleType('sensor_msgs')
    msg_module = types.ModuleType('sensor_msgs.msg')

    class Image:  # pragma: no cover - スタブのみ
        height = 0
        width = 0
        encoding = 'rgb8'
        data = b''

    msg_module.Image = Image
    sensor_msgs.msg = msg_module
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = msg_module


def _install_numpy_stub() -> None:
    numpy_stub = types.ModuleType('numpy')

    def _frombuffer(data, dtype):  # pragma: no cover - スタブのみ
        return []

    numpy_stub.frombuffer = _frombuffer
    sys.modules['numpy'] = numpy_stub


def _install_pil_stub() -> None:
    pil_module = types.ModuleType('PIL')
    image_module = types.ModuleType('PIL.Image')
    imagetk_module = types.ModuleType('PIL.ImageTk')

    class _DummyImage:  # pragma: no cover - スタブのみ
        mode = 'RGB'
        width = 1
        height = 1
        info: dict = {}

        def resize(self, size, _filter=None):
            self.width, self.height = size
            return self

        def copy(self):
            return self

        def paste(self, _other, box=None, mask=None):
            return None

        def convert(self, _mode):
            return self

        def alpha_composite(self, _other):
            return None

    def _fromarray(_array, mode=None):
        return _DummyImage()

    def _new(mode, size, color):
        return _DummyImage()

    image_module.Image = _DummyImage
    image_module.NEAREST = 0
    image_module.fromarray = _fromarray
    image_module.new = _new
    imagetk_module.PhotoImage = _DummyImage
    pil_module.Image = image_module
    pil_module.ImageTk = imagetk_module
    sys.modules['PIL'] = pil_module
    sys.modules['PIL.Image'] = image_module
    sys.modules['PIL.ImageTk'] = imagetk_module


try:
    import sensor_msgs.msg  # noqa: F401
except ImportError:
    _install_sensor_stubs()

try:
    import numpy  # noqa: F401
except ImportError:
    _install_numpy_stub()

try:
    import PIL  # noqa: F401
except ImportError:
    _install_pil_stub()


_THIS_DIR = Path(__file__).resolve().parent


# isolated/ は通常の収集から外す。QWebEngineView を生成するテストを置いており、
# 同じインタプリタで他のQtテストと混ぜるとセグメンテーションフォルトを起こす。
# test_isolated_suite.py が TC2026_RUN_ISOLATED を設定して subprocess で実行する。
# 本 conftest は isolated/ を直接指定した場合にも読み込まれるため、環境変数で分岐する。
collect_ignore = [] if os.environ.get('TC2026_RUN_ISOLATED') else ['isolated']


def pytest_collection_modifyitems(items: list) -> None:
    """`isolated/` 配下のテストだけを `pytest.mark.forked` 対象にする。

    本ディレクトリ直下のテストは fork しない。`QWebEngineView` を生成するテストを
    `isolated/` へ分けたことで、同一インタプリタ内で Qt の状態が汚染されなくなった。
    fork する対象を減らすほど、親プロセスのスレッドを引き継いで停止する危険が減る。

    `isolated/` 側は fork が要る。同一インタプリタで複数の `QWebEngineView` を
    跨ぐとセグメンテーションフォルトを起こすためである。ただしその実行は
    `test_isolated_suite.py` が起こす子プロセスであり、fork 元の親は 3 ファイルしか
    収集していないため、他のテストのスレッドを持ち込まない。

    `pytest-forked` はルートの `pytest.ini` で必須プラグインとして宣言する。
    """

    for item in items:
        item_path = Path(str(item.fspath)).resolve()
        if item_path.parent == _THIS_DIR / 'isolated':
            item.add_marker(pytest.mark.forked)
