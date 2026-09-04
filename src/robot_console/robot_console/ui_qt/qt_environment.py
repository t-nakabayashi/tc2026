"""PyQt5起動前に整えておくべきQt関連の環境設定。

`opencv-python` は `import cv2` の時点で `QT_QPA_PLATFORM_PLUGIN_PATH` を
自身が同梱するQtプラグイン（PyQt5とは異なるQtビルド）へ書き換える。
`robot_console.utils`（`core.launch_profile` 等が依存する）は `cv2` を
importするため、同一プロセスでPyQt5の `QApplication` を作成する経路では、
書き換え後の環境変数のままだとQtライブラリのバージョン不整合により
（タイミング依存で）セグメンテーションフォルトを起こし得る。

`QApplication` を作成する前に本モジュールの `fix_qt_plugin_path_conflict()`
を呼び、PyQt5自身のプラグインパスへ明示的に戻す。

`ui_qt.widgets.map_view.MapView`（`QWebEngineView`）を使う経路では、これに加えて
`enable_qtwebengine_shared_opengl_contexts()` も `QApplication` 生成前に呼ぶ。
QtWebEngineは内部でOpenGLコンテキストを使うため、`Qt::AA_ShareOpenGLContexts`
属性を`QApplication`生成前に設定しておかないと、他のQtウィジェットが先に
生成された状態で`QWebEngineView`を生成した際にセグメンテーションフォルトを
起こし得る。
"""

from __future__ import annotations

import os


def fix_qt_plugin_path_conflict() -> None:
    """`cv2` によるQtプラグインパスの書き換えをPyQt5自身のパスへ戻す。

    `cv2` が未importの環境やimportに失敗する環境では何もしない。
    """

    try:
        import cv2  # noqa: F401
    except ImportError:
        return

    try:
        from PyQt5 import QtCore
    except ImportError:
        return

    plugin_root = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    if not plugin_root:
        return
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = os.path.join(plugin_root, 'platforms')
    os.environ['QT_PLUGIN_PATH'] = plugin_root


def enable_qtwebengine_shared_opengl_contexts() -> None:
    """`QWebEngineView` に必要な共有OpenGLコンテキスト設定を行う。

    `QApplication` インスタンス生成前に呼ぶ必要がある。PyQt5が未importの
    環境では何もしない。
    """

    try:
        from PyQt5 import QtCore, QtWidgets
    except ImportError:
        return

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts)
