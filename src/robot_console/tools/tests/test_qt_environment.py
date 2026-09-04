"""qt_environment.fix_qt_plugin_path_conflict の単体テスト。

`cv2` によるQtプラグインパスの書き換えを、PyQt5自身のパスへ戻せることを
確認する。`cv2` / `PyQt5` のどちらも利用できない環境では skip する。
"""

import os

import pytest

pytest.importorskip('cv2')
from PyQt5 import QtCore  # noqa: E402

from robot_console.ui_qt.qt_environment import fix_qt_plugin_path_conflict  # noqa: E402


def test_fix_qt_plugin_path_conflict_restores_pyqt5_plugin_path(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM_PLUGIN_PATH', '/bogus/cv2/qt/plugins/platforms')
    monkeypatch.setenv('QT_PLUGIN_PATH', '/bogus/cv2/qt/plugins')

    fix_qt_plugin_path_conflict()

    expected_root = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    assert os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] == os.path.join(expected_root, 'platforms')
    assert os.environ['QT_PLUGIN_PATH'] == expected_root


def test_fix_qt_plugin_path_conflict_is_idempotent():
    fix_qt_plugin_path_conflict()
    first = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH')

    fix_qt_plugin_path_conflict()
    second = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH')

    assert first == second
