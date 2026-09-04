"""ConsoleLogTab / LogView の単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import pytest
from PyQt5 import QtGui, QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.launch_profile import LaunchProfileState
from robot_console.core.snapshot_model import ConsoleSnapshot
from robot_console.ui_qt.console_log_tab import ConsoleLogTab
from robot_console.ui_qt.widgets.log_view import LogView
from robot_console.utils import NodeLaunchStatus


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _sample_snapshot() -> ConsoleSnapshot:
    return ConsoleSnapshot(
        logs={
            'route_manager': ['[INFO] start', '[WARN] slow segment', '[ERROR] replan failed'],
            'route_follower': ['[INFO] following', '[DEBUG] tick'],
        },
        launch_profiles={
            'route_manager': LaunchProfileState(
                profile_id='route_manager', status=NodeLaunchStatus.RUNNING, process_id=111
            ),
            'route_follower': LaunchProfileState(
                profile_id='route_follower', status=NodeLaunchStatus.STOPPED
            ),
        },
        log_paths={'route_manager': '/tmp/route_manager.log'},
    )


def test_update_snapshot_populates_profile_table(qt_app):
    tab = ConsoleLogTab()
    tab.update_snapshot(_sample_snapshot())

    assert tab._profile_table.rowCount() == 2
    row_texts = {
        tab._profile_table.item(row, 0).text(): tab._profile_table.item(row, 1).text()
        for row in range(tab._profile_table.rowCount())
    }
    assert row_texts == {'route_follower': 'STOPPED', 'route_manager': 'RUNNING'}


def test_selecting_profile_row_updates_log_view_and_path(qt_app):
    tab = ConsoleLogTab()
    tab.update_snapshot(_sample_snapshot())

    tab.select_profile('route_manager')

    assert tab._log_view.toPlainText() == '[INFO] start\n[WARN] slow segment\n[ERROR] replan failed\n'
    assert tab._log_path_label.text() == 'ログファイル: /tmp/route_manager.log'


def test_profile_filter_combo_stays_in_sync_with_table_selection(qt_app):
    tab = ConsoleLogTab()
    tab.update_snapshot(_sample_snapshot())

    tab._profile_table.selectRow(0)  # route_follower（アルファベット順で先頭）

    assert tab._selected_profile_id == 'route_follower'
    assert tab._profile_filter_combo.currentText() == 'route_follower'


def test_level_filter_hides_non_matching_lines(qt_app):
    tab = ConsoleLogTab()
    tab.update_snapshot(_sample_snapshot())
    tab.select_profile('route_manager')

    tab._level_filter_combo.setCurrentText('ERROR')

    assert tab._log_view.toPlainText() == '[ERROR] replan failed\n'


def test_warn_error_panel_shows_only_selected_profile_when_selected(qt_app):
    tab = ConsoleLogTab()
    tab.update_snapshot(_sample_snapshot())
    tab.select_profile('route_manager')

    text = tab._warn_error_view.toPlainText()
    assert '[route_manager] [WARN] slow segment' in text
    assert '[route_manager] [ERROR] replan failed' in text
    assert 'route_follower' not in text


def test_warn_error_panel_covers_all_profiles_when_none_selected(qt_app):
    tab = ConsoleLogTab()
    tab.update_snapshot(_sample_snapshot())
    tab._profile_filter_combo.setCurrentText('すべて')

    text = tab._warn_error_view.toPlainText()
    assert '[route_manager] [ERROR] replan failed' in text


def test_no_profile_selected_shows_placeholder_path(qt_app):
    tab = ConsoleLogTab()
    tab.update_snapshot(_sample_snapshot())
    tab._profile_filter_combo.setCurrentText('すべて')

    assert tab._log_view.toPlainText() == ''
    assert tab._log_path_label.text() == 'ログファイル: -'


def test_log_view_set_lines_colors_by_level(qt_app):
    view = LogView()
    view.set_lines(['[WARN] caution', '[ERROR] boom'])

    cursor = QtGui.QTextCursor(view.document())
    cursor.setPosition(0)
    cursor.movePosition(QtGui.QTextCursor.EndOfLine, QtGui.QTextCursor.KeepAnchor)
    warn_line_color = cursor.charFormat().foreground().color().name()

    second_line_start = view.document().findBlockByNumber(1).position()
    cursor.setPosition(second_line_start)
    cursor.movePosition(QtGui.QTextCursor.EndOfLine, QtGui.QTextCursor.KeepAnchor)
    error_line_color = cursor.charFormat().foreground().color().name()

    assert warn_line_color != error_line_color
    assert error_line_color == '#f44336'


def test_log_view_highlight_marks_matching_text(qt_app):
    view = LogView()
    view.set_lines(['[INFO] hello world', '[INFO] another line'], highlight_term='world')

    document = view.document()
    cursor = document.find('world')
    assert not cursor.isNull()
    assert cursor.charFormat().background().color().name() != '#000000'


def test_log_view_tail_follow_scrolls_to_bottom(qt_app):
    view = LogView()
    view.resize(200, 60)
    many_lines = [f'[INFO] line {i}' for i in range(200)]

    view.set_lines(many_lines, tail_follow=True)

    scrollbar = view.verticalScrollBar()
    assert scrollbar.value() == scrollbar.maximum()


def test_log_view_without_tail_follow_preserves_scroll_position(qt_app):
    view = LogView()
    view.resize(200, 60)
    many_lines = [f'[INFO] line {i}' for i in range(200)]
    view.set_lines(many_lines, tail_follow=True)
    scrollbar = view.verticalScrollBar()
    scrollbar.setValue(0)

    view.set_lines(many_lines + ['[INFO] extra line'], tail_follow=False)

    assert scrollbar.value() == 0
