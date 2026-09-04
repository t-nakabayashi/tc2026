"""コンソールログタブ。

起動後の詳細確認、異常調査、開発時のデバッグを目的とする画面
（robot_console_gui_screen_function_design.md 5章）。走行中に常時見る画面
ではなく、profile別ログ、WARN/ERROR統合ログ、検索/フィルタ、tail追従を提供する。
"""

from __future__ import annotations

from typing import Dict, List, Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.launch_profile import LaunchProfileState
from robot_console.core.log_manager import count_levels, filter_levels
from robot_console.core.snapshot_model import ConsoleSnapshot

from .widgets.log_view import LogView

PROFILE_FILTER_ALL = 'すべて'
LEVEL_FILTER_ALL = 'すべて'
LEVEL_FILTER_OPTIONS = [LEVEL_FILTER_ALL, 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']
PROFILE_TABLE_COLUMNS = ['Profile', '状態', 'PID', 'エラー', 'WARN', 'ERROR']
PROFILE_ID_ROLE = QtCore.Qt.UserRole


class ConsoleLogTab(QtWidgets.QWidget):
    """起動後の詳細確認・異常調査・開発時デバッグ用の画面。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)

        self._logs: Dict[str, List[str]] = {}
        self._launch_states: Dict[str, LaunchProfileState] = {}
        self._log_paths: Dict[str, Optional[str]] = {}
        self._selected_profile_id: Optional[str] = None

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(self._build_toolbar())

        splitter = QtWidgets.QHBoxLayout()
        splitter.addWidget(self._build_profile_list_panel(), 1)
        splitter.addWidget(self._build_log_panel(), 2)
        layout.addLayout(splitter, 1)

        layout.addWidget(self._build_warn_error_panel())

    # ---------- ツールバー（5.2節） ----------
    def _build_toolbar(self) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()

        self._profile_filter_combo = QtWidgets.QComboBox()
        self._profile_filter_combo.addItem(PROFILE_FILTER_ALL)
        self._profile_filter_combo.currentTextChanged.connect(self._on_profile_filter_changed)

        self._level_filter_combo = QtWidgets.QComboBox()
        self._level_filter_combo.addItems(LEVEL_FILTER_OPTIONS)
        self._level_filter_combo.currentTextChanged.connect(lambda _text: self._refresh_log_view())

        self._search_edit = QtWidgets.QLineEdit()
        self._search_edit.setPlaceholderText('検索語')
        self._search_edit.textChanged.connect(lambda _text: self._refresh_log_view())

        self._tail_follow_check = QtWidgets.QCheckBox('tail追従')
        self._tail_follow_check.setChecked(True)
        self._tail_follow_check.toggled.connect(lambda _checked: self._refresh_log_view())

        row.addWidget(QtWidgets.QLabel('Profile:'))
        row.addWidget(self._profile_filter_combo)
        row.addWidget(QtWidgets.QLabel('Level:'))
        row.addWidget(self._level_filter_combo)
        row.addWidget(QtWidgets.QLabel('検索:'))
        row.addWidget(self._search_edit)
        row.addWidget(self._tail_follow_check)
        row.addStretch(1)
        return row

    # ---------- profile一覧（5.3節） ----------
    def _build_profile_list_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('profile一覧')
        self._profile_table = QtWidgets.QTableWidget(0, len(PROFILE_TABLE_COLUMNS))
        self._profile_table.setHorizontalHeaderLabels(PROFILE_TABLE_COLUMNS)
        self._profile_table.horizontalHeader().setStretchLastSection(True)
        self._profile_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self._profile_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._profile_table.itemSelectionChanged.connect(self._on_profile_table_selection_changed)

        layout = QtWidgets.QVBoxLayout(group)
        layout.addWidget(self._profile_table)
        return group

    # ---------- 選択中profileのリアルタイムログ（5.3節） ----------
    def _build_log_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('選択中profileのリアルタイムログ')
        self._log_view = LogView()
        self._log_path_label = QtWidgets.QLabel('ログファイル: -')
        layout = QtWidgets.QVBoxLayout(group)
        layout.addWidget(self._log_view)
        layout.addWidget(self._log_path_label)
        return group

    # ---------- WARN/ERROR統合ログ（5.2節） ----------
    def _build_warn_error_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('WARN/ERROR統合ログ')
        self._warn_error_view = LogView()
        layout = QtWidgets.QVBoxLayout(group)
        layout.addWidget(self._warn_error_view)
        return group

    # ---------- Snapshot反映 ----------
    def update_snapshot(self, snapshot: ConsoleSnapshot) -> None:
        """`ConsoleSnapshot` のlogs/launch_profiles/log_pathsを反映する。"""

        self._logs = snapshot.logs
        self._launch_states = snapshot.launch_profiles
        self._log_paths = snapshot.log_paths
        self._refresh_profile_filter_options()
        self._refresh_profile_table()
        self._refresh_log_view()
        self._refresh_warn_error_view()

    def _known_profile_ids(self) -> List[str]:
        return sorted(self._logs.keys() | self._launch_states.keys())

    def _refresh_profile_filter_options(self) -> None:
        current = self._profile_filter_combo.currentText()
        self._profile_filter_combo.blockSignals(True)
        self._profile_filter_combo.clear()
        self._profile_filter_combo.addItem(PROFILE_FILTER_ALL)
        self._profile_filter_combo.addItems(self._known_profile_ids())
        index = self._profile_filter_combo.findText(current)
        self._profile_filter_combo.setCurrentIndex(index if index >= 0 else 0)
        self._profile_filter_combo.blockSignals(False)

    def _refresh_profile_table(self) -> None:
        profile_ids = self._known_profile_ids()
        self._profile_table.blockSignals(True)
        self._profile_table.setRowCount(len(profile_ids))
        for row, profile_id in enumerate(profile_ids):
            state = self._launch_states.get(profile_id)
            counts = count_levels(self._logs.get(profile_id, []))

            status_text = state.status.name if state else 'UNKNOWN'
            pid_text = str(state.process_id) if state and state.process_id else '-'
            error_text = state.error_message if state and state.error_message else '-'

            values = [
                profile_id,
                status_text,
                pid_text,
                error_text,
                str(counts.warn),
                str(counts.error),
            ]
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setData(PROFILE_ID_ROLE, profile_id)
                self._profile_table.setItem(row, column, item)
        for column in range(len(PROFILE_TABLE_COLUMNS) - 1):
            self._profile_table.resizeColumnToContents(column)
        self._profile_table.blockSignals(False)

        if self._selected_profile_id:
            self._select_profile_row(self._selected_profile_id)

    def _select_profile_row(self, profile_id: str) -> None:
        for row in range(self._profile_table.rowCount()):
            item = self._profile_table.item(row, 0)
            if item is not None and item.text() == profile_id:
                self._profile_table.blockSignals(True)
                self._profile_table.selectRow(row)
                self._profile_table.blockSignals(False)
                return

    def _on_profile_table_selection_changed(self) -> None:
        rows = self._profile_table.selectionModel().selectedRows()
        if not rows:
            return
        item = self._profile_table.item(rows[0].row(), 0)
        if item is None:
            return
        self._set_selected_profile(item.text())

    def _on_profile_filter_changed(self, text: str) -> None:
        if text == PROFILE_FILTER_ALL:
            self._selected_profile_id = None
            self._refresh_log_view()
            return
        self._set_selected_profile(text)

    def _set_selected_profile(self, profile_id: str) -> None:
        self._selected_profile_id = profile_id
        self._select_profile_row(profile_id)
        index = self._profile_filter_combo.findText(profile_id)
        if index >= 0:
            self._profile_filter_combo.blockSignals(True)
            self._profile_filter_combo.setCurrentIndex(index)
            self._profile_filter_combo.blockSignals(False)
        self._refresh_log_view()

    def select_profile(self, profile_id: str) -> None:
        """外部（Node Healthカード等）からのprofile指定選択に応じる。"""

        self._set_selected_profile(profile_id)

    def _refresh_log_view(self) -> None:
        tail_follow = self._tail_follow_check.isChecked()
        if self._selected_profile_id is None:
            self._log_view.set_lines([], tail_follow=tail_follow)
            self._log_path_label.setText('ログファイル: -')
            return

        lines = self._logs.get(self._selected_profile_id, [])
        level = self._level_filter_combo.currentText()
        if level != LEVEL_FILTER_ALL:
            lines = filter_levels(lines, (level,))

        self._log_view.set_lines(
            lines, highlight_term=self._search_edit.text().strip(), tail_follow=tail_follow
        )
        log_path = self._log_paths.get(self._selected_profile_id)
        self._log_path_label.setText(f'ログファイル: {log_path or "-"}')

    def _refresh_warn_error_view(self) -> None:
        target_ids = (
            [self._selected_profile_id]
            if self._selected_profile_id
            else self._known_profile_ids()
        )
        merged: List[str] = []
        for profile_id in target_ids:
            lines = filter_levels(self._logs.get(profile_id, []), ('WARN', 'ERROR', 'FATAL'))
            merged.extend(f'[{profile_id}] {line.rstrip()}' for line in lines)
        self._warn_error_view.set_lines(
            merged, tail_follow=self._tail_follow_check.isChecked()
        )
