"""起動・設定タブ。

業務開始時に最初に使う画面。業務モード選択とプリセット適用、起動候補ツリー、
起動予定ノード一覧、ノード設定編集パネル、起動内容プレビューで構成する
（robot_console_gui_screen_function_design.md 4章）。

本タブは純粋な設定タブであり、ノードの起動・停止操作は置かない。実際の
起動・停止操作はダッシュボードタブの起動操作カード（`LaunchControlCard`）
が担い、本タブが管理する起動予定ノード一覧（`LaunchPlan`）を`plan_changed`
シグナルで参照する。
"""

from __future__ import annotations

from typing import Dict, List, Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.business_mode import (
    DRIVE_MODES,
    ENVIRONMENTS,
    LaunchPlan,
    get_preset,
)
from robot_console.core.launch_profile import (
    LaunchProfile,
    LaunchProfileState,
    LaunchProfileStore,
    build_initial_states,
    build_launch_args,
    resolve_effective_overrides,
)

from .widgets.argument_widget_hints import ENUM_ARGUMENTS, NUMBER_ARGUMENTS, widget_kind

# architecture_design.md 9.2節の category 定義に対応する表示名。
CATEGORY_LABELS: Dict[str, str] = {
    'real_robot_base': '実機基盤',
    'gps_gnss': 'GPS/GNSS',
    'simulation_base': 'シミュレーション基盤',
    'route_stack': '経路計画・追従',
    'drive_stack': '走行制御・mux',
    'obstacle_stack': '障害物監視',
    'perception_stack': '認識・判定',
    'visualization': '可視化',
}

PLAN_TABLE_COLUMNS = ['順', 'Profile', 'カテゴリ', 'Config', 'Override', '状態', 'Health', '']
PROFILE_ID_ROLE = QtCore.Qt.UserRole


class LaunchSettingsTab(QtWidgets.QWidget):
    """業務開始時に使う起動・設定画面。"""

    plan_changed = QtCore.pyqtSignal()
    business_mode_changed = QtCore.pyqtSignal(str, str)
    param_changed = QtCore.pyqtSignal(str, str)
    alternate_toggled = QtCore.pyqtSignal(str, bool)
    simulator_toggled = QtCore.pyqtSignal(str, bool)
    argument_changed = QtCore.pyqtSignal(str, str, str)

    def __init__(
        self,
        profile_store: Optional[LaunchProfileStore] = None,
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)

        self._profile_store = profile_store or LaunchProfileStore()
        self._profiles: List[LaunchProfile] = self._profile_store.load()
        self._profiles_by_id: Dict[str, LaunchProfile] = {
            profile.profile_id: profile for profile in self._profiles
        }
        self._states: Dict[str, LaunchProfileState] = build_initial_states(self._profiles)
        self._plan = LaunchPlan()
        self._selected_profile_id: Optional[str] = None
        self._argument_widgets: Dict[str, QtWidgets.QWidget] = {}

        outer = QtWidgets.QVBoxLayout(self)
        outer.addLayout(self._build_toolbar())

        splitter = QtWidgets.QHBoxLayout()
        splitter.addWidget(self._build_candidate_tree_panel(), 1)
        splitter.addWidget(self._build_plan_panel(), 2)
        splitter.addWidget(self._build_config_panel(), 2)
        outer.addLayout(splitter, 1)

        outer.addWidget(self._build_preview_panel())

        self._rebuild_tree()
        self._refresh_plan_table()
        self._update_config_panel()

    # ---------- ツールバー（4.3節 業務モード選択） ----------
    def _build_toolbar(self) -> QtWidgets.QHBoxLayout:
        layout = QtWidgets.QHBoxLayout()

        self._environment_combo = QtWidgets.QComboBox()
        self._environment_combo.addItems(ENVIRONMENTS)
        self._drive_mode_combo = QtWidgets.QComboBox()
        self._drive_mode_combo.addItems(DRIVE_MODES)

        # 業務モードはプリセット適用の入力であると同時に、ダッシュボードの
        # 運行フェーズ領域が表示する実行環境・走行モードの元データでもあるため、
        # プリセット適用を待たず選択時点で通知する。
        self._environment_combo.currentTextChanged.connect(self._on_business_mode_changed)
        self._drive_mode_combo.currentTextChanged.connect(self._on_business_mode_changed)

        apply_preset_button = QtWidgets.QPushButton('プリセット適用')
        apply_preset_button.clicked.connect(self._on_apply_preset_clicked)

        layout.addWidget(QtWidgets.QLabel('実行環境:'))
        layout.addWidget(self._environment_combo)
        layout.addWidget(QtWidgets.QLabel('走行モード:'))
        layout.addWidget(self._drive_mode_combo)
        layout.addWidget(apply_preset_button)
        layout.addStretch(1)
        return layout

    def apply_preset(self, environment: str, drive_mode: str) -> None:
        """指定した業務モードのプリセットを適用する（ダッシュボードからの呼び出し用）。

        コンボボックスの選択をトップバーの「プリセット適用」ボタンと同じ値へ
        揃えたうえで、同じ適用ロジック（`_on_apply_preset_clicked`）を呼ぶ。
        起動予定ノード一覧（`LaunchPlan`）の更新経路を一本化し、ダッシュボード
        側で別の実体を持たせないようにするための入口である。
        """

        environment_index = self._environment_combo.findText(environment)
        if environment_index >= 0:
            self._environment_combo.setCurrentIndex(environment_index)
        drive_mode_index = self._drive_mode_combo.findText(drive_mode)
        if drive_mode_index >= 0:
            self._drive_mode_combo.setCurrentIndex(drive_mode_index)
        self._on_apply_preset_clicked()

    def _on_business_mode_changed(self, _text: str) -> None:
        """業務モードコンボの選択変更を外部（ConsoleCore・ダッシュボード）へ通知する。"""

        self.business_mode_changed.emit(
            self._environment_combo.currentText(), self._drive_mode_combo.currentText()
        )

    def _on_apply_preset_clicked(self) -> None:
        if self._plan.ordered_profile_ids:
            confirmed = QtWidgets.QMessageBox.question(
                self,
                'プリセット適用確認',
                '現在の起動予定を破棄してプリセットを適用します。よろしいですか。',
            )
            if confirmed != QtWidgets.QMessageBox.Yes:
                return

        environment = self._environment_combo.currentText()
        drive_mode = self._drive_mode_combo.currentText()
        preset = get_preset(environment, drive_mode)

        self._plan.apply_preset(preset)
        for entry in preset:
            state = self._states.get(entry.profile_id)
            if state is None:
                continue
            state.simulator_enabled = entry.use_simulator_alternate
            for key, value in entry.overrides.items():
                state.override_inputs[key] = value

        self._sync_tree_checkboxes_with_plan()
        self._refresh_plan_table()
        self._update_config_panel()

    # ---------- 起動候補ツリー（4.4節） ----------
    def _build_candidate_tree_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('起動候補ツリー')
        self._tree = QtWidgets.QTreeWidget()
        self._tree.setHeaderLabels(['Profile', '状態', 'Health'])
        self._tree.itemChanged.connect(self._on_tree_item_changed)

        layout = QtWidgets.QVBoxLayout(group)
        layout.addWidget(self._tree)
        return group

    def _rebuild_tree(self) -> None:
        self._tree.blockSignals(True)
        self._tree.clear()
        self._tree_items: Dict[str, QtWidgets.QTreeWidgetItem] = {}

        by_category: Dict[str, List[LaunchProfile]] = {}
        for profile in self._profiles:
            by_category.setdefault(profile.category, []).append(profile)

        for category, profiles in by_category.items():
            category_item = QtWidgets.QTreeWidgetItem(
                [CATEGORY_LABELS.get(category, category), '', '']
            )
            self._tree.addTopLevelItem(category_item)
            for profile in profiles:
                leaf = QtWidgets.QTreeWidgetItem([profile.display_name, 'STOPPED', 'UNKNOWN'])
                leaf.setData(0, PROFILE_ID_ROLE, profile.profile_id)
                leaf.setFlags(leaf.flags() | QtCore.Qt.ItemIsUserCheckable)
                leaf.setCheckState(0, QtCore.Qt.Unchecked)
                category_item.addChild(leaf)
                self._tree_items[profile.profile_id] = leaf
            category_item.setExpanded(True)

        for column in range(self._tree.columnCount()):
            self._tree.resizeColumnToContents(column)
        self._tree.blockSignals(False)

    def _on_tree_item_changed(self, item: QtWidgets.QTreeWidgetItem, column: int) -> None:
        if column != 0:
            return
        profile_id = item.data(0, PROFILE_ID_ROLE)
        if not profile_id:
            return
        if item.checkState(0) == QtCore.Qt.Checked:
            self._plan.add(profile_id)
        else:
            self._plan.remove(profile_id)
        self._refresh_plan_table()

    def _sync_tree_checkboxes_with_plan(self) -> None:
        self._tree.blockSignals(True)
        for profile_id, item in self._tree_items.items():
            state = QtCore.Qt.Checked if self._plan.contains(profile_id) else QtCore.Qt.Unchecked
            item.setCheckState(0, state)
        self._tree.blockSignals(False)

    # ---------- 起動予定ノード一覧（4.5節） ----------
    def _build_plan_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('起動予定ノード一覧')
        self._plan_table = QtWidgets.QTableWidget(0, len(PLAN_TABLE_COLUMNS))
        self._plan_table.setHorizontalHeaderLabels(PLAN_TABLE_COLUMNS)
        self._plan_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self._plan_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._plan_table.itemSelectionChanged.connect(self._on_plan_selection_changed)

        move_up_button = QtWidgets.QPushButton('起動順を上げる')
        move_up_button.clicked.connect(lambda: self._move_selected_plan_entry(-1))
        move_down_button = QtWidgets.QPushButton('起動順を下げる')
        move_down_button.clicked.connect(lambda: self._move_selected_plan_entry(1))
        button_row = QtWidgets.QHBoxLayout()
        button_row.addWidget(move_up_button)
        button_row.addWidget(move_down_button)

        layout = QtWidgets.QVBoxLayout(group)
        layout.addWidget(self._plan_table)
        layout.addLayout(button_row)
        return group

    def _refresh_plan_table(self) -> None:
        self._plan_table.blockSignals(True)
        self._plan_table.setRowCount(len(self._plan.ordered_profile_ids))
        for row, profile_id in enumerate(self._plan.ordered_profile_ids):
            profile = self._profiles_by_id.get(profile_id)
            state = self._states.get(profile_id)
            if profile is None or state is None:
                continue

            self._plan_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(row + 1)))
            self._plan_table.setItem(row, 1, QtWidgets.QTableWidgetItem(profile.display_name))
            self._plan_table.setItem(
                row, 2, QtWidgets.QTableWidgetItem(CATEGORY_LABELS.get(profile.category, profile.category))
            )
            self._plan_table.setItem(
                row, 3, QtWidgets.QTableWidgetItem(state.selected_param or '-')
            )
            overrides = resolve_effective_overrides(profile, state)
            override_text = ', '.join(f'{k}={v}' for k, v in overrides.items()) or '-'
            self._plan_table.setItem(row, 4, QtWidgets.QTableWidgetItem(override_text))
            self._plan_table.setItem(row, 5, QtWidgets.QTableWidgetItem(state.status.name))
            self._plan_table.setItem(row, 6, QtWidgets.QTableWidgetItem('UNKNOWN'))

            remove_button = QtWidgets.QPushButton('一覧から外す')
            remove_button.clicked.connect(
                lambda _checked=False, pid=profile_id: self._on_remove_from_plan_clicked(pid)
            )
            self._plan_table.setCellWidget(row, 7, remove_button)

            for column in range(7):
                item = self._plan_table.item(row, column)
                if item is not None:
                    item.setData(PROFILE_ID_ROLE, profile_id)
        for column in range(7):
            self._plan_table.resizeColumnToContents(column)
        self._plan_table.setColumnWidth(7, 140)
        self._plan_table.blockSignals(False)
        self._update_config_panel()
        self.plan_changed.emit()

    def _on_remove_from_plan_clicked(self, profile_id: str) -> None:
        self._plan.remove(profile_id)
        self._sync_tree_checkboxes_with_plan()
        self._refresh_plan_table()

    def _move_selected_plan_entry(self, direction: int) -> None:
        profile_id = self._selected_profile_id
        if not profile_id:
            return
        if direction < 0:
            self._plan.move_up(profile_id)
        else:
            self._plan.move_down(profile_id)
        self._refresh_plan_table()
        self._select_plan_row_for(profile_id)

    def _on_plan_selection_changed(self) -> None:
        rows = self._plan_table.selectionModel().selectedRows()
        if not rows:
            self._selected_profile_id = None
        else:
            item = self._plan_table.item(rows[0].row(), 0)
            self._selected_profile_id = item.data(PROFILE_ID_ROLE) if item else None
        self._update_config_panel()

    def _select_plan_row_for(self, profile_id: str) -> None:
        for row in range(self._plan_table.rowCount()):
            item = self._plan_table.item(row, 0)
            if item is not None and item.data(PROFILE_ID_ROLE) == profile_id:
                self._plan_table.selectRow(row)
                return

    # ---------- ノード設定編集パネル（4.6節・4.7節） ----------
    def _build_config_panel(self) -> QtWidgets.QGroupBox:
        self._config_group = QtWidgets.QGroupBox('ノード設定編集パネル')
        self._config_form = QtWidgets.QFormLayout()
        self._config_group.setLayout(self._config_form)
        return self._config_group

    def _clear_config_form(self) -> None:
        """ノード設定編集パネルの既存ウィジェットを破棄する。

        `QFormLayout.removeRow()` はウィジェットを即座に削除するため、引数編集
        ウィジェット自身のシグナル処理中（`_on_argument_changed()` →
        `_refresh_plan_table()` → `_update_config_panel()` → 本メソッド）に
        呼ばれると、シグナル送出元のウィジェットが処理の途中で破棄される。
        `takeRow()` でレイアウトから外し、破棄を `deleteLater()` で次のイベント
        ループへ遅らせることでこの再入を避ける。
        """

        while self._config_form.rowCount():
            row = self._config_form.takeRow(0)
            for item in (row.labelItem, row.fieldItem):
                if item is None:
                    continue
                widget = item.widget()
                if widget is None:
                    continue
                widget.setParent(None)
                widget.deleteLater()

    def _update_config_panel(self) -> None:
        self._clear_config_form()
        self._argument_widgets = {}

        profile_id = self._selected_profile_id
        profile = self._profiles_by_id.get(profile_id) if profile_id else None
        state = self._states.get(profile_id) if profile_id else None
        if profile is None or state is None:
            self._config_group.setTitle('ノード設定編集パネル（未選択）')
            self._update_preview()
            return
        self._config_group.setTitle(f'ノード設定編集パネル: {profile.display_name}')

        param_edit = QtWidgets.QLineEdit(state.selected_param or '')
        param_edit.editingFinished.connect(
            lambda: self._on_param_path_edited(profile_id, param_edit.text())
        )
        self._config_form.addRow('config:', param_edit)

        if profile.alternate_launch_file:
            alternate_check = QtWidgets.QCheckBox(profile.launch_toggle_label or '代替launchを使用')
            alternate_check.setChecked(state.use_alternate_launch)
            alternate_check.toggled.connect(
                lambda checked, pid=profile_id: self._on_alternate_toggled(pid, checked)
            )
            self._config_form.addRow('', alternate_check)

        if profile.simulator_launch_file:
            simulator_check = QtWidgets.QCheckBox('Simulator代替を使用')
            simulator_check.setChecked(state.simulator_enabled)
            simulator_check.toggled.connect(
                lambda checked, pid=profile_id: self._on_simulator_toggled(pid, checked)
            )
            self._config_form.addRow('', simulator_check)

        for argument_name in profile.user_arguments:
            widget = self._build_argument_widget(profile_id, state, argument_name)
            self._argument_widgets[argument_name] = widget
            self._config_form.addRow(f'{argument_name}:', widget)

        self._update_preview()

    def _build_argument_widget(
        self, profile_id: str, state: LaunchProfileState, argument_name: str
    ) -> QtWidgets.QWidget:
        kind = widget_kind(argument_name)
        current_value = state.override_inputs.get(argument_name, '')

        if kind == 'bool':
            widget = QtWidgets.QCheckBox()
            widget.setChecked(current_value.lower() == 'true')
            widget.toggled.connect(
                lambda checked, pid=profile_id, name=argument_name: self._on_argument_changed(
                    pid, name, 'true' if checked else 'false'
                )
            )
            return widget

        if kind == 'enum':
            widget = QtWidgets.QComboBox()
            options = ENUM_ARGUMENTS[argument_name]
            widget.addItems(options)
            if current_value in options:
                widget.setCurrentText(current_value)
            widget.currentTextChanged.connect(
                lambda text, pid=profile_id, name=argument_name: self._on_argument_changed(
                    pid, name, text
                )
            )
            return widget

        if kind == 'number':
            number_type = NUMBER_ARGUMENTS[argument_name]
            if number_type is int:
                spin: QtWidgets.QAbstractSpinBox = QtWidgets.QSpinBox()
                spin.setRange(-1000000, 1000000)
                if current_value:
                    spin.setValue(int(float(current_value)))
            else:
                spin = QtWidgets.QDoubleSpinBox()
                spin.setRange(-1000000.0, 1000000.0)
                if current_value:
                    spin.setValue(float(current_value))
            spin.valueChanged.connect(
                lambda value, pid=profile_id, name=argument_name: self._on_argument_changed(
                    pid, name, str(value)
                )
            )
            return spin

        widget = QtWidgets.QLineEdit(current_value)
        widget.editingFinished.connect(
            lambda pid=profile_id, name=argument_name, w=widget: self._on_argument_changed(
                pid, name, w.text()
            )
        )
        return widget

    def _on_param_path_edited(self, profile_id: str, text: str) -> None:
        state = self._states.get(profile_id)
        if state is None:
            return
        state.selected_param = text or None
        self._refresh_plan_table()
        self.param_changed.emit(profile_id, text)

    def _on_alternate_toggled(self, profile_id: str, checked: bool) -> None:
        state = self._states.get(profile_id)
        if state is None:
            return
        state.use_alternate_launch = checked
        self._update_preview()
        self.alternate_toggled.emit(profile_id, checked)

    def _on_simulator_toggled(self, profile_id: str, checked: bool) -> None:
        state = self._states.get(profile_id)
        if state is None:
            return
        state.simulator_enabled = checked
        self._update_preview()
        self.simulator_toggled.emit(profile_id, checked)

    def _on_argument_changed(self, profile_id: str, argument_name: str, value: str) -> None:
        state = self._states.get(profile_id)
        if state is None:
            return
        state.override_inputs[argument_name] = value
        self._refresh_plan_table()
        self.argument_changed.emit(profile_id, argument_name, value)

    # ---------- 起動内容プレビュー（4.8節） ----------
    def _build_preview_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('起動内容プレビュー')
        self._preview_text = QtWidgets.QPlainTextEdit()
        self._preview_text.setReadOnly(True)
        self._preview_text.setMaximumHeight(120)
        layout = QtWidgets.QVBoxLayout(group)
        layout.addWidget(self._preview_text)
        return group

    def _update_preview(self) -> None:
        profile_id = self._selected_profile_id
        profile = self._profiles_by_id.get(profile_id) if profile_id else None
        state = self._states.get(profile_id) if profile_id else None
        if profile is None or state is None:
            self._preview_text.setPlainText('起動予定ノード一覧からノードを選択してください。')
            return

        overrides = resolve_effective_overrides(profile, state)
        args = build_launch_args(
            profile,
            param_path=state.selected_param,
            use_alternate=state.use_alternate_launch,
            overrides=overrides,
        )
        order = (
            self._plan.ordered_profile_ids.index(profile_id) + 1
            if self._plan.contains(profile_id)
            else None
        )

        lines = [f'Profile: {profile.display_name} ({profile.profile_id})']
        lines.append(f'起動順: {order if order is not None else "起動予定に未追加"}')
        lines.append('Command:')
        lines.append('  ' + ' '.join(args))
        if profile.health_topics:
            lines.append('Health:')
            lines.extend(f'  {topic}' for topic in profile.health_topics)
        self._preview_text.setPlainText('\n'.join(lines))

    # ---------- 外部公開API ----------
    @property
    def plan(self) -> LaunchPlan:
        """起動予定ノード一覧（`LaunchPlan`）を返す。"""

        return self._plan

    @property
    def profiles_by_id(self) -> Dict[str, LaunchProfile]:
        """profile_idをキーとした`LaunchProfile`辞書を返す。"""

        return dict(self._profiles_by_id)

    @property
    def environment(self) -> str:
        """現在選択中の実行環境を返す。"""

        return self._environment_combo.currentText()

    @property
    def drive_mode(self) -> str:
        """現在選択中の走行モードを返す。"""

        return self._drive_mode_combo.currentText()

    def state_for(self, profile_id: str) -> Optional[LaunchProfileState]:
        """指定profileの現在の編集状態を返す。"""

        return self._states.get(profile_id)

    def update_launch_states(self, states: Dict[str, LaunchProfileState]) -> None:
        """外部から通知された起動状態でツリー・一覧の状態表示を更新する。"""

        for profile_id, state in states.items():
            if profile_id not in self._states:
                continue
            self._states[profile_id].status = state.status
            self._states[profile_id].process_id = state.process_id
            self._states[profile_id].error_message = state.error_message
            item = self._tree_items.get(profile_id)
            if item is not None:
                item.setText(1, state.status.name)
        self._refresh_plan_table()
