"""起動操作カード: ダッシュボードから業務モード選択・一斉/個別のノード起動・停止を行う。

起動予定ノード一覧（`LaunchPlan`）は起動・設定タブ（`LaunchSettingsTab`）が
唯一管理する実体であり、本カードはそれを複製しない。実行環境・走行モードの
選択とプリセット適用はダッシュボードからも行えるようにするが、
`apply_preset_requested` シグナルで起動・設定タブ側の同一ロジック
（`LaunchSettingsTab.apply_preset`）を呼び出す形とし、起動予定ノード一覧が
複数箇所で食い違わないようにする。QWidgetは`ros2 launch`を直接実行せず、
操作はpyqtSignalで外部（将来の `ros/console_node.py` ⇔ `ConsoleCore`）へ
通知する。
"""

from __future__ import annotations

from typing import Dict, List, Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.business_mode import DRIVE_MODES, ENVIRONMENTS
from robot_console.core.launch_profile import LaunchProfile

PROFILE_ID_ROLE = QtCore.Qt.UserRole


def _set_combo_text(combo: QtWidgets.QComboBox, text: str) -> None:
    """シグナルを発火させずにコンボボックスの選択値を設定する。"""

    index = combo.findText(text)
    if index < 0:
        return
    combo.blockSignals(True)
    combo.setCurrentIndex(index)
    combo.blockSignals(False)


class LaunchControlCard(QtWidgets.QGroupBox):
    """実行環境・走行モードの選択と、一斉/個別の起動・停止操作を提供するカード。"""

    apply_preset_requested = QtCore.pyqtSignal(str, str)
    launch_all_requested = QtCore.pyqtSignal(list)
    stop_all_requested = QtCore.pyqtSignal(list)
    launch_requested = QtCore.pyqtSignal(str)
    stop_requested = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__('起動操作', parent)

        self._ordered_profile_ids: List[str] = []

        self._environment_combo = QtWidgets.QComboBox()
        self._environment_combo.addItems(ENVIRONMENTS)
        self._drive_mode_combo = QtWidgets.QComboBox()
        self._drive_mode_combo.addItems(DRIVE_MODES)
        apply_preset_button = QtWidgets.QPushButton('プリセット適用')
        apply_preset_button.clicked.connect(
            lambda: self.apply_preset_requested.emit(
                self._environment_combo.currentText(), self._drive_mode_combo.currentText()
            )
        )

        self._node_combo = QtWidgets.QComboBox()
        self._node_combo.setMinimumWidth(220)

        launch_all_button = QtWidgets.QPushButton('起動予定ノードを一斉起動')
        launch_all_button.clicked.connect(
            lambda: self.launch_all_requested.emit(list(self._ordered_profile_ids))
        )
        stop_all_button = QtWidgets.QPushButton('起動予定ノードを一斉停止')
        stop_all_button.clicked.connect(
            lambda: self.stop_all_requested.emit(list(self._ordered_profile_ids))
        )
        launch_one_button = QtWidgets.QPushButton('選択ノードを起動')
        launch_one_button.clicked.connect(self._on_launch_selected_clicked)
        stop_one_button = QtWidgets.QPushButton('選択ノードを停止')
        stop_one_button.clicked.connect(self._on_stop_selected_clicked)

        # Node Healthカードと縦積みするため列幅が狭く、横1行に詰め込むと
        # ボタン文字が見切れる。業務モード選択行、個別操作行、一斉操作行の
        # 3行に分けて縦方向の余裕を使う。個別操作（ノードを選んで都度確認
        # しながら起動する操作）を先に、一斉操作（プリセット済みの構成を
        # まとめて起動する操作）を後に置く。
        mode_row = QtWidgets.QHBoxLayout()
        mode_row.addWidget(QtWidgets.QLabel('実行環境:'))
        mode_row.addWidget(self._environment_combo)
        mode_row.addWidget(QtWidgets.QLabel('走行モード:'))
        mode_row.addWidget(self._drive_mode_combo)
        mode_row.addWidget(apply_preset_button)
        mode_row.addStretch(1)

        individual_row = QtWidgets.QHBoxLayout()
        individual_row.addWidget(self._node_combo, 1)
        individual_row.addWidget(launch_one_button)
        individual_row.addWidget(stop_one_button)

        all_row = QtWidgets.QHBoxLayout()
        all_row.addWidget(launch_all_button)
        all_row.addWidget(stop_all_button)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(mode_row)
        layout.addLayout(individual_row)
        layout.addLayout(all_row)

    def update_plan(
        self,
        *,
        environment: str,
        drive_mode: str,
        ordered_profile_ids: List[str],
        profiles_by_id: Dict[str, LaunchProfile],
    ) -> None:
        """起動・設定タブの業務モードと起動予定ノード一覧を反映する。"""

        _set_combo_text(self._environment_combo, environment)
        _set_combo_text(self._drive_mode_combo, drive_mode)
        self._ordered_profile_ids = list(ordered_profile_ids)

        current = self._node_combo.currentData()
        self._node_combo.blockSignals(True)
        self._node_combo.clear()
        for profile_id in self._ordered_profile_ids:
            profile = profiles_by_id.get(profile_id)
            label = profile.display_name if profile else profile_id
            self._node_combo.addItem(label, profile_id)
        if current is not None:
            index = self._node_combo.findData(current)
            if index >= 0:
                self._node_combo.setCurrentIndex(index)
        self._node_combo.blockSignals(False)

    def _on_launch_selected_clicked(self) -> None:
        profile_id = self._node_combo.currentData()
        if profile_id:
            self.launch_requested.emit(profile_id)

    def _on_stop_selected_clicked(self) -> None:
        profile_id = self._node_combo.currentData()
        if profile_id:
            self.stop_requested.emit(profile_id)
