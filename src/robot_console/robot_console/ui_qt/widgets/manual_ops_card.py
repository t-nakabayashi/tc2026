"""Manual Opsカード: タブ切替方式の手動介入操作カード。

`manual_start` / `sig_recog` / `road_blocked` / `obstacle_hint` / `frame_image`
の5タブで構成する（screen_function_design.md 6.9節）。QWidgetはROS
publisherを直接持たず、送信操作はすべて `pyqtSignal` で外部（ConsoleCore/
CommandQueue）へ通知する。
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.snapshot_model import ManualControlsView

TAB_TITLE_MANUAL_START = 'manual_start'
TAB_TITLE_SIGNAL = 'signal'
TAB_TITLE_ROAD_BLOCKED = 'road_blocked'
TAB_TITLE_OBSTACLE_HINT = 'obstacle_hint'
TAB_TITLE_FRAME_IMAGE = 'frame_image'

SIG_RECOG_GO = 1
SIG_RECOG_STOP = 2


def _format_timestamp(value: Optional[datetime]) -> str:
    if value is None:
        return '未送信'
    return value.strftime('%H:%M:%S')


class ManualOpsCard(QtWidgets.QGroupBox):
    """走行中の手動介入操作をタブ切替で提供するカード。"""

    manual_start_requested = QtCore.pyqtSignal(bool)
    sig_recog_requested = QtCore.pyqtSignal(int)
    road_blocked_requested = QtCore.pyqtSignal(bool)
    obstacle_hint_override_requested = QtCore.pyqtSignal(bool, float, float, float)
    obstacle_hint_stop_requested = QtCore.pyqtSignal()
    frame_image_requested = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__('Manual Ops', parent)

        self._tabs = QtWidgets.QTabWidget()
        self._tabs.addTab(self._build_manual_start_tab(), TAB_TITLE_MANUAL_START)
        self._tabs.addTab(self._build_signal_tab(), TAB_TITLE_SIGNAL)
        self._tabs.addTab(self._build_road_blocked_tab(), TAB_TITLE_ROAD_BLOCKED)
        self._tabs.addTab(self._build_obstacle_hint_tab(), TAB_TITLE_OBSTACLE_HINT)
        self._tabs.addTab(self._build_frame_image_tab(), TAB_TITLE_FRAME_IMAGE)
        self._tabs.setCurrentIndex(0)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self._tabs)
        self.setLayout(layout)

    # ---------- manual_start ----------
    def _build_manual_start_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        self._manual_start_value_label = QtWidgets.QLabel('-')
        self._manual_start_time_label = QtWidgets.QLabel('未送信')

        send_button = QtWidgets.QPushButton('manual_start = True 送信')
        send_button.clicked.connect(lambda: self.manual_start_requested.emit(True))
        clear_button = QtWidgets.QPushButton('manual_start = False 送信')
        clear_button.clicked.connect(lambda: self.manual_start_requested.emit(False))

        status_row = QtWidgets.QHBoxLayout()
        status_row.addWidget(QtWidgets.QLabel('現在値:'))
        status_row.addWidget(self._manual_start_value_label)
        status_row.addWidget(QtWidgets.QLabel('最終送信:'))
        status_row.addWidget(self._manual_start_time_label)
        status_row.addStretch(1)

        layout = QtWidgets.QVBoxLayout(widget)
        layout.addLayout(status_row)
        layout.addWidget(send_button)
        layout.addWidget(clear_button)
        layout.addStretch(1)
        return widget

    # ---------- signal ----------
    def _build_signal_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        self._sig_recog_value_label = QtWidgets.QLabel('-')
        self._sig_recog_time_label = QtWidgets.QLabel('未送信')

        self._sig_go_radio = QtWidgets.QRadioButton('GO')
        self._sig_stop_radio = QtWidgets.QRadioButton('STOP')
        self._sig_go_radio.setChecked(True)
        radio_group = QtWidgets.QButtonGroup(widget)
        radio_group.addButton(self._sig_go_radio)
        radio_group.addButton(self._sig_stop_radio)

        send_button = QtWidgets.QPushButton('sig_recog 送信')
        send_button.clicked.connect(self._on_sig_recog_send_clicked)

        status_row = QtWidgets.QHBoxLayout()
        status_row.addWidget(QtWidgets.QLabel('現在値:'))
        status_row.addWidget(self._sig_recog_value_label)
        status_row.addWidget(QtWidgets.QLabel('最終送信:'))
        status_row.addWidget(self._sig_recog_time_label)
        status_row.addStretch(1)

        radio_row = QtWidgets.QHBoxLayout()
        radio_row.addWidget(self._sig_go_radio)
        radio_row.addWidget(self._sig_stop_radio)
        radio_row.addStretch(1)

        layout = QtWidgets.QVBoxLayout(widget)
        layout.addLayout(status_row)
        layout.addLayout(radio_row)
        layout.addWidget(send_button)
        layout.addStretch(1)
        return widget

    def _on_sig_recog_send_clicked(self) -> None:
        value = SIG_RECOG_GO if self._sig_go_radio.isChecked() else SIG_RECOG_STOP
        self.sig_recog_requested.emit(value)

    # ---------- road_blocked ----------
    def _build_road_blocked_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        self._road_blocked_value_label = QtWidgets.QLabel('-')
        self._road_blocked_time_label = QtWidgets.QLabel('未送信')
        self._road_blocked_source_label = QtWidgets.QLabel('-')

        self._road_blocked_true_radio = QtWidgets.QRadioButton('True（封鎖）')
        self._road_blocked_false_radio = QtWidgets.QRadioButton('False（解除）')
        self._road_blocked_false_radio.setChecked(True)
        radio_group = QtWidgets.QButtonGroup(widget)
        radio_group.addButton(self._road_blocked_true_radio)
        radio_group.addButton(self._road_blocked_false_radio)

        send_button = QtWidgets.QPushButton('road_blocked 送信')
        send_button.clicked.connect(self._on_road_blocked_send_clicked)

        status_row = QtWidgets.QHBoxLayout()
        status_row.addWidget(QtWidgets.QLabel('現在値:'))
        status_row.addWidget(self._road_blocked_value_label)
        status_row.addWidget(QtWidgets.QLabel('入力元:'))
        status_row.addWidget(self._road_blocked_source_label)
        status_row.addWidget(QtWidgets.QLabel('最終送信:'))
        status_row.addWidget(self._road_blocked_time_label)
        status_row.addStretch(1)

        radio_row = QtWidgets.QHBoxLayout()
        radio_row.addWidget(self._road_blocked_true_radio)
        radio_row.addWidget(self._road_blocked_false_radio)
        radio_row.addStretch(1)

        layout = QtWidgets.QVBoxLayout(widget)
        layout.addLayout(status_row)
        layout.addLayout(radio_row)
        layout.addWidget(send_button)
        layout.addStretch(1)
        return widget

    def _on_road_blocked_send_clicked(self) -> None:
        value = self._road_blocked_true_radio.isChecked()
        if value:
            confirmed = QtWidgets.QMessageBox.question(
                self,
                'road_blocked 送信確認',
                'road_blocked = True を送信します。よろしいですか。',
            )
            if confirmed != QtWidgets.QMessageBox.Yes:
                return
        self.road_blocked_requested.emit(value)

    # ---------- obstacle_hint ----------
    def _build_obstacle_hint_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()

        self._obstacle_clearance_spin = QtWidgets.QDoubleSpinBox()
        self._obstacle_clearance_spin.setRange(0.0, 100.0)
        self._obstacle_clearance_spin.setSuffix(' m')
        self._obstacle_left_spin = QtWidgets.QDoubleSpinBox()
        self._obstacle_left_spin.setRange(-10.0, 10.0)
        self._obstacle_left_spin.setSuffix(' m')
        self._obstacle_right_spin = QtWidgets.QDoubleSpinBox()
        self._obstacle_right_spin.setRange(-10.0, 10.0)
        self._obstacle_right_spin.setSuffix(' m')

        spin_row = QtWidgets.QHBoxLayout()
        spin_row.addWidget(QtWidgets.QLabel('前方余裕:'))
        spin_row.addWidget(self._obstacle_clearance_spin)
        spin_row.addWidget(QtWidgets.QLabel('左オフセット:'))
        spin_row.addWidget(self._obstacle_left_spin)
        spin_row.addWidget(QtWidgets.QLabel('右オフセット:'))
        spin_row.addWidget(self._obstacle_right_spin)
        spin_row.addStretch(1)

        self._obstacle_front_blocked_check = QtWidgets.QCheckBox('front_blocked')
        start_button = QtWidgets.QPushButton('固定送出 開始')
        start_button.clicked.connect(self._on_obstacle_hint_start_clicked)
        stop_button = QtWidgets.QPushButton('固定送出 停止')
        stop_button.clicked.connect(self._on_obstacle_hint_stop_clicked)

        control_row = QtWidgets.QHBoxLayout()
        control_row.addWidget(self._obstacle_front_blocked_check)
        control_row.addWidget(start_button)
        control_row.addWidget(stop_button)
        control_row.addStretch(1)

        self._obstacle_override_state_label = QtWidgets.QLabel('送出中: いいえ')
        self._obstacle_time_label = QtWidgets.QLabel('未送信')
        status_row = QtWidgets.QHBoxLayout()
        status_row.addWidget(self._obstacle_override_state_label)
        status_row.addWidget(QtWidgets.QLabel('最終送信:'))
        status_row.addWidget(self._obstacle_time_label)
        status_row.addStretch(1)

        layout = QtWidgets.QVBoxLayout(widget)
        layout.addLayout(spin_row)
        layout.addLayout(control_row)
        layout.addLayout(status_row)
        layout.addStretch(1)
        return widget

    def _on_obstacle_hint_start_clicked(self) -> None:
        confirmed = QtWidgets.QMessageBox.question(
            self,
            'obstacle_hint 固定送出確認',
            'obstacle_avoidance_hint の固定送出を開始します。よろしいですか。',
        )
        if confirmed != QtWidgets.QMessageBox.Yes:
            return
        self.obstacle_hint_override_requested.emit(
            self._obstacle_front_blocked_check.isChecked(),
            self._obstacle_clearance_spin.value(),
            self._obstacle_left_spin.value(),
            self._obstacle_right_spin.value(),
        )

    def _on_obstacle_hint_stop_clicked(self) -> None:
        confirmed = QtWidgets.QMessageBox.question(
            self,
            'obstacle_hint 固定送出停止確認',
            'obstacle_avoidance_hint の固定送出を停止します。よろしいですか。',
        )
        if confirmed != QtWidgets.QMessageBox.Yes:
            return
        self.obstacle_hint_stop_requested.emit()

    # ---------- frame_image ----------
    def _build_frame_image_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        self._frame_image_path_edit = QtWidgets.QLineEdit()
        self._frame_image_path_edit.setPlaceholderText('静止画ファイルパス')
        browse_button = QtWidgets.QPushButton('参照')
        browse_button.clicked.connect(self._on_frame_image_browse_clicked)
        send_button = QtWidgets.QPushButton('frame_image_path 送信')
        send_button.clicked.connect(self._on_frame_image_send_clicked)

        path_row = QtWidgets.QHBoxLayout()
        path_row.addWidget(self._frame_image_path_edit)
        path_row.addWidget(browse_button)

        layout = QtWidgets.QVBoxLayout(widget)
        layout.addLayout(path_row)
        layout.addWidget(send_button)
        layout.addStretch(1)
        return widget

    def _on_frame_image_browse_clicked(self) -> None:
        path, _filter = QtWidgets.QFileDialog.getOpenFileName(self, '静止画ファイルを選択')
        if path:
            self._frame_image_path_edit.setText(path)

    def _on_frame_image_send_clicked(self) -> None:
        path = self._frame_image_path_edit.text().strip()
        if path:
            self.frame_image_requested.emit(path)

    # ---------- Snapshot反映 ----------
    def update_snapshot(self, manual_controls: ManualControlsView) -> None:
        """`ManualControlsView` の内容を各タブへ反映する。"""

        self._manual_start_value_label.setText(str(manual_controls.manual_start_value))
        self._manual_start_time_label.setText(
            _format_timestamp(manual_controls.manual_start_last_sent_at)
        )

        sig_text = {SIG_RECOG_GO: 'GO', SIG_RECOG_STOP: 'STOP'}.get(
            manual_controls.sig_recog_value, '-'
        )
        self._sig_recog_value_label.setText(sig_text)
        self._sig_recog_time_label.setText(
            _format_timestamp(manual_controls.sig_recog_last_sent_at)
        )

        self._road_blocked_value_label.setText(str(manual_controls.road_blocked_value))
        self._road_blocked_source_label.setText(manual_controls.road_blocked_source)
        self._road_blocked_time_label.setText(
            _format_timestamp(manual_controls.road_blocked_last_sent_at)
        )

        override_text = 'はい' if manual_controls.obstacle_hint_override_active else 'いいえ'
        self._obstacle_override_state_label.setText(f'送出中: {override_text}')
        self._obstacle_time_label.setText(
            _format_timestamp(manual_controls.obstacle_hint_last_sent_at)
        )
