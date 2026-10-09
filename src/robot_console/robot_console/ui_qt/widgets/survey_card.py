"""経路採取の開始・保存と受信状態。"""
import math
from PyQt5 import QtCore, QtWidgets


class SurveyCard(QtWidgets.QGroupBox):
    command_requested = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__('ルート記録')
        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        row = QtWidgets.QHBoxLayout()
        self.start = QtWidgets.QPushButton('ルート記録開始')
        self.finish = QtWidgets.QPushButton('終了・保存')
        self.start.setMinimumHeight(72)
        self.finish.setMinimumHeight(72)
        font = self.status.font()
        font.setPointSize(24)
        self.status.setFont(font)
        self.start.clicked.connect(lambda: self.command_requested.emit('start'))
        self.finish.clicked.connect(lambda: self.command_requested.emit('finish'))
        row.addWidget(self.start)
        row.addWidget(self.finish)
        layout.addLayout(row)
        self.width = QtWidgets.QLabel()
        self.width.setWordWrap(True)
        font = self.width.font(); font.setPointSize(18); self.width.setFont(font)
        layout.addWidget(self.width)
        self.directory = QtWidgets.QLabel()
        self.directory.setWordWrap(True)
        self.directory.setTextFormat(QtCore.Qt.PlainText)
        self.directory.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.directory)
        self.update_state({})

    def update_state(self, state):
        connected = state.get('connected', False)
        active = state.get('active', False)
        pose = state.get('pose_fresh', False)
        source = 'ICP' if state.get('localization_mode') == 'icp' else 'GNSS融合'
        if not connected:
            text = '記録ノード未接続（起動・設定から実機・手動走行を起動）'
        elif state.get('error'):
            text = '保存エラー: ' + state['error']
        elif active:
            text = '● 記録中' + ('／自己位置の更新待ち' if not pose else '')
        elif state.get('saved'):
            text = '保存完了'
        elif state.get('saving'):
            text = '保存中'
        else:
            text = '記録開始できます' if pose else '自己位置の受信待ち'
        self.status.setText(text + (f"　{state.get('count', 0)}点［{source}］" if connected else ''))
        self.start.setEnabled(connected and pose and not active and not state.get('error'))
        self.finish.setEnabled(connected and (active or bool(state.get('error'))))
        self.directory.setText(state.get('directory', '') if connected else '')
        width = state.get('width', {}) if connected else {}
        reasons = {'unknown': '未観測', 'no_surface_reference': '路面基準未確定',
            'rough_reference': '基準面が粗い', 'material_change': '材質変化（芝生等）',
            'rough_or_obstacle': '粗面・障害物', 'curb_or_drop': '段差・落下端',
            'slope': '勾配', 'range_limit': '観測範囲端', 'material_unavailable': '反射強度未取得'}
        def side(name):
            try:
                value = float(width.get(name, 0.))
            except (TypeError, ValueError):
                value = 0.
            if not math.isfinite(value) or value < 0:
                value = 0.
            reason = reasons.get(width.get(name+'_reason'), '未確認')
            return f'{value:.2f} m（{reason}）'
        self.width.setText('回避幅候補：左 '+side('left')+' ／ 右 '+side('right')+
            '\n車体中心の横移動量。0 mは回避領域を確定できない状態です。')
