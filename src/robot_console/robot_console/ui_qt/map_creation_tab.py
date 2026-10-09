"""Recorded bag mapping UI. Mapping owns no ROS node or driving command."""
import json
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from icart_mapping.job import DEFAULTS, atomic_json, create_job


class MapCreationTab(QtWidgets.QWidget):
    map_selected = QtCore.pyqtSignal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.process = None
        self.job = None
        self.result = None
        layout = QtWidgets.QVBoxLayout(self)
        guide = QtWidgets.QLabel(
            '記録済みbagから、FIX位置とICPループ閉合で点群地図を作ります。\n'
            '① bag・保存先を選択 → ② 地図作成 → ③ 俯瞰を確認 → ④ ICP走行の地図に設定')
        guide.setWordWrap(True)
        layout.addWidget(guide)
        self.fields = {}
        form = QtWidgets.QFormLayout()
        layout.addLayout(form)
        for key, label in [('bag', '入力bagフォルダ'), ('output_root', '地図・ログの保存先')]:
            self._field(form, key, label, True)
        advanced = QtWidgets.QGroupBox('詳細設定（経路の原点・記録時の取付・専用実行環境）')
        advanced.setCheckable(True)
        advanced.setChecked(False)
        advanced_layout = QtWidgets.QVBoxLayout(advanced)
        details = QtWidgets.QWidget()
        detail_form = QtWidgets.QFormLayout(details)
        for key, label, directory in [('projection', '経路のprojection.yaml', False),
                                      ('hardware', '記録時のhardware.yaml', False),
                                      ('python', '地図作成用Python', False),
                                      ('backend', 'small_gicpフォルダ', True)]:
            self._field(detail_form, key, label, directory)
        advanced_layout.addWidget(details)
        details.setVisible(False)
        advanced.toggled.connect(details.setVisible)
        layout.addWidget(advanced)
        self.advanced = advanced
        controls = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton('地図作成開始')
        self.cancel_button = QtWidgets.QPushButton('中止')
        self.cancel_button.setEnabled(False)
        self.open_button = QtWidgets.QPushButton('結果を開く')
        self.apply_button = QtWidgets.QPushButton('ICP走行の地図に設定')
        self.apply_button.setEnabled(False)
        for widget in (self.start_button, self.cancel_button, self.open_button, self.apply_button):
            controls.addWidget(widget)
        layout.addLayout(controls)
        self.phase = QtWidgets.QLabel('未実行：元bag・現在の地図を保持し、新規フォルダへ出力します。')
        self.phase.setWordWrap(True)
        layout.addWidget(self.phase)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setFormat('処理段階 %p%（残り時間の割合ではありません）')
        self.progress.setValue(0)
        layout.addWidget(self.progress)
        self.summary = QtWidgets.QLabel('必要な記録：FAST-LIO raw姿勢・Livox点群・IMU・FIX・RTK状態')
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.preview = QtWidgets.QLabel('完成地図の俯瞰をここに表示します')
        self.preview.setAlignment(QtCore.Qt.AlignCenter)
        self.preview.setMinimumHeight(190)
        layout.addWidget(self.preview, 1)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        self.log.setMaximumHeight(110)
        layout.addWidget(self.log)
        self.start_button.clicked.connect(self.start)
        self.cancel_button.clicked.connect(self.cancel)
        self.open_button.clicked.connect(self.open_result)
        self.apply_button.clicked.connect(self.apply_result)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.refresh)

    def _field(self, form, key, label, directory):
        line = QtWidgets.QLineEdit(DEFAULTS[key])
        self.fields[key] = line
        row = QtWidgets.QHBoxLayout()
        row.addWidget(line, 1)
        button = QtWidgets.QPushButton('選択…')
        def choose():
            value = (QtWidgets.QFileDialog.getExistingDirectory(self, label, line.text()) if directory
                     else QtWidgets.QFileDialog.getOpenFileName(self, label, line.text())[0])
            if value:
                line.setText(value)
        button.clicked.connect(choose)
        row.addWidget(button)
        form.addRow(label, row)

    @property
    def busy(self):
        return self.process is not None and self.process.state() != QtCore.QProcess.NotRunning

    def _controls(self, running):
        self.start_button.setEnabled(not running)
        self.cancel_button.setEnabled(running)
        self.open_button.setEnabled(not running)
        self.apply_button.setEnabled(not running and self.result is not None)
        for field in self.fields.values():
            field.setEnabled(not running)
        self.advanced.setEnabled(not running)

    def start(self):
        if self.busy:
            return
        self.result = None
        self.apply_button.setEnabled(False)
        try:
            job = create_job({k: field.text().strip() for k, field in self.fields.items()})
        except Exception as error:
            self.phase.setText('開始できません：'+str(error))
            return
        self.job = job
        self.result = None
        self.preview.clear()
        self.log.clear()
        self.summary.setText('入力を読み取り、元データとは別の場所に地図を作成します。')
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_output)
        self.process.finished.connect(self._finished)
        self.process.errorOccurred.connect(self._process_error)
        self.process.start('/usr/bin/python3', ['-u', '-m', 'icart_mapping.job', '--job', str(job)])
        self._controls(True)
        self.timer.start()
        self.refresh()

    def _read_output(self):
        self.log.moveCursor(QtGui.QTextCursor.End)
        self.log.insertPlainText(bytes(self.process.readAllStandardOutput()).decode('utf-8', errors='replace'))

    def _process_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            atomic_json(self.job.parent/'status.json', dict(state='failed', phase='起動失敗', percent=0,
                                                           error=self.process.errorString()))
            self._finished(1, QtCore.QProcess.CrashExit)

    def _finished(self, code, exit_status):
        self._read_output()
        self.timer.stop()
        if self.job and (code != 0 or exit_status != QtCore.QProcess.NormalExit):
            status = json.loads((self.job.parent/'status.json').read_text())
            if status.get('state') not in ('failed', 'cancelled'):
                status.update(state='failed', phase='処理終了', error=f'終了コード {code}')
                atomic_json(self.job.parent/'status.json', status)
        self.refresh()
        self._controls(False)

    def refresh(self):
        if not self.job:
            return
        try:
            status = json.loads((self.job.parent/'status.json').read_text())
            self.phase.setText(status['phase']+'\n保存先：'+str(self.job.parent)+
                               ('\n'+status['error'] if status.get('error') else ''))
            self.progress.setValue(int(status.get('percent', 0)))
            if status['state'] == 'complete' and not self.busy:
                self._load_complete(status)
        except (OSError, ValueError, KeyError) as error:
            self.result = None
            self.apply_button.setEnabled(False)
            self.phase.setText('結果を確認できません：'+str(error))

    def _load_complete(self, status):
        from icp_localization.registration import load_manifest
        manifest = self.job.parent/'map_manifest.json'
        config = json.loads(self.job.read_text())
        load_manifest(manifest, config['projection'])
        self.result = (str(manifest), config['backend'])
        s = status['summary']
        residual = s['fit_residual_m']
        self.summary.setText(
            f"地図 {s['points']:,}点／姿勢 {s['nodes']:,}／FIX拘束 {s['GNSS_factors']}／"
            f"ICP接続 {s['join_factors']}・ループ {s['loop_factors']}\n"
            f"採用FIXとの差：平均 {residual['mean']:.3f} m、最大 {residual['max']:.3f} m"
            '（最適化に使用した点の残差。独立した精度評価ではありません）')
        pixmap = QtGui.QPixmap(str(self.job.parent/'overview.png'))
        self.preview.setPixmap(pixmap.scaled(950, 390, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        self.apply_button.setEnabled(True)

    def open_result(self):
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, '地図作成結果を選択', self.fields['output_root'].text())
        if directory:
            self.load_result(Path(directory))

    def load_result(self, directory):
        if self.busy:
            return
        self.result = None
        self.apply_button.setEnabled(False)
        self.preview.clear()
        self.job = Path(directory)/'job.json'
        self.refresh()

    def apply_result(self):
        if self.result is not None and not self.busy:
            # Recheck hashes if files have changed since the result was opened.
            self.result = None
            self.refresh()
            if self.result is not None:
                self.map_selected.emit(*self.result)

    def cancel(self):
        if not self.busy:
            return
        self.phase.setText('中止処理中…')
        self.cancel_button.setEnabled(False)
        process = self.process
        process.terminate()
        QtCore.QTimer.singleShot(5000, lambda: process.kill() if process.state() != QtCore.QProcess.NotRunning else None)

    def shutdown(self):
        self.timer.stop()
        if self.busy:
            self.process.terminate()
            if not self.process.waitForFinished(2000):
                self.process.kill()
                self.process.waitForFinished(1000)
