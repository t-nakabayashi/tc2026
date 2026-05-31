"""PS3 controller 相当の /joy を publish する開発用 GUI ノード。"""

from __future__ import annotations

import signal
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy

from drive_mode_manager.ps3_joy_sim_core import (
    Ps3JoySimConfig,
    Ps3JoySimCore,
    Ps3JoyState,
    key_name_from_text_or_code,
)


class Ps3JoySimRosNode(Node):
    """キーボード入力状態を Joy topic へ周期 publish する。"""

    def __init__(self) -> None:
        super().__init__('ps3_joy_sim_node')
        self._config = self._load_config()
        self._core = Ps3JoySimCore(self._config)
        self._pressed_keys: set[str] = set()
        self._stick_x = 0.0
        self._stick_y = 0.0
        self._lock = threading.Lock()
        self._publish_count = 0
        self._last_state = self._core.compute(set(), self._stick_x, self._stick_y)

        self._joy_topic = str(self.get_parameter('joy_topic').value)
        self._publisher = self.create_publisher(Joy, self._joy_topic, 10)
        period_s = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period_s, self._on_timer)
        self.get_logger().info(
            f'ps3_joy_sim_node started: topic={self._joy_topic} rate={1.0 / period_s:.1f}Hz'
        )

    @property
    def joy_topic(self) -> str:
        """publish 先 topic 名を返す。"""

        return self._joy_topic

    @property
    def publish_rate_hz(self) -> float:
        """publish 周期設定を Hz で返す。"""

        return float(self.get_parameter('publish_rate_hz').value)

    def press_key(self, key: str) -> None:
        """GUI thread からキー押下を反映する。"""

        normalized = self._normalize_key_name(key)
        with self._lock:
            if normalized == self._config.key_reset.lower():
                self._pressed_keys.clear()
                self._stick_x = 0.0
                self._stick_y = 0.0
            elif self._core.is_stick_key(normalized):
                self._stick_x, self._stick_y = self._core.update_stick_for_key(
                    self._stick_x,
                    self._stick_y,
                    normalized,
                )
            else:
                self._pressed_keys.add(normalized)
            self._last_state = self._core.compute(
                self._pressed_keys,
                self._stick_x,
                self._stick_y,
            )

    def release_key(self, key: str) -> None:
        """GUI thread からキー解放を反映する。"""

        normalized = self._normalize_key_name(key)
        with self._lock:
            if not self._core.is_stick_key(normalized):
                self._pressed_keys.discard(normalized)
            self._last_state = self._core.compute(
                self._pressed_keys,
                self._stick_x,
                self._stick_y,
            )

    def is_stick_key(self, key: str) -> bool:
        """指定キーが stick 操作用かを返す。"""

        return self._core.is_stick_key(self._normalize_key_name(key))

    def snapshot(self) -> tuple[Ps3JoyState, int, tuple[str, ...]]:
        """GUI 表示用の最新状態を返す。"""

        with self._lock:
            return self._last_state, self._publish_count, tuple(sorted(self._pressed_keys))

    def _load_config(self) -> Ps3JoySimConfig:
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('num_axes', 6)
        self.declare_parameter('num_buttons', 17)
        self.declare_parameter('left_stick_x_axis', 0)
        self.declare_parameter('left_stick_y_axis', 1)
        self.declare_parameter('invert_left_stick_x', True)
        self.declare_parameter('invert_left_stick_y', False)
        self.declare_parameter('stick_step', 0.1)
        self.declare_parameter('l1_button_index', 4)
        self.declare_parameter('ps_button_index', 5)
        self.declare_parameter('key_l1', 'l')
        self.declare_parameter('key_ps', 'p')
        self.declare_parameter('key_stick_forward', 'w')
        self.declare_parameter('key_stick_backward', 's')
        self.declare_parameter('key_stick_left', 'a')
        self.declare_parameter('key_stick_right', 'd')
        self.declare_parameter('key_reset', 'space')
        self.declare_parameter('normalize_diagonal_stick', True)
        self.declare_parameter('cmd_vel_linear_scale', 1.2)
        self.declare_parameter('cmd_vel_angular_scale', 1.5)
        self.declare_parameter('cmd_vel_deadzone', 0.05)
        self.declare_parameter('cmd_vel_linear_axis_invert', False)
        self.declare_parameter('cmd_vel_angular_axis_invert', False)
        return Ps3JoySimConfig(
            num_axes=int(self.get_parameter('num_axes').value),
            num_buttons=int(self.get_parameter('num_buttons').value),
            left_stick_x_axis=int(self.get_parameter('left_stick_x_axis').value),
            left_stick_y_axis=int(self.get_parameter('left_stick_y_axis').value),
            invert_left_stick_x=bool(self.get_parameter('invert_left_stick_x').value),
            invert_left_stick_y=bool(self.get_parameter('invert_left_stick_y').value),
            stick_step=float(self.get_parameter('stick_step').value),
            l1_button_index=int(self.get_parameter('l1_button_index').value),
            ps_button_index=int(self.get_parameter('ps_button_index').value),
            key_l1=str(self.get_parameter('key_l1').value).lower(),
            key_ps=str(self.get_parameter('key_ps').value).lower(),
            key_stick_forward=str(self.get_parameter('key_stick_forward').value).lower(),
            key_stick_backward=str(self.get_parameter('key_stick_backward').value).lower(),
            key_stick_left=str(self.get_parameter('key_stick_left').value).lower(),
            key_stick_right=str(self.get_parameter('key_stick_right').value).lower(),
            key_reset=str(self.get_parameter('key_reset').value).lower(),
            normalize_diagonal_stick=bool(
                self.get_parameter('normalize_diagonal_stick').value
            ),
            cmd_vel_linear_scale=float(self.get_parameter('cmd_vel_linear_scale').value),
            cmd_vel_angular_scale=float(self.get_parameter('cmd_vel_angular_scale').value),
            cmd_vel_deadzone=float(self.get_parameter('cmd_vel_deadzone').value),
            cmd_vel_linear_axis_invert=bool(
                self.get_parameter('cmd_vel_linear_axis_invert').value
            ),
            cmd_vel_angular_axis_invert=bool(
                self.get_parameter('cmd_vel_angular_axis_invert').value
            ),
        )

    def _on_timer(self) -> None:
        with self._lock:
            state = self._core.compute(self._pressed_keys, self._stick_x, self._stick_y)
            self._last_state = state
            self._publish_count += 1
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(state.axes)
        msg.buttons = list(state.buttons)
        self._publisher.publish(msg)

    @staticmethod
    def _normalize_key_name(key: str) -> str:
        if key == ' ':
            return 'space'
        return key.lower()


class StickView:  # pragma: no cover - GUI は手動/統合確認で扱う
    """左 stick の現在位置を描画する小さな widget。"""

    def __init__(self, qt_widgets: object, qt_gui: object, qt_core: object) -> None:
        QtWidgets = qt_widgets
        self._qt_widgets = qt_widgets
        self._qt_gui = qt_gui
        self._qt_core = qt_core
        self._widget = QtWidgets.QWidget()
        self._widget.setMinimumSize(180, 180)
        self._x = 0.0
        self._y = 0.0
        self._widget.paintEvent = self._paint_event

    @property
    def widget(self) -> object:
        """Qt widget を返す。"""

        return self._widget

    def update(self, stick_x: float, stick_y: float) -> None:
        """描画位置を更新する。"""

        self._x = stick_x
        self._y = stick_y
        self._widget.update()

    def _paint_event(self, _event: object) -> None:
        QtCore = self._qt_core
        QtGui = self._qt_gui
        painter = QtGui.QPainter(self._widget)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self._widget.rect().adjusted(12, 12, -12, -12)
        size = min(rect.width(), rect.height())
        cx = rect.center().x()
        cy = rect.center().y()
        radius = size * 0.45
        painter.setPen(QtGui.QPen(QtGui.QColor('#8892a0'), 2))
        painter.setBrush(QtGui.QBrush(QtGui.QColor('#20242a')))
        painter.drawEllipse(QtCore.QPointF(cx, cy), radius, radius)
        painter.setPen(QtGui.QPen(QtGui.QColor('#56606f'), 1))
        painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))
        px = cx + self._x * radius
        py = cy - self._y * radius
        painter.setPen(QtGui.QPen(QtGui.QColor('#ffffff'), 2))
        painter.setBrush(QtGui.QBrush(QtGui.QColor('#4cc9f0')))
        painter.drawEllipse(QtCore.QPointF(px, py), 10, 10)
        painter.end()


class Ps3JoySimWindow:  # pragma: no cover - GUI は手動/統合確認で扱う
    """PyQt5 で Joy simulator の状態と key bind を表示する。"""

    def __init__(self, ros_node: Ps3JoySimRosNode) -> None:
        from PyQt5 import QtCore, QtGui, QtWidgets

        self._qt = QtCore, QtGui, QtWidgets
        self._ros_node = ros_node
        self._app = QtWidgets.QApplication(sys.argv)
        self._window = QtWidgets.QMainWindow()
        self._window.setWindowTitle('PS3 Joy Simulator')
        self._window.setFocusPolicy(QtCore.Qt.StrongFocus)
        self._central = QtWidgets.QWidget()
        self._central.setFocusPolicy(QtCore.Qt.StrongFocus)
        self._window.setCentralWidget(self._central)
        self._labels: dict[str, object] = {}
        self._stick_view = StickView(QtWidgets, QtGui, QtCore)
        self._build_layout()
        self._central.keyPressEvent = self._on_key_press
        self._central.keyReleaseEvent = self._on_key_release
        self._window.keyPressEvent = self._on_key_press
        self._window.keyReleaseEvent = self._on_key_release
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)
        self._central.setFocus()

    def run(self) -> int:
        """GUI を開始し、終了コードを返す。"""

        self._window.resize(820, 620)
        self._window.show()
        self._refresh()
        return int(self._app.exec_())

    def quit(self) -> None:
        """外部停止要求を Qt event loop の終了へ変換する。"""

        self._timer.stop()
        self._app.quit()

    def _build_layout(self) -> None:
        QtWidgets = self._qt[2]
        layout = QtWidgets.QVBoxLayout(self._central)
        title = QtWidgets.QLabel('PS3 Joy Simulator')
        title.setStyleSheet('font-size: 24px; font-weight: bold;')
        layout.addWidget(title)

        grid = QtWidgets.QGridLayout()
        layout.addLayout(grid)
        rows = [
            ('topic', 'Publish Topic'),
            ('rate', 'Publish Rate'),
            ('l1', 'L1'),
            ('ps', 'R1 (mode)'),
            ('combo', 'L1 + R1'),
            ('stick_x', 'Left Stick X'),
            ('stick_y', 'Left Stick Y'),
            ('preview_v', 'Preview cmd_vel v'),
            ('preview_w', 'Preview cmd_vel w'),
            ('count', 'Publish Count'),
            ('focus', 'Focus'),
            ('keys', 'Pressed Keys'),
        ]
        for row, (key, label_text) in enumerate(rows):
            label = QtWidgets.QLabel(label_text)
            value = QtWidgets.QLabel('-')
            value.setTextInteractionFlags(self._qt[0].Qt.TextSelectableByMouse)
            grid.addWidget(label, row, 0)
            grid.addWidget(value, row, 1)
            self._labels[key] = value
        grid.addWidget(self._stick_view.widget, 0, 2, len(rows), 1)

        self._axes_text = QtWidgets.QPlainTextEdit()
        self._axes_text.setReadOnly(True)
        self._buttons_text = QtWidgets.QPlainTextEdit()
        self._buttons_text.setReadOnly(True)
        layout.addWidget(QtWidgets.QLabel('axes[]'))
        layout.addWidget(self._axes_text)
        layout.addWidget(QtWidgets.QLabel('buttons[]'))
        layout.addWidget(self._buttons_text)
        bind = QtWidgets.QLabel('keys: l=L1, p=R1(mode), w/s/a/d=left stick, space=reset')
        bind.setStyleSheet('color: #555555;')
        layout.addWidget(bind)

    def _on_key_press(self, event: object) -> None:
        key = self._event_key_name(event)
        if event.isAutoRepeat() and not self._ros_node.is_stick_key(key):
            return
        if key:
            self._ros_node.press_key(key)
            self._refresh()

    def _on_key_release(self, event: object) -> None:
        if event.isAutoRepeat():
            return
        key = self._event_key_name(event)
        if key:
            self._ros_node.release_key(key)
            self._refresh()

    def _event_key_name(self, event: object) -> str:
        text = event.text()
        return self._key_name_from_text_or_code(text, int(event.key()))

    @staticmethod
    def _key_name_from_text_or_code(text: str, key_code: int) -> str:
        return key_name_from_text_or_code(text, key_code)

    def _refresh(self) -> None:
        state, publish_count, pressed_keys = self._ros_node.snapshot()
        focused = self._window.isActiveWindow() and (
            self._central.hasFocus() or self._window.hasFocus()
        )
        self._labels['topic'].setText(self._ros_node.joy_topic)
        self._labels['rate'].setText(f'{self._ros_node.publish_rate_hz:.1f} Hz')
        self._labels['l1'].setText('ON' if state.l1_pressed else 'OFF')
        self._labels['ps'].setText('ON' if state.ps_pressed else 'OFF')
        self._labels['combo'].setText('ON' if state.l1_pressed and state.ps_pressed else 'OFF')
        self._labels['stick_x'].setText(f'{state.left_stick_x:+.2f}')
        self._labels['stick_y'].setText(f'{state.left_stick_y:+.2f}')
        self._labels['preview_v'].setText(f'{state.preview_linear_x:+.2f} m/s')
        self._labels['preview_w'].setText(f'{state.preview_angular_z:+.2f} rad/s')
        self._labels['count'].setText(str(publish_count))
        self._labels['focus'].setText('ON' if focused else 'OFF')
        self._labels['keys'].setText(', '.join(pressed_keys) if pressed_keys else '-')
        self._axes_text.setPlainText(str(list(state.axes)))
        self._buttons_text.setPlainText(str(list(state.buttons)))
        self._stick_view.update(state.left_stick_x, state.left_stick_y)


def main() -> None:
    rclpy.init()
    node = Ps3JoySimRosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    window = Ps3JoySimWindow(node)
    previous_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda _sig, _frame: window.quit())
    try:
        exit_code = window.run()
    except KeyboardInterrupt:
        exit_code = 0
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
