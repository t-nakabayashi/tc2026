"""PyQt5版 robot_console のスタンドアロン起動エントリポイント。

`ConsoleCore` を生成し、`ros/console_node.py::start_ros_thread()` でQt
イベントループとは別スレッドのrclpy executorを起動する
（robot_console_gui_architecture_design.md 14.1節）。`QTimer` で
`ConsoleCore.build_snapshot()` を定期ポーリングし、`MainWindow.update_snapshot()`
経由で5タブへ配布する。本entry pointが正式UIであり、旧tkinter版
（`robot_console`）は当面コードを残すが正式UIとしては扱わない。
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from typing import List, Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.console_core import ConsoleCore
from robot_console.ros.console_node import start_ros_thread
from robot_console.ui_qt.qt_environment import (
    enable_qtwebengine_shared_opengl_contexts,
    fix_qt_plugin_path_conflict,
)

# HTML遠隔観測UI（web/static/app.js の SNAPSHOT_POLL_MS）と同じ更新周期にする。
SNAPSHOT_POLL_MS = 1000


def _parse_args(argv: List[str]) -> argparse.Namespace:
    """Qtへ渡す引数と混在させないため、既知の引数だけを取り出す。"""

    parser = argparse.ArgumentParser(description='robot_console PyQt5 UI（正式UI）')
    parser.add_argument(
        '--console-log-directory',
        default=os.environ.get('ROBOT_CONSOLE_LOG_DIR'),
        help=(
            'robot_console 管理の子プロセス stdout/stderr 保存先。'
            '未指定時は ROBOT_CONSOLE_LOG_DIR を参照します'
        ),
    )
    parser.add_argument('--business-environment', choices=['実機（融合）', 'デジタルツイン'],
                        help='共通起動から渡す初期環境。未指定なら従来の初期値を使います')
    parser.add_argument('--start-gnss', action='store_true', help='明示した地域・補正局でGNSSのみ起動する')
    parser.add_argument('--gnss-site', choices=['稲城', 'つくば'])
    parser.add_argument('--gnss-station', default='地域の既定局')
    known, _ = parser.parse_known_args(argv)
    if known.start_gnss and not known.gnss_site:
        parser.error('--start-gnssには--gnss-siteが必要です')
    return known


def main(argv: Optional[List[str]] = None) -> int:
    """PyQt5 UIを起動する。"""

    raw_argv = argv if argv is not None else sys.argv
    args = _parse_args(raw_argv[1:])

    fix_qt_plugin_path_conflict()
    enable_qtwebengine_shared_opengl_contexts()
    # 移設したQtの資源設定後にWebEngineを読み込む。
    from robot_console.ui_qt.main_window import MainWindow

    app = QtWidgets.QApplication(raw_argv)

    core = ConsoleCore(log_directory=args.console_log_directory)
    app.aboutToQuit.connect(core.bag_recorder.close)
    ros_handle = start_ros_thread(core, node_name='robot_console_qt')
    app.aboutToQuit.connect(ros_handle.stop)

    window = MainWindow(core=core)
    if args.business_environment:
        window.launch_settings_tab.set_business_mode(args.business_environment, '自律走行')

    timer = QtCore.QTimer(window)
    timer.timeout.connect(lambda: window.update_snapshot(core.build_snapshot()))
    timer.start(SNAPSHOT_POLL_MS)

    window.show()
    if args.start_gnss:
        window.launch_settings_tab.set_business_mode('実機', '手動走行')
        window.launch_settings_tab.configure_gnss(args.gnss_site, args.gnss_station)
        QtCore.QTimer.singleShot(0, lambda: core.request_launch('rtk_gps_um982'))
    # launchからの終了通知をQtの終了イベントへ変換し、ROSスレッドも停止する。
    previous = {sig: signal.signal(sig, lambda *_: app.quit())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        return app.exec_()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    sys.exit(main())
