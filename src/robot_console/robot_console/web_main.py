"""HTML遠隔観測UIのスタンドアロン起動エントリポイント。

`ConsoleCore` を生成し、`ros/console_node.py::start_ros_thread()` でHTTP
サーバスレッドとは別のrclpy executorスレッドを起動する
（robot_console_gui_architecture_design.md 3章・15章）。`WebObservationServer`
の`snapshot_provider`/`image_store`は`ConsoleCore`が保持するものを直接使う。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional

from robot_console.core.console_core import ConsoleCore
from robot_console.ros.console_node import start_ros_thread
from robot_console.web.server import DEFAULT_HOST, DEFAULT_PORT, WebObservationServer


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='robot_console HTML遠隔観測UI（閲覧専用）')
    parser.add_argument(
        '--host', default=DEFAULT_HOST, help=f'bindするホスト（既定: {DEFAULT_HOST}）'
    )
    parser.add_argument(
        '--port', type=int, default=DEFAULT_PORT, help=f'bindするポート（既定: {DEFAULT_PORT}）'
    )
    parser.add_argument(
        '--console-log-directory',
        default=os.environ.get('ROBOT_CONSOLE_LOG_DIR'),
        help=(
            'robot_console 管理の子プロセス stdout/stderr 保存先。'
            '未指定時は ROBOT_CONSOLE_LOG_DIR を参照します'
        ),
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    """HTML遠隔観測UIサーバを起動し、Ctrl+Cまでブロックする。"""

    args = _parse_args(argv if argv is not None else sys.argv[1:])

    core = ConsoleCore(log_directory=args.console_log_directory)
    ros_handle = start_ros_thread(core, node_name='robot_console_web')

    server = WebObservationServer(
        core.build_snapshot, core.image_store, host=args.host, port=args.port
    )
    server.start()
    address = server.address
    if address is not None:
        print(f'robot_console HTML遠隔観測UIを起動しました: http://{address[0]}:{address[1]}/')
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        ros_handle.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
