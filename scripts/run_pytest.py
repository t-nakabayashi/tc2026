#!/usr/bin/env python3
"""pytest.ini の全対象を、GUI とその他で別プロセスに分けて実行する。

ROS 通信の初期化後に Qt テストを fork すると DDS のロックを引き継ぐため、
robot_console の親 pytest プロセスでは他パッケージの ROS テストを実行しない。
追加の pytest オプションはコマンドラインから渡せる。
"""

import configparser
import os
import signal
import subprocess
import sys
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    config = configparser.ConfigParser()
    config.read(root / 'pytest.ini')
    targets = config['pytest']['testpaths'].split()
    console = 'src/robot_console/tools/tests'
    groups = ([target for target in targets if target == console],
              [target for target in targets if target != console])
    failed = False
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '193')
    env.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    for group in groups:
        if not group:
            continue
        command = [sys.executable, '-m', 'pytest', *group, *sys.argv[1:]]
        print('Running:', ' '.join(command), flush=True)
        with subprocess.Popen(command, cwd=root, env=env, start_new_session=True) as process:
            try:
                code = process.wait(timeout=600)
                failed = failed or code != 0
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                if isinstance(exc, KeyboardInterrupt):
                    return 130
                print('pytest exceeded 600 seconds', file=sys.stderr, flush=True)
                failed = True
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
