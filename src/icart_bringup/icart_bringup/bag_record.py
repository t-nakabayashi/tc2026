"""走行の rosbag 記録を起動側から自動で開始する。

robot_console の記録は操作者の明示操作で始まるため、自律走行では記録を忘れると走行後に
検証する材料が残らない。本モジュールは共通起動から記録を開始し、GUI の記録と同じ保存先・
同じ命名・同じロックを使って二重記録を防ぐ。

ロックは robot_console の `BagRecorder` と同じファイルを使う。先に本モジュールが確保する
ため、GUI 側は「同じ保存先で別の UI が記録中です」として開始を断る。
"""

from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence
import argparse
import fcntl
import os
import shutil
import signal
import subprocess
import sys
import uuid

# robot_console の BagRecorder と共有する設定。片方だけ変えると二重記録を防げない。
DIRECTORY_ENVIRONMENT = 'ROBOT_CONSOLE_BAG_DIR'
DEFAULT_DIRECTORY = '~/rosbags'
LOCK_NAME = '.robot_console_record.lock'
MINIMUM_FREE_BYTES = 1024 ** 3
DEFAULT_MAX_BAG_SIZE = 1024 ** 3


def resolve_directory(value: Optional[str] = None) -> Path:
    """保存先を決める。引数、環境変数、既定値の順に採る."""
    candidate = value or os.environ.get(DIRECTORY_ENVIRONMENT) or DEFAULT_DIRECTORY
    return Path(candidate).expanduser().resolve()


def build_output_name(now: Optional[datetime] = None,
                      token: Optional[str] = None) -> str:
    """保存先のディレクトリ名を作る。

    robot_console の記録と同じ `bag_<日時>_<8桁>` 形式にする。転送や集計のツールが
    名前で bag を拾うため、記録元によって形式を変えない。
    """
    stamp = (now or datetime.now()).strftime('bag_%Y%m%d_%H%M%S_')
    return stamp + (token or uuid.uuid4().hex[:8])


def record_command(output: Path, max_bag_size: int = DEFAULT_MAX_BAG_SIZE) -> List[str]:
    """記録プロセスの起動コマンドを返す。

    `ros2 bag record` を直接呼ばず robot_console の記録を使う。累積地図と Sensor Viewer を
    間引く中継を同じ条件で通し、GUI 記録と bag の内容を揃えるためである。
    """
    return ['ros2', 'run', 'robot_console', 'record_bag_with_map',
            '--output', str(output), '--max-bag-size', str(int(max_bag_size))]


def acquire_lock(directory: Path):
    """保存先のロックを取る。取れない場合は None を返す."""
    directory.mkdir(parents=True, exist_ok=True)
    handle = (directory / LOCK_NAME).open('a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    return handle


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', default='', help='保存先。空なら環境変数か既定値')
    parser.add_argument('--max-bag-size', type=int, default=DEFAULT_MAX_BAG_SIZE,
                        help='分割サイズ [byte]')
    return parser.parse_args(list(argv) if argv is not None else None)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """記録を開始し、停止信号を受けるまで待つ。

    記録を開始できない場合も 0 で終わる。走行そのものを止めないためであり、理由は
    標準出力へ残す。
    """
    args = parse_args(argv)
    directory = resolve_directory(args.directory or None)
    try:
        lock = acquire_lock(directory)
    except OSError as error:
        print(f'記録を開始できない: 保存先を作成できない {directory} ({error})', flush=True)
        return 0
    if lock is None:
        print(f'記録を開始しない: 同じ保存先で別の記録が動作中 {directory}', flush=True)
        return 0
    free = shutil.disk_usage(directory).free
    if free < MINIMUM_FREE_BYTES:
        print(f'記録を開始しない: 空き容量が不足 {free / 1024 ** 3:.1f} GiB', flush=True)
        lock.close()
        return 0

    output = directory / build_output_name()
    print(f'記録を開始する: {output}', flush=True)
    process = subprocess.Popen(record_command(output, args.max_bag_size),
                               stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        process.wait()
    except KeyboardInterrupt:
        # 停止信号は記録プロセスへ渡し、metadata.yaml の書き出しを待つ。
        _stop(process)
    finally:
        lock.close()
    return 0


def _stop(process: subprocess.Popen) -> None:
    """記録プロセスを保存付きで終了させる."""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except (OSError, ProcessLookupError):
        return
    try:
        process.wait(timeout=30.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            return
        process.wait(timeout=5.0)


if __name__ == '__main__':
    sys.exit(main())
