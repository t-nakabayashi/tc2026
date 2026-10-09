"""rosbag2子プロセスを管理する。開始は明示操作のみ、停止はSIGINTで保存を完了する。"""
from dataclasses import dataclass, replace
from datetime import datetime
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import uuid


@dataclass
class BagState:
    state: str = 'IDLE'
    directory: str = ''
    output: str = ''
    elapsed_s: float = 0.
    size_bytes: int = 0
    free_bytes: int = 0
    message: str = ''


class BagRecorder:
    def __init__(self, directory=None):
        self.directory = Path(directory or os.environ.get('ROBOT_CONSOLE_BAG_DIR',
                                                         '~/rosbags')).expanduser().resolve()
        self._lock = threading.RLock()
        self._state = BagState(directory=str(self.directory))
        self._process = None
        self._file_lock = None
        self._thread = None
        self._started = 0.
        self._error = ''

    def snapshot(self):
        with self._lock:
            return replace(self._state)

    def start(self, directory=None):
        with self._lock:
            if self._process is not None:
                return False
            self._state = BagState(directory=str(self.directory))
            try:
                if directory:
                    self.directory = Path(directory).expanduser().resolve()
                    self._state.directory = str(self.directory)
                self.directory.mkdir(parents=True, exist_ok=True)
                lock = (self.directory/'.robot_console_record.lock').open('a')
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    lock.close()
                    raise RuntimeError('同じ保存先で別のUIが記録中です')
                self._file_lock = lock
                free = shutil.disk_usage(self.directory).free
                if free < 1024**3:
                    raise RuntimeError('空き容量が1 GiB未満です')
                name = datetime.now().strftime('bag_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8]
                output = self.directory/name
                self._state = BagState(state='STARTING', directory=str(self.directory),
                                       output=str(output), free_bytes=free)
                self._started = time.monotonic()
                self._error = ''
                # 専用relayで累積地図は10秒、Sensor Viewerは1秒間隔にする。
                log = (self.directory/(name+'.log')).open('w')
                try:
                    self._process = subprocess.Popen([
                        'ros2', 'run', 'robot_console', 'record_bag_with_map',
                        '--output', str(output), '--max-bag-size', str(1024**3)], stdin=subprocess.DEVNULL,
                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                finally:
                    log.close()
                self._thread = threading.Thread(target=self._watch, daemon=True)
                self._thread.start()
                return True
            except (OSError, RuntimeError) as exc:
                self._state.state = 'ERROR'
                self._state.message = str(exc)
                self._release_lock()
                return False

    def stop(self):
        with self._lock:
            if self._process is None or self._state.state == 'STOPPING':
                return False
            self._state.state = 'STOPPING'
            self._state.message = '保存処理中です'
            try:
                os.killpg(self._process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            return True

    def _watch(self):
        while True:
            with self._lock:
                process = self._process
                result = process.poll()
                output = Path(self._state.output)
                self._state.elapsed_s = time.monotonic()-self._started
                try:
                    self._state.size_bytes = sum(p.stat().st_size for p in output.glob('*') if p.is_file())
                    self._state.free_bytes = shutil.disk_usage(self.directory).free
                except OSError:
                    pass
                if result is not None:
                    saved = result == 0 and (output/'metadata.yaml').is_file()
                    self._state.state = 'SAVED' if saved and not self._error else 'ERROR'
                    self._state.message = (self._error or ('保存完了' if saved else
                        f'記録終了異常（code={result}）。同名.logを確認してください'))
                    self._process = None
                    self._release_lock()
                    return
                if self._state.free_bytes < 512*1024**2 and self._state.state != 'STOPPING':
                    self._error = '空き容量低下のため記録を停止しました'
                    self.stop()
                elif self._state.state == 'STARTING' and any(output.glob('*.db3')):
                    self._state.state = 'RECORDING'
            time.sleep(.25)

    def _release_lock(self):
        if self._file_lock is not None:
            self._file_lock.close()
            self._file_lock = None

    def close(self):
        self.stop()
        thread = self._thread
        if thread:
            thread.join(timeout=20.)
            with self._lock:
                if self._process is not None:
                    self._error = '終了待ちがタイムアウトしました。bagの復旧確認が必要です'
                    try:
                        os.killpg(self._process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
            thread.join(timeout=5.)
