"""profile定義駆動でノードのlaunchプロセスを起動・停止するモジュール。

`ros2 launch` を subprocess として起動・監視し、標準出力/標準エラーの収集と
ログファイルへの保存を行う。QWidget/tkinter などUI部品への依存を持たない。
"""

from __future__ import annotations

import os
import platform
import signal
import subprocess
import threading
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Union

from ..utils import NodeLaunchStatus
from .launch_profile import (
    LaunchProfile,
    build_launch_args,
    build_simulator_launch_args,
    resolve_param_path,
)

StatusCallback = Callable[[str, NodeLaunchStatus, Optional[int], Optional[str]], None]
LogCallback = Callable[[str, str], None]


class LaunchManager:
    """subprocess ベースでprofile定義に基づきノードを起動・停止する。"""

    def __init__(
        self,
        status_callback: StatusCallback,
        log_callback: LogCallback,
        log_directory: Optional[Union[str, Path]] = None,
    ) -> None:
        self._status_callback = status_callback
        self._log_callback = log_callback
        self._lock = threading.Lock()
        self._processes: Dict[str, subprocess.Popen[str]] = {}
        self._sim_processes: Dict[str, subprocess.Popen[str]] = {}
        self._threads: List[threading.Thread] = []
        self._log_meta_lock = threading.Lock()
        self._log_paths: Dict[str, Path] = {}
        self._log_locks: Dict[str, threading.Lock] = {}
        self._log_stream_counters: Dict[str, int] = {}
        self._latest_log_paths: Dict[str, Path] = {}
        self._base_log_directory: Optional[Path] = None
        self._run_log_directory: Optional[Path] = None
        if log_directory:
            self._base_log_directory = Path(log_directory).expanduser()

    def launch(
        self,
        profile: LaunchProfile,
        *,
        param_path: Optional[str] = None,
        use_alternate: bool = False,
        simulator_enabled: bool = False,
        overrides: Optional[Dict[str, str]] = None,
    ) -> None:
        """指定profileのノードを起動する。既に起動中の場合は一度停止してから起動する。"""

        with self._lock:
            self._cleanup_finished_process_locked()
            already_running = profile.profile_id in self._processes
        if already_running:
            self.stop(profile.profile_id)

        with self._lock:
            self._cleanup_finished_process_locked()
            if profile.profile_id in self._processes:
                raise RuntimeError(f"{profile.profile_id} の停止完了を待っています")
            self._status_callback(profile.profile_id, NodeLaunchStatus.STARTING, None, None)

            args = build_launch_args(
                profile,
                param_path=resolve_param_path(profile, param_path),
                use_alternate=use_alternate,
                overrides=overrides,
            )

            process = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=self._build_subprocess_env(),
                **self._build_popen_options(),
            )
            self._processes[profile.profile_id] = process
            self._status_callback(profile.profile_id, NodeLaunchStatus.RUNNING, process.pid, None)
            self._start_reader_threads(profile.profile_id, process)
            self._start_monitor_thread(
                dict_key=profile.profile_id,
                status_id=profile.profile_id,
                process=process,
                is_simulator=False,
            )

        if simulator_enabled and profile.simulator_launch_file:
            self._launch_simulator(profile, overrides)

    def _launch_simulator(
        self, profile: LaunchProfile, overrides: Optional[Dict[str, str]]
    ) -> None:
        """profile.simulator_argument_map に従い引数を変換して代替launchを起動する。

        simulator_argument_map に掲載の無いoverrideキーは、simulator側launchが
        受け付けない引数である可能性が高いため転送しない。
        """

        sim_args = build_simulator_launch_args(profile, overrides)

        sim_process = subprocess.Popen(
            sim_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=self._build_subprocess_env(),
            **self._build_popen_options(),
        )
        with self._lock:
            self._sim_processes[profile.profile_id] = sim_process
        self._start_monitor_thread(
            dict_key=profile.profile_id,
            status_id=f"{profile.profile_id}:sim",
            process=sim_process,
            is_simulator=True,
        )
        self._start_reader_threads(f"{profile.profile_id}:sim", sim_process)
        self._status_callback(
            f"{profile.profile_id}:sim",
            NodeLaunchStatus.RUNNING,
            sim_process.pid,
            None,
        )

    def stop(self, profile_id: str) -> None:
        """ノードを停止する。"""

        with self._lock:
            process = self._processes.get(profile_id)
            sim_process = self._sim_processes.get(profile_id)

        if process is not None:
            self._status_callback(profile_id, NodeLaunchStatus.STOPPING, process.pid, None)
            self._terminate_process(process)
        else:
            self._status_callback(profile_id, NodeLaunchStatus.STOPPED, None, None)

        if sim_process is not None:
            self._status_callback(
                f"{profile_id}:sim",
                NodeLaunchStatus.STOPPING,
                sim_process.pid,
                None,
            )
            self._terminate_process(sim_process)
        else:
            self._status_callback(f"{profile_id}:sim", NodeLaunchStatus.STOPPED, None, None)
        with self._lock:
            self._cleanup_finished_process_locked()

    def is_running(self, profile_id: str) -> bool:
        """指定ノードが稼働中かどうかを判定する。"""

        with self._lock:
            self._cleanup_finished_process_locked()
            if profile_id in self._processes:
                return True
            return profile_id in self._sim_processes

    def get_latest_log_path(self, profile_id: str) -> Optional[Path]:
        """指定ノードの直近のログファイルパスを返す。"""

        with self._log_meta_lock:
            path = self._latest_log_paths.get(profile_id)
        return path

    def _terminate_process(self, process: subprocess.Popen[str]) -> None:
        """プロセスに対し SIGINT → SIGTERM → SIGKILL の順で停止要求を行う。"""

        if process.poll() is not None:
            return
        self._send_signal(process, signal.SIGINT)
        if self._wait_process(process, timeout=5.0):
            return
        self._send_signal(process, signal.SIGTERM)
        if self._wait_process(process, timeout=3.0):
            return
        self._send_signal(process, signal.SIGKILL)
        self._wait_process(process, timeout=1.0, suppress_timeout=True)

    @staticmethod
    def _build_popen_options() -> Dict[str, object]:
        """サブプロセス起動時のオプションを構築する。"""

        options: Dict[str, object] = {}
        if os.name == 'nt':
            options['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            options['preexec_fn'] = os.setsid
        return options

    @staticmethod
    def _build_subprocess_env() -> Dict[str, str]:
        """ROS launch 子プロセス向けの環境変数を構築する。

        rviz2 など Qt を利用するノードを ``ros2 launch`` 経由で起動する場合に
        備え、PyQt5 が利用可能であれば Qt プラットフォームプラグインの探索先を
        補完する。PyQt5 が無い環境では素通しする。
        """

        env = os.environ.copy()
        try:
            from PyQt5 import QtCore  # type: ignore[import-not-found]
        except ImportError:
            return env

        plugin_root = Path(QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath))
        platform_root = plugin_root / 'platforms'
        qt_lib_root = plugin_root.parent / 'lib'
        if platform_root.exists():
            env['QT_QPA_PLATFORM_PLUGIN_PATH'] = str(platform_root)
            env['QT_PLUGIN_PATH'] = str(plugin_root)
        if 'QT_QPA_PLATFORM' not in env:
            env['QT_QPA_PLATFORM'] = 'xcb'
        if qt_lib_root.exists():
            current_library_path = env.get('LD_LIBRARY_PATH')
            if current_library_path:
                env['LD_LIBRARY_PATH'] = f"{qt_lib_root}:{current_library_path}"
            else:
                env['LD_LIBRARY_PATH'] = str(qt_lib_root)
        return env

    def _send_signal(self, process: subprocess.Popen[str], sig: signal.Signals) -> None:
        """プロセスグループにシグナルを送信し、失敗時は個別プロセスに送る。"""

        if process.poll() is not None:
            return
        if os.name != 'nt':
            try:
                os.killpg(os.getpgid(process.pid), sig)
                return
            except (ProcessLookupError, PermissionError, AttributeError, OSError):
                pass
        try:
            process.send_signal(sig)
        except (ProcessLookupError, AttributeError, ValueError):
            try:
                process.terminate()
            except (ProcessLookupError, AttributeError, ValueError):
                return

    @staticmethod
    def _wait_process(
        process: subprocess.Popen[str], timeout: float, suppress_timeout: bool = False
    ) -> bool:
        """指定時間でプロセス終了を待ち、終了したら True を返す。"""

        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            if suppress_timeout:
                return False
            return False

    def _start_reader_threads(self, profile_id: str, process: subprocess.Popen[str]) -> None:
        """stdout/stderr を読み取るスレッドを起動する。"""

        streams = []
        if process.stdout is not None:
            streams.append((process.stdout, 'INFO'))
        if process.stderr is not None:
            streams.append((process.stderr, 'ERR'))

        if streams:
            self._prepare_log_file(profile_id, process.pid, len(streams))

        def _reader(stream, prefix: str) -> None:
            try:
                for line in iter(stream.readline, ''):
                    formatted = f"[{prefix}] {line.rstrip()}\n"
                    self._log_callback(profile_id, formatted)
                    self._append_log_line(profile_id, formatted)
            finally:
                stream.close()
                self._notify_stream_closed(profile_id)

        for stream, prefix in streams:
            thread = threading.Thread(target=_reader, args=(stream, prefix), daemon=True)
            thread.start()
            self._threads.append(thread)

    def _cleanup_finished_process_locked(self) -> None:
        """監視用辞書から終了済みプロセスを除去する。"""

        for key, process in list(self._processes.items()):
            if process.poll() is not None:
                self._processes.pop(key, None)
        for key, process in list(self._sim_processes.items()):
            if process.poll() is not None:
                self._sim_processes.pop(key, None)

    def _prepare_log_file(self, profile_id: str, pid: int, stream_count: int) -> None:
        """ログファイルの作成準備を行う。"""

        if self._base_log_directory is None:
            return
        run_dir = self._ensure_run_log_directory()
        if run_dir is None:
            return
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        sanitized = profile_id.replace(':', '_')
        file_name = f"{sanitized}-{pid}-{timestamp}.log"
        path = run_dir / file_name
        with self._log_meta_lock:
            self._log_paths[profile_id] = path
            self._log_locks[profile_id] = threading.Lock()
            self._log_stream_counters[profile_id] = stream_count
            self._latest_log_paths[profile_id] = path

    def _append_log_line(self, profile_id: str, line: str) -> None:
        """ログ行をファイルへ追記する。"""

        with self._log_meta_lock:
            path = self._log_paths.get(profile_id)
            lock = self._log_locks.get(profile_id)
        if path is None or lock is None:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        with lock:
            with path.open('a', encoding='utf-8') as handle:
                handle.write(line)

    def _notify_stream_closed(self, profile_id: str) -> None:
        """ストリーム終了時にファイル管理情報を更新する。"""

        with self._log_meta_lock:
            counter = self._log_stream_counters.get(profile_id)
            if counter is None:
                return
            if counter <= 1:
                self._log_stream_counters.pop(profile_id, None)
                self._log_locks.pop(profile_id, None)
                self._log_paths.pop(profile_id, None)
            else:
                self._log_stream_counters[profile_id] = counter - 1

    def _ensure_run_log_directory(self) -> Optional[Path]:
        """ROS2 標準に倣ったランログディレクトリを生成する。"""

        if self._base_log_directory is None:
            return None
        if self._run_log_directory is not None:
            return self._run_log_directory
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        hostname = platform.node() or 'unknown_host'
        run_dir = self._base_log_directory / f"{timestamp}_{hostname}"
        run_dir.mkdir(parents=True, exist_ok=True)
        self._run_log_directory = run_dir
        return run_dir

    def _start_monitor_thread(
        self,
        dict_key: str,
        status_id: str,
        process: subprocess.Popen[str],
        is_simulator: bool,
    ) -> None:
        """終了状態を監視し辞書の整合性を保つスレッドを起動する。"""

        def _monitor() -> None:
            return_code = process.wait()
            with self._lock:
                target = self._sim_processes if is_simulator else self._processes
                target.pop(dict_key, None)
            status = NodeLaunchStatus.STOPPED
            message = None
            if return_code != 0:
                status = NodeLaunchStatus.ERROR
                message = f"プロセスが異常終了しました (return code={return_code})"
            self._status_callback(status_id, status, None, message)

        thread = threading.Thread(target=_monitor, daemon=True)
        thread.start()
        self._threads.append(thread)
