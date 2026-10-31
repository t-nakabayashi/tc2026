# -*- coding: utf-8 -*-
"""外部 CLI を非同期に起動し、タイムアウトで確実に終了させる ROS 非依存のコア.

AI の推論は `claude -p` 相当の CLI を機上 PC で起動して行う。応答は数秒から十数秒かかり、
制御周期とは別の時間軸になるため、呼び出しは専用スレッドで行い、結果を取りに行く側は
待たない。

作業ディレクトリは実行のたびに作る専用ディレクトリとする。リポジトリ内で起動すると
リポジトリの設定や指示をエージェントが読み込み、助言と無関係な指示が混入するためである。
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence
import os
import signal
import subprocess
import threading
import time
import uuid

# 要求文を差し込む位置を表す置換語。コマンド列のどこへ渡すかを設定で変えられるようにする。
PROMPT_PLACEHOLDER = '{prompt}'

STATUS_RUNNING = 'running'
STATUS_DONE = 'done'
STATUS_TIMEOUT = 'timeout'
STATUS_FAILED = 'failed'
STATUS_CANCELED = 'canceled'


@dataclass
class CliResult:
    """CLI の実行結果."""

    status: str = STATUS_RUNNING
    stdout: str = ''
    stderr: str = ''
    returncode: Optional[int] = None
    elapsed_s: float = 0.0


@dataclass(frozen=True)
class CliConfig:
    """CLI 実行の設定.

    Attributes:
        command (Sequence[str]): 起動するコマンド列. `{prompt}` を要求文で置換する.
        timeout_sec (float): 応答を待つ上限 [s].
        workspace_root (Path): 実行ごとの作業ディレクトリを作る親ディレクトリ.
        keep_workspace (bool): 実行後に作業ディレクトリを残すか.
        env_overrides (Dict[str, str]): 追加で与える環境変数.
    """

    command: Sequence[str] = ('claude', '-p', PROMPT_PLACEHOLDER)
    timeout_sec: float = 20.0
    workspace_root: Path = Path('.')
    keep_workspace: bool = True
    env_overrides: Dict[str, str] = field(default_factory=dict)


class CliInvocation:
    """1 回の CLI 実行。スレッドで起動し、呼び出し側は結果を取りに来る."""

    def __init__(self, config: CliConfig, prompt: str, request_id: str,
                 workspace: Path) -> None:
        self._config = config
        self._prompt = prompt
        self._workspace = workspace
        self.request_id = request_id
        self.result = CliResult()
        self._process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        """実行を開始する."""
        self._thread.start()

    def wait(self, timeout: Optional[float] = None) -> bool:
        """完了を待つ. 完了していれば True を返す."""
        return self._done.wait(timeout)

    @property
    def finished(self) -> bool:
        return self._done.is_set()

    @property
    def workspace(self) -> Path:
        return self._workspace

    def cancel(self) -> None:
        """実行中のプロセスを終了させる.

        子プロセスが残らないよう、プロセスグループごと終了させる。
        """
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                if not self._done.is_set():
                    self.result.status = STATUS_CANCELED
                    self._done.set()
                return
            self.result.status = STATUS_CANCELED
        _terminate_group(process)

    def _run(self) -> None:
        started = time.monotonic()
        command = [self._prompt if part == PROMPT_PLACEHOLDER else part
                   for part in self._config.command]
        environment = dict(os.environ)
        environment.update(self._config.env_overrides)
        try:
            process = subprocess.Popen(
                command,
                cwd=str(self._workspace),
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.DEVNULL,
                text=True,
                start_new_session=True,
            )
        except (OSError, ValueError) as error:
            self.result.status = STATUS_FAILED
            self.result.stderr = str(error)
            self.result.elapsed_s = time.monotonic() - started
            self._done.set()
            return

        with self._lock:
            self._process = process
            canceled = self.result.status == STATUS_CANCELED
        if canceled:
            _terminate_group(process)

        try:
            stdout, stderr = process.communicate(timeout=self._config.timeout_sec)
            timed_out = False
        except subprocess.TimeoutExpired:
            _terminate_group(process)
            stdout, stderr = process.communicate()
            timed_out = True

        self.result.stdout = stdout or ''
        self.result.stderr = stderr or ''
        self.result.returncode = process.returncode
        self.result.elapsed_s = time.monotonic() - started
        if timed_out:
            self.result.status = STATUS_TIMEOUT
        elif self.result.status != STATUS_CANCELED:
            self.result.status = (STATUS_DONE if process.returncode == 0
                                  else STATUS_FAILED)
        self._done.set()


class CliRunner:
    """CLI 実行の生成と、同時実行数 1 の制御を行う.

    同時に複数の推論を走らせない。機上 PC では他のノードと計算資源を共有しており、
    複数起動が制御周期へ影響するためである。
    """

    def __init__(self, config: CliConfig) -> None:
        self._config = config
        self._current: Optional[CliInvocation] = None

    @property
    def busy(self) -> bool:
        """実行中の呼び出しがあるか."""
        return self._current is not None and not self._current.finished

    def start(self, prompt: str, files: Optional[Dict[str, bytes]] = None,
              request_id: Optional[str] = None) -> Optional[CliInvocation]:
        """作業ディレクトリを作り、CLI を起動する.

        Args:
            prompt (str): 要求文.
            files (Optional[Dict[str, bytes]]): 作業ディレクトリへ置くファイル.
            request_id (Optional[str]): 要求の識別子. 省略時は生成する.

        Returns:
            Optional[CliInvocation]: 起動した呼び出し. 実行中の場合は None.
        """
        if self.busy:
            return None
        identifier = request_id or uuid.uuid4().hex[:12]
        workspace = Path(self._config.workspace_root) / identifier
        workspace.mkdir(parents=True, exist_ok=True)
        for name, content in (files or {}).items():
            (workspace / name).write_bytes(content)
        (workspace / 'prompt.txt').write_text(prompt, encoding='utf-8')
        invocation = CliInvocation(self._config, prompt, identifier, workspace)
        self._current = invocation
        invocation.start()
        return invocation

    def cancel(self) -> None:
        """実行中の呼び出しを終了させる."""
        if self._current is not None:
            self._current.cancel()


def _terminate_group(process: subprocess.Popen) -> None:
    """プロセスグループへ終了を要求し、残る場合は強制終了する."""
    try:
        group = os.getpgid(process.pid)
    except (OSError, ProcessLookupError):
        return
    for sig, wait in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(group, sig)
        except (OSError, ProcessLookupError):
            return
        try:
            process.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue
