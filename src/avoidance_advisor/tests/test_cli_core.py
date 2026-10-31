"""外部 CLI の起動、タイムアウト、取り消しの挙動を固定する。ROS グラフを使わない。

実際に子プロセスを起動する。AI の CLI は使わず、同じ振る舞いをする短いスクリプトへ
置き換えて確認する。確認したいのは、応答を待つ側が固まらないこと、タイムアウトで
プロセスが残らないこと、作業ディレクトリがリポジトリ外に作られることである。
"""

from pathlib import Path
import os
import signal
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from avoidance_advisor.cli_core import (PROMPT_PLACEHOLDER, STATUS_CANCELED,
                                        STATUS_DONE, STATUS_FAILED, STATUS_TIMEOUT,
                                        CliConfig, CliRunner)


def config_for(tmp_path: Path, command, timeout_sec: float = 5.0) -> CliConfig:
    return CliConfig(command=command, timeout_sec=timeout_sec,
                     workspace_root=tmp_path / 'sessions')


def test_output_is_captured_and_status_is_done(tmp_path: Path) -> None:
    """CLI の標準出力を取得し、正常終了を記録する."""
    runner = CliRunner(config_for(tmp_path, ['printf', '%s', '{"action": "wait"}']))

    invocation = runner.start('要求文')
    assert invocation.wait(timeout=10.0)

    assert invocation.result.status == STATUS_DONE
    assert invocation.result.stdout == '{"action": "wait"}'
    assert invocation.result.returncode == 0


def test_prompt_placeholder_is_replaced_in_the_command(tmp_path: Path) -> None:
    """コマンド列の置換語が要求文へ差し替わる."""
    runner = CliRunner(config_for(tmp_path, ['echo', PROMPT_PLACEHOLDER]))

    invocation = runner.start('左右どちらへ避けるか')
    invocation.wait(timeout=10.0)

    assert invocation.result.stdout.strip() == '左右どちらへ避けるか'


def test_workspace_holds_the_prompt_and_given_files(tmp_path: Path) -> None:
    """作業ディレクトリに要求文と入力ファイルが残り、後から再現できる."""
    runner = CliRunner(config_for(tmp_path, ['true']))

    invocation = runner.start('要求文', files={'layers.png': b'\x89PNG'},
                              request_id='abc123')
    invocation.wait(timeout=10.0)

    workspace = invocation.workspace
    assert workspace.name == 'abc123'
    assert (workspace / 'prompt.txt').read_text(encoding='utf-8') == '要求文'
    assert (workspace / 'layers.png').read_bytes() == b'\x89PNG'


def test_timeout_terminates_the_process(tmp_path: Path) -> None:
    """応答が上限を超えた場合、プロセスを終了させて timeout を記録する."""
    runner = CliRunner(config_for(tmp_path, ['sleep', '30'], timeout_sec=0.5))

    invocation = runner.start('要求文')
    assert invocation.wait(timeout=10.0)

    assert invocation.result.status == STATUS_TIMEOUT
    assert invocation.result.elapsed_s < 5.0


def test_timeout_kills_child_processes_of_the_cli(tmp_path: Path) -> None:
    """CLI が起こした子プロセスも残さない.

    プロセスグループごと終了させる。グループを使わないと、CLI 本体だけが消え、
    推論を続ける子プロセスが機上 PC に残る。
    """
    marker = tmp_path / 'child.pid'
    command = ['sh', '-c', f'sleep 30 & echo $! > {marker}; wait']
    runner = CliRunner(config_for(tmp_path, command, timeout_sec=0.8))

    invocation = runner.start('要求文')
    invocation.wait(timeout=10.0)
    deadline = time.monotonic() + 3.0
    while not marker.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    child_pid = int(marker.read_text().strip())

    time.sleep(0.5)
    with pytest.raises(OSError):
        os.kill(child_pid, 0)


def test_cancel_stops_a_running_invocation(tmp_path: Path) -> None:
    """取り消しで実行中のプロセスを終了させる（action の cancel に対応する）."""
    runner = CliRunner(config_for(tmp_path, ['sleep', '30'], timeout_sec=30.0))

    invocation = runner.start('要求文')
    time.sleep(0.3)
    runner.cancel()

    assert invocation.wait(timeout=10.0)
    assert invocation.result.status == STATUS_CANCELED


def test_runner_rejects_a_second_invocation_while_busy(tmp_path: Path) -> None:
    """実行中は次の呼び出しを受け付けない（同時実行数 1）."""
    runner = CliRunner(config_for(tmp_path, ['sleep', '2'], timeout_sec=5.0))

    first = runner.start('要求文')
    second = runner.start('要求文')

    assert first is not None
    assert second is None
    runner.cancel()


def test_missing_command_is_reported_as_failure(tmp_path: Path) -> None:
    """CLI が見つからない場合も例外にせず、失敗として記録する.

    実行環境に CLI が入っていない構成でもノードを落とさない。
    """
    runner = CliRunner(config_for(tmp_path, ['no_such_command_for_test']))

    invocation = runner.start('要求文')
    assert invocation.wait(timeout=10.0)

    assert invocation.result.status == STATUS_FAILED
    assert invocation.result.stderr


def test_nonzero_exit_is_reported_as_failure(tmp_path: Path) -> None:
    """異常終了した CLI の出力は採用しない."""
    runner = CliRunner(config_for(tmp_path, ['false']))

    invocation = runner.start('要求文')
    invocation.wait(timeout=10.0)

    assert invocation.result.status == STATUS_FAILED
