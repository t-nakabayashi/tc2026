"""走行記録の自動開始を固定する。rosbag もロボットも起動しない。

確認するのは、保存先と命名が robot_console の記録と一致すること、二重記録を防ぐこと、
記録を始められない場合でも走行を止めないことである。
"""

from pathlib import Path
import re
import shutil
import sys

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / 'robot_console'))
from icart_bringup import bag_record


def test_directory_prefers_argument_then_environment(monkeypatch, tmp_path: Path) -> None:
    """保存先は引数、環境変数、既定値の順に決まる."""
    monkeypatch.setenv(bag_record.DIRECTORY_ENVIRONMENT, str(tmp_path / 'env'))

    assert bag_record.resolve_directory(str(tmp_path / 'arg')) == tmp_path / 'arg'
    assert bag_record.resolve_directory() == tmp_path / 'env'

    monkeypatch.delenv(bag_record.DIRECTORY_ENVIRONMENT)
    assert bag_record.resolve_directory() == Path(bag_record.DEFAULT_DIRECTORY).expanduser().resolve()


def test_output_name_matches_the_console_naming() -> None:
    """保存先の名前は robot_console の記録と同じ形式にする.

    転送や集計のツールは名前で bag を拾う。記録元によって形式が変わると拾えなくなる。
    """
    assert re.fullmatch(r'bag_\d{8}_\d{6}_[0-9a-f]{8}', bag_record.build_output_name())


def test_record_command_uses_the_console_recorder(tmp_path: Path) -> None:
    """記録は robot_console の record_bag_with_map 経由で行う.

    `ros2 bag record` を直接呼ぶと、累積地図と Sensor Viewer の間引きが掛からず、
    GUI 記録と bag の内容が変わる。
    """
    command = bag_record.record_command(tmp_path / 'bag_x', 123)

    assert command[:5] == ['ros2', 'run', 'robot_console', 'record_bag_with_map', '--output']
    assert command[5] == str(tmp_path / 'bag_x')
    assert command[-2:] == ['--max-bag-size', '123']


def test_lock_is_shared_with_the_console_recorder(tmp_path: Path) -> None:
    """起動側が記録していると、GUI は同じ保存先で記録を始められない.

    ロックファイル名を片方だけ変えると二重記録になり、同じ topic を 2 本の
    `ros2 bag record` が書き込む。
    """
    from robot_console.core.bag_recorder import BagRecorder

    held = bag_record.acquire_lock(tmp_path)
    recorder = BagRecorder(str(tmp_path))

    assert held is not None
    assert recorder.start() is False
    assert '記録中' in recorder.snapshot().message
    held.close()


def test_second_launch_side_recorder_does_not_start(tmp_path: Path, capsys) -> None:
    """起動側の記録が二重に動かない。開始しない場合も終了コードは 0 とする."""
    held = bag_record.acquire_lock(tmp_path)

    result = bag_record.main(['--directory', str(tmp_path)])

    assert result == 0
    assert '別の記録が動作中' in capsys.readouterr().out
    held.close()


def test_low_free_space_skips_recording_without_stopping_the_run(
        monkeypatch, tmp_path: Path, capsys) -> None:
    """空き容量が足りない場合は記録せず、走行は続ける.

    記録は走行の付帯処理であり、失敗で走行を止めない。
    """
    monkeypatch.setattr(bag_record.shutil, 'disk_usage',
                        lambda _: shutil._ntuple_diskusage(0, 0, 1024 ** 2))

    result = bag_record.main(['--directory', str(tmp_path)])

    assert result == 0
    assert '空き容量が不足' in capsys.readouterr().out


def test_run_session_passes_launch_arguments_through() -> None:
    """run_session の末尾に並べた `key:=value` は共通launchへ渡る."""
    from icart_bringup.session_core import validate_launch_arguments

    assert validate_launch_arguments(['record_bag:=false', 'bag_directory:=/tmp/x']) == [
        'record_bag:=false', 'bag_directory:=/tmp/x']


def test_run_session_rejects_malformed_or_reserved_arguments() -> None:
    """形式違いと、run_session が決める引数の指定を拒否する.

    綴り誤りをそのまま渡すと launch 側で黙って無視され、指定したつもりの設定で走る。
    """
    import pytest
    from icart_bringup.session_core import validate_launch_arguments

    with pytest.raises(ValueError, match='key:=value'):
        validate_launch_arguments(['record_bag=false'])
    with pytest.raises(ValueError, match='environment'):
        validate_launch_arguments(['environment:=simulation'])
