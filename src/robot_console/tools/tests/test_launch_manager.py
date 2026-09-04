"""LaunchManager の単体テスト。

`subprocess.Popen` をフェイクへ差し替え、実際の `ros2 launch` は起動せずに
起動コマンドの組み立てとsimulator引数変換のみを検証する。
"""

import queue
import subprocess
import threading
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from robot_console.core.launch_manager import LaunchManager
from robot_console.core.launch_profile import LaunchProfile
from robot_console.utils import NodeLaunchStatus


class _EmptyStream:
    """readline() が即座に空文字を返すダミーストリーム。"""

    def readline(self) -> str:
        return ''

    def close(self) -> None:
        return None


class _FakePopen:
    """subprocess.Popen の代替として起動引数と終了通知のみを扱う。"""

    def __init__(self, args, **_kwargs) -> None:
        self.args = args
        self.pid = 424242
        self.stdout = _EmptyStream()
        self.stderr = _EmptyStream()
        self._returncode = None
        self._stopped = threading.Event()

    def poll(self):
        return self._returncode

    def wait(self, timeout=None):
        if self._returncode is None:
            if not self._stopped.wait(timeout):
                raise subprocess.TimeoutExpired(self.args, timeout or 0)
        return self._returncode

    def send_signal(self, _sig) -> None:
        self._returncode = 0
        self._stopped.set()

    def terminate(self) -> None:
        self._returncode = 0
        self._stopped.set()

    def kill(self) -> None:
        self._returncode = -9
        self._stopped.set()


def _make_profile(**overrides) -> LaunchProfile:
    defaults = dict(
        profile_id='robot_navigator',
        category='drive_stack',
        display_name='Robot Navigator',
        package='robot_navigator',
        launch_file='robot_navigator.launch.py',
        param_argument='param_file',
        simulator_package='robot_navigator',
        simulator_launch_file='robot_simulator.launch.py',
        simulator_argument_map={'pose_enu_topic': 'pose_topic', 'odom_topic': 'odom_topic'},
        user_arguments=['cmd_vel_topic', 'odom_topic', 'pose_enu_topic'],
    )
    defaults.update(overrides)
    return LaunchProfile(**defaults)


def _make_manager(monkeypatch, recorded_popens):
    def _fake_popen(args, **kwargs):
        process = _FakePopen(args, **kwargs)
        recorded_popens.append(process)
        return process

    monkeypatch.setattr('robot_console.core.launch_manager.subprocess.Popen', _fake_popen)

    events: "queue.Queue" = queue.Queue()

    def _status_callback(profile_id, status, pid, error):
        events.put((profile_id, status, pid, error))

    def _log_callback(_profile_id, _line):
        return None

    manager = LaunchManager(_status_callback, _log_callback)
    return manager, events


def test_launch_builds_expected_command(monkeypatch):
    recorded = []
    manager, events = _make_manager(monkeypatch, recorded)
    profile = _make_profile()

    manager.launch(
        profile,
        param_path='params/custom.yaml',
        overrides={'cmd_vel_topic': '/cmd_vel/autonomous'},
    )

    assert len(recorded) == 1
    args = recorded[0].args
    assert args[:4] == ['ros2', 'launch', 'robot_navigator', 'robot_navigator.launch.py']
    # param_pathはパッケージ相対のフラグメントのため、起動直前にパッケージ共有
    # ディレクトリ基準の絶対パスへ解決される（相対のままだとサブプロセスの
    # 作業ディレクトリ次第で解決に失敗するため）。
    expected_param_path = str(
        Path(get_package_share_directory('robot_navigator')) / 'params/custom.yaml'
    )
    assert f'param_file:={expected_param_path}' in args
    assert 'cmd_vel_topic:=/cmd_vel/autonomous' in args

    status_id, status, _pid, _error = events.get(timeout=1.0)
    assert (status_id, status) == ('robot_navigator', NodeLaunchStatus.STARTING)
    status_id, status, pid, _error = events.get(timeout=1.0)
    assert (status_id, status, pid) == ('robot_navigator', NodeLaunchStatus.RUNNING, recorded[0].pid)

    manager.stop('robot_navigator')


def test_launch_uses_alternate_launch_file(monkeypatch):
    recorded = []
    manager, _events = _make_manager(monkeypatch, recorded)
    profile = _make_profile(
        profile_id='road_blockage_detector',
        package='road_blockage_detector',
        launch_file='road_blockage_perception.launch.py',
        alternate_launch_file='road_blockage_perception_yolo.launch.py',
        param_argument='detector_param_file',
        simulator_package=None,
        simulator_launch_file=None,
        simulator_argument_map={},
        user_arguments=[],
    )

    manager.launch(profile, use_alternate=True)

    assert recorded[0].args[3] == 'road_blockage_perception_yolo.launch.py'
    manager.stop('road_blockage_detector')


def test_simulator_only_forwards_mapped_overrides(monkeypatch):
    recorded = []
    manager, _events = _make_manager(monkeypatch, recorded)
    profile = _make_profile()

    manager.launch(
        profile,
        simulator_enabled=True,
        overrides={
            'cmd_vel_topic': '/cmd_vel/autonomous',
            'odom_topic': '/ypspur_ros/odom',
            'pose_enu_topic': '/localization/pose_enu',
        },
    )

    assert len(recorded) == 2
    sim_args = recorded[1].args
    assert sim_args[:4] == ['ros2', 'launch', 'robot_navigator', 'robot_simulator.launch.py']
    # cmd_vel_topic は simulator_argument_map に無いため転送されない
    assert not any(arg.startswith('cmd_vel_topic:=') for arg in sim_args)
    assert 'odom_topic:=/ypspur_ros/odom' in sim_args
    assert 'pose_topic:=/localization/pose_enu' in sim_args

    manager.stop('robot_navigator')


def test_simulator_not_launched_when_disabled(monkeypatch):
    recorded = []
    manager, _events = _make_manager(monkeypatch, recorded)
    profile = _make_profile()

    manager.launch(profile, simulator_enabled=False)

    assert len(recorded) == 1
    manager.stop('robot_navigator')


def test_stop_without_running_process_reports_stopped(monkeypatch):
    recorded = []
    manager, events = _make_manager(monkeypatch, recorded)

    manager.stop('unknown_profile')

    collected = []
    while True:
        try:
            collected.append(events.get_nowait())
        except queue.Empty:
            break
    assert ('unknown_profile', NodeLaunchStatus.STOPPED, None, None) in collected
    assert ('unknown_profile:sim', NodeLaunchStatus.STOPPED, None, None) in collected


def test_is_running_reflects_process_lifecycle(monkeypatch):
    recorded = []
    manager, _events = _make_manager(monkeypatch, recorded)
    profile = _make_profile()

    assert manager.is_running(profile.profile_id) is False
    manager.launch(profile)
    assert manager.is_running(profile.profile_id) is True
    manager.stop(profile.profile_id)
