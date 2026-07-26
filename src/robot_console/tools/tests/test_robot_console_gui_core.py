"""GuiCore の単体テスト。"""

import math
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

if 'ament_index_python' not in sys.modules:
    mock_root = types.ModuleType('ament_index_python')
    packages_mod = types.ModuleType('ament_index_python.packages')

    def _get_package_share_directory(_name: str) -> str:
        raise RuntimeError('ament_index_python is unavailable in tests')

    packages_mod.get_package_share_directory = _get_package_share_directory
    mock_root.packages = packages_mod
    sys.modules['ament_index_python'] = mock_root
    sys.modules['ament_index_python.packages'] = packages_mod


def _install_geometry_stubs() -> None:
    geometry_msgs = types.ModuleType('geometry_msgs')
    msg_module = types.ModuleType('geometry_msgs.msg')

    class PoseStamped:  # pragma: no cover - スタブのみ
        def __init__(self) -> None:
            class _Pose:
                def __init__(self) -> None:
                    class _Point:
                        def __init__(self) -> None:
                            self.x = 0.0
                            self.y = 0.0
                            self.z = 0.0

                    self.position = _Point()

            self.pose = _Pose()

    class PoseWithCovarianceStamped:  # pragma: no cover - スタブのみ
        def __init__(self) -> None:
            class _PoseWithCovariance:
                def __init__(self) -> None:
                    class _Pose:
                        def __init__(self) -> None:
                            class _Point:
                                def __init__(self) -> None:
                                    self.x = 0.0
                                    self.y = 0.0
                                    self.z = 0.0

                            self.position = _Point()

                    self.pose = _Pose()

            self.pose = _PoseWithCovariance()

    msg_module.PoseStamped = PoseStamped
    msg_module.PoseWithCovarianceStamped = PoseWithCovarianceStamped
    geometry_msgs.msg = msg_module
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = msg_module


def _install_sensor_stubs() -> None:
    sensor_msgs = types.ModuleType('sensor_msgs')
    msg_module = types.ModuleType('sensor_msgs.msg')

    class Image:  # pragma: no cover - スタブのみ
        height = 0
        width = 0
        encoding = 'rgb8'
        data = b''

    msg_module.Image = Image
    sensor_msgs.msg = msg_module
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = msg_module


if 'geometry_msgs' not in sys.modules:
    _install_geometry_stubs()

if 'sensor_msgs' not in sys.modules:
    _install_sensor_stubs()

import geometry_msgs.msg

try:
    import numpy  # noqa: F401
except ImportError:
    numpy_stub = types.ModuleType('numpy')

    def _frombuffer(data, dtype):  # pragma: no cover - スタブのみ
        return []

    numpy_stub.frombuffer = _frombuffer
    sys.modules['numpy'] = numpy_stub

if 'PIL' not in sys.modules:
    pil_module = types.ModuleType('PIL')
    image_module = types.ModuleType('PIL.Image')
    imagetk_module = types.ModuleType('PIL.ImageTk')

    class _DummyImage:  # pragma: no cover - スタブのみ
        mode = 'RGB'
        width = 1
        height = 1
        info = {}

        def resize(self, size, _filter=None):
            self.width, self.height = size
            return self

        def copy(self):
            return self

        def paste(self, _other, box=None, mask=None):
            return None

        def convert(self, _mode):
            return self

        def alpha_composite(self, _other):
            return None

    def _fromarray(_array, mode=None):
        return _DummyImage()

    def _new(mode, size, color):
        return _DummyImage()

    image_module.Image = _DummyImage
    image_module.NEAREST = 0
    image_module.fromarray = _fromarray
    image_module.new = _new
    imagetk_module.PhotoImage = _DummyImage
    pil_module.Image = image_module
    pil_module.ImageTk = imagetk_module
    sys.modules['PIL'] = pil_module
    sys.modules['PIL.Image'] = image_module
    sys.modules['PIL.ImageTk'] = imagetk_module

from robot_console.gui_core import GuiCore, default_launch_profiles
from robot_console.ui_main import UiMain, compute_route_progress
from robot_console.utils import ConsoleLogBuffer, GuiCommandType, FollowerStateView, RouteStateView

def _make_follower_state(**overrides):
    """GuiCore.update_follower_state へ渡すテスト用メッセージを生成する。"""

    defaults = {
        'state': 'RUNNING',
        'active_waypoint_index': 0,
        'active_waypoint_label': '',
        'front_blocked': False,
        'left_offset_m': 0.0,
        'right_offset_m': 0.0,
        'last_stagnation_reason': '',
        'avoidance_attempt_count': 0,
        'active_target_distance_m': 0.0,
        'segment_length_m': 0.0,
        'signal_stop_active': False,
        'line_stop_active': False,
    }
    defaults.update(overrides)
    return types.SimpleNamespace(**defaults)


def _make_route_state(**overrides):
    """GuiCore.update_route_state へ渡すテスト用メッセージを生成する。"""

    defaults = {
        'route_version': 1,
        'total_waypoints': 10,
        'current_index': 0,
        'status': 2,
        'message': '',
        'current_label': 'WP00',
    }
    defaults.update(overrides)
    return types.SimpleNamespace(**defaults)


def test_console_log_buffer_capacity() -> None:
    buffer = ConsoleLogBuffer(capacity=3)
    buffer.append('line1')
    buffer.append('line2')
    buffer.append('line3')
    buffer.append('line4')
    snapshot = buffer.snapshot()
    assert snapshot == ['line2', 'line3', 'line4']


def test_command_queue_order() -> None:
    core = GuiCore(launch_profiles=[])
    core.request_manual_start(True)
    core.request_sig_recog(True)
    core.start_obstacle_override(True, 2.0, 0.1, -0.1)

    commands = [core.get_next_command() for _ in range(3)]
    types = [cmd.command_type for cmd in commands if cmd]
    assert types == [
        GuiCommandType.MANUAL_START,
        GuiCommandType.SIG_RECOG,
        GuiCommandType.OBSTACLE_HINT_OVERRIDE,
    ]


def test_snapshot_is_copy() -> None:
    core = GuiCore(launch_profiles=[])
    snapshot = core.snapshot()
    snapshot.route_state.manager_state = 'modified'
    snapshot.route_state.route_status = 'modified'
    new_snapshot = core.snapshot()
    assert new_snapshot.route_state.manager_state != 'modified'
    assert new_snapshot.route_state.route_status != 'modified'


def test_manager_status_does_not_override_event() -> None:
    core = GuiCore(launch_profiles=[])
    core.update_route_state(_make_route_state(message='route_ready', current_index=1))
    core.update_manager_status(
        types.SimpleNamespace(state='running', decision='shift', last_cause='', route_version=1)
    )
    core.update_manager_status(
        types.SimpleNamespace(state='running', decision='none', last_cause='', route_version=1)
    )
    snapshot = core.snapshot()
    assert snapshot.route_state.last_replan_reason == 'route_ready'
    assert snapshot.route_state.manager_decision == ''
    assert snapshot.route_state.manager_cause == ''


def test_manager_status_records_cause() -> None:
    core = GuiCore(launch_profiles=[])
    core.update_manager_status(
        types.SimpleNamespace(state='holding', decision='failed', last_cause='front_blocked', route_version=1)
    )
    snapshot = core.snapshot()
    assert snapshot.route_state.manager_cause == 'front_blocked'


def test_route_state_message_retained_on_empty_update() -> None:
    core = GuiCore(launch_profiles=[])
    core.update_route_state(_make_route_state(message='shift_right(0.5m)', current_index=2))
    core.update_route_state(_make_route_state(message='', current_index=3))
    snapshot = core.snapshot()
    assert snapshot.route_state.last_replan_reason == 'shift_right(0.5m)'


def test_compute_distance_with_pose_with_covariance() -> None:
    core = GuiCore(launch_profiles=[])
    target = geometry_msgs.msg.PoseStamped()
    target.pose.position.x = 2.0  # type: ignore[attr-defined]
    pose_cov = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose_cov.pose.pose.position.x = 1.0  # type: ignore[attr-defined]
    distance = core._compute_distance(target, pose_cov)
    assert math.isclose(distance, 1.0)


def test_target_distance_falls_back_to_follower_state_value() -> None:
    core = GuiCore(launch_profiles=[])
    follower_msg = _make_follower_state(active_target_distance_m=7.5)
    core.update_follower_state(follower_msg)
    snapshot = core.snapshot()
    assert math.isclose(snapshot.target_distance.current_distance_m, 7.5)


def test_target_distance_prefers_pose_when_available() -> None:
    core = GuiCore(launch_profiles=[])
    target = geometry_msgs.msg.PoseStamped()
    target.pose.position.x = 10.0  # type: ignore[attr-defined]
    core.update_active_target(target)
    pose_cov = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose_cov.pose.pose.position.x = 4.0  # type: ignore[attr-defined]
    core.update_pose_enu(pose_cov)
    follower_msg = _make_follower_state(active_target_distance_m=123.0)
    core.update_follower_state(follower_msg)
    snapshot = core.snapshot()
    assert math.isclose(snapshot.target_distance.current_distance_m, 6.0)


def test_follower_finished_marks_route_and_manager_completed() -> None:
    core = GuiCore(launch_profiles=[])
    core.update_route_state(_make_route_state(status=2))
    core.update_manager_status(
        types.SimpleNamespace(state='running', decision='none', last_cause='', route_version=1)
    )
    follower_msg = _make_follower_state(state='FINISHED')
    core.update_follower_state(follower_msg)
    snapshot = core.snapshot()
    assert snapshot.route_state.route_status == 'completed'
    assert snapshot.route_state.manager_state == 'finished'


def test_compute_route_progress_uses_route_index() -> None:
    """ルート進捗計算が RouteState のインデックスを基準にすることを確認する。"""

    route_state = RouteStateView(total_waypoints=5, current_index=2)
    follower_state = FollowerStateView(active_waypoint_index=-1)
    ratio, display_index, total = compute_route_progress(route_state, follower_state)
    assert math.isclose(ratio, 0.6)
    assert display_index == 3  # RouteStateのインデックスを1始まりで表示
    assert total == 5


def test_compute_route_progress_handles_zero_total() -> None:
    """経路数が0の場合でも例外なく0表記を返すことを確認する。"""

    route_state = RouteStateView(total_waypoints=0, current_index=3)
    follower_state = FollowerStateView(active_waypoint_index=1)
    ratio, display_index, total = compute_route_progress(route_state, follower_state)
    assert math.isclose(ratio, 0.0)
    assert display_index == 0
    assert total == 0


def test_ui_main_source_aliases_target_label_to_label() -> None:
    ui_path = Path(__file__).resolve().parents[2] / 'robot_console' / 'ui_main.py'
    source = ui_path.read_text(encoding='utf-8')
    assert "'target_label': follower_label" in source
    assert "self._follower_vars['target_label']" in source


def test_ui_main_waiting_stop_appends_reason_suffix() -> None:
    ui_path = Path(__file__).resolve().parents[2] / 'robot_console' / 'ui_main.py'
    source = ui_path.read_text(encoding='utf-8')
    assert "(信号)" in source
    assert "(停止線)" in source


def test_ui_main_source_defines_progress_percent_var() -> None:
    ui_path = Path(__file__).resolve().parents[2] / 'robot_console' / 'ui_main.py'
    source = ui_path.read_text(encoding='utf-8')
    assert "'progress_percent': tk.DoubleVar" not in source
    assert "'progress_percent': tk.StringVar" in source
    assert "self._target_vars['progress_percent']" in source


def test_ui_main_route_vars_use_attribute_access() -> None:
    """Routeカード変数がデータクラス属性アクセスで統一されていることを検証する。"""

    ui_path = Path(__file__).resolve().parents[2] / 'robot_console' / 'ui_main.py'
    source = ui_path.read_text(encoding='utf-8')
    assert '._route_state_vars.' in source
    assert '._route_state_vars[' not in source


def test_ui_main_runtime_state_initializer_sets_defaults() -> None:
    """UiMainの内部状態初期化ヘルパーが安全な既定値を設定することを確認する。"""

    ui_main = UiMain.__new__(UiMain)
    UiMain._initialize_runtime_state(ui_main)
    assert ui_main._closing is False
    assert ui_main._shutdown_pending is False
    assert ui_main._update_job is None
    assert ui_main._shutdown_check_job is None
    assert ui_main._latest_snapshot is None
    assert ui_main._line_stop_active_since is None
    assert ui_main._last_line_stop_state is False


def test_default_launch_profiles_use_perception_packages() -> None:
    """認識系カードの起動先を確認する。"""

    profiles = {profile.profile_id: profile for profile in default_launch_profiles()}

    assert 'yolo_detector' not in profiles
    road = profiles['road_blockage_detector']
    assert road.package == 'road_blockage_detector'
    assert road.launch_file == 'road_blockage_perception.launch.py'
    assert road.alternate_launch_file == 'road_blockage_perception_yolo.launch.py'
    assert road.param_argument == 'detector_param_file'
    assert road.simulator_package == 'yolo_detector'

    signal = profiles['traffic_signal_recognizer']
    assert signal.package == 'traffic_signal_recognizer'
    assert signal.launch_file == 'traffic_signal_perception.launch.py'
    assert signal.alternate_launch_file is None
    assert signal.param_argument == 'recognizer_param_file'
    assert signal.simulator_package == 'yolo_detector'
