"""launch/robot_console.launch.py の単体テスト。

`robot_console.launch.py` は購読トピックごとに launch引数（`remappings`）を
公開し、operatorが実際の配線（例: ypspur_ros2/robot_navigatorのodom_topic設定）に
合わせて上書きできるようにする方針である
（`robot_console_gui_architecture_design.md` 7.1節）。結合動作確認で
`drive_mode_status` / `cmd_vel/autonomous` / `odom` の購読を追加した際に
launch引数の追加を漏らし、`ros2 launch` 経由では上書き手段が無い状態になって
いたため、`_TOPIC_CONFIGS` に列挙されたrelative名が `RobotConsoleNode` の
実際の購読トピック名と一致することを回帰確認する。
"""

import importlib.util
import re
import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from robot_console.core.console_core import ConsoleCore  # noqa: E402
from robot_console.core.launch_profile import LaunchProfileStore  # noqa: E402
from robot_console.ros.console_node import RobotConsoleNode  # noqa: E402

LAUNCH_FILE = Path(__file__).resolve().parents[2] / 'launch' / 'robot_console.launch.py'
REPO_PROFILE_PATH = Path(__file__).resolve().parents[2] / 'config' / 'node_launch_profiles.yaml'

# `RobotConsoleNode.__init__` の `self.create_subscription(<Type>, '<relative_topic>', ...)` /
# `self.create_publisher(<Type>, '<relative_topic>', ...)` を抽出する。QoSプロファイル変数
# （`_QOS_ACTIVE_ROUTE` 等）を渡す購読も対象に含める。
_TOPIC_USAGE_PATTERN = re.compile(
    r"self\.create_(?:subscription|publisher)\(\s*\w+,\s*['\"]([^'\"]+)['\"]"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('robot_console_launch_under_test', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _make_core() -> ConsoleCore:
    return ConsoleCore(profile_store=LaunchProfileStore(REPO_PROFILE_PATH))


def _relative_topics_used_by_node() -> set:
    source = Path(__file__).resolve().parents[2] / 'robot_console' / 'ros' / 'console_node.py'
    text = source.read_text()
    topics = set(_TOPIC_USAGE_PATTERN.findall(text))
    # 絶対パス指定（先頭 '/'）はremap対象外のため除外する。
    return {topic for topic in topics if not topic.startswith('/')}


def test_topic_configs_have_unique_launch_arguments():
    module = _load_launch_module()

    arg_names = [entry[0] for entry in module._TOPIC_CONFIGS]

    assert len(arg_names) == len(set(arg_names))


def test_topic_configs_cover_drive_and_odom_subscriptions_added_for_operation_phase():
    """運行フェーズ判定・Drive/CmdVelカードのために追加した購読が

    launchレベルでも上書きできることを確認する（結合確認で `odom_topic` の
    remap手段が無いことが判明したため、再発防止のため固定する）。
    """

    module = _load_launch_module()
    relative_names = {entry[1] for entry in module._TOPIC_CONFIGS}

    for topic in ('drive_mode_status', 'cmd_vel/autonomous', 'odom'):
        assert topic in relative_names, f'{topic} の launch remap引数が未定義'


def test_topic_configs_relative_names_match_actual_subscriptions():
    """`_TOPIC_CONFIGS` の相対名が `RobotConsoleNode` の実際の購読名とズレていないか確認する。

    どちらかだけを変更して同期を忘れると、launch経由の上書きが無効化される
    （remap先の内部名が存在しないため）。
    """

    module = _load_launch_module()
    configured = {entry[1] for entry in module._TOPIC_CONFIGS}
    actual = _relative_topics_used_by_node()

    unmatched = configured - actual
    assert not unmatched, f'launchで宣言されているが実際には購読していない相対名: {unmatched}'


def test_generate_launch_description_builds_without_error():
    module = _load_launch_module()

    description = module.generate_launch_description()

    assert description is not None
