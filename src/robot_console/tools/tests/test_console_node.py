"""ros/console_node.py の最小テスト。

`RobotConsoleNode` はROS 2 topic購読のみを持つ薄いラッパーであり、実際の
購読・メッセージ受信を伴う動作確認は `ros2-local-run` スキルに従いノード単体
起動で行う（本テストではimportとNode生成のみを確認する）。ROS 2環境が無い
（`rclpy` をimportできない）環境では自動的にskipする。
"""

from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from robot_console.core.console_core import ConsoleCore  # noqa: E402
from robot_console.core.launch_profile import LaunchProfileStore  # noqa: E402
from robot_console.ros.console_node import RobotConsoleNode, start_ros_thread  # noqa: E402

REPO_PROFILE_PATH = Path(__file__).resolve().parents[2] / 'config' / 'node_launch_profiles.yaml'


def _make_core() -> ConsoleCore:
    return ConsoleCore(profile_store=LaunchProfileStore(REPO_PROFILE_PATH))


def test_robot_console_node_is_a_rclpy_node():
    if not rclpy.ok():
        rclpy.init()
    try:
        node = RobotConsoleNode(_make_core(), node_name='test_robot_console_node')
        assert node.get_name() == 'test_robot_console_node'
    finally:
        node.destroy_node()


def test_start_ros_thread_returns_running_handle_and_stops_cleanly():
    handle = start_ros_thread(_make_core(), node_name='test_robot_console_node_thread')

    assert handle.thread.is_alive()

    handle.stop()

    assert not handle.thread.is_alive()


def test_active_route_subscription_uses_transient_local_durability():
    """/active_route はroute_managerがTransient Local（ラッチ配信）で送るため、

    購読側が既定のVOLATILEのままだと、route_manager の publish 完了より購読の
    確立が遅れた場合に初回のRouteを二度と受け取れなくなる（durability replayは
    購読側もTRANSIENT_LOCALでなければ行われないため）。起動順に依存せず必ず
    受け取れることを、実際に生成した購読のQoSから確認する。
    """
    from rclpy.qos import DurabilityPolicy

    if not rclpy.ok():
        rclpy.init()
    node = RobotConsoleNode(_make_core(), node_name='test_active_route_qos_node')
    try:
        infos = node.get_subscriptions_info_by_topic('active_route')
        assert len(infos) == 1
        assert infos[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    finally:
        node.destroy_node()


def test_obstacle_hint_subscription_uses_best_effort_reliability():
    """obstacle_monitorは obstacle_avoidance_hint を BEST_EFFORT で配信するため、

    購読側が既定のRELIABLEのままだとQoS非互換で接続が成立せず、hintを一切
    受信できない。BEST_EFFORT購読はRELIABLE配信元とも互換であることを利用し、
    購読側を配信側に合わせていることを確認する。
    """
    from rclpy.qos import ReliabilityPolicy

    if not rclpy.ok():
        rclpy.init()
    node = RobotConsoleNode(_make_core(), node_name='test_obstacle_hint_qos_node')
    try:
        infos = node.get_subscriptions_info_by_topic('obstacle_avoidance_hint')
        assert len(infos) == 1
        assert infos[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    finally:
        node.destroy_node()


def test_image_subscriptions_use_best_effort_reliability():
    """road_blockage_detectorのdecision_imageはBEST_EFFORT配信であり、

    RELIABLE購読ではQoS非互換で画像が届かない。画像購読は取りこぼしを許容できる
    ため一律BEST_EFFORTとしていることを確認する。
    """
    from rclpy.qos import ReliabilityPolicy

    if not rclpy.ok():
        rclpy.init()
    node = RobotConsoleNode(_make_core(), node_name='test_image_qos_node')
    try:
        for topic in (
            'sensor_viewer',
            'perception/road_blockage/decision_image',
            'perception/traffic_signal/decision_image',
        ):
            infos = node.get_subscriptions_info_by_topic(topic)
            assert len(infos) == 1, topic
            assert infos[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT, topic
    finally:
        node.destroy_node()


def test_drive_and_manual_start_topics_are_subscribed():
    """Drive/CmdVelカードと運行フェーズ判定に必要なtopicを購読していることを確認する。"""

    if not rclpy.ok():
        rclpy.init()
    node = RobotConsoleNode(_make_core(), node_name='test_drive_subscription_node')
    try:
        for topic in ('drive_mode_status', 'cmd_vel', 'cmd_vel/autonomous', 'odom', 'manual_start'):
            assert node.get_subscriptions_info_by_topic(topic), topic
    finally:
        node.destroy_node()
