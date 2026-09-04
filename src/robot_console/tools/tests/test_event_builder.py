"""core/event_builder.py（Eventカードのイベント生成）の単体テスト。

screen_function_design.md 6.7節が定義する表示対象と優先順位を確認する。
"""

from datetime import datetime, timezone

from robot_console.core.event_builder import build_event_banners
from robot_console.core.event_priority import sort_by_priority
from robot_console.core.launch_profile import LaunchProfileState
from robot_console.core.snapshot_model import (
    FollowerView,
    ManualControlsView,
    ObstacleStateView,
    RouteView,
)
from robot_console.utils import NodeLaunchStatus

NOW = datetime(2026, 9, 4, 12, 0, 0, tzinfo=timezone.utc)


def _build(**overrides):
    kwargs = {
        'launch_states': {},
        'route': RouteView(),
        'follower': FollowerView(),
        'obstacle': ObstacleStateView(),
        'manual_controls': ManualControlsView(),
        'operation_phase': '走行中',
        'lost_topics': None,
        'now': NOW,
    }
    kwargs.update(overrides)
    return build_event_banners(**kwargs)


def _types(banners):
    return [b.event_type for b in banners]


def test_no_events_in_steady_driving_state():
    assert _build() == []


def test_profile_error_is_reported_with_detail():
    states = {
        'route_follower': LaunchProfileState(
            profile_id='route_follower',
            status=NodeLaunchStatus.ERROR,
            error_message='process has died\nsecond line',
        )
    }

    banners = _build(launch_states=states)

    assert _types(banners) == ['profile_error']
    assert banners[0].severity == 'error'
    assert 'route_follower ERROR' in banners[0].message
    assert 'process has died' in banners[0].message
    assert 'second line' not in banners[0].message  # 先頭行のみ


def test_topic_lost_is_reported_with_elapsed_seconds():
    banners = _build(lost_topics={'/route_state': 12.34})

    assert _types(banners) == ['topic_lost']
    assert '/route_state LOST 12.3s' in banners[0].message


def test_road_blocked_reports_input_source():
    controls = ManualControlsView(road_blocked_value=True, road_blocked_source='external')

    banners = _build(manual_controls=controls)

    assert _types(banners) == ['road_blocked']
    assert 'ROAD BLOCKED external' in banners[0].message


def test_front_blocked_uses_follower_values_when_follower_detects():
    follower = FollowerView(
        state='RUNNING', front_blocked=True, front_clearance_m=0.8,
        left_offset_m=0.3, right_offset_m=-0.1,
    )

    banners = _build(follower=follower)

    assert _types(banners) == ['front_blocked']
    assert 'clearance=0.8m' in banners[0].message
    assert banners[0].source == 'follower'


def test_front_blocked_falls_back_to_obstacle_hint():
    obstacle = ObstacleStateView(front_blocked=True, front_clearance_m=1.2)

    banners = _build(obstacle=obstacle)

    assert _types(banners) == ['front_blocked']
    assert banners[0].source == 'obstacle_hint'


def test_follower_waiting_state_is_reported_as_signal_stop():
    follower = FollowerView(state='WAITING_STOP', active_waypoint_label='A-24')

    banners = _build(follower=follower)

    assert _types(banners) == ['signal_stop']
    assert 'WAITING_STOP A-24' in banners[0].message


def test_manual_start_pending_is_reported_only_while_ready():
    banners = _build(operation_phase='走行準備完了')

    assert _types(banners) == ['manual_start_pending']

    sent = ManualControlsView(manual_start_value=True)
    assert _build(operation_phase='走行準備完了', manual_controls=sent) == []


def test_signal_go_is_reported_with_input_source():
    controls = ManualControlsView(sig_recog_value=1, input_source='external')

    banners = _build(manual_controls=controls)

    assert _types(banners) == ['signal_go']
    assert 'SIGNAL GO external' in banners[0].message


def test_route_update_is_reported_with_version_and_reason():
    route = RouteView(route_version=5, last_decision='replan_first', last_replan_reason='blocked')

    banners = _build(route=route)

    assert _types(banners) == ['route_update']
    assert 'ROUTE REPLAN_FIRST v5 blocked' in banners[0].message


def test_route_decision_none_does_not_create_event():
    assert _build(route=RouteView(last_decision='none')) == []


def test_events_are_ordered_by_documented_priority():
    """複数同時発生時、6.7節の優先順位で並ぶことを確認する。"""

    states = {
        'route_manager': LaunchProfileState(
            profile_id='route_manager', status=NodeLaunchStatus.ERROR
        )
    }
    controls = ManualControlsView(road_blocked_value=True, sig_recog_value=1)
    follower = FollowerView(state='WAITING_STOP', front_blocked=True)

    banners = sort_by_priority(
        _build(
            launch_states=states,
            manual_controls=controls,
            follower=follower,
            lost_topics={'/cmd_vel': 5.0},
            route=RouteView(last_decision='update'),
        )
    )

    assert _types(banners) == [
        'profile_error',
        'topic_lost',
        'road_blocked',
        'front_blocked',
        'signal_stop',
        'signal_go',
        'route_update',
    ]
