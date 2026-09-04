"""core/operation_phase.py（運行フェーズ判定）の単体テスト。

screen_function_design.md 6.3節の運行フェーズ列挙値ごとに、判定条件と
優先順位（重い状態が優先されること）を確認する。
"""

from typing import Dict

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.launch_profile import LaunchProfileState
from robot_console.core.operation_phase import (
    PHASE_DRIVING,
    PHASE_ERROR,
    PHASE_NOT_STARTED,
    PHASE_PAUSED,
    PHASE_READY,
    PHASE_SHUTTING_DOWN,
    PHASE_STARTING,
    build_operation_state,
    resolve_waypoint_labels,
)
from robot_console.core.snapshot_model import (
    DriveModeStateView,
    FollowerView,
    RouteView,
    RouteWaypointView,
)
from robot_console.utils import NodeLaunchStatus


def _launch_states(**statuses: NodeLaunchStatus) -> Dict[str, LaunchProfileState]:
    return {
        profile_id: LaunchProfileState(profile_id=profile_id, status=status)
        for profile_id, status in statuses.items()
    }


def _build(**overrides):
    kwargs = {
        'environment': '実機',
        'drive_mode_selection': '自律走行',
        'launch_states': {},
        'route': RouteView(),
        'follower': FollowerView(),
        'drive_mode': DriveModeStateView(),
        'manual_start': False,
        'route_freshness': FreshnessLevel.UNKNOWN,
        'follower_freshness': FreshnessLevel.UNKNOWN,
    }
    kwargs.update(overrides)
    return build_operation_state(**kwargs)


def test_phase_is_not_started_when_nothing_launched_and_no_topic_received():
    assert _build().phase == PHASE_NOT_STARTED


def test_phase_is_starting_while_profiles_are_starting_without_topics():
    state = _build(launch_states=_launch_states(route_manager=NodeLaunchStatus.STARTING))

    assert state.phase == PHASE_STARTING


def test_phase_is_starting_when_profiles_run_but_operation_topics_not_received_yet():
    state = _build(launch_states=_launch_states(route_manager=NodeLaunchStatus.RUNNING))

    assert state.phase == PHASE_STARTING


def test_phase_is_ready_when_operation_topics_received_and_manual_start_not_sent():
    state = _build(
        route=RouteView(state='idle', total_waypoints=3),
        follower=FollowerView(state='IDLE'),
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_READY


def test_phase_is_driving_after_manual_start_while_follower_is_running():
    state = _build(
        route=RouteView(state='running', total_waypoints=3),
        follower=FollowerView(state='RUNNING'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_DRIVING
    assert state.pause_reason == ''


def test_phase_is_ready_when_manual_start_sent_but_follower_still_idle():
    state = _build(
        route=RouteView(state='idle', total_waypoints=3),
        follower=FollowerView(state='IDLE'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_READY


def test_phase_is_paused_with_reason_while_follower_waits_at_stop_waypoint():
    state = _build(
        route=RouteView(state='running', total_waypoints=3),
        follower=FollowerView(state='WAITING_STOP'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_PAUSED
    assert '停止waypoint' in state.pause_reason


def test_phase_is_paused_while_route_manager_is_holding():
    state = _build(
        route=RouteView(state='holding', total_waypoints=3, last_replan_reason='front_blocked'),
        follower=FollowerView(state='RUNNING'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_PAUSED
    assert 'front_blocked' in state.pause_reason


def test_phase_is_paused_when_operator_takes_manual_control():
    state = _build(
        route=RouteView(state='running', total_waypoints=3),
        follower=FollowerView(state='RUNNING'),
        drive_mode=DriveModeStateView(mode='manual'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_PAUSED
    assert 'manual' in state.pause_reason


def test_phase_is_shutting_down_when_route_completed():
    state = _build(
        route=RouteView(state='completed', total_waypoints=3),
        follower=FollowerView(state='FINISHED'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_SHUTTING_DOWN


def test_phase_is_shutting_down_while_profiles_are_stopping():
    state = _build(
        launch_states=_launch_states(route_manager=NodeLaunchStatus.STOPPING),
        route=RouteView(state='running', total_waypoints=3),
        follower=FollowerView(state='RUNNING'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_SHUTTING_DOWN


def test_phase_is_error_when_a_profile_failed_even_while_driving():
    state = _build(
        launch_states=_launch_states(
            route_manager=NodeLaunchStatus.RUNNING, route_follower=NodeLaunchStatus.ERROR
        ),
        route=RouteView(state='running', total_waypoints=3),
        follower=FollowerView(state='RUNNING'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_ERROR


def test_phase_is_error_when_follower_reports_error():
    state = _build(
        follower=FollowerView(state='ERROR'),
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_ERROR


def test_topic_only_usage_reaches_driving_without_any_launched_profile():
    """HTML遠隔観測UI単独起動（profileを起動しない使い方）でも走行中を判定できる。"""

    state = _build(
        launch_states={},
        route=RouteView(state='running', total_waypoints=3),
        follower=FollowerView(state='RUNNING'),
        manual_start=True,
        route_freshness=FreshnessLevel.OK,
        follower_freshness=FreshnessLevel.OK,
    )

    assert state.phase == PHASE_DRIVING


def test_topic_loss_after_reception_does_not_fall_back_to_not_started():
    state = _build(
        route=RouteView(state='running', total_waypoints=3),
        follower=FollowerView(state='RUNNING'),
        manual_start=True,
        route_freshness=FreshnessLevel.LOST,
        follower_freshness=FreshnessLevel.LOST,
    )

    assert state.phase == PHASE_DRIVING


def test_drive_mode_display_prefers_drive_mode_status_over_selection():
    state = _build(
        drive_mode_selection='自律走行',
        drive_mode=DriveModeStateView(mode='manual'),
    )

    assert state.drive_mode == '手動'


def test_drive_mode_display_falls_back_to_business_mode_selection():
    state = _build(drive_mode_selection='自律走行')

    assert state.drive_mode == '自律'
    assert state.environment == '実機'


def test_route_progress_comes_from_route_state():
    state = _build(route=RouteView(total_waypoints=4, current_index=1, progress_ratio=0.25))

    assert state.route_progress == 0.25


def test_waypoint_labels_use_route_label_and_next_waypoint_label():
    route = RouteView(
        current_index=1,
        current_label='A-11',
        total_waypoints=3,
        waypoints=[
            RouteWaypointView(index=0, label='A-10'),
            RouteWaypointView(index=1, label='A-11'),
            RouteWaypointView(index=2, label='A-12'),
        ],
    )

    assert resolve_waypoint_labels(route, FollowerView()) == ('A-11', 'A-12')


def test_waypoint_labels_fall_back_to_index_notation_without_labels():
    route = RouteView(current_index=1, total_waypoints=5)

    assert resolve_waypoint_labels(route, FollowerView()) == ('#1', '#2')


def test_waypoint_labels_show_goal_at_last_waypoint():
    route = RouteView(current_index=2, current_label='A-12', total_waypoints=3)

    assert resolve_waypoint_labels(route, FollowerView()) == ('A-12', 'goal')


def test_waypoint_labels_are_undetermined_before_any_route_reception():
    assert resolve_waypoint_labels(RouteView(), FollowerView()) == ('-', '-')
