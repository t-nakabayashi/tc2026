"""business_mode モジュールの単体テスト。"""

from robot_console.core.business_mode import (
    DRIVE_MODES,
    ENVIRONMENTS,
    LAUNCH_PRESETS,
    LaunchPlan,
    LaunchPresetEntry,
    get_preset,
)


def test_all_environment_drive_mode_combinations_have_presets():
    for environment in ENVIRONMENTS:
        for drive_mode in DRIVE_MODES:
            assert (environment, drive_mode) in LAUNCH_PRESETS


def test_real_robot_autonomous_preset_matches_architecture_doc_group_order():
    preset = get_preset('実機', '自律走行')
    assert [entry.profile_id for entry in preset] == [
        'rtk_gps_um982',
        'ypspur_ros2',
        'drive_mode_manager',
        'route_planner',
        'route_manager',
        'geo_pose_converter',
        'route_follower',
        'obstacle_monitor',
        'robot_navigator',
        'road_blockage_detector',
        'traffic_signal_recognizer',
    ]


def test_desktop_check_autonomous_preset_uses_simulator_alternates():
    preset = get_preset('机上確認', '自律走行')
    simulator_profiles = {entry.profile_id for entry in preset if entry.use_simulator_alternate}
    assert simulator_profiles == {
        'robot_navigator',
        'obstacle_monitor',
        'road_blockage_detector',
        'traffic_signal_recognizer',
    }
    assert 'ypspur_ros2' not in {entry.profile_id for entry in preset}
    assert 'rtk_gps_um982' not in {entry.profile_id for entry in preset}
    assert 'obstacle_route_sim' not in {entry.profile_id for entry in preset}


def test_desktop_check_presets_use_ps3_joy_sim():
    for drive_mode in DRIVE_MODES:
        preset = get_preset('机上確認', drive_mode)
        drive_mode_entry = next(
            entry for entry in preset if entry.profile_id == 'drive_mode_manager'
        )
        assert drive_mode_entry.overrides.get('joy_input') == 'ps3_joy_sim'


def test_get_preset_returns_empty_list_for_unknown_combination():
    assert get_preset('unknown', 'unknown') == []


def test_launch_plan_add_is_idempotent_and_preserves_order():
    plan = LaunchPlan()
    plan.add('route_manager')
    plan.add('route_follower')
    plan.add('route_manager')

    assert plan.ordered_profile_ids == ['route_manager', 'route_follower']
    assert plan.contains('route_manager') is True
    assert plan.contains('robot_navigator') is False


def test_launch_plan_remove_does_not_raise_for_missing_profile():
    plan = LaunchPlan()
    plan.add('route_manager')
    plan.remove('unknown')
    assert plan.ordered_profile_ids == ['route_manager']


def test_launch_plan_move_up_and_down():
    plan = LaunchPlan(['a', 'b', 'c'])

    plan.move_up('b')
    assert plan.ordered_profile_ids == ['b', 'a', 'c']

    plan.move_down('b')
    assert plan.ordered_profile_ids == ['a', 'b', 'c']


def test_launch_plan_move_up_at_head_is_noop():
    plan = LaunchPlan(['a', 'b'])
    plan.move_up('a')
    assert plan.ordered_profile_ids == ['a', 'b']


def test_launch_plan_move_down_at_tail_is_noop():
    plan = LaunchPlan(['a', 'b'])
    plan.move_down('b')
    assert plan.ordered_profile_ids == ['a', 'b']


def test_launch_plan_apply_preset_replaces_existing_plan():
    plan = LaunchPlan(['old_profile'])
    plan.apply_preset([LaunchPresetEntry('a'), LaunchPresetEntry('b')])
    assert plan.ordered_profile_ids == ['a', 'b']


def test_launch_plan_clear():
    plan = LaunchPlan(['a', 'b'])
    plan.clear()
    assert plan.ordered_profile_ids == []
