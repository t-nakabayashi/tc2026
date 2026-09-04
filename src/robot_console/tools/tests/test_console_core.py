"""ConsoleCore（ROS非依存の状態集約Facade）の単体テスト。"""

from pathlib import Path
from types import SimpleNamespace

from robot_console.core.console_core import ConsoleCore
from robot_console.core.freshness import FreshnessLevel
from robot_console.core.launch_profile import LaunchProfileStore
from robot_console.core.snapshot_model import ConsoleSnapshot
from robot_console.utils import NodeLaunchStatus

REPO_PROFILE_PATH = Path(__file__).resolve().parents[2] / 'config' / 'node_launch_profiles.yaml'


def _make_core() -> ConsoleCore:
    return ConsoleCore(profile_store=LaunchProfileStore(REPO_PROFILE_PATH))


def test_build_snapshot_returns_console_snapshot_with_default_views():
    core = _make_core()

    snapshot = core.build_snapshot()

    assert isinstance(snapshot, ConsoleSnapshot)
    assert snapshot.operation_state.phase == '未起動'
    assert snapshot.route_state.waypoints == []


def test_build_snapshot_includes_health_for_all_profiles():
    core = _make_core()

    snapshot = core.build_snapshot()

    profile_ids = {item.profile_id for item in snapshot.health}
    assert profile_ids == {profile.profile_id for profile in core._profiles}
    assert len(snapshot.health) == 15
    assert all(item.status == 'STOPPED' for item in snapshot.health)


def test_launch_status_callback_updates_health_and_launch_profiles():
    core = _make_core()
    profile_id = core._profiles[0].profile_id

    core._on_launch_status(profile_id, NodeLaunchStatus.RUNNING, 4242, None)
    snapshot = core.build_snapshot()

    assert snapshot.launch_profiles[profile_id].status == NodeLaunchStatus.RUNNING
    assert snapshot.launch_profiles[profile_id].process_id == 4242
    health = next(item for item in snapshot.health if item.profile_id == profile_id)
    assert health.status == 'RUNNING'


def test_launch_status_callback_handles_simulator_suffix_separately():
    core = _make_core()
    profile_id = core._profiles[0].profile_id

    core._on_launch_status(f'{profile_id}:sim', NodeLaunchStatus.RUNNING, 555, None)
    snapshot = core.build_snapshot()

    state = snapshot.launch_profiles[profile_id]
    assert state.simulator_status == NodeLaunchStatus.RUNNING
    assert state.simulator_process_id == 555
    assert state.status == NodeLaunchStatus.STOPPED  # 本体側は未変更


def test_launch_status_callback_ignores_unknown_profile_id():
    core = _make_core()

    core._on_launch_status('unknown_profile', NodeLaunchStatus.RUNNING, 1, None)

    assert 'unknown_profile' not in core.build_snapshot().launch_profiles


def test_launch_log_callback_forwards_to_log_manager():
    core = _make_core()
    profile_id = core._profiles[0].profile_id

    core._on_launch_log(profile_id, '[INFO] hello\n')

    assert core.build_snapshot().logs[profile_id] == ['[INFO] hello\n']


def test_request_launch_ignores_unknown_profile_id():
    core = _make_core()

    core.request_launch('unknown_profile')  # 例外を送出しないことのみ確認


def test_request_launch_uses_state_updated_by_launch_settings_setters():
    core = _make_core()
    profile_id = 'route_planner'
    calls: list = []
    core.launch_manager.launch = lambda profile, **kwargs: calls.append((profile, kwargs))

    core.update_selected_param(profile_id, 'params/tsukuba.yaml')
    core.update_use_alternate_launch(profile_id, True)
    core.update_simulator_enabled(profile_id, True)

    core.request_launch(profile_id)

    assert len(calls) == 1
    profile, kwargs = calls[0]
    assert profile.profile_id == profile_id
    assert kwargs['param_path'] == 'params/tsukuba.yaml'
    assert kwargs['use_alternate'] is True
    assert kwargs['simulator_enabled'] is True


def test_request_launch_explicit_kwargs_override_state():
    core = _make_core()
    profile_id = 'route_planner'
    calls: list = []
    core.launch_manager.launch = lambda profile, **kwargs: calls.append((profile, kwargs))

    core.update_simulator_enabled(profile_id, True)
    core.request_launch(profile_id, simulator_enabled=False)

    assert calls[0][1]['simulator_enabled'] is False


def test_update_launch_override_reflects_in_request_launch_overrides():
    core = _make_core()
    profile_id = 'drive_mode_manager'
    calls: list = []
    core.launch_manager.launch = lambda profile, **kwargs: calls.append((profile, kwargs))

    core.update_launch_override(profile_id, 'joy_input', 'ps3_joy_sim')
    core.request_launch(profile_id)

    assert calls[0][1]['overrides'].get('joy_input') == 'ps3_joy_sim'


def _waypoint(index, *, has_geo_pose=False, latitude=None, longitude=None):
    geo_pose = SimpleNamespace(point=SimpleNamespace(latitude=latitude, longitude=longitude))
    return SimpleNamespace(index=index, has_geo_pose=has_geo_pose, geo_pose=geo_pose)


def test_update_route_state_and_manager_status_merge_into_same_route_view():
    core = _make_core()

    core.update_route_state(
        SimpleNamespace(status=2, route_version=1, current_index=2, total_waypoints=4)
    )
    core.update_manager_status(SimpleNamespace(decision='avoid', last_cause='obstacle_detected'))
    route = core.build_snapshot().route_state

    assert route.state == 'running'
    assert route.progress_ratio == 0.5
    assert route.last_decision == 'avoid'
    assert route.last_replan_reason == 'obstacle_detected'


def test_update_route_populates_waypoints():
    core = _make_core()

    core.update_route(
        SimpleNamespace(
            version=3,
            waypoints=[_waypoint(0, has_geo_pose=True, latitude=36.083, longitude=140.113)],
        )
    )
    route = core.build_snapshot().route_state

    assert route.route_version == 3
    assert route.waypoints[0].latitude == 36.083


def test_update_follower_state_reflects_in_snapshot():
    core = _make_core()

    core.update_follower_state(
        SimpleNamespace(
            state='following', active_waypoint_index=2, active_waypoint_label='A-2',
            last_stagnation_reason='', avoidance_attempt_count=0, front_blocked=False,
            front_clearance_m=5.0, left_offset_m=0.0, right_offset_m=0.0,
        )
    )

    assert core.build_snapshot().follower_state.active_waypoint_label == 'A-2'


def test_update_obstacle_hint_sets_freshness_ok_immediately_after_update():
    core = _make_core()

    core.update_obstacle_hint(
        SimpleNamespace(front_blocked=True, front_clearance_m=0.3, left_offset_m=0.0, right_offset_m=0.0)
    )
    obstacle = core.build_snapshot().obstacle_state

    assert obstacle.front_blocked is True
    assert obstacle.freshness == FreshnessLevel.OK


def _pose_enu_msg(x, y, z=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )


def _pose_llh_msg(latitude, longitude):
    return SimpleNamespace(
        pose=SimpleNamespace(
            point=SimpleNamespace(latitude=latitude, longitude=longitude, altitude=0.0, has_altitude=False),
            has_yaw_enu=False,
            yaw_enu_rad=0.0,
            child_frame_id='base_link',
        )
    )


def test_update_pose_enu_then_pose_llh_merge_into_single_localization_view():
    core = _make_core()

    core.update_pose_enu(_pose_enu_msg(1.0, 2.0))
    core.update_pose_llh(_pose_llh_msg(36.083, 140.113))
    localization = core.build_snapshot().localization_state

    assert localization.x_m == 1.0  # pose_enu由来の値が失われていない
    assert localization.y_m == 2.0
    assert localization.latitude == 36.083
    assert localization.longitude == 140.113
    assert localization.source == 'pose_llh'
    assert localization.freshness == FreshnessLevel.OK


def test_update_gps_status_reflects_in_snapshot_with_ok_freshness():
    core = _make_core()

    core.update_gps_status(
        SimpleNamespace(
            rtk_state=4, rtk_state_raw='RTK_FIX', num_satellites=19, hdop=0.7,
            correction_age_s=0.9, rtcm_bytes_received=1000, heading_deg=87.3,
            heading_stddev_deg=0.6, baseline_length_m=1.2, latitude=36.083,
            longitude=140.113, altitude=25.0,
        )
    )
    gps = core.build_snapshot().gps_state

    assert gps.rtk_state == 'RTK_FIX'
    assert gps.fix_freshness == FreshnessLevel.OK
    assert gps.heading_freshness == FreshnessLevel.OK


def test_update_active_target_computes_distance_from_current_localization():
    core = _make_core()
    core.update_pose_enu(_pose_enu_msg(0.0, 0.0))

    core.update_active_target(
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=0.0)))
    )
    target = core.build_snapshot().target_state

    assert target.distance_m == 5.0  # 3-4-5の直角三角形
    assert target.freshness == FreshnessLevel.OK


def test_update_active_target_without_localization_uses_zero_distance():
    core = _make_core()

    core.update_active_target(
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=0.0)))
    )
    target = core.build_snapshot().target_state

    assert target.distance_m == 0.0


def _image_msg(width=2, height=2, encoding='rgb8'):
    channels = 3
    return SimpleNamespace(
        width=width, height=height, encoding=encoding, data=bytes([255]) * (width * height * channels)
    )


def test_update_sensor_image_stores_image_and_metadata():
    core = _make_core()

    core.update_sensor_image('sensor_viewer', 'Sensor Viewer', '/sensor_viewer', _image_msg())
    snapshot = core.build_snapshot()

    assert core.image_store.get('sensor_viewer') is not None
    panel = next(p for p in snapshot.sensor_panels if p.panel_id == 'sensor_viewer')
    assert panel.title == 'Sensor Viewer'
    assert panel.width == 2
    assert panel.height == 2
    assert panel.freshness == FreshnessLevel.OK


def test_update_sensor_image_with_undecodable_message_keeps_metadata_without_image():
    core = _make_core()

    core.update_sensor_image('sensor_viewer', 'Sensor Viewer', '/sensor_viewer', _image_msg(width=0, height=0))
    snapshot = core.build_snapshot()

    assert core.image_store.get('sensor_viewer') is None
    panel = next(p for p in snapshot.sensor_panels if p.panel_id == 'sensor_viewer')
    assert panel.width == 0


def test_send_manual_start_updates_manual_controls_and_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(manual_start=lambda value: sent.append(value))

    core.send_manual_start(True)
    manual_controls = core.build_snapshot().manual_controls

    assert sent == [True]
    assert manual_controls.manual_start_value is True
    assert manual_controls.manual_start_last_sent_at is not None
    assert manual_controls.input_source == 'gui'


def test_send_manual_start_without_bound_publisher_only_updates_state():
    core = _make_core()

    core.send_manual_start(True)  # publisher未登録でも例外を送出しないことを確認

    assert core.build_snapshot().manual_controls.manual_start_value is True


def test_send_sig_recog_updates_manual_controls_and_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(sig_recog=lambda value: sent.append(value))

    core.send_sig_recog(2)

    assert sent == [2]
    assert core.build_snapshot().manual_controls.sig_recog_value == 2


def test_send_road_blocked_updates_manual_controls_and_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(road_blocked=lambda value: sent.append(value))

    core.send_road_blocked(True)
    manual_controls = core.build_snapshot().manual_controls

    assert sent == [True]
    assert manual_controls.road_blocked_value is True
    assert manual_controls.road_blocked_source == 'gui'


def test_send_obstacle_hint_override_calls_publisher_with_all_fields():
    core = _make_core()
    sent = []
    core.bind_publishers(obstacle_hint=lambda *args: sent.append(args))

    core.send_obstacle_hint_override(True, 1.5, 0.1, 0.2)
    manual_controls = core.build_snapshot().manual_controls

    assert sent == [(True, 1.5, 0.1, 0.2)]
    assert manual_controls.obstacle_hint_override_active is True


def test_send_obstacle_hint_stop_publishes_cleared_values():
    core = _make_core()
    sent = []
    core.bind_publishers(obstacle_hint=lambda *args: sent.append(args))

    core.send_obstacle_hint_stop()

    assert sent == [(False, 0.0, 0.0, 0.0)]
    assert core.build_snapshot().manual_controls.obstacle_hint_override_active is False


def test_send_frame_image_request_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(frame_image=lambda path: sent.append(path))

    core.send_frame_image_request('/tmp/frame.png')

    assert sent == ['/tmp/frame.png']


def _drive_mode_status_msg(mode=1, output_source=1, auto_resume_pending=False):
    return SimpleNamespace(
        mode=mode, output_source=output_source, auto_resume_pending=auto_resume_pending
    )


def _twist_msg(linear_x=0.0, angular_z=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=linear_x, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=angular_z),
    )


def _follower_state_msg(state='RUNNING', index=1, label='A-11'):
    return SimpleNamespace(
        state=state, active_waypoint_index=index, active_waypoint_label=label,
        last_stagnation_reason='', avoidance_attempt_count=0, front_blocked=False,
        front_clearance_m=5.0, left_offset_m=0.0, right_offset_m=0.0,
    )


def test_update_drive_mode_status_reflects_in_snapshot():
    core = _make_core()

    core.update_drive_mode_status(
        _drive_mode_status_msg(mode=2, output_source=2, auto_resume_pending=True)
    )
    drive = core.build_snapshot().drive_mode_state

    assert drive.mode == 'manual'
    assert drive.output_source == 'manual_cmd'
    assert drive.auto_resume_pending is True


def test_update_cmd_vel_sets_values_and_freshness():
    core = _make_core()

    core.update_cmd_vel(_twist_msg(0.5, 0.0))
    drive = core.build_snapshot().drive_mode_state

    assert drive.cmd_vel_linear_mps == 0.5
    assert drive.cmd_vel_freshness == FreshnessLevel.OK


def test_update_cmd_vel_autonomous_does_not_overwrite_mux_output():
    core = _make_core()

    core.update_cmd_vel(_twist_msg(0.5, 0.0))
    core.update_cmd_vel_autonomous(_twist_msg(0.8, 0.0))
    drive = core.build_snapshot().drive_mode_state

    assert drive.cmd_vel_linear_mps == 0.5
    assert drive.cmd_vel_autonomous_linear_mps == 0.8


def test_update_odom_records_topic_name_and_freshness():
    core = _make_core()

    core.update_odom(SimpleNamespace(), topic='/ypspur_ros/odom')
    drive = core.build_snapshot().drive_mode_state

    assert drive.odom_topic == '/ypspur_ros/odom'
    assert drive.odom_freshness == FreshnessLevel.OK


def test_update_manual_start_echo_confirms_gui_send():
    core = _make_core()
    core.send_manual_start(True)

    assert core.build_snapshot().manual_controls.send_result == 'waiting_echo'

    core.update_manual_start(SimpleNamespace(data=True))
    manual_controls = core.build_snapshot().manual_controls

    assert manual_controls.manual_start_value is True
    assert manual_controls.send_result == 'confirmed'
    assert manual_controls.input_source == 'gui'


def test_update_manual_start_from_external_sender_is_marked_external():
    core = _make_core()

    core.update_manual_start(SimpleNamespace(data=True))
    manual_controls = core.build_snapshot().manual_controls

    assert manual_controls.manual_start_value is True
    assert manual_controls.input_source == 'external'


def test_update_business_mode_reflects_in_operation_state():
    core = _make_core()

    core.update_business_mode('シミュレーション', '自律走行')
    operation = core.build_snapshot().operation_state

    assert operation.environment == 'シミュレーション'
    assert operation.drive_mode == '自律'


def test_operation_state_transitions_from_not_started_to_driving():
    """運行フェーズがROSメッセージ受信とmanual_startに追従して更新される。"""

    core = _make_core()

    assert core.build_snapshot().operation_state.phase == '未起動'

    core.update_route_state(
        SimpleNamespace(
            status=1, route_version=1, current_index=0, current_label='A-10', total_waypoints=3
        )
    )
    core.update_follower_state(_follower_state_msg(state='IDLE', index=0, label='A-10'))

    assert core.build_snapshot().operation_state.phase == '走行準備完了'

    core.send_manual_start(True)
    core.update_route_state(
        SimpleNamespace(
            status=2, route_version=1, current_index=1, current_label='A-11', total_waypoints=3
        )
    )
    core.update_follower_state(_follower_state_msg(state='RUNNING', index=1, label='A-11'))
    operation = core.build_snapshot().operation_state

    assert operation.phase == '走行中'
    assert operation.manual_start is True
    assert operation.route_progress > 0.0
    assert operation.current_waypoint == 'A-11'


def test_operation_state_reports_pause_reason_while_follower_waits():
    core = _make_core()
    core.send_manual_start(True)
    core.update_route_state(
        SimpleNamespace(
            status=2, route_version=1, current_index=1, current_label='A-11', total_waypoints=3
        )
    )
    core.update_follower_state(_follower_state_msg(state='WAITING_STOP'))
    operation = core.build_snapshot().operation_state

    assert operation.phase == '一時停止'
    assert operation.pause_reason != ''


def test_route_progress_reaches_100_percent_when_route_completes():
    """goal到達後も 95.2% のまま止まらないことを確認する。"""

    core = _make_core()
    core.update_route_state(
        SimpleNamespace(
            status=2, route_version=100, current_index=20, current_label='30', total_waypoints=21
        )
    )
    assert core.build_snapshot().route_state.progress_ratio < 1.0

    core.update_route_state(
        SimpleNamespace(
            status=5, route_version=100, current_index=20, current_label='30', total_waypoints=21
        )
    )
    snapshot = core.build_snapshot()

    assert snapshot.route_state.progress_ratio == 1.0
    assert snapshot.route_state.is_completed is True
    assert snapshot.operation_state.route_progress == 1.0


def _active_target_llh_msg(latitude, longitude):
    return SimpleNamespace(
        target_label='30',
        target_index=20,
        route_version=100,
        target_pose=SimpleNamespace(
            point=SimpleNamespace(
                latitude=latitude, longitude=longitude, altitude=0.0, has_altitude=False
            )
        ),
        distance_m=3.5,
        bearing_deg=12.0,
    )


def _active_target_msg(x, y, z=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def test_update_active_target_llh_provides_map_coordinates():
    """地図描画用の目標緯度経度は geo_pose_converter 由来のtopicから受け取る。"""

    core = _make_core()

    core.update_active_target_llh(_active_target_llh_msg(36.0833, 140.0769))
    target = core.build_snapshot().target_state

    assert target.latitude == 36.0833
    assert target.longitude == 140.0769


def test_active_target_enu_update_keeps_latitude_longitude_from_llh_topic():
    """ENU目標は高頻度で更新されるため、LLH由来の緯度経度を消さないことを確認する。"""

    core = _make_core()
    core.update_active_target_llh(_active_target_llh_msg(36.0833, 140.0769))

    core.update_active_target(_active_target_msg(28000.0, 44640.0))
    target = core.build_snapshot().target_state

    assert target.latitude == 36.0833
    assert target.longitude == 140.0769
    assert target.x_m == 28000.0


def test_update_sig_recog_from_external_sender_is_marked_external():
    """信号認識ノードからの送信を検知し、入力元を区別できることを確認する。"""

    core = _make_core()

    core.update_sig_recog(SimpleNamespace(data=1))
    manual_controls = core.build_snapshot().manual_controls

    assert manual_controls.sig_recog_value == 1
    assert manual_controls.input_source == 'external'


def test_update_sig_recog_echo_confirms_gui_send():
    core = _make_core()
    core.send_sig_recog(1)

    core.update_sig_recog(SimpleNamespace(data=1))
    manual_controls = core.build_snapshot().manual_controls

    assert manual_controls.send_result == 'confirmed'
    assert manual_controls.input_source == 'gui'


def test_update_road_blocked_from_detector_is_marked_external():
    """road_blockage_detector からの送信を external として区別する（12.2節）。"""

    core = _make_core()

    core.update_road_blocked(SimpleNamespace(data=True))
    manual_controls = core.build_snapshot().manual_controls

    assert manual_controls.road_blocked_value is True
    assert manual_controls.road_blocked_source == 'external'


def test_update_road_blocked_echo_confirms_gui_send():
    core = _make_core()
    core.send_road_blocked(True)

    core.update_road_blocked(SimpleNamespace(data=True))
    manual_controls = core.build_snapshot().manual_controls

    assert manual_controls.road_blocked_source == 'gui'
    assert manual_controls.send_result == 'confirmed'


def test_snapshot_includes_event_banners_generated_from_state():
    """Eventカードが常に空だった問題の回帰確認（6.7節）。"""

    core = _make_core()
    core.update_road_blocked(SimpleNamespace(data=True))

    events = core.build_snapshot().event_banners

    assert [e.event_type for e in events] == ['road_blocked']


def test_snapshot_reports_profile_error_as_event():
    core = _make_core()
    profile_id = core._profiles[0].profile_id

    core._on_launch_status(profile_id, NodeLaunchStatus.ERROR, None, 'process has died')
    events = core.build_snapshot().event_banners

    assert any(e.event_type == 'profile_error' and profile_id in e.message for e in events)
