"""web/json_codec.py の単体テスト。"""

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import (
    ConsoleSnapshot,
    DriveModeStateView,
    GpsStateView,
    HealthSummaryView,
    ImageReference,
    LocalizationStateView,
    ManualControlsView,
    OperationStateView,
    RouteView,
    RouteWaypointView,
    TargetView,
)
from robot_console.web.json_codec import (
    build_health_payload,
    build_map_state_payload,
    build_sensor_panels_payload,
    build_snapshot_payload,
)


def _sample_snapshot() -> ConsoleSnapshot:
    return ConsoleSnapshot(
        operation_state=OperationStateView(
            environment='実機', drive_mode='自律', phase='走行中', route_progress=0.5,
            current_waypoint='A-10', next_waypoint='A-11',
        ),
        gps_state=GpsStateView(rtk_state='RTK_FIX', num_satellites=18),
        localization_state=LocalizationStateView(
            source='pose_enu', x_m=1.0, y_m=2.0, freshness=FreshnessLevel.OK
        ),
        target_state=TargetView(distance_m=3.5, within_arrival_threshold=False),
        route_state=RouteView(
            current_index=1,
            total_waypoints=3,
            waypoints=[
                RouteWaypointView(index=0, latitude=36.083, longitude=140.113),
                RouteWaypointView(index=1, latitude=36.0831, longitude=140.1131),
                RouteWaypointView(index=2, latitude=None, longitude=None),
            ],
        ),
        manual_controls=ManualControlsView(manual_start_value=True),
        sensor_panels=[
            ImageReference(
                panel_id='sensor_viewer', title='Sensor Viewer', topic='/sensor_viewer',
                freshness=FreshnessLevel.OK,
            )
        ],
        health=[
            HealthSummaryView(profile_id='route_manager', category='route_stack', status='RUNNING')
        ],
    )


def test_snapshot_payload_contains_documented_sections():
    payload = build_snapshot_payload(_sample_snapshot())

    assert payload['operation']['phase'] == '走行中'
    assert payload['gps']['rtk_state'] == 'RTK_FIX'
    assert payload['localization']['x_m'] == 1.0
    assert payload['target']['distance_m'] == 3.5
    assert payload['sensor_panels'][0]['panel_id'] == 'sensor_viewer'
    assert payload['health'][0]['profile_id'] == 'route_manager'


def test_snapshot_payload_excludes_manual_controls_and_launch_profiles():
    payload = build_snapshot_payload(_sample_snapshot())

    assert 'manual_controls' not in payload
    assert 'launch_profiles' not in payload
    assert 'logs' not in payload
    assert 'log_paths' not in payload


def test_health_payload_excludes_override_inputs():
    payload = build_health_payload(_sample_snapshot())

    for profile in payload['profiles']:
        assert 'override_inputs' not in profile
        assert set(profile.keys()) == {
            'profile_id', 'category', 'status', 'health', 'required_but_not_selected'
        }


def test_freshness_enums_are_serialized_as_plain_strings():
    payload = build_snapshot_payload(_sample_snapshot())

    assert payload['localization']['freshness'] == 'OK'
    assert isinstance(payload['gps']['fix_freshness'], str)


def test_map_state_payload_uses_localization_and_target():
    payload = build_map_state_payload(_sample_snapshot())

    assert payload['current_position']['x_m'] == 1.0
    assert payload['target_position']['x_m'] is None  # サンプルではx_m未設定
    assert payload['route_progress']['progress_ratio'] == 0.0  # RouteViewは既定値


def test_snapshot_payload_route_includes_waypoints_with_null_coordinates():
    payload = build_snapshot_payload(_sample_snapshot())

    waypoints = payload['route']['waypoints']
    assert len(waypoints) == 3
    assert waypoints[0] == {'index': 0, 'latitude': 36.083, 'longitude': 140.113}
    assert waypoints[2] == {'index': 2, 'latitude': None, 'longitude': None}


def test_map_state_payload_route_includes_waypoints():
    payload = build_map_state_payload(_sample_snapshot())

    waypoints = payload['route']['waypoints']
    assert len(waypoints) == 3
    assert waypoints[1] == {'index': 1, 'latitude': 36.0831, 'longitude': 140.1131}


def test_sensor_panels_payload_matches_snapshot_panels():
    payload = build_sensor_panels_payload(_sample_snapshot())

    assert len(payload['panels']) == 1
    assert payload['panels'][0]['panel_id'] == 'sensor_viewer'


def test_timestamp_is_iso_formatted_string():
    payload = build_snapshot_payload(_sample_snapshot())

    assert 'T' in payload['timestamp']


def test_snapshot_payload_includes_manual_start_and_pause_reason():
    """HTML観測UIでも一時停止理由とmanual_start現在値を確認できる（6.3節）。"""

    snapshot = _sample_snapshot()
    snapshot.operation_state = OperationStateView(
        phase='一時停止', manual_start=True, pause_reason='停止waypointで待機中（信号/停止線）'
    )

    payload = build_snapshot_payload(snapshot)

    assert payload['operation']['manual_start'] is True
    assert payload['operation']['pause_reason'] == '停止waypointで待機中（信号/停止線）'


def test_snapshot_payload_includes_drive_mode_section():
    snapshot = _sample_snapshot()
    snapshot.drive_mode_state = DriveModeStateView(
        mode='autonomous',
        output_source='autonomous_cmd',
        cmd_vel_linear_mps=0.4,
        cmd_vel_angular_dps=12.0,
        cmd_vel_freshness=FreshnessLevel.OK,
        odom_topic='/odom',
        odom_freshness=FreshnessLevel.STALE,
    )

    payload = build_snapshot_payload(snapshot)

    assert payload['drive']['mode'] == 'autonomous'
    assert payload['drive']['cmd_vel_linear_mps'] == 0.4
    assert payload['drive']['cmd_vel_freshness'] == 'OK'
    assert payload['drive']['odom_freshness'] == 'STALE'


def test_snapshot_payload_exposes_traveled_waypoint_count_for_map_coloring():
    """完走時に最終waypointが未走行の色で残らないよう、走行済み点数をCore側から渡す。"""

    snapshot = _sample_snapshot()
    snapshot.route_state = RouteView(
        current_index=2, total_waypoints=3, is_completed=True, progress_ratio=1.0
    )

    payload = build_snapshot_payload(snapshot)

    assert payload['route']['is_completed'] is True
    assert payload['route']['traveled_waypoint_count'] == 3
    assert payload['route']['progress_ratio'] == 1.0


def test_map_state_payload_exposes_traveled_waypoint_count():
    snapshot = _sample_snapshot()
    snapshot.route_state = RouteView(current_index=1, total_waypoints=3)

    payload = build_map_state_payload(snapshot)

    assert payload['route_progress']['traveled_waypoint_count'] == 1
    assert payload['route_progress']['is_completed'] is False
