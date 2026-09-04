"""snapshot_model モジュールの単体テスト。"""

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import ConsoleSnapshot, GpsStateView, ImageReference


def test_console_snapshot_default_construction():
    snapshot = ConsoleSnapshot()

    assert snapshot.gps_state.rtk_state == 'UNKNOWN'
    assert snapshot.gps_state.fix_freshness == FreshnessLevel.UNKNOWN
    assert snapshot.sensor_panels == []
    assert snapshot.event_banners == []
    assert snapshot.launch_profiles == {}
    assert snapshot.logs == {}
    assert snapshot.health == []


def test_console_snapshot_mutable_fields_are_independent_per_instance():
    first = ConsoleSnapshot()
    second = ConsoleSnapshot()

    first.sensor_panels.append(ImageReference(panel_id='route_map'))

    assert first.sensor_panels != second.sensor_panels
    assert second.sensor_panels == []


def test_gps_state_view_holds_rtk_status_fields():
    gps = GpsStateView(
        rtk_state='RTK_FIX',
        rtk_state_raw='RTK_FIX',
        num_satellites=18,
        hdop=0.8,
        correction_age_s=1.2,
        rtcm_bytes_received=125034,
        heading_deg=123.4,
        heading_stddev_deg=0.8,
        baseline_length_m=0.5,
        latitude=36.08,
        longitude=140.11,
        altitude=25.0,
    )

    assert gps.rtk_state == 'RTK_FIX'
    assert gps.num_satellites == 18
    assert gps.rtcm_bytes_received == 125034
