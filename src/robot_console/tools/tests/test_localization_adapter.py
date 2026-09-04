"""core/localization_adapter.py の単体テスト（ROS非依存）。"""

import math
from types import SimpleNamespace

import pytest

from robot_console.core.localization_adapter import (
    gps_view_from_rtk_status_msg,
    localization_view_from_pose_enu_msg,
    localization_view_from_pose_llh_msg,
    rtk_state_label,
)


def _pose_enu_msg(*, x=1.0, y=2.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame_id='map'):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
            )
        ),
    )


def _pose_llh_msg(*, latitude=36.083, longitude=140.113, altitude=25.0, has_altitude=True,
                   has_yaw_enu=False, yaw_enu_rad=0.0, child_frame_id='base_link'):
    return SimpleNamespace(
        pose=SimpleNamespace(
            point=SimpleNamespace(
                latitude=latitude, longitude=longitude, altitude=altitude,
                has_altitude=has_altitude,
            ),
            has_yaw_enu=has_yaw_enu,
            yaw_enu_rad=yaw_enu_rad,
            child_frame_id=child_frame_id,
        )
    )


def _rtk_status_msg(**overrides):
    defaults = dict(
        rtk_state=4, rtk_state_raw='RTK_FIX', num_satellites=19, hdop=0.7,
        correction_age_s=0.9, rtcm_bytes_received=482913, heading_deg=87.3,
        heading_stddev_deg=0.6, baseline_length_m=1.2, latitude=36.083,
        longitude=140.113, altitude=25.0,
    )
    defaults.update(overrides)
    return SimpleNamespace(**defaults)


def test_rtk_state_label_maps_known_values():
    assert rtk_state_label(0) == 'UNKNOWN'
    assert rtk_state_label(4) == 'RTK_FIX'


def test_rtk_state_label_falls_back_to_unknown_for_unmapped_value():
    assert rtk_state_label(99) == 'UNKNOWN'


def test_gps_view_from_rtk_status_msg_maps_all_fields():
    view = gps_view_from_rtk_status_msg(_rtk_status_msg())

    assert view.rtk_state == 'RTK_FIX'
    assert view.rtk_state_raw == 'RTK_FIX'
    assert view.num_satellites == 19
    assert view.hdop == 0.7
    assert view.latitude == 36.083
    assert view.longitude == 140.113


def test_localization_view_from_pose_enu_msg_extracts_position_and_frame():
    view = localization_view_from_pose_enu_msg(_pose_enu_msg(x=1.5, y=2.5, z=0.1))

    assert view.source == 'pose_enu'
    assert view.x_m == 1.5
    assert view.y_m == 2.5
    assert view.latitude is None


def test_localization_view_from_pose_enu_msg_computes_yaw_from_quaternion():
    # yaw=90degのクォータニオン
    half = math.sin(math.radians(45.0))
    view = localization_view_from_pose_enu_msg(_pose_enu_msg(qz=half, qw=math.cos(math.radians(45.0))))

    assert view.yaw_deg == pytest.approx(90.0)


def test_localization_view_from_pose_llh_msg_extracts_lat_lon():
    view = localization_view_from_pose_llh_msg(_pose_llh_msg(latitude=36.09, longitude=140.12))

    assert view.source == 'pose_llh'
    assert view.latitude == 36.09
    assert view.longitude == 140.12
    assert view.yaw_deg is None


def test_localization_view_from_pose_llh_msg_sets_yaw_when_available():
    view = localization_view_from_pose_llh_msg(_pose_llh_msg(has_yaw_enu=True, yaw_enu_rad=math.pi / 2))

    assert view.yaw_deg == pytest.approx(90.0)


def test_localization_view_from_pose_llh_msg_altitude_none_when_unavailable():
    view = localization_view_from_pose_llh_msg(_pose_llh_msg(has_altitude=False))

    assert view.altitude is None
