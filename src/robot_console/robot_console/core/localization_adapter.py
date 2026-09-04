"""GPS/自己位置のROSメッセージを表示用Viewへ変換する純粋関数群。

`ros/console_node.py` から渡されるROSメッセージ（またはテスト用の同形の
ダミーオブジェクト）を受け取り、`core/snapshot_model.py` の
`GpsStateView` / `LocalizationStateView` を組み立てる。ROSメッセージ型を
importしないため、`rclpy` が無い環境でも単体テストできる。

鮮度（`FreshnessLevel`）の判定は `ConsoleCore` が `FreshnessMonitor` を用いて
一元的に行うため、本モジュールでは設定しない。
"""

from __future__ import annotations

import math
from typing import Any

from .snapshot_model import GpsStateView, LocalizationStateView

_RTK_STATE_LABELS = {
    0: 'UNKNOWN',
    1: 'STANDALONE',
    2: 'DGPS',
    3: 'RTK_FLOAT',
    4: 'RTK_FIX',
}


def rtk_state_label(value: int) -> str:
    """`rtk_gps_um982_msgs/RtkStatus.rtk_state`（uint8）を表示用文字列へ変換する。"""

    return _RTK_STATE_LABELS.get(int(value), 'UNKNOWN')


def gps_view_from_rtk_status_msg(msg: Any) -> GpsStateView:
    """`rtk_gps_um982_msgs/RtkStatus` から `GpsStateView` を組み立てる。"""

    return GpsStateView(
        rtk_state=rtk_state_label(msg.rtk_state),
        rtk_state_raw=str(getattr(msg, 'rtk_state_raw', '')),
        num_satellites=int(msg.num_satellites),
        hdop=float(msg.hdop),
        correction_age_s=float(msg.correction_age_s),
        rtcm_bytes_received=int(msg.rtcm_bytes_received),
        heading_deg=float(msg.heading_deg),
        heading_stddev_deg=float(msg.heading_stddev_deg),
        baseline_length_m=float(msg.baseline_length_m),
        latitude=float(msg.latitude),
        longitude=float(msg.longitude),
        altitude=float(msg.altitude),
    )


def _yaw_deg_from_quaternion(orientation: Any) -> float:
    """クォータニオン（東ゼロ・反時計回りが正のENU）からyaw角 [deg] を算出する。"""

    yaw_rad = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )
    return math.degrees(yaw_rad)


def localization_view_from_pose_enu_msg(msg: Any) -> LocalizationStateView:
    """`geometry_msgs/PoseWithCovarianceStamped`（`/localization/pose_enu`）から
    `LocalizationStateView` を組み立てる。緯度経度は持たないためNoneのまま返す。
    """

    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return LocalizationStateView(
        source='pose_enu',
        x_m=float(position.x),
        y_m=float(position.y),
        z_m=float(position.z),
        yaw_deg=_yaw_deg_from_quaternion(orientation),
        frame_id=str(msg.header.frame_id),
    )


def localization_view_from_pose_llh_msg(msg: Any) -> LocalizationStateView:
    """`tc_geo_msgs/GeoPoseWithQuality`（`/localization/pose_llh`）から
    `LocalizationStateView` を組み立てる。x_m/y_mはENU側の値を別途保持する
    想定のため設定しない。
    """

    point = msg.pose.point
    yaw_deg = None
    if getattr(msg.pose, 'has_yaw_enu', False):
        yaw_deg = math.degrees(msg.pose.yaw_enu_rad)
    altitude = float(point.altitude) if getattr(point, 'has_altitude', False) else None

    return LocalizationStateView(
        source='pose_llh',
        latitude=float(point.latitude),
        longitude=float(point.longitude),
        altitude=altitude,
        yaw_deg=yaw_deg,
        frame_id=str(getattr(msg.pose, 'child_frame_id', '')),
    )
