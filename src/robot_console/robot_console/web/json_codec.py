"""`ConsoleSnapshot` をHTML遠隔観測UI向けJSONペイロードへ変換するモジュール。

HTML UIは観測専用であるため、操作系の状態（`ManualControlsView`）や、
起動時overrideに任意の値（NTRIPパスワード等）が入り得る
`LaunchProfileState.override_inputs` はペイロードに含めない
（screen_function_design.md 8.3節、architecture_design.md 16.6節）。
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Optional

from ..core.event_priority import sort_by_priority
from ..core.route_adapter import traveled_waypoint_count
from ..core.snapshot_model import (
    ConsoleSnapshot,
    EventBanner,
    HealthSummaryView,
    ImageReference,
    RouteWaypointView,
)


def _iso(value: Optional[datetime]) -> Optional[str]:
    return value.isoformat() if value is not None else None


def _panel_payload(panel: ImageReference) -> Dict[str, Any]:
    return {
        'panel_id': panel.panel_id,
        'title': panel.title,
        'topic': panel.topic,
        'width': panel.width,
        'height': panel.height,
        'updated_at': _iso(panel.updated_at),
        'freshness': panel.freshness.value,
    }


def _waypoint_payload(waypoint: RouteWaypointView) -> Dict[str, Any]:
    return {
        'index': waypoint.index,
        'latitude': waypoint.latitude,
        'longitude': waypoint.longitude,
    }


def _event_payload(event: EventBanner) -> Dict[str, Any]:
    return {
        'event_type': event.event_type,
        'message': event.message,
        'severity': event.severity,
        'source': event.source,
        'occurred_at': _iso(event.occurred_at),
    }


def _health_payload(item: HealthSummaryView) -> Dict[str, Any]:
    return {
        'profile_id': item.profile_id,
        'category': item.category,
        'status': item.status,
        'health': item.health.value,
        'required_but_not_selected': item.required_but_not_selected,
    }


def build_snapshot_payload(snapshot: ConsoleSnapshot) -> Dict[str, Any]:
    """`GET /snapshot.json` のペイロードを組み立てる（8.2節 表示内容）。"""

    operation = snapshot.operation_state
    gps = snapshot.gps_state
    localization = snapshot.localization_state
    route = snapshot.route_state
    follower = snapshot.follower_state
    target = snapshot.target_state
    drive = snapshot.drive_mode_state

    return {
        'timestamp': _iso(snapshot.timestamp),
        'operation': {
            'environment': operation.environment,
            'drive_mode': operation.drive_mode,
            'phase': operation.phase,
            'route_progress': operation.route_progress,
            'current_waypoint': operation.current_waypoint,
            'next_waypoint': operation.next_waypoint,
            'manual_start': operation.manual_start,
            'pause_reason': operation.pause_reason,
        },
        'drive': {
            'mode': drive.mode,
            'output_source': drive.output_source,
            'auto_resume_pending': drive.auto_resume_pending,
            'cmd_vel_linear_mps': drive.cmd_vel_linear_mps,
            'cmd_vel_angular_dps': drive.cmd_vel_angular_dps,
            'cmd_vel_freshness': drive.cmd_vel_freshness.value,
            'odom_topic': drive.odom_topic,
            'odom_freshness': drive.odom_freshness.value,
        },
        'gps': {
            'rtk_state': gps.rtk_state,
            'num_satellites': gps.num_satellites,
            'hdop': gps.hdop,
            'correction_age_s': gps.correction_age_s,
            'rtcm_bytes_received': gps.rtcm_bytes_received,
            'heading_deg': gps.heading_deg,
            'heading_stddev_deg': gps.heading_stddev_deg,
            'latitude': gps.latitude,
            'longitude': gps.longitude,
            'altitude': gps.altitude,
            'fix_freshness': gps.fix_freshness.value,
            'heading_freshness': gps.heading_freshness.value,
            'status_freshness': gps.status_freshness.value,
            'display_level': gps.display_level,
        },
        'localization': {
            'source': localization.source,
            'latitude': localization.latitude,
            'longitude': localization.longitude,
            'altitude': localization.altitude,
            'x_m': localization.x_m,
            'y_m': localization.y_m,
            'z_m': localization.z_m,
            'yaw_deg': localization.yaw_deg,
            'freshness': localization.freshness.value,
            'updated_at': _iso(localization.updated_at),
        },
        'route': {
            'state': route.state,
            'route_version': route.route_version,
            'current_index': route.current_index,
            'total_waypoints': route.total_waypoints,
            'progress_ratio': route.progress_ratio,
            'is_completed': route.is_completed,
            'traveled_waypoint_count': traveled_waypoint_count(route),
            'coordinate_kind': route.coordinate_kind,
            'waypoints': [_waypoint_payload(waypoint) for waypoint in route.waypoints],
        },
        'follower': {
            'state': follower.state,
            'active_waypoint_index': follower.active_waypoint_index,
            'active_waypoint_label': follower.active_waypoint_label,
        },
        'target': {
            'latitude': target.latitude,
            'longitude': target.longitude,
            'x_m': target.x_m,
            'y_m': target.y_m,
            'distance_m': target.distance_m,
            'bearing_deg': target.bearing_deg,
            'within_arrival_threshold': target.within_arrival_threshold,
            'freshness': target.freshness.value,
        },
        'events': [_event_payload(event) for event in sort_by_priority(snapshot.event_banners)],
        'sensor_panels': [_panel_payload(panel) for panel in snapshot.sensor_panels],
        'health': [_health_payload(item) for item in snapshot.health],
    }


def build_map_state_payload(snapshot: ConsoleSnapshot) -> Dict[str, Any]:
    """`GET /map_state.json` のペイロードを組み立てる（8.4節・16.3節）。

    waypoint列は `route.waypoints`（`geo_pose` を持つもののみ緯度経度が入る）
    として提供する。走行済み/未走行の判定は `route_progress.current_index`
    との比較で行う。
    """

    localization = snapshot.localization_state
    target = snapshot.target_state
    route = snapshot.route_state

    return {
        'timestamp': _iso(snapshot.timestamp),
        'current_position': {
            'latitude': localization.latitude,
            'longitude': localization.longitude,
            'x_m': localization.x_m,
            'y_m': localization.y_m,
            'yaw_deg': localization.yaw_deg,
            'freshness': localization.freshness.value,
        },
        'target_position': {
            'latitude': target.latitude,
            'longitude': target.longitude,
            'x_m': target.x_m,
            'y_m': target.y_m,
            'freshness': target.freshness.value,
        },
        'route_progress': {
            'current_index': route.current_index,
            'total_waypoints': route.total_waypoints,
            'progress_ratio': route.progress_ratio,
            'is_completed': route.is_completed,
            'traveled_waypoint_count': traveled_waypoint_count(route),
        },
        'route': {
            'waypoints': [_waypoint_payload(waypoint) for waypoint in route.waypoints],
        },
    }


def build_sensor_panels_payload(snapshot: ConsoleSnapshot) -> Dict[str, Any]:
    """`GET /sensor_panels.json` のペイロードを組み立てる。"""

    return {'panels': [_panel_payload(panel) for panel in snapshot.sensor_panels]}


def build_health_payload(snapshot: ConsoleSnapshot) -> Dict[str, Any]:
    """`GET /health.json` のペイロードを組み立てる（topic鮮度・profile稼働状態）。"""

    return {
        'timestamp': _iso(snapshot.timestamp),
        'gps_freshness': {
            'fix': snapshot.gps_state.fix_freshness.value,
            'heading': snapshot.gps_state.heading_freshness.value,
            'status': snapshot.gps_state.status_freshness.value,
        },
        'localization_freshness': snapshot.localization_state.freshness.value,
        'sensor_panels': [
            {'panel_id': panel.panel_id, 'freshness': panel.freshness.value}
            for panel in snapshot.sensor_panels
        ],
        'profiles': [_health_payload(item) for item in snapshot.health],
    }


__all__: List[str] = [
    'build_snapshot_payload',
    'build_map_state_payload',
    'build_sensor_panels_payload',
    'build_health_payload',
]
