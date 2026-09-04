"""PyQt5 UI / HTML UI 共通の Snapshot / ViewModel データモデル。

UIはROSメッセージ型を直接参照せず、本モジュールで定義する `ConsoleSnapshot` を
読む。個々の状態を実際のROS topicから集約する処理（Adapter/StateStore/
ViewModelBuilder）は本モジュールの対象外であり、後続フェーズで実装する。
フィールド構成は docs/robot_console_gui_architecture_design.md 8章、
docs/robot_console_gui_screen_function_design.md 6〜7章を正とする。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional

from .freshness import FreshnessLevel
from .launch_profile import LaunchProfileState


def _utc_now() -> datetime:
    return datetime.now(tz=timezone.utc)


@dataclass
class ImageReference:
    """画像本体を含まない、画像パネル向けの参照情報。

    画像本体は `ImageStore`（後続フェーズで実装）が保持し、PyQt5 UIは
    `ImageStore.get_qimage(image_id)`、HTML UIは `GET /images/{panel_id}` で取得する。
    """

    panel_id: str = ''
    title: str = ''
    topic: str = ''
    image_id: Optional[str] = None
    width: int = 0
    height: int = 0
    updated_at: Optional[datetime] = None
    freshness: FreshnessLevel = FreshnessLevel.UNKNOWN


@dataclass
class OperationStateView:
    """運行フェーズ領域（ダッシュボード上段）の表示用データ。"""

    environment: str = 'unknown'  # 実機 / シミュレーション / 机上確認
    drive_mode: str = 'unknown'  # 手動 / 自律
    phase: str = '未起動'
    route_progress: float = 0.0
    current_waypoint: str = ''
    next_waypoint: str = ''
    manual_start: bool = False
    pause_reason: str = ''
    updated_at: Optional[datetime] = None


@dataclass
class RouteWaypointView:
    """地図重畳用のwaypoint 1点分の表示データ（8.4節 waypoint列表示）。

    `tc_route_msgs/Waypoint` の `geo_pose`（`has_geo_pose=True` の場合）に
    対応する。緯度経度が無い場合（ENUのみでLLH未確定の場合）は地図上には
    描画しない。
    """

    index: int = 0
    label: str = ''
    latitude: Optional[float] = None
    longitude: Optional[float] = None


@dataclass
class RouteView:
    """route_manager 由来の運行状態（Route/Followerカードのroute_manager部分）。"""

    state: str = 'unknown'
    route_version: int = 0
    last_decision: str = ''
    last_replan_reason: str = ''
    current_index: int = 0
    current_label: str = ''
    total_waypoints: int = 0
    progress_ratio: float = 0.0
    # route_manager が完走を通知したか（`RouteState.STATUS_COMPLETED`）。
    # `current_index` は最終waypointのindex（0起算）で止まるため、完走判定は
    # indexだけでは行えない（進捗が 100% にならず、最終waypointも未走行扱いになる）。
    is_completed: bool = False
    coordinate_kind: str = 'enu'  # enu / llh
    waypoints: List[RouteWaypointView] = field(default_factory=list)
    updated_at: Optional[datetime] = None


@dataclass
class FollowerView:
    """route_follower 由来の運行状態（Route/Followerカードのfollower部分）。"""

    state: str = 'unknown'
    active_waypoint_index: int = 0
    active_waypoint_label: str = ''
    stagnation_reason: str = ''
    avoidance_attempt_count: int = 0
    front_blocked: bool = False
    front_clearance_m: float = float('inf')
    left_offset_m: float = 0.0
    right_offset_m: float = 0.0
    signal_stop_active: bool = False
    line_stop_active: bool = False
    updated_at: Optional[datetime] = None


@dataclass
class TargetView:
    """active_target 由来の目標情報（距離ゲージ・到達判定に利用する）。"""

    target_id: str = ''
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    x_m: Optional[float] = None
    y_m: Optional[float] = None
    z_m: Optional[float] = None
    distance_m: float = 0.0
    bearing_deg: Optional[float] = None
    arrival_threshold_m: Optional[float] = None
    within_arrival_threshold: bool = False
    freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    updated_at: Optional[datetime] = None


@dataclass
class DriveModeStateView:
    """Drive / CmdVelカードの表示用データ。"""

    mode: str = 'unknown'  # autonomous / manual
    output_source: str = ''
    auto_resume_pending: bool = False
    cmd_vel_linear_mps: float = 0.0
    cmd_vel_angular_dps: float = 0.0
    cmd_vel_freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    cmd_vel_autonomous_linear_mps: Optional[float] = None
    cmd_vel_autonomous_angular_dps: Optional[float] = None
    odom_topic: str = ''
    odom_freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    updated_at: Optional[datetime] = None


@dataclass
class ObstacleStateView:
    """obstacle_avoidance_hint 由来の障害物状態。"""

    front_blocked: bool = False
    front_clearance_m: float = 0.0
    left_offset_m: float = 0.0
    right_offset_m: float = 0.0
    freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    updated_at: Optional[datetime] = None


@dataclass
class ManualControlsView:
    """Manual Opsカード向けの手動操作状態（12.2節 手動操作状態）。"""

    manual_start_value: bool = False
    manual_start_last_sent_at: Optional[datetime] = None
    sig_recog_value: Optional[int] = None
    sig_recog_last_sent_at: Optional[datetime] = None
    road_blocked_value: bool = False
    road_blocked_source: str = 'unknown'  # GUI / external / unknown
    road_blocked_last_sent_at: Optional[datetime] = None
    obstacle_hint_override_active: bool = False
    obstacle_hint_last_sent_at: Optional[datetime] = None
    input_source: str = 'unknown'
    send_result: str = 'unknown'  # sent / waiting_echo / confirmed / timeout


@dataclass
class EventBanner:
    """Eventカードに表示するイベント1件分のデータ。"""

    event_type: str = ''
    message: str = ''
    severity: str = 'info'  # info / notice / warn / error
    source: str = ''
    occurred_at: Optional[datetime] = None
    expires_at: Optional[datetime] = None
    sticky: bool = False


@dataclass
class HealthSummaryView:
    """Node Healthカードに表示するprofile1件分のサマリ。"""

    profile_id: str = ''
    category: str = ''
    status: str = 'STOPPED'  # STOPPED / STARTING / RUNNING / ERROR
    health: FreshnessLevel = FreshnessLevel.UNKNOWN
    last_log_level: Optional[str] = None
    required_but_not_selected: bool = False


@dataclass
class GpsStateView:
    """GPS / Poseカードの表示用データ（architecture_design.md 8.1節）。"""

    rtk_state: str = 'UNKNOWN'  # UNKNOWN / STANDALONE / DGPS / RTK_FLOAT / RTK_FIX
    rtk_state_raw: str = ''
    num_satellites: int = 0
    hdop: float = 0.0
    correction_age_s: float = 0.0
    rtcm_bytes_received: int = 0
    heading_deg: float = 0.0
    heading_stddev_deg: float = 0.0
    baseline_length_m: float = 0.0
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    fix_freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    heading_freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    status_freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    display_level: str = 'UNKNOWN'  # NORMAL / NOTICE / WARN / ERROR / UNKNOWN


@dataclass
class LocalizationStateView:
    """自己位置の表示用データ（architecture_design.md 6.2節 LocalizationView）。"""

    source: str = 'pose_enu'  # pose_enu（現行互換） / pose_llh（将来正本）
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    x_m: Optional[float] = None
    y_m: Optional[float] = None
    z_m: Optional[float] = None
    yaw_deg: Optional[float] = None
    frame_id: str = ''
    covariance_summary: Optional[float] = None
    freshness: FreshnessLevel = FreshnessLevel.UNKNOWN
    updated_at: Optional[datetime] = None


@dataclass
class ConsoleSnapshot:
    """PyQt5 UI / HTML UI 共通のSnapshot（architecture_design.md 8章）。"""

    timestamp: datetime = field(default_factory=_utc_now)
    operation_state: OperationStateView = field(default_factory=OperationStateView)
    gps_state: GpsStateView = field(default_factory=GpsStateView)
    localization_state: LocalizationStateView = field(default_factory=LocalizationStateView)
    route_state: RouteView = field(default_factory=RouteView)
    target_state: TargetView = field(default_factory=TargetView)
    follower_state: FollowerView = field(default_factory=FollowerView)
    obstacle_state: ObstacleStateView = field(default_factory=ObstacleStateView)
    drive_mode_state: DriveModeStateView = field(default_factory=DriveModeStateView)
    sensor_panels: List[ImageReference] = field(default_factory=list)
    event_banners: List[EventBanner] = field(default_factory=list)
    manual_controls: ManualControlsView = field(default_factory=ManualControlsView)
    launch_profiles: Dict[str, LaunchProfileState] = field(default_factory=dict)
    logs: Dict[str, List[str]] = field(default_factory=dict)
    log_paths: Dict[str, Optional[str]] = field(default_factory=dict)
    health: List[HealthSummaryView] = field(default_factory=list)
