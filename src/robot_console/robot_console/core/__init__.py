"""robot_console のUI非依存Core層。

ROS 2 topic購読・publish、GUI描画から独立した状態集約・起動管理・ログ管理・
Snapshot生成を提供する。PyQt5 UIおよびHTML UIは、本パッケージが提供する
Snapshotのみを参照し、ROSメッセージ型を直接解釈しない。
"""

from .business_mode import (
    DRIVE_MODES,
    ENVIRONMENTS,
    LAUNCH_PRESETS,
    LaunchPlan,
    LaunchPresetEntry,
    get_preset,
)
from .console_core import ConsoleCore
from .freshness import (
    DEFAULT_LOST_SEC,
    DEFAULT_STALE_SEC,
    FreshnessLevel,
    FreshnessMonitor,
    FreshnessThresholds,
)
from .image_store import ImageStore
from .launch_manager import LaunchManager
from .launch_profile import (
    LaunchProfile,
    LaunchProfileError,
    LaunchProfileState,
    LaunchProfileStore,
    build_initial_states,
    build_launch_args,
    build_simulator_launch_args,
    resolve_effective_overrides,
)
from .log_manager import (
    LogLevelCounts,
    LogManager,
    count_levels,
    detect_log_level,
    filter_levels,
)
from .metrics import (
    Position3D,
    TripMetrics,
    TripSnapshot,
    compute_progress_ratio,
    euclidean_distance,
    is_within_arrival_threshold,
)
from .snapshot_model import (
    ConsoleSnapshot,
    DriveModeStateView,
    EventBanner,
    FollowerView,
    GpsStateView,
    HealthSummaryView,
    ImageReference,
    LocalizationStateView,
    ManualControlsView,
    ObstacleStateView,
    OperationStateView,
    RouteView,
    RouteWaypointView,
    TargetView,
)

__all__ = [
    'DRIVE_MODES',
    'ENVIRONMENTS',
    'LAUNCH_PRESETS',
    'LaunchPlan',
    'LaunchPresetEntry',
    'get_preset',
    'ConsoleCore',
    'DEFAULT_LOST_SEC',
    'DEFAULT_STALE_SEC',
    'FreshnessLevel',
    'FreshnessMonitor',
    'FreshnessThresholds',
    'ImageStore',
    'LaunchManager',
    'LaunchProfile',
    'LaunchProfileError',
    'LaunchProfileState',
    'LaunchProfileStore',
    'build_initial_states',
    'build_launch_args',
    'build_simulator_launch_args',
    'resolve_effective_overrides',
    'LogLevelCounts',
    'LogManager',
    'count_levels',
    'detect_log_level',
    'filter_levels',
    'Position3D',
    'TripMetrics',
    'TripSnapshot',
    'compute_progress_ratio',
    'euclidean_distance',
    'is_within_arrival_threshold',
    'ConsoleSnapshot',
    'DriveModeStateView',
    'EventBanner',
    'FollowerView',
    'GpsStateView',
    'HealthSummaryView',
    'ImageReference',
    'LocalizationStateView',
    'ManualControlsView',
    'ObstacleStateView',
    'OperationStateView',
    'RouteView',
    'RouteWaypointView',
    'TargetView',
]
