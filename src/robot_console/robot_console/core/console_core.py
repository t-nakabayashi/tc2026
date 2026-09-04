"""ROS 2非依存の状態集約Facade。

`robot_console_gui_architecture_design.md` 3章が定義する `ConsoleCore` に
相当する。`ros/console_node.py`（ROS 2 Node）から届いたROSメッセージを
`update_*()` で受け取り、`core/localization_adapter.py` / `core/route_adapter.py`
を通じて `ConsoleSnapshot` の各Viewへ変換・格納する。PyQt5 UI / HTML UIは
`build_snapshot()` の戻り値だけを参照し、ROSメッセージ型・`LaunchManager`
などの内部実装には触れない。

起動管理（`LaunchManager`）・ログ収集（`LogManager`）・画像保持
（`ImageStore`）・鮮度判定（`FreshnessMonitor`）は既存モジュールをそのまま
束ねるのみで、本モジュールはROSなしでも単体テストできる（16章 受け入れ条件）。
"""

from __future__ import annotations

import threading
from dataclasses import replace
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional

from ..utils import NodeLaunchStatus, convert_image_message
from .drive_mode_adapter import (
    apply_cmd_vel_autonomous_msg,
    apply_cmd_vel_msg,
    apply_drive_mode_status_msg,
)
from .event_builder import build_event_banners
from .freshness import FreshnessLevel, FreshnessMonitor
from .image_store import ImageStore
from .launch_manager import LaunchManager
from .launch_profile import (
    LaunchProfile,
    LaunchProfileState,
    LaunchProfileStore,
    build_initial_states,
    resolve_effective_overrides,
)
from .localization_adapter import (
    gps_view_from_rtk_status_msg,
    localization_view_from_pose_enu_msg,
    localization_view_from_pose_llh_msg,
)
from .log_manager import LogManager
from .metrics import euclidean_distance
from .operation_phase import build_operation_state
from .route_adapter import (
    apply_active_target_llh_msg,
    apply_manager_status_msg,
    apply_route_msg,
    apply_route_state_msg,
    follower_view_from_msg,
    obstacle_view_from_hint_msg,
    target_view_from_pose_msg,
)
from .snapshot_model import (
    ConsoleSnapshot,
    DriveModeStateView,
    FollowerView,
    GpsStateView,
    HealthSummaryView,
    ImageReference,
    LocalizationStateView,
    ManualControlsView,
    ObstacleStateView,
    RouteView,
    TargetView,
)

_LAUNCH_STATUS_TO_HEALTH: Dict[NodeLaunchStatus, FreshnessLevel] = {
    NodeLaunchStatus.RUNNING: FreshnessLevel.OK,
    NodeLaunchStatus.STARTING: FreshnessLevel.STALE,
    NodeLaunchStatus.STOPPING: FreshnessLevel.STALE,
    NodeLaunchStatus.STOPPED: FreshnessLevel.UNKNOWN,
    NodeLaunchStatus.ERROR: FreshnessLevel.LOST,
}

_SIMULATOR_SUFFIX = ':sim'


def _utc_now() -> datetime:
    return datetime.now(tz=timezone.utc)


class ConsoleCore:
    """PyQt5 UI / HTML UI 共通の状態集約Facade（ROS非依存）。"""

    def __init__(
        self,
        *,
        profile_store: Optional[LaunchProfileStore] = None,
        log_directory: Optional[str] = None,
    ) -> None:
        self._lock = threading.Lock()

        self._profile_store = profile_store or LaunchProfileStore()
        self._profiles: List[LaunchProfile] = self._profile_store.load()
        self._launch_states: Dict[str, LaunchProfileState] = build_initial_states(self._profiles)

        self.log_manager = LogManager(profile_ids=[p.profile_id for p in self._profiles])
        self.image_store = ImageStore()
        self.freshness = FreshnessMonitor()
        self.launch_manager = LaunchManager(
            status_callback=self._on_launch_status,
            log_callback=self._on_launch_log,
            log_directory=log_directory,
        )

        # 業務モード（実行環境・走行モード）は起動・設定タブの選択値であり、
        # ROS topicからは得られないため `update_business_mode()` で受け取る。
        # 運行フェーズ領域の他のフィールドは `build_snapshot()` 時点の状態から
        # `core/operation_phase.py` が毎回算出する。
        self._environment = 'unknown'
        self._drive_mode_selection = 'unknown'
        self._gps_state = GpsStateView()
        self._localization_state = LocalizationStateView()
        self._route_state = RouteView()
        self._target_state = TargetView()
        self._follower_state = FollowerView()
        self._obstacle_state = ObstacleStateView()
        self._drive_mode_state = DriveModeStateView()
        self._manual_controls = ManualControlsView()
        self._sensor_panel_refs: Dict[str, ImageReference] = {}

        # `ros/console_node.py` が bind_publishers() で登録するROS publish関数。
        # 未登録（ROSなしでのテスト等）の場合は状態更新のみ行い送信は行わない。
        self._manual_start_publisher: Optional[Callable[[bool], None]] = None
        self._sig_recog_publisher: Optional[Callable[[int], None]] = None
        self._road_blocked_publisher: Optional[Callable[[bool], None]] = None
        self._obstacle_hint_publisher: Optional[Callable[[bool, float, float, float], None]] = None
        self._frame_image_publisher: Optional[Callable[[str], None]] = None

    def bind_publishers(
        self,
        *,
        manual_start: Optional[Callable[[bool], None]] = None,
        sig_recog: Optional[Callable[[int], None]] = None,
        road_blocked: Optional[Callable[[bool], None]] = None,
        obstacle_hint: Optional[Callable[[bool, float, float, float], None]] = None,
        frame_image: Optional[Callable[[str], None]] = None,
    ) -> None:
        """`ros/console_node.py` が生成したROS publisherの送信関数を登録する。"""

        if manual_start is not None:
            self._manual_start_publisher = manual_start
        if sig_recog is not None:
            self._sig_recog_publisher = sig_recog
        if road_blocked is not None:
            self._road_blocked_publisher = road_blocked
        if obstacle_hint is not None:
            self._obstacle_hint_publisher = obstacle_hint
        if frame_image is not None:
            self._frame_image_publisher = frame_image

    # ---------- 手動操作コマンド（ManualOpsCardから） ----------
    def send_manual_start(self, value: bool) -> None:
        """`manual_start` topicへ送信し、送信状態を記録する。"""

        if self._manual_start_publisher is not None:
            self._manual_start_publisher(value)
        with self._lock:
            self._manual_controls = replace(
                self._manual_controls,
                manual_start_value=value,
                manual_start_last_sent_at=_utc_now(),
                input_source='gui',
                send_result='waiting_echo',
            )

    def send_sig_recog(self, value: int) -> None:
        """`sig_recog` topicへ送信し、送信状態を記録する。"""

        if self._sig_recog_publisher is not None:
            self._sig_recog_publisher(value)
        with self._lock:
            self._manual_controls = replace(
                self._manual_controls,
                sig_recog_value=value,
                sig_recog_last_sent_at=_utc_now(),
                input_source='gui',
            )

    def send_road_blocked(self, value: bool) -> None:
        """`road_blocked` topicへ送信し、送信状態を記録する。"""

        if self._road_blocked_publisher is not None:
            self._road_blocked_publisher(value)
        with self._lock:
            self._manual_controls = replace(
                self._manual_controls,
                road_blocked_value=value,
                road_blocked_source='gui',
                road_blocked_last_sent_at=_utc_now(),
                input_source='gui',
            )

    def send_obstacle_hint_override(
        self,
        active: bool,
        front_clearance_m: float,
        left_offset_m: float,
        right_offset_m: float,
    ) -> None:
        """`obstacle_avoidance_hint` topicへ手動override値を送信する。"""

        if self._obstacle_hint_publisher is not None:
            self._obstacle_hint_publisher(active, front_clearance_m, left_offset_m, right_offset_m)
        with self._lock:
            self._manual_controls = replace(
                self._manual_controls,
                obstacle_hint_override_active=active,
                obstacle_hint_last_sent_at=_utc_now(),
                input_source='gui',
            )

    def send_obstacle_hint_stop(self) -> None:
        """`obstacle_avoidance_hint` のoverrideを解除する（前方クリア相当を送信）。"""

        self.send_obstacle_hint_override(False, 0.0, 0.0, 0.0)

    def send_frame_image_request(self, path: str) -> None:
        """`/frame_image_path` topicへ単発フレーム保存先を送信する。"""

        if self._frame_image_publisher is not None:
            self._frame_image_publisher(path)

    # ---------- センサ・画像更新（ros/console_node.pyから） ----------
    def update_sensor_image(self, panel_id: str, title: str, topic: str, msg: Any) -> None:
        """センサ・画像パネル向けのROS `sensor_msgs/Image` を反映する。"""

        image = convert_image_message(msg)
        if image is not None:
            self.image_store.set(panel_id, image)
        with self._lock:
            self._sensor_panel_refs[panel_id] = ImageReference(
                panel_id=panel_id,
                title=title,
                topic=topic,
                image_id=panel_id,
                width=image.width if image is not None else 0,
                height=image.height if image is not None else 0,
            )
        self.freshness.mark_received(f'image.{panel_id}')

    # ---------- 起動管理コマンド（LaunchControlCard/LaunchSettingsTabから） ----------
    def request_launch(
        self,
        profile_id: str,
        *,
        use_alternate: Optional[bool] = None,
        simulator_enabled: Optional[bool] = None,
        overrides: Optional[Dict[str, str]] = None,
    ) -> None:
        """指定profileの起動を要求する。未知のprofile_idは無視する。

        `use_alternate` / `simulator_enabled` / `overrides` を省略した場合は、
        `update_selected_param()` 等で更新済みの `LaunchProfileState` の現在値を使う。
        """

        profile = self._profile_store.get(profile_id)
        if profile is None:
            return
        state = self._launch_states.get(profile_id)
        param_path = state.selected_param if state else profile.default_param
        resolved_use_alternate = (
            use_alternate if use_alternate is not None
            else (state.use_alternate_launch if state else False)
        )
        resolved_simulator_enabled = (
            simulator_enabled if simulator_enabled is not None
            else (state.simulator_enabled if state else False)
        )
        resolved_overrides = (
            overrides if overrides is not None
            else (resolve_effective_overrides(profile, state) if state else None)
        )
        self.launch_manager.launch(
            profile,
            param_path=param_path,
            use_alternate=resolved_use_alternate,
            simulator_enabled=resolved_simulator_enabled,
            overrides=resolved_overrides,
        )

    def request_stop(self, profile_id: str) -> None:
        """指定profileの停止を要求する。"""

        self.launch_manager.stop(profile_id)

    def update_business_mode(self, environment: str, drive_mode: str) -> None:
        """起動・設定タブで選択中の業務モード（実行環境・走行モード）を反映する。

        運行フェーズ領域の業務モード表示（6.3節）はこの選択値を元にする。
        HTML遠隔観測UI単独起動のように選択操作が無い場合は `unknown` のままとなる。
        """

        with self._lock:
            self._environment = environment or 'unknown'
            self._drive_mode_selection = drive_mode or 'unknown'

    def update_selected_param(self, profile_id: str, param_path: Optional[str]) -> None:
        """起動・設定タブで選択されたパラメータパスを反映する。"""

        with self._lock:
            state = self._launch_states.get(profile_id)
            if state is not None:
                state.selected_param = param_path

    def update_use_alternate_launch(self, profile_id: str, enabled: bool) -> None:
        """起動・設定タブの代替launch切替を反映する。"""

        with self._lock:
            state = self._launch_states.get(profile_id)
            if state is not None:
                state.use_alternate_launch = enabled

    def update_simulator_enabled(self, profile_id: str, enabled: bool) -> None:
        """起動・設定タブのsimulator切替を反映する。"""

        with self._lock:
            state = self._launch_states.get(profile_id)
            if state is not None:
                state.simulator_enabled = enabled

    def update_launch_override(self, profile_id: str, key: str, value: str) -> None:
        """起動・設定タブの引数override入力を反映する。"""

        with self._lock:
            state = self._launch_states.get(profile_id)
            if state is not None:
                state.override_inputs[key] = value

    def request_launch_all(self) -> None:
        """`launch_order` 順に全profileの起動を要求する。"""

        for profile in self._profiles:
            self.request_launch(profile.profile_id)

    def request_stop_all(self) -> None:
        """全profileの停止を要求する。"""

        for profile in self._profiles:
            self.request_stop(profile.profile_id)

    def _on_launch_status(
        self,
        status_id: str,
        status: NodeLaunchStatus,
        process_id: Optional[int],
        error_message: Optional[str],
    ) -> None:
        """`LaunchManager` からの状態通知を`LaunchProfileState`へ反映する。"""

        is_simulator = status_id.endswith(_SIMULATOR_SUFFIX)
        profile_id = status_id[: -len(_SIMULATOR_SUFFIX)] if is_simulator else status_id

        with self._lock:
            state = self._launch_states.get(profile_id)
            if state is None:
                return
            if is_simulator:
                state.simulator_status = status
                state.simulator_process_id = process_id
            else:
                state.status = status
                state.process_id = process_id
            if error_message:
                state.error_message = error_message
            state.last_action_time = _utc_now()
        self.freshness.mark_received(f'launch.{status_id}')

    def _on_launch_log(self, status_id: str, line: str) -> None:
        """`LaunchManager` からのログ行を`LogManager`へ転送する。"""

        self.log_manager.append(status_id, line)

    # ---------- ROSメッセージ反映（ros/console_node.pyから呼ばれる） ----------
    def update_route_state(self, msg: Any) -> None:
        """`tc_route_msgs/RouteState`（`route_state`）を反映する。"""

        with self._lock:
            self._route_state = apply_route_state_msg(self._route_state, msg)
        self.freshness.mark_received('route_state')

    def update_manager_status(self, msg: Any) -> None:
        """`tc_route_msgs/ManagerStatus`（`manager_status`）を反映する。"""

        with self._lock:
            self._route_state = apply_manager_status_msg(self._route_state, msg)

    def update_route(self, msg: Any) -> None:
        """`tc_route_msgs/Route`（`active_route`）を反映する。"""

        with self._lock:
            self._route_state = apply_route_msg(self._route_state, msg)
        self.freshness.mark_received('route')

    def update_follower_state(self, msg: Any) -> None:
        """`tc_route_msgs/FollowerState`（`follower_state`）を反映する。"""

        with self._lock:
            self._follower_state = follower_view_from_msg(msg)
        self.freshness.mark_received('follower_state')

    def update_drive_mode_status(self, msg: Any) -> None:
        """`tc_route_msgs/DriveModeStatus`（`drive_mode_status`）を反映する。"""

        with self._lock:
            self._drive_mode_state = apply_drive_mode_status_msg(self._drive_mode_state, msg)
        self.freshness.mark_received('drive_mode_status')

    def update_cmd_vel(self, msg: Any) -> None:
        """`geometry_msgs/Twist`（`cmd_vel`）を反映する（mux後の最終指令）。"""

        with self._lock:
            self._drive_mode_state = apply_cmd_vel_msg(self._drive_mode_state, msg)
        self.freshness.mark_received('cmd_vel')

    def update_cmd_vel_autonomous(self, msg: Any) -> None:
        """`geometry_msgs/Twist`（`cmd_vel/autonomous`）を反映する。"""

        with self._lock:
            self._drive_mode_state = apply_cmd_vel_autonomous_msg(self._drive_mode_state, msg)
        self.freshness.mark_received('cmd_vel_autonomous')

    def update_odom(self, msg: Any, *, topic: str = '') -> None:
        """`nav_msgs/Odometry`（`odom`）の受信を鮮度として反映する。

        6.5節のodom表示はfreshness確認が目的であり、内容そのものは表示しない
        （自己位置は `localization/pose_enu` を正とする）。
        """

        with self._lock:
            if topic and self._drive_mode_state.odom_topic != topic:
                self._drive_mode_state = replace(self._drive_mode_state, odom_topic=topic)
        self.freshness.mark_received('odom')

    def update_manual_start(self, msg: Any) -> None:
        """`std_msgs/Bool`（`manual_start`）の現在値を反映する。

        自身の送信分もDDS経由でエコーバックされるため、GUI送信済みの場合は
        送信結果（`send_result`）を `confirmed` にする。GUI以外の送信元
        （joyコンソール等）から届いた場合は入力元を `external` として扱う。
        """

        value = bool(msg.data)
        with self._lock:
            sent_from_gui = self._manual_controls.manual_start_last_sent_at is not None
            confirmed = sent_from_gui and self._manual_controls.manual_start_value == value
            self._manual_controls = replace(
                self._manual_controls,
                manual_start_value=value,
                input_source='gui' if confirmed else 'external',
                send_result='confirmed' if confirmed else self._manual_controls.send_result,
            )
        self.freshness.mark_received('manual_start')

    def update_sig_recog(self, msg: Any) -> None:
        """`std_msgs/Int32`（`sig_recog`）の現在値を反映する。

        `manual_start` と同様、自身の送信分もエコーバックされるため、GUI送信済みの
        値と一致する場合は送信確認として扱い、それ以外は外部送信元（信号認識ノード
        等）からの入力として扱う。
        """

        value = int(msg.data)
        with self._lock:
            sent_from_gui = self._manual_controls.sig_recog_last_sent_at is not None
            confirmed = sent_from_gui and self._manual_controls.sig_recog_value == value
            self._manual_controls = replace(
                self._manual_controls,
                sig_recog_value=value,
                input_source='gui' if confirmed else 'external',
                send_result='confirmed' if confirmed else self._manual_controls.send_result,
            )
        self.freshness.mark_received('sig_recog')

    def update_road_blocked(self, msg: Any) -> None:
        """`std_msgs/Bool`（`road_blocked`）の現在値を反映する。

        道路封鎖はGUIだけでなく `road_blockage_detector` も送信するため、入力元を
        `road_blocked_source` として区別する（screen_function_design.md 12.2節）。
        """

        value = bool(msg.data)
        with self._lock:
            sent_from_gui = self._manual_controls.road_blocked_last_sent_at is not None
            confirmed = sent_from_gui and self._manual_controls.road_blocked_value == value
            self._manual_controls = replace(
                self._manual_controls,
                road_blocked_value=value,
                road_blocked_source='gui' if confirmed else 'external',
                send_result='confirmed' if confirmed else self._manual_controls.send_result,
            )
        self.freshness.mark_received('road_blocked')

    def update_obstacle_hint(self, msg: Any) -> None:
        """`tc_route_msgs/ObstacleAvoidanceHint`（`obstacle_avoidance_hint`）を反映する。"""

        with self._lock:
            self._obstacle_state = obstacle_view_from_hint_msg(msg)
        self.freshness.mark_received('obstacle_hint')

    def update_pose_enu(self, msg: Any) -> None:
        """`geometry_msgs/PoseWithCovarianceStamped`（`localization/pose_enu`）を反映する。

        `pose_llh` が別途届いている場合の緯度経度は保持したまま、ENU由来の
        フィールドだけを更新する。
        """

        partial = localization_view_from_pose_enu_msg(msg)
        with self._lock:
            self._localization_state = replace(
                self._localization_state,
                source='pose_enu',
                x_m=partial.x_m,
                y_m=partial.y_m,
                z_m=partial.z_m,
                yaw_deg=partial.yaw_deg,
                frame_id=partial.frame_id,
            )
        self.freshness.mark_received('localization.pose_enu')

    def update_pose_llh(self, msg: Any) -> None:
        """`tc_geo_msgs/GeoPoseWithQuality`（`localization/pose_llh`）を反映する。

        `pose_enu` が別途届いている場合のENU座標は保持したまま、緯度経度
        フィールドだけを更新する。
        """

        partial = localization_view_from_pose_llh_msg(msg)
        with self._lock:
            updates: Dict[str, Any] = {
                'source': 'pose_llh',
                'latitude': partial.latitude,
                'longitude': partial.longitude,
                'altitude': partial.altitude,
            }
            if partial.yaw_deg is not None:
                updates['yaw_deg'] = partial.yaw_deg
            self._localization_state = replace(self._localization_state, **updates)
        self.freshness.mark_received('localization.pose_llh')

    def update_gps_status(self, msg: Any) -> None:
        """`rtk_gps_um982_msgs/RtkStatus`（`rtk_gps/.../rtk_status`）を反映する。"""

        with self._lock:
            self._gps_state = gps_view_from_rtk_status_msg(msg)
        self.freshness.mark_received('gps')

    def update_active_target(self, msg: Any) -> None:
        """`geometry_msgs/PoseStamped`（`active_target`）を反映する。

        目標までの距離は自己位置（ENU）との差分から算出する。到達判定
        しきい値（`arrival_threshold_m`）は現状のtopicからは得られないため
        `None`のままとする。
        """

        partial = target_view_from_pose_msg(msg)
        with self._lock:
            current = self._localization_state
            distance_m = partial.distance_m
            if current.x_m is not None and current.y_m is not None:
                distance_m = euclidean_distance(
                    (current.x_m, current.y_m, current.z_m or 0.0),
                    (partial.x_m, partial.y_m, partial.z_m),
                )
            # 緯度経度は `route/active_target_llh`（geo_pose_converterが変換）が正本の
            # ため、ENU側の更新で消さずに保持する。
            previous = self._target_state
            self._target_state = replace(
                partial,
                distance_m=distance_m,
                target_id=previous.target_id,
                latitude=previous.latitude,
                longitude=previous.longitude,
                altitude=previous.altitude,
                bearing_deg=previous.bearing_deg,
            )
        self.freshness.mark_received('target')

    def update_active_target_llh(self, msg: Any) -> None:
        """`tc_route_msgs/ActiveTargetLlh`（`route/active_target_llh`）を反映する。

        地図描画に必要な目標の緯度経度は、ENU→LLH変換を担う geo_pose_converter
        （`route_geo_projector_node`）が配信する本topicから受け取る。
        """

        with self._lock:
            self._target_state = apply_active_target_llh_msg(self._target_state, msg)
        self.freshness.mark_received('target')

    # ---------- Snapshot生成 ----------
    def build_snapshot(self) -> ConsoleSnapshot:
        """現在の内部状態から `ConsoleSnapshot` を組み立てる。

        `freshness`系フィールドは、更新時刻を焼き込むのではなく本メソッド
        呼び出し時点で`FreshnessMonitor.evaluate()`を用いて評価する
        （経過時間は「最後に更新されてからどれだけ経ったか」で決まるため）。
        """

        now = _utc_now()
        with self._lock:
            launch_profiles = dict(self._launch_states)
            environment = self._environment
            drive_mode_selection = self._drive_mode_selection
            gps_state = self._gps_state
            localization_state = self._localization_state
            route_state = self._route_state
            target_state = self._target_state
            follower_state = self._follower_state
            obstacle_state = self._obstacle_state
            drive_mode_state = self._drive_mode_state
            manual_controls = self._manual_controls
            sensor_panel_refs = dict(self._sensor_panel_refs)

        sensor_panels = [
            replace(ref, freshness=self.freshness.evaluate(f'image.{panel_id}', now=now))
            for panel_id, ref in sensor_panel_refs.items()
        ]

        localization_key = f'localization.{localization_state.source}'
        localization_state = replace(
            localization_state,
            freshness=self.freshness.evaluate(localization_key, now=now),
        )
        gps_freshness = self.freshness.evaluate('gps', now=now)
        gps_state = replace(
            gps_state,
            fix_freshness=gps_freshness,
            heading_freshness=gps_freshness,
            status_freshness=gps_freshness,
        )
        obstacle_state = replace(
            obstacle_state,
            freshness=self.freshness.evaluate('obstacle_hint', now=now),
        )
        target_state = replace(
            target_state,
            freshness=self.freshness.evaluate('target', now=now),
        )
        drive_mode_state = replace(
            drive_mode_state,
            cmd_vel_freshness=self.freshness.evaluate('cmd_vel', now=now),
            odom_freshness=self.freshness.evaluate('odom', now=now),
        )

        # 運行フェーズは単独のtopicでは表せず、起動状態・route/follower・drive mode・
        # manual_startの組み合わせから決まるため、Snapshot生成時にまとめて算出する。
        operation_state = build_operation_state(
            environment=environment,
            drive_mode_selection=drive_mode_selection,
            launch_states=launch_profiles,
            route=route_state,
            follower=follower_state,
            drive_mode=drive_mode_state,
            manual_start=manual_controls.manual_start_value,
            route_freshness=self.freshness.evaluate('route_state', now=now),
            follower_freshness=self.freshness.evaluate('follower_state', now=now),
            now=now,
        )

        event_banners = build_event_banners(
            launch_states=launch_profiles,
            route=route_state,
            follower=follower_state,
            obstacle=obstacle_state,
            manual_controls=manual_controls,
            operation_phase=operation_state.phase,
            lost_topics=self._collect_lost_topics(now),
            now=now,
        )

        log_paths: Dict[str, Optional[str]] = {}
        for profile in self._profiles:
            path = self.launch_manager.get_latest_log_path(profile.profile_id)
            log_paths[profile.profile_id] = str(path) if path is not None else None

        return ConsoleSnapshot(
            timestamp=now,
            operation_state=operation_state,
            gps_state=gps_state,
            localization_state=localization_state,
            route_state=route_state,
            target_state=target_state,
            follower_state=follower_state,
            obstacle_state=obstacle_state,
            drive_mode_state=drive_mode_state,
            sensor_panels=sensor_panels,
            event_banners=event_banners,
            manual_controls=manual_controls,
            launch_profiles=launch_profiles,
            logs=self.log_manager.snapshot_all(),
            log_paths=log_paths,
            health=self._build_health_summaries(launch_profiles),
        )

    # Eventカードの `topic lost` 判定対象。走行判断に直結し、途絶に気付かないと
    # 古い値を現在値と誤認する運行系topicに限定する（6.7節）。
    _LOST_WATCH_TOPICS: Dict[str, str] = {
        'route_state': '/route_state',
        'follower_state': '/follower_state',
        'localization.pose_enu': '/localization/pose_enu',
        'localization.pose_llh': '/localization/pose_llh',
        'cmd_vel': '/cmd_vel',
        'drive_mode_status': '/drive_mode_status',
        'gps': '/rtk_gps/rtk_status',
    }

    def _collect_lost_topics(self, now: datetime) -> Dict[str, float]:
        """`LOST` 判定になった監視対象topicと、その経過秒を返す。

        一度も受信していないtopic（`UNKNOWN`）は「未使用の構成」と区別できない
        ため対象にしない。受信実績があるのに途絶えたものだけを通知する。
        """

        lost: Dict[str, float] = {}
        for key, display_name in self._LOST_WATCH_TOPICS.items():
            if self.freshness.evaluate(key, now=now) != FreshnessLevel.LOST:
                continue
            elapsed = self.freshness.elapsed_seconds(key, now=now)
            if elapsed is not None:
                lost[display_name] = elapsed
        return lost

    def _build_health_summaries(
        self, launch_profiles: Dict[str, LaunchProfileState]
    ) -> List[HealthSummaryView]:
        summaries: List[HealthSummaryView] = []
        for profile in self._profiles:
            state = launch_profiles.get(profile.profile_id)
            if state is None:
                continue
            status_name = 'STOPPED' if state.status == NodeLaunchStatus.STOPPING else state.status.name
            summaries.append(
                HealthSummaryView(
                    profile_id=profile.profile_id,
                    category=profile.category,
                    status=status_name,
                    health=_LAUNCH_STATUS_TO_HEALTH.get(state.status, FreshnessLevel.UNKNOWN),
                    required_but_not_selected=False,
                )
            )
        return summaries
