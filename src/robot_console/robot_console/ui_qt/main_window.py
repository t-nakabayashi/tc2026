"""PyQt5ローカルUIのメインウィンドウ。"""

from __future__ import annotations

from typing import List, Optional

from PyQt5 import QtWidgets

from ..core.console_core import ConsoleCore
from ..core.snapshot_model import ConsoleSnapshot
from ..utils import NodeLaunchStatus
from .console_log_tab import ConsoleLogTab
from .dashboard_tab import DashboardTab
from .gnss_tab import GnssTab
from .launch_settings_tab import LaunchSettingsTab
from .localization_sensor_tab import LocalizationSensorTab
from .widgets.scaled_canvas import ScaledCanvas
from .widgets.clock_sync_warning import ClockSyncWarning
from .widgets.typography import BASE_FONT_POINT_SIZE

WINDOW_TITLE = 'robot_console (PyQt5)'
DEFAULT_WINDOW_WIDTH = 1280
DEFAULT_WINDOW_HEIGHT = 720

TAB_TITLE_DASHBOARD = 'ダッシュボード'
TAB_TITLE_LOCALIZATION_SENSOR = '自己位置・センサ情報'
TAB_TITLE_LAUNCH_SETTINGS = '起動・設定'
TAB_TITLE_CONSOLE_LOG = 'コンソールログ'


class MainWindow(QtWidgets.QMainWindow):
    """5タブ構成のPyQt5メインウィンドウ。

    robot_console_gui_screen_function_design.md 2章の方針に従い、全タブ共通の
    常設の上部ステータスバーは設けない。時刻同期警告は異常時のみ全タブ上部に表示する。タブ内容はダッシュボードタブを既定表示とし、
    アプリ内コンテンツ領域全体を16:9の論理キャンバスとして拡縮する。
    """

    def __init__(
        self,
        parent: Optional[QtWidgets.QWidget] = None,
        *,
        core: Optional[ConsoleCore] = None,
    ) -> None:
        super().__init__(parent)
        self._core = core
        self._apply_base_font()
        self.setWindowTitle(WINDOW_TITLE)
        self.resize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT)

        self.dashboard_tab = DashboardTab()
        # 画像パネルはメタ情報を `ConsoleSnapshot` から、画像本体を `ImageStore` から
        # 取得する2系統構成のため、`ConsoleCore` が保持する `ImageStore` をそのまま
        # 渡す。渡さない場合はタブ側が空の `ImageStore` を生成し、画像を受信しても
        # 全パネルが `No Image` のままになる。
        self.localization_sensor_tab = LocalizationSensorTab(
            image_store=core.image_store if core is not None else None
        )
        self.launch_settings_tab = LaunchSettingsTab()
        self.console_log_tab = ConsoleLogTab()
        self.gnss_tab = GnssTab()

        self.tab_widget = QtWidgets.QTabWidget()
        self.tab_widget.addTab(self.dashboard_tab, TAB_TITLE_DASHBOARD)
        self.tab_widget.addTab(self.localization_sensor_tab, TAB_TITLE_LOCALIZATION_SENSOR)
        self.tab_widget.addTab(self.launch_settings_tab, TAB_TITLE_LAUNCH_SETTINGS)
        self.tab_widget.addTab(self.console_log_tab, TAB_TITLE_CONSOLE_LOG)
        self.tab_widget.addTab(self.gnss_tab, 'GNSS・基地局')
        self.recording_tab = QtWidgets.QWidget()
        recording_layout = QtWidgets.QVBoxLayout(self.recording_tab)
        recording_layout.setContentsMargins(36, 36, 36, 36)
        guide = QtWidgets.QLabel('① 起動・設定で「実機／手動走行」を適用し、場所を選択\n② ダッシュボードで一斉起動\n③ 融合位置の受信後に「ルート記録開始」→ 手動走行 →「終了・保存」')
        guide.setWordWrap(True)
        recording_layout.addWidget(guide)
        recording_layout.addWidget(self.dashboard_tab.survey_card)
        recording_layout.addStretch(1)
        self.tab_widget.addTab(self.recording_tab, 'ルート記録')
        self.tab_widget.setCurrentWidget(self.dashboard_tab)

        self.clock_sync_warning = ClockSyncWarning()
        self.clock_sync_warning.set_environment(self.launch_settings_tab.environment)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)
        content_layout.addWidget(self.clock_sync_warning)
        content_layout.addWidget(self.tab_widget, 1)
        self.setCentralWidget(ScaledCanvas(content))

        self.dashboard_tab.node_health_card.profile_selected.connect(
            self._on_node_health_profile_selected
        )
        self.launch_settings_tab.plan_changed.connect(self._on_launch_plan_changed)
        self.launch_settings_tab.business_mode_changed.connect(self._on_business_mode_changed)
        self.dashboard_tab.launch_control_card.apply_preset_requested.connect(
            self.launch_settings_tab.apply_preset
        )
        self._on_launch_plan_changed()

        if self._core is not None:
            self.dashboard_tab.survey_card.command_requested.connect(self._core.send_survey_command)
            self.dashboard_tab.bag_card.start_requested.connect(self._start_bag)
            self.dashboard_tab.bag_card.stop_requested.connect(self._stop_bag)
            self.dashboard_tab.launch_control_card.launch_requested.connect(
                self._core.request_launch
            )
            self.dashboard_tab.launch_control_card.stop_requested.connect(
                self._core.request_stop
            )
            self.dashboard_tab.launch_control_card.launch_all_requested.connect(
                self._on_launch_all_requested
            )
            self.dashboard_tab.launch_control_card.stop_all_requested.connect(
                self._on_stop_all_requested
            )
            self.dashboard_tab.manual_ops_card.manual_start_requested.connect(
                self._core.send_manual_start
            )
            self.dashboard_tab.manual_ops_card.sig_recog_requested.connect(
                self._core.send_sig_recog
            )
            self.dashboard_tab.manual_ops_card.road_blocked_requested.connect(
                self._core.send_road_blocked
            )
            self.dashboard_tab.manual_ops_card.obstacle_hint_override_requested.connect(
                self._core.send_obstacle_hint_override
            )
            self.dashboard_tab.manual_ops_card.obstacle_hint_stop_requested.connect(
                self._core.send_obstacle_hint_stop
            )
            self.dashboard_tab.manual_ops_card.frame_image_requested.connect(
                self._core.send_frame_image_request
            )
            self.launch_settings_tab.param_changed.connect(
                lambda profile_id, text: self._core.update_selected_param(
                    profile_id, text or None
                )
            )
            self.launch_settings_tab.alternate_toggled.connect(
                self._core.update_use_alternate_launch
            )
            self.launch_settings_tab.simulator_toggled.connect(
                self._core.update_simulator_enabled
            )
            self.launch_settings_tab.argument_changed.connect(
                self._core.update_launch_override
            )
            # 起動時点のコンボ選択値（既定値）もConsoleCoreへ反映しておく。
            # 以降の変更は business_mode_changed シグナル経由で伝わる。
            self._core.update_business_mode(
                self.launch_settings_tab.environment, self.launch_settings_tab.drive_mode
            )

    def _start_bag(self, directory: str) -> None:
        self._core.bag_recorder.start(directory)
        self.dashboard_tab.bag_card.update_state(self._core.bag_recorder.snapshot())

    def _stop_bag(self) -> None:
        self._core.bag_recorder.stop()
        self.dashboard_tab.bag_card.update_state(self._core.bag_recorder.snapshot())

    def update_snapshot(self, snapshot: ConsoleSnapshot) -> None:
        """`ConsoleSnapshot` を各タブへ配布する（QTimer駆動でConsoleCoreから呼ばれる）。"""

        self.dashboard_tab.update_snapshot(snapshot)
        self.gnss_tab.update_snapshot(snapshot)
        self.localization_sensor_tab.update_snapshot(snapshot)
        self.console_log_tab.update_snapshot(snapshot)
        self.launch_settings_tab.update_launch_states(snapshot.launch_profiles)
        active = [pid for pid, state in snapshot.launch_profiles.items()
                  if state.status in (NodeLaunchStatus.RUNNING, NodeLaunchStatus.STARTING,
                                      NodeLaunchStatus.STOPPING)]
        if active != getattr(self, '_active_profile_ids', []):
            self._active_profile_ids = active
            self._on_launch_plan_changed()

    def _on_launch_all_requested(self, profile_ids: List[str]) -> None:
        """起動予定ノード一覧（プラン）の一括起動要求を反映する。"""

        if self._core is None:
            return
        for profile_id in profile_ids:
            self._core.request_launch(profile_id)

    def _on_stop_all_requested(self, profile_ids: List[str]) -> None:
        """起動予定ノード一覧（プラン）の一括停止要求を反映する。"""

        if self._core is None:
            return
        for profile_id in profile_ids:
            self._core.request_stop(profile_id)

    def _on_business_mode_changed(self, environment: str, drive_mode: str) -> None:
        """起動・設定タブの業務モード選択を、ConsoleCoreと起動操作カードへ反映する。"""

        self.clock_sync_warning.set_environment(environment)
        if self._core is not None:
            self._core.update_business_mode(environment, drive_mode)
        self._on_launch_plan_changed()

    def _on_node_health_profile_selected(self, profile_id: str) -> None:
        """Node Healthカードのチップ選択を受け、コンソールログタブへ遷移する（9章 画面間導線）。"""

        self.console_log_tab.select_profile(profile_id)
        self.tab_widget.setCurrentWidget(self.console_log_tab)

    def _on_launch_plan_changed(self) -> None:
        """起動・設定タブの起動予定ノード一覧を、ダッシュボードの起動操作カードへ反映する。"""

        self.dashboard_tab.launch_control_card.update_plan(
            environment=self.launch_settings_tab.environment,
            drive_mode=self.launch_settings_tab.drive_mode,
            ordered_profile_ids=list(self.launch_settings_tab.plan.ordered_profile_ids),
            profiles_by_id=self.launch_settings_tab.profiles_by_id,
            active_profile_ids=getattr(self, '_active_profile_ids', []),
        )

    @staticmethod
    def _apply_base_font() -> None:
        """走行中に数m離れた位置からでも判読しやすいよう、既定フォントを拡大する。"""

        app = QtWidgets.QApplication.instance()
        if app is None:
            return
        font = app.font()
        font.setPointSize(BASE_FONT_POINT_SIZE)
        app.setFont(font)
