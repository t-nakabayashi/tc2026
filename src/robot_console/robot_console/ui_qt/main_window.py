"""PyQt5ローカルUIのメインウィンドウ。"""

from __future__ import annotations

from typing import List, Optional

from PyQt5 import QtWidgets

from ..core.console_core import ConsoleCore
from ..core.snapshot_model import ConsoleSnapshot
from .console_log_tab import ConsoleLogTab
from .dashboard_tab import DashboardTab
from .launch_settings_tab import LaunchSettingsTab
from .localization_sensor_tab import LocalizationSensorTab
from .widgets.scaled_canvas import ScaledCanvas
from .widgets.typography import BASE_FONT_POINT_SIZE

WINDOW_TITLE = 'robot_console (PyQt5)'
DEFAULT_WINDOW_WIDTH = 1280
DEFAULT_WINDOW_HEIGHT = 720

TAB_TITLE_DASHBOARD = 'ダッシュボード'
TAB_TITLE_LOCALIZATION_SENSOR = '自己位置・センサ情報'
TAB_TITLE_LAUNCH_SETTINGS = '起動・設定'
TAB_TITLE_CONSOLE_LOG = 'コンソールログ'


class MainWindow(QtWidgets.QMainWindow):
    """4タブ構成のPyQt5メインウィンドウ。

    robot_console_gui_screen_function_design.md 2章の方針に従い、全タブ共通の
    上部ステータスバーは設けない。タブ内容はダッシュボードタブを既定表示とし、
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
        self.localization_sensor_tab = LocalizationSensorTab()
        self.launch_settings_tab = LaunchSettingsTab()
        self.console_log_tab = ConsoleLogTab()

        self.tab_widget = QtWidgets.QTabWidget()
        self.tab_widget.addTab(self.dashboard_tab, TAB_TITLE_DASHBOARD)
        self.tab_widget.addTab(self.localization_sensor_tab, TAB_TITLE_LOCALIZATION_SENSOR)
        self.tab_widget.addTab(self.launch_settings_tab, TAB_TITLE_LAUNCH_SETTINGS)
        self.tab_widget.addTab(self.console_log_tab, TAB_TITLE_CONSOLE_LOG)
        self.tab_widget.setCurrentWidget(self.dashboard_tab)

        self.setCentralWidget(ScaledCanvas(self.tab_widget))

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

    def update_snapshot(self, snapshot: ConsoleSnapshot) -> None:
        """`ConsoleSnapshot` を各タブへ配布する（QTimer駆動でConsoleCoreから呼ばれる）。"""

        self.dashboard_tab.update_snapshot(snapshot)
        self.localization_sensor_tab.update_snapshot(snapshot)
        self.console_log_tab.update_snapshot(snapshot)
        self.launch_settings_tab.update_launch_states(snapshot.launch_profiles)

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
