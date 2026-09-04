#!/usr/bin/env python3
"""robot_console 次期PyQt5 UI（ConsoleCore ⇔ Snapshot ⇔ MainWindow）の
route stack 結合評価ツール。

`gui_route_stack_eval.py` が旧tkinter版 `GuiCore`/`UiMain` を対象とするのに対し、
本ツールは次期UIのROS統合実装（`ros/console_node.py::RobotConsoleNode` ⇔
`core/console_core.py::ConsoleCore` ⇔ `ui_qt/main_window.py::MainWindow`）を対象と
する。次期UIにはまだ `ui_main.py` の `automation_*` に相当する公開APIが無いため、
実際の起動操作カード・起動設定タブが呼ぶのと同じ `ConsoleCore` の公開メソッド
（`update_selected_param()` 等）と、起動・設定タブの入力ハンドラ
（`LaunchSettingsTab._on_param_path_edited()` 等）を直接呼び出して駆動する。
MainWindow は実際に表示し、Snapshotの反映を目視確認できるようにする。
ローカルデスクトップまたはX11転送が有効な環境での実行を前提とする。
"""

from __future__ import annotations

import argparse
import os
import threading
import time
from pathlib import Path
from typing import Optional, Sequence

import rclpy
from PyQt5 import QtCore, QtWidgets
from rclpy.executors import MultiThreadedExecutor

from headless_route_stack_eval import (
    DEFAULT_LAUNCH_ORDER,
    STOPPED_STATUSES,
    EvalConfig,
    TopicMonitor,
    config_from_args,
    parse_launch_order,
    positive_float,
)
from robot_console.core.console_core import ConsoleCore
from robot_console.ros.console_node import RobotConsoleNode
from robot_console.ui_qt.main_window import MainWindow
from robot_console.ui_qt.qt_environment import (
    enable_qtwebengine_shared_opengl_contexts,
    fix_qt_plugin_path_conflict,
)

SNAPSHOT_POLL_MS = 1000


def _resolve_param_path(core: ConsoleCore, profile_id: str, requested: str) -> str:
    """`tsukuba.yaml` のようなファイル名指定を profile の既定パラメータと同じ
    ディレクトリ配下のパスへ解決する。既にパス区切りを含む場合はそのまま使う。"""

    if "/" in requested:
        return requested
    profile = core._profile_store.get(profile_id)
    if profile is None or not profile.default_param:
        return requested
    default_dir = str(Path(profile.default_param).parent)
    if default_dir in ("", "."):
        return requested
    return f"{default_dir}/{requested}"


class QtRouteStackEvaluator:
    """MainWindow を表示しつつ ConsoleCore 経由で route stack を評価する。"""

    def __init__(self, config: EvalConfig, app: QtWidgets.QApplication) -> None:
        self._config = config
        self._app = app
        self._core = ConsoleCore(log_directory=config.console_log_directory)
        self._ros_node = RobotConsoleNode(self._core, node_name="robot_console_qt_eval")
        self._monitor = TopicMonitor(config.goal_label, config)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._ros_node)
        self._executor.add_node(self._monitor)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)

        self._window = MainWindow(core=self._core)
        self._snapshot_timer = QtCore.QTimer(self._window)
        self._snapshot_timer.timeout.connect(self._on_snapshot_timer)
        self._snapshot_timer.start(SNAPSHOT_POLL_MS)

        self._seen_logs: dict[str, int] = {}
        self._result = 1
        self._goal_reached = False
        self._stop_started_at: Optional[float] = None
        self._timeout_deadline: Optional[float] = None

    def run(self) -> int:
        """GUI 評価を実行し、終了コードを返す。"""

        self._executor_thread.start()
        self._window.show()
        try:
            QtCore.QTimer.singleShot(500, self._configure_launch_profiles)
            self._app.exec_()
            return self._result
        finally:
            self._executor.shutdown()
            self._ros_node.destroy_node()
            self._monitor.destroy_node()
            self._executor_thread.join(timeout=2.0)

    def _configure_launch_profiles(self) -> None:
        launch_settings_tab = self._window.launch_settings_tab
        try:
            self._configure_param("route_planner", self._config.route_planner_param)
            self._configure_param("route_manager", self._config.route_manager_param)
            self._configure_param("route_follower", self._config.route_follower_param)
            self._configure_param("robot_navigator", self._config.robot_navigator_param)
            launch_settings_tab._on_argument_changed(
                "route_manager", "start_label", self._config.start_label
            )
            launch_settings_tab._on_argument_changed(
                "route_manager", "goal_label", self._config.goal_label
            )
            launch_settings_tab._on_argument_changed(
                "drive_mode_manager", "start_gui", "false"
            )
            launch_settings_tab._on_argument_changed(
                "drive_mode_manager", "joy_input", "joy_node"
            )
            launch_settings_tab._on_argument_changed(
                "robot_navigator", "cmd_vel_topic", "/cmd_vel/autonomous"
            )
            launch_settings_tab._on_argument_changed(
                "robot_navigator", "odom_topic", "/ypspur_ros/odom"
            )
            launch_settings_tab._on_argument_changed(
                "robot_navigator", "pose_enu_topic", "/localization/pose_enu"
            )
            launch_settings_tab._on_simulator_toggled(
                "robot_navigator", self._config.simulator
            )
        except Exception as exc:  # pylint: disable=broad-except
            print(f"[qt-test] configure failed: {exc}", flush=True)
            self._result = 4
            self._app.quit()
            return
        QtCore.QTimer.singleShot(500, self._after_configured)

    def _configure_param(self, profile_id: str, requested: str) -> None:
        param_path = _resolve_param_path(self._core, profile_id, requested)
        self._window.launch_settings_tab._on_param_path_edited(profile_id, param_path)

    def _after_configured(self) -> None:
        self._print_launch_selection()
        self._launch_profile_at(0)

    def _print_launch_selection(self) -> None:
        snapshot = self._core.build_snapshot()
        for profile_id in self._config.launch_order:
            state = snapshot.launch_profiles.get(profile_id)
            if state is None:
                continue
            print(
                f"[qt-test] profile={profile_id} "
                f"param={state.selected_param!r} "
                f"simulator={state.simulator_enabled} overrides={state.override_inputs}",
                flush=True,
            )

    def _launch_profile_at(self, index: int) -> None:
        if index >= len(self._config.launch_order):
            if self._config.manual_start:
                print("[qt-test] manual_start=True request", flush=True)
                self._core.send_manual_start(True)
            self._timeout_deadline = time.monotonic() + self._config.timeout_sec
            QtCore.QTimer.singleShot(500, self._poll_goal)
            return
        profile_id = self._config.launch_order[index]
        print(f"[qt-test] launch request: {profile_id}", flush=True)
        self._core.request_launch(profile_id)
        delay_ms = int(self._config.startup_wait_sec * 1000)
        QtCore.QTimer.singleShot(delay_ms, lambda: self._launch_profile_at(index + 1))

    def _poll_goal(self) -> None:
        self._drain_console_logs()
        if self._monitor.goal_reached_time is not None:
            self._goal_reached = True
            delay_ms = int(self._config.post_goal_wait_sec * 1000)
            QtCore.QTimer.singleShot(delay_ms, self._start_stop_sequence)
            return
        assert self._timeout_deadline is not None
        if time.monotonic() >= self._timeout_deadline:
            print(
                f"[qt-test] timeout waiting for goal label {self._config.goal_label!r}",
                flush=True,
            )
            self._result = 2
            self._start_stop_sequence()
            return
        QtCore.QTimer.singleShot(500, self._poll_goal)

    def _start_stop_sequence(self) -> None:
        print("[qt-test] stop request for launched profiles", flush=True)
        self._stop_started_at = time.monotonic()
        for profile_id in reversed(self._config.launch_order):
            self._core.request_stop(profile_id)
        QtCore.QTimer.singleShot(500, self._poll_stopped)

    def _poll_stopped(self) -> None:
        self._drain_console_logs()
        if self._all_launch_profiles_stopped():
            self._print_remaining_launch_states()
            if self._goal_reached and self._result == 1:
                self._result = 0
            print("[qt-test] shutdown completed", flush=True)
            self._app.quit()
            return
        assert self._stop_started_at is not None
        if time.monotonic() - self._stop_started_at >= self._config.stop_timeout_sec:
            self._print_remaining_launch_states()
            print("[qt-test] launched profiles did not stop cleanly", flush=True)
            if self._result == 1:
                self._result = 3
            self._app.quit()
            return
        QtCore.QTimer.singleShot(500, self._poll_stopped)

    def _all_launch_profiles_stopped(self) -> bool:
        snapshot = self._core.build_snapshot()
        for profile_id in self._config.launch_order:
            state = snapshot.launch_profiles.get(profile_id)
            if state is None:
                continue
            if state.status not in STOPPED_STATUSES:
                return False
            if state.process_id is not None or state.simulator_process_id is not None:
                return False
        return True

    def _print_remaining_launch_states(self) -> None:
        snapshot = self._core.build_snapshot()
        for profile_id in self._config.launch_order:
            state = snapshot.launch_profiles.get(profile_id)
            if state is None:
                continue
            print(
                f"[qt-test] stop state: {profile_id} status={state.status.name} "
                f"pid={state.process_id} sim_pid={state.simulator_process_id} "
                f"error={state.error_message!r}",
                flush=True,
            )

    def _drain_console_logs(self) -> None:
        snapshot = self._core.build_snapshot()
        keywords = (
            "ERROR",
            "Error",
            "error",
            "failed",
            "Failed",
            "Traceback",
            "started",
            "Using",
            "generated temporary yaml",
        )
        for profile_id, lines in snapshot.logs.items():
            start = self._seen_logs.get(profile_id, 0)
            for line in lines[start:]:
                if any(key in line for key in keywords):
                    print(f"[console:{profile_id}] {line}", flush=True)
            self._seen_logs[profile_id] = len(lines)

    def _on_snapshot_timer(self) -> None:
        self._window.update_snapshot(self._core.build_snapshot())


def build_arg_parser() -> argparse.ArgumentParser:
    """CLI 引数パーサを構築する。"""

    parser = argparse.ArgumentParser(
        description="robot_console 次期PyQt5 UI（ConsoleCore経由）を使う route stack 評価ツール"
    )
    parser.add_argument("--start-label", default="10", help="route_manager の開始ラベル")
    parser.add_argument("--goal-label", default="30", help="route_manager の終了ラベル")
    parser.add_argument("--route-planner-param", default="tsukuba.yaml")
    parser.add_argument("--route-manager-param", default="tsukuba.yaml")
    parser.add_argument("--route-follower-param", default="default.yaml")
    parser.add_argument("--robot-navigator-param", default="default.yaml")
    parser.add_argument("--timeout-sec", type=positive_float, default=180.0)
    parser.add_argument("--post-goal-wait-sec", type=positive_float, default=10.0)
    parser.add_argument("--startup-wait-sec", type=positive_float, default=3.0)
    parser.add_argument("--stop-timeout-sec", type=positive_float, default=20.0)
    parser.add_argument("--summary-period-sec", type=positive_float, default=5.0)
    parser.add_argument("--cmd-vel-period-sec", type=positive_float, default=5.0)
    parser.add_argument(
        "--console-log-directory",
        default=os.environ.get("ROBOT_CONSOLE_LOG_DIR"),
        help=(
            "robot_console 管理の子プロセス stdout/stderr 保存先。"
            "未指定時は ROBOT_CONSOLE_LOG_DIR を参照します"
        ),
    )
    parser.add_argument(
        "--launch-order",
        type=parse_launch_order,
        default=DEFAULT_LAUNCH_ORDER,
        help="起動する profile ID のカンマ区切り一覧",
    )
    parser.add_argument("--no-simulator", action="store_true")
    parser.add_argument("--no-manual-start", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI エントリポイント。"""

    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if not os.environ.get("DISPLAY"):
        print(
            "[qt-test] DISPLAY が未設定です。"
            "ローカルデスクトップまたは X11 転送ありの環境で"
            "実行してください。",
            flush=True,
        )
        return 4
    config = config_from_args(args)
    rclpy.init()
    try:
        fix_qt_plugin_path_conflict()
        enable_qtwebengine_shared_opengl_contexts()
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        evaluator = QtRouteStackEvaluator(config, app)
        return evaluator.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
