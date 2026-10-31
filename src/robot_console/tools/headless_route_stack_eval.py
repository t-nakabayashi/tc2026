#!/usr/bin/env python3
"""robot_console の ConsoleCore で route stack を headless 評価する.

GUI を生成せず、起動操作カード・起動設定タブが呼ぶのと同じ `ConsoleCore` の
公開メソッドへ GUI 操作相当の入力を与える。
既存 route stack 回帰用の簡易構成を対象に、route_planner、route_manager、
route_follower、robot_navigator、robot_simulator を起動し、topic の流れを監視する。

GUI あり評価は `qt_route_stack_eval.py` を使う。
"""

from __future__ import annotations

import argparse
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tc_route_msgs.msg import DriveModeStatus, FollowerState, Route, RouteState
from std_msgs.msg import Bool

from robot_console.core.console_core import ConsoleCore
from robot_console.ros.console_node import start_ros_thread
from robot_console.utils import NodeLaunchStatus


DEFAULT_LAUNCH_ORDER = (
    "route_planner",
    "route_manager",
    "route_follower",
    "drive_mode_manager",
    "robot_navigator",
)
STOPPED_STATUSES = {NodeLaunchStatus.STOPPED, NodeLaunchStatus.ERROR}


@dataclass
class EvalConfig:
    """headless 評価の実行設定."""

    start_label: str
    goal_label: str
    route_planner_param: str
    route_manager_param: str
    route_follower_param: str
    robot_navigator_param: str
    timeout_sec: float
    post_goal_wait_sec: float
    startup_wait_sec: float
    stop_timeout_sec: float
    summary_period_sec: float
    cmd_vel_period_sec: float
    simulator: bool
    manual_start: bool
    # drive_mode_manager の走行状態GUIを表示するか。GUIあり評価でのみ意味を持つ。
    show_drive_status_gui: bool
    launch_order: Sequence[str]
    console_log_directory: Optional[str]


class TopicMonitor(Node):
    """route stack の主要 topic を購読し、進行状況を標準出力へ出す."""

    def __init__(self, target_label: str, config: EvalConfig) -> None:
        super().__init__("headless_route_stack_monitor")
        self._target_label = target_label
        self._config = config
        self._start_time = time.monotonic()
        self.goal_reached_time: Optional[float] = None
        self.last_route_label: Optional[str] = None
        self.last_follower_label: Optional[str] = None
        self._last_cmd_print = 0.0
        self.route_state_count = 0
        self.follower_state_count = 0
        self.cmd_vel_count = 0
        self.autonomous_cmd_vel_count = 0
        self.drive_mode_status_count = 0
        self.active_route_count = 0
        self.manual_start_count = 0

        self.create_subscription(RouteState, "/route_state", self._on_route_state, 10)
        self.create_subscription(Route, "/active_route", self._on_active_route, 10)
        self.create_subscription(
            FollowerState, "/follower_state", self._on_follower_state, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(
            Twist, "/cmd_vel/autonomous", self._on_autonomous_cmd_vel, 10
        )
        self.create_subscription(
            DriveModeStatus, "/drive_mode_status", self._on_drive_mode_status, 10
        )
        self.create_subscription(Bool, "/manual_start", self._on_manual_start, 10)
        self.create_timer(config.summary_period_sec, self._on_summary)

    def elapsed(self) -> float:
        """評価開始からの経過秒を返す."""

        return time.monotonic() - self._start_time

    def _on_route_state(self, msg: RouteState) -> None:
        self.route_state_count += 1
        label = msg.current_label
        if label != self.last_route_label:
            self.last_route_label = label
            print(
                f"[monitor] {self.elapsed():6.1f}s /route_state "
                f"current_label={label!r} status={msg.status} "
                f"version={msg.route_version}",
                flush=True,
            )
        if label == self._target_label and self.goal_reached_time is None:
            self.goal_reached_time = time.monotonic()
            print(
                f"[monitor] {self.elapsed():6.1f}s goal label "
                f"{self._target_label!r} reached by /route_state",
                flush=True,
            )

    def _on_active_route(self, msg: Route) -> None:
        self.active_route_count += 1
        labels = [wp.label for wp in msg.waypoints]
        head = labels[0] if labels else ""
        tail = labels[-1] if labels else ""
        print(
            f"[monitor] {self.elapsed():6.1f}s /active_route "
            f"waypoints={len(labels)} start={head!r} goal={tail!r} "
            f"version={msg.version}",
            flush=True,
        )

    def _on_follower_state(self, msg: FollowerState) -> None:
        self.follower_state_count += 1
        label = msg.active_waypoint_label
        if label != self.last_follower_label:
            self.last_follower_label = label
            print(
                f"[monitor] {self.elapsed():6.1f}s /follower_state "
                f"active_label={label!r} state={msg.state} "
                f"index={msg.active_waypoint_index}",
                flush=True,
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel_count += 1
        now_time = time.monotonic()
        if now_time - self._last_cmd_print >= self._config.cmd_vel_period_sec:
            self._last_cmd_print = now_time
            print(
                f"[monitor] {self.elapsed():6.1f}s /cmd_vel "
                f"linear.x={msg.linear.x:.3f} angular.z={msg.angular.z:.3f}",
                flush=True,
            )

    def _on_autonomous_cmd_vel(self, msg: Twist) -> None:
        self.autonomous_cmd_vel_count += 1

    def _on_drive_mode_status(self, msg: DriveModeStatus) -> None:
        self.drive_mode_status_count += 1
        if self.drive_mode_status_count == 1 or msg.auto_resume_pending:
            print(
                f"[monitor] {self.elapsed():6.1f}s /drive_mode_status "
                f"mode={msg.mode} source={msg.output_source} reason={msg.reason!r}",
                flush=True,
            )

    def _on_manual_start(self, msg: Bool) -> None:
        self.manual_start_count += 1
        print(
            f"[monitor] {self.elapsed():6.1f}s /manual_start data={msg.data}",
            flush=True,
        )

    def _on_summary(self) -> None:
        print(
            f"[monitor] {self.elapsed():6.1f}s counts "
            f"route_state={self.route_state_count} "
            f"active_route={self.active_route_count} "
            f"follower_state={self.follower_state_count} "
            f"cmd_vel={self.cmd_vel_count} "
            f"cmd_vel_autonomous={self.autonomous_cmd_vel_count} "
            f"drive_mode_status={self.drive_mode_status_count} "
            f"manual_start={self.manual_start_count}",
            flush=True,
        )


class HeadlessRouteStackEvaluator:
    """ConsoleCore 操作と ROS topic 監視をまとめる実行器."""

    def __init__(self, config: EvalConfig) -> None:
        self._config = config
        self._core = ConsoleCore(log_directory=config.console_log_directory)
        self._ros_handle = start_ros_thread(
            self._core, node_name='robot_console_headless_eval'
        )
        self._monitor = TopicMonitor(config.goal_label, config)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._monitor)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._seen_logs: dict[str, int] = {}

    def run(self) -> int:
        """評価を実行し、終了コードを返す."""

        self._thread.start()
        try:
            self._configure_launch_profiles()
            self._launch_profiles()
            if self._config.manual_start:
                print("[test] manual_start=True request", flush=True)
                self._core.send_manual_start(True)
            goal_reached = self._wait_for_goal()
            stop_ok = self._stop_profiles()
            self._drain_console_logs()
            if not goal_reached:
                print(
                    f"[test] timeout waiting for goal label "
                    f"{self._config.goal_label!r}",
                    flush=True,
                )
                return 2
            if not stop_ok:
                print("[test] launched profiles did not stop cleanly", flush=True)
                return 3
            print("[test] goal reached and shutdown completed", flush=True)
            return 0
        finally:
            self._executor.shutdown()
            self._monitor.destroy_node()
            self._thread.join(timeout=2.0)
            self._ros_handle.stop()

    def _configure_launch_profiles(self) -> None:
        core = self._core
        self._select_param("route_planner", self._config.route_planner_param)
        self._select_param("route_manager", self._config.route_manager_param)
        self._select_param("route_follower", self._config.route_follower_param)
        self._select_param("robot_navigator", self._config.robot_navigator_param)
        core.update_launch_override("route_manager", "start_label", self._config.start_label)
        core.update_launch_override("route_manager", "goal_label", self._config.goal_label)
        core.update_launch_override(
            "drive_mode_manager",
            "start_gui",
            "true" if self._config.show_drive_status_gui else "false",
        )
        core.update_launch_override("drive_mode_manager", "joy_input", "joy_node")
        core.update_launch_override("robot_navigator", "cmd_vel_topic", "/cmd_vel/autonomous")
        core.update_launch_override("robot_navigator", "odom_topic", "/ypspur_ros/odom")
        core.update_launch_override("robot_navigator", "pose_enu_topic", "/localization/pose_enu")
        core.update_simulator_enabled("robot_navigator", self._config.simulator)
        self._pump_for(1.0)
        self._print_launch_selection()

    def _select_param(self, profile_id: str, requested: str) -> None:
        self._core.update_selected_param(
            profile_id, self._resolve_param_path(profile_id, requested)
        )

    def _resolve_param_path(self, profile_id: str, requested: str) -> str:
        """`inagi.yaml` のようなファイル名指定を、profile の既定パラメータと同じ
        ディレクトリ配下のパスへ解決する。既にパス区切りを含む場合はそのまま使う
        （`qt_route_stack_eval.py` と同じ扱い）。"""

        if "/" in requested:
            return requested
        profile = self._core._profile_store.get(profile_id)
        if profile is None:
            raise RuntimeError(f"profile が見つかりません: {profile_id}")
        if not profile.default_param:
            return requested
        default_dir = str(Path(profile.default_param).parent)
        if default_dir in ("", "."):
            return requested
        return f"{default_dir}/{requested}"

    def _print_launch_selection(self) -> None:
        snapshot = self._core.build_snapshot()
        for profile_id in self._config.launch_order:
            state = snapshot.launch_profiles.get(profile_id)
            if state is None:
                continue
            print(
                f"[test] profile={profile_id} param={state.selected_param!r} "
                f"simulator={state.simulator_enabled} overrides={state.override_inputs}",
                flush=True,
            )

    def _launch_profiles(self) -> None:
        for profile_id in self._config.launch_order:
            print(f"[test] launch request: {profile_id}", flush=True)
            self._core.request_launch(profile_id)
            self._pump_for(self._config.startup_wait_sec)

    def _wait_for_goal(self) -> bool:
        deadline = time.monotonic() + self._config.timeout_sec
        while time.monotonic() < deadline:
            self._pump_for(0.5)
            if self._monitor.goal_reached_time is not None:
                self._pump_for(self._config.post_goal_wait_sec)
                return True
        return False

    def _stop_profiles(self) -> bool:
        print("[test] stopping launched profiles", flush=True)
        # ConsoleCore は profile 単位の停止のみを公開するため、起動順の逆順で停止する。
        for profile_id in reversed(tuple(self._config.launch_order)):
            self._core.request_stop(profile_id)
        deadline = time.monotonic() + self._config.stop_timeout_sec
        while time.monotonic() < deadline:
            self._pump_for(0.5)
            if self._all_launch_profiles_stopped():
                self._print_remaining_launch_states()
                return True
        self._print_remaining_launch_states()
        return False

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
                f"[test] stop state: {profile_id} status={state.status.name} "
                f"pid={state.process_id} sim_pid={state.simulator_process_id} "
                f"error={state.error_message!r}",
                flush=True,
            )

    def _pump_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._drain_console_logs()
            time.sleep(0.2)

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
            "drive mode",
            "Using",
            "generated temporary yaml",
        )
        for profile_id, lines in snapshot.logs.items():
            start = self._seen_logs.get(profile_id, 0)
            for line in lines[start:]:
                if any(key in line for key in keywords):
                    print(f"[console:{profile_id}] {line}", flush=True)
            self._seen_logs[profile_id] = len(lines)


def positive_float(value: str) -> float:
    """正の float 引数を検証する."""

    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("0 より大きい値を指定してください")
    return parsed


def parse_launch_order(value: str) -> Sequence[str]:
    """カンマ区切りの profile ID 一覧を返す."""

    profiles = tuple(item.strip() for item in value.split(",") if item.strip())
    if not profiles:
        raise argparse.ArgumentTypeError("1 件以上の profile ID を指定してください")
    return profiles


def build_arg_parser() -> argparse.ArgumentParser:
    """CLI 引数パーサを構築する."""

    parser = argparse.ArgumentParser(
        description="robot_console ConsoleCore を使う headless route stack 評価ツール"
    )
    parser.add_argument("--start-label", default="10", help="route_manager の開始ラベル")
    parser.add_argument("--goal-label", default="30", help="route_manager の終了ラベル")
    parser.add_argument(
        "--route-planner-param",
        default="inagi.yaml",
        help="route_planner のパラメータ表示名または YAML パス",
    )
    parser.add_argument(
        "--route-manager-param",
        default="inagi.yaml",
        help="route_manager のパラメータ表示名または YAML パス",
    )
    parser.add_argument(
        "--route-follower-param",
        default="default.yaml",
        help="route_follower のパラメータ表示名または YAML パス",
    )
    parser.add_argument(
        "--robot-navigator-param",
        default="default.yaml",
        help="robot_navigator のパラメータ表示名または YAML パス",
    )
    parser.add_argument(
        "--timeout-sec",
        type=positive_float,
        default=180.0,
        help="goal 到達待ちの上限秒",
    )
    parser.add_argument(
        "--post-goal-wait-sec",
        type=positive_float,
        default=10.0,
        help="goal 到達後に追加監視する秒数",
    )
    parser.add_argument(
        "--startup-wait-sec",
        type=positive_float,
        default=3.0,
        help="各 profile 起動要求後に待つ秒数",
    )
    parser.add_argument(
        "--stop-timeout-sec",
        type=positive_float,
        default=20.0,
        help="停止要求後に各 profile の停止を待つ秒数",
    )
    parser.add_argument(
        "--summary-period-sec",
        type=positive_float,
        default=5.0,
        help="topic 受信数サマリの出力周期",
    )
    parser.add_argument(
        "--cmd-vel-period-sec",
        type=positive_float,
        default=5.0,
        help="/cmd_vel の代表値出力周期",
    )
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
    parser.add_argument(
        "--show-drive-status-gui",
        action="store_true",
        help="drive_mode_manager の走行状態GUIを表示する",
    )
    parser.add_argument(
        "--no-simulator",
        action="store_true",
        help="robot_navigator の simulator 同時起動を無効にする",
    )
    parser.add_argument(
        "--no-manual-start",
        action="store_true",
        help="起動後の manual_start=True 送信を省略する",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> EvalConfig:
    """argparse の結果から EvalConfig を作成する."""

    return EvalConfig(
        start_label=args.start_label,
        goal_label=args.goal_label,
        route_planner_param=args.route_planner_param,
        route_manager_param=args.route_manager_param,
        route_follower_param=args.route_follower_param,
        robot_navigator_param=args.robot_navigator_param,
        timeout_sec=args.timeout_sec,
        post_goal_wait_sec=args.post_goal_wait_sec,
        startup_wait_sec=args.startup_wait_sec,
        stop_timeout_sec=args.stop_timeout_sec,
        summary_period_sec=args.summary_period_sec,
        cmd_vel_period_sec=args.cmd_vel_period_sec,
        simulator=not args.no_simulator,
        manual_start=not args.no_manual_start,
        show_drive_status_gui=getattr(args, "show_drive_status_gui", False),
        launch_order=args.launch_order,
        console_log_directory=args.console_log_directory,
    )


def main(argv: Optional[Iterable[str]] = None) -> int:
    """CLI エントリポイント."""

    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = config_from_args(args)
    rclpy.init()
    evaluator = None
    try:
        evaluator = HeadlessRouteStackEvaluator(config)
        return evaluator.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
