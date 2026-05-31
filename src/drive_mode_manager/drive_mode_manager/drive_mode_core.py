"""走行モード切替と速度指令 mux の ROS 非依存ロジック。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


MODE_AUTONOMOUS = 1
MODE_MANUAL = 2
SOURCE_ZERO = 0
SOURCE_AUTONOMOUS_CMD = 1
SOURCE_MANUAL_CMD = 2


@dataclass(frozen=True)
class DriveModeConfig:
    """走行モード管理の設定。"""

    initial_mode: str = 'autonomous'
    manual_transition_hold_s: float = 2.0
    manual_to_auto_l1_released_s: float = 1.0
    auto_resume_delay_s: float = 5.0
    autonomous_cmd_timeout_s: float = 0.5
    manual_cmd_timeout_s: float = 0.3
    joy_timeout_s: float = 0.5
    l1_button_index: int = 4  # L1 (buttons[4]) デッドマン
    ps_button_index: int = 5  # モード遷移トリガ。実機 PS3 は PS 不可のため R1 (buttons[5])


@dataclass(frozen=True)
class JoySnapshot:
    """状態遷移判定用 Joy 入力。"""

    buttons: Sequence[int]
    stamp_s: float


@dataclass(frozen=True)
class CommandSnapshot:
    """速度指令の最新値。"""

    linear_x: float
    linear_y: float
    angular_z: float
    stamp_s: float


@dataclass(frozen=True)
class DriveModeOutput:
    """走行モード管理の出力。"""

    mode: int
    output_source: int
    linear_x: float
    linear_y: float
    angular_z: float
    joy_available: bool
    l1_pressed: bool
    ps_button_pressed: bool
    ps_hold_progress_s: float
    manual_input_active: bool
    manual_cmd_alive: bool
    autonomous_cmd_alive: bool
    auto_resume_pending: bool
    auto_resume_remaining_s: float
    pending_autonomous_linear_x: float
    pending_autonomous_angular_z: float
    reason: str


class DriveModeCore:
    """自律・手動の状態遷移と最終速度指令を決定する。"""

    def __init__(self, config: DriveModeConfig) -> None:
        self._config = config
        self._mode = MODE_MANUAL if config.initial_mode.lower() == 'manual' else MODE_AUTONOMOUS
        self._manual_hold_start_s: float | None = None
        self._l1_release_start_s: float | None = None
        self._auto_resume_until_s: float | None = None

    @property
    def mode(self) -> int:
        """現在の走行モードを返す。"""

        return self._mode

    def update(
        self,
        now_s: float,
        joy: JoySnapshot | None,
        autonomous_cmd: CommandSnapshot | None,
        manual_cmd: CommandSnapshot | None,
    ) -> DriveModeOutput:
        """現在時刻と最新入力から出力を算出する。"""

        joy_available, l1_pressed, ps_pressed, joy_reason = self._resolve_joy(now_s, joy)
        autonomous_alive = self._cmd_alive(now_s, autonomous_cmd, self._config.autonomous_cmd_timeout_s)
        manual_alive = self._cmd_alive(now_s, manual_cmd, self._config.manual_cmd_timeout_s)
        manual_input_active = manual_alive and manual_cmd is not None and self._is_nonzero(manual_cmd)

        reason = joy_reason
        if self._mode == MODE_AUTONOMOUS:
            reason = self._update_autonomous_transition(now_s, joy_available, l1_pressed, ps_pressed, reason)
        else:
            reason = self._update_manual_transition(now_s, l1_pressed, autonomous_alive, reason)

        return self._select_output(
            now_s=now_s,
            joy_available=joy_available,
            l1_pressed=l1_pressed,
            ps_pressed=ps_pressed,
            autonomous_alive=autonomous_alive,
            manual_alive=manual_alive,
            manual_input_active=manual_input_active,
            autonomous_cmd=autonomous_cmd,
            manual_cmd=manual_cmd,
            reason=reason,
        )

    def _update_autonomous_transition(
        self,
        now_s: float,
        joy_available: bool,
        l1_pressed: bool,
        ps_pressed: bool,
        reason: str,
    ) -> str:
        if joy_available and l1_pressed and ps_pressed:
            if self._manual_hold_start_s is None:
                self._manual_hold_start_s = now_s
            hold_s = now_s - self._manual_hold_start_s
            if hold_s >= self._config.manual_transition_hold_s:
                self._mode = MODE_MANUAL
                self._manual_hold_start_s = None
                self._l1_release_start_s = None
                self._auto_resume_until_s = None
                return 'manual_transition'
            return 'manual_transition_hold'
        self._manual_hold_start_s = None
        return reason or 'autonomous'

    def _update_manual_transition(
        self,
        now_s: float,
        l1_pressed: bool,
        autonomous_alive: bool,
        reason: str,
    ) -> str:
        self._manual_hold_start_s = None
        if l1_pressed:
            self._l1_release_start_s = None
            return 'manual_l1_hold'
        if self._l1_release_start_s is None:
            self._l1_release_start_s = now_s
        released_s = now_s - self._l1_release_start_s
        if released_s >= self._config.manual_to_auto_l1_released_s:
            if autonomous_alive:
                self._mode = MODE_AUTONOMOUS
                self._l1_release_start_s = None
                self._auto_resume_until_s = now_s + self._config.auto_resume_delay_s
                return 'auto_resume_countdown'
            return 'autonomous_cmd_stale'
        return reason or 'manual_l1_released_wait'

    def _select_output(
        self,
        now_s: float,
        joy_available: bool,
        l1_pressed: bool,
        ps_pressed: bool,
        autonomous_alive: bool,
        manual_alive: bool,
        manual_input_active: bool,
        autonomous_cmd: CommandSnapshot | None,
        manual_cmd: CommandSnapshot | None,
        reason: str,
    ) -> DriveModeOutput:
        auto_remaining = 0.0
        auto_pending = False
        source = SOURCE_ZERO
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        pending_linear_x = autonomous_cmd.linear_x if autonomous_cmd is not None else 0.0
        pending_angular_z = autonomous_cmd.angular_z if autonomous_cmd is not None else 0.0

        if self._mode == MODE_AUTONOMOUS:
            if self._auto_resume_until_s is not None:
                auto_remaining = max(0.0, self._auto_resume_until_s - now_s)
                auto_pending = auto_remaining > 0.0
                if not auto_pending:
                    self._auto_resume_until_s = None
            if auto_pending:
                source = SOURCE_ZERO
                reason = reason or 'auto_resume_countdown'
                if not autonomous_alive:
                    reason = 'autonomous_cmd_stale'
            elif autonomous_alive and autonomous_cmd is not None:
                source = SOURCE_AUTONOMOUS_CMD
                linear_x = autonomous_cmd.linear_x
                linear_y = autonomous_cmd.linear_y
                angular_z = autonomous_cmd.angular_z
                reason = 'autonomous_cmd'
            else:
                source = SOURCE_ZERO
                reason = 'autonomous_cmd_timeout'
        else:
            self._auto_resume_until_s = None
            if not l1_pressed:
                source = SOURCE_ZERO
                reason = 'manual_l1_released'
            elif manual_alive and manual_cmd is not None:
                source = SOURCE_MANUAL_CMD
                linear_x = manual_cmd.linear_x
                linear_y = manual_cmd.linear_y
                angular_z = manual_cmd.angular_z
                reason = 'manual_cmd'
            else:
                source = SOURCE_ZERO
                reason = 'manual_cmd_timeout'

        return DriveModeOutput(
            mode=self._mode,
            output_source=source,
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            joy_available=joy_available,
            l1_pressed=l1_pressed,
            ps_button_pressed=ps_pressed,
            ps_hold_progress_s=self._ps_hold_progress(now_s),
            manual_input_active=manual_input_active,
            manual_cmd_alive=manual_alive,
            autonomous_cmd_alive=autonomous_alive,
            auto_resume_pending=auto_pending,
            auto_resume_remaining_s=auto_remaining,
            pending_autonomous_linear_x=pending_linear_x,
            pending_autonomous_angular_z=pending_angular_z,
            reason=reason,
        )

    def _resolve_joy(self, now_s: float, joy: JoySnapshot | None) -> tuple[bool, bool, bool, str]:
        if joy is None:
            return False, False, False, 'joy_not_received'
        if now_s - joy.stamp_s > self._config.joy_timeout_s:
            return False, False, False, 'joy_timeout'
        l1, l1_ok = self._button_pressed(joy.buttons, self._config.l1_button_index)
        ps, ps_ok = self._button_pressed(joy.buttons, self._config.ps_button_index)
        if not l1_ok or not ps_ok:
            return True, l1 if l1_ok else False, ps if ps_ok else False, 'joy_button_index_out_of_range'
        return True, l1, ps, ''

    def _ps_hold_progress(self, now_s: float) -> float:
        if self._manual_hold_start_s is None:
            return 0.0
        return max(0.0, now_s - self._manual_hold_start_s)

    @staticmethod
    def _button_pressed(buttons: Sequence[int], index: int) -> tuple[bool, bool]:
        if index < 0:
            return True, True
        if index >= len(buttons):
            return False, False
        return bool(buttons[index]), True

    @staticmethod
    def _cmd_alive(now_s: float, cmd: CommandSnapshot | None, timeout_s: float) -> bool:
        if cmd is None or now_s - cmd.stamp_s > timeout_s:
            return False
        return all(math.isfinite(value) for value in (cmd.linear_x, cmd.linear_y, cmd.angular_z))

    @staticmethod
    def _is_nonzero(cmd: CommandSnapshot) -> bool:
        return abs(cmd.linear_x) > 1e-6 or abs(cmd.linear_y) > 1e-6 or abs(cmd.angular_z) > 1e-6
