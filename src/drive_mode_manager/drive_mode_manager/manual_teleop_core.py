"""手動走行用 Joy 入力変換の ROS 非依存ロジック。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class ManualTeleopConfig:
    """Joy から Twist 相当値へ変換する設定。"""

    linear_axis: int = 1
    angular_axis: int = 0
    linear_y_axis: int = -1
    linear_scale: float = 1.2
    angular_scale: float = 1.5
    linear_y_scale: float = 0.5
    deadzone: float = 0.05
    linear_axis_invert: bool = False
    angular_axis_invert: bool = False
    enable_button: int = 4  # L1 (buttons[4]) デッドマン
    turbo_button: int = 6  # L2 (buttons[6]) ターボ
    turbo_ratio: float = 1.5
    joy_timeout_s: float = 0.5


@dataclass(frozen=True)
class JoyInput:
    """時刻付き Joy 入力。"""

    axes: Sequence[float]
    buttons: Sequence[int]
    stamp_s: float


@dataclass(frozen=True)
class ManualCommand:
    """手動速度指令の計算結果。"""

    linear_x: float
    linear_y: float
    angular_z: float
    enabled: bool
    joy_available: bool
    reason: str

    @staticmethod
    def zero(reason: str, joy_available: bool = False, enabled: bool = False) -> 'ManualCommand':
        """ゼロ速度の結果を生成する。"""

        return ManualCommand(
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            enabled=enabled,
            joy_available=joy_available,
            reason=reason,
        )


class ManualTeleopCore:
    """Joy 入力を手動速度指令へ変換する。"""

    def __init__(self, config: ManualTeleopConfig) -> None:
        self._config = config
        self._last_joy: JoyInput | None = None

    def update_joy(self, axes: Sequence[float], buttons: Sequence[int], stamp_s: float) -> None:
        """最新 Joy 入力を保存する。"""

        self._last_joy = JoyInput(tuple(axes), tuple(buttons), stamp_s)

    def compute(self, now_s: float) -> ManualCommand:
        """現在時刻で publish すべき手動速度指令を返す。"""

        if self._last_joy is None:
            return ManualCommand.zero('joy_not_received')
        if now_s - self._last_joy.stamp_s > self._config.joy_timeout_s:
            return ManualCommand.zero('joy_timeout')

        joy = self._last_joy
        enabled, button_ok = self._button_pressed(joy.buttons, self._config.enable_button)
        if not button_ok:
            return ManualCommand.zero('enable_button_index_out_of_range', True)
        if not enabled:
            return ManualCommand.zero('deadman_released', True)

        linear_x = self._read_axis(joy.axes, self._config.linear_axis, self._config.linear_scale)
        angular_z = self._read_axis(joy.axes, self._config.angular_axis, self._config.angular_scale)
        linear_y = 0.0
        if self._config.linear_y_axis >= 0:
            linear_y = self._read_axis(joy.axes, self._config.linear_y_axis, self._config.linear_y_scale)

        if self._config.linear_axis_invert:
            linear_x *= -1.0
        if self._config.angular_axis_invert:
            angular_z *= -1.0

        turbo = False
        if self._config.turbo_button >= 0:
            turbo, _ = self._button_pressed(joy.buttons, self._config.turbo_button)
        if turbo:
            linear_x *= self._config.turbo_ratio
            linear_y *= self._config.turbo_ratio
            angular_z *= self._config.turbo_ratio

        if not all(math.isfinite(value) for value in (linear_x, linear_y, angular_z)):
            return ManualCommand.zero('invalid_axis_value', True, True)

        return ManualCommand(
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            enabled=True,
            joy_available=True,
            reason='manual_cmd',
        )

    def _read_axis(self, axes: Sequence[float], index: int, scale: float) -> float:
        if index < 0 or index >= len(axes):
            return 0.0
        value = float(axes[index])
        if not math.isfinite(value):
            return float('nan')
        if abs(value) < self._config.deadzone:
            return 0.0
        return value * scale

    @staticmethod
    def _button_pressed(buttons: Sequence[int], index: int) -> tuple[bool, bool]:
        if index < 0:
            return True, True
        if index >= len(buttons):
            return False, False
        return bool(buttons[index]), True
