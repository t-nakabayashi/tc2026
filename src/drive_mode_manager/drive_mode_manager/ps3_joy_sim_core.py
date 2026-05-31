"""PS3 controller 相当の Joy 入力を生成する ROS 非依存ロジック。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import FrozenSet, Sequence


def key_name_from_text_or_code(text: str, key_code: int) -> str:
    """Qt の文字列または key code から simulator 用キー名を得る。"""

    if text:
        if text == ' ':
            return 'space'
        return text.lower()
    key_map = {
        0x20: 'space',
        0x41: 'a',
        0x44: 'd',
        0x4C: 'l',
        0x50: 'p',
        0x53: 's',
        0x57: 'w',
        0x01000012: 'a',
        0x01000013: 'w',
        0x01000014: 'd',
        0x01000015: 's',
    }
    return key_map.get(key_code, '')


@dataclass(frozen=True)
class Ps3JoySimConfig:
    """キーボード入力から Joy 配列を生成する設定。"""

    num_axes: int = 6
    num_buttons: int = 17
    left_stick_x_axis: int = 0
    left_stick_y_axis: int = 1
    invert_left_stick_x: bool = True
    invert_left_stick_y: bool = False
    stick_step: float = 0.1
    l1_button_index: int = 4
    ps_button_index: int = 5  # 実機モード遷移トリガ R1 (buttons[5]) に合わせる
    key_l1: str = 'l'
    key_ps: str = 'p'
    key_stick_forward: str = 'w'
    key_stick_backward: str = 's'
    key_stick_left: str = 'a'
    key_stick_right: str = 'd'
    key_reset: str = 'space'
    normalize_diagonal_stick: bool = True
    cmd_vel_linear_scale: float = 1.2
    cmd_vel_angular_scale: float = 1.5
    cmd_vel_deadzone: float = 0.05
    cmd_vel_linear_axis_invert: bool = False
    cmd_vel_angular_axis_invert: bool = False


@dataclass(frozen=True)
class Ps3JoyState:
    """publish 対象の Joy 配列と表示用状態。"""

    axes: tuple[float, ...]
    buttons: tuple[int, ...]
    left_stick_x: float
    left_stick_y: float
    l1_pressed: bool
    ps_pressed: bool
    preview_linear_x: float
    preview_angular_z: float


class Ps3JoySimCore:
    """押下キー集合から PS3 controller 相当の Joy 状態を生成する。"""

    def __init__(self, config: Ps3JoySimConfig) -> None:
        self._config = config

    @property
    def config(self) -> Ps3JoySimConfig:
        """現在の変換設定を返す。"""

        return self._config

    def compute(
        self,
        pressed_keys: Sequence[str] | FrozenSet[str] | set[str],
        stick_x: float = 0.0,
        stick_y: float = 0.0,
    ) -> Ps3JoyState:
        """押下中キーと stick 保持値から Joy 状態を算出する。

        Args:
            pressed_keys (Sequence[str] | FrozenSet[str] | set[str]): 押下中キー名.
            stick_x (float): 左 stick 横軸の保持値.
            stick_y (float): 左 stick 縦軸の保持値.

        Returns:
            Ps3JoyState: Joy 配列と表示用の正規化状態.
        """

        keys = {key.lower() for key in pressed_keys}
        axes = [0.0] * max(0, self._config.num_axes)
        buttons = [0] * max(0, self._config.num_buttons)

        display_stick_x = self._clamp_axis(stick_x)
        display_stick_y = self._clamp_axis(stick_y)
        joy_stick_x = -display_stick_x if self._config.invert_left_stick_x else display_stick_x
        joy_stick_y = display_stick_y
        if self._config.invert_left_stick_y:
            joy_stick_y *= -1.0
        if self._config.normalize_diagonal_stick:
            joy_stick_x, joy_stick_y = self._normalize_diagonal(joy_stick_x, joy_stick_y)
            display_stick_x, display_stick_y = self._normalize_diagonal(
                display_stick_x,
                display_stick_y,
            )

        self._write_axis(axes, self._config.left_stick_x_axis, joy_stick_x)
        self._write_axis(axes, self._config.left_stick_y_axis, joy_stick_y)

        l1_pressed = self._config.key_l1.lower() in keys
        ps_pressed = self._config.key_ps.lower() in keys
        self._write_button(buttons, self._config.l1_button_index, l1_pressed)
        self._write_button(buttons, self._config.ps_button_index, ps_pressed)

        preview_linear_x, preview_angular_z = self._preview_cmd_vel(joy_stick_x, joy_stick_y)

        return Ps3JoyState(
            axes=tuple(axes),
            buttons=tuple(buttons),
            left_stick_x=display_stick_x,
            left_stick_y=display_stick_y,
            l1_pressed=l1_pressed,
            ps_pressed=ps_pressed,
            preview_linear_x=preview_linear_x,
            preview_angular_z=preview_angular_z,
        )

    def _preview_cmd_vel(self, stick_x: float, stick_y: float) -> tuple[float, float]:
        linear_x = 0.0 if abs(stick_y) < self._config.cmd_vel_deadzone else stick_y
        angular_z = 0.0 if abs(stick_x) < self._config.cmd_vel_deadzone else stick_x
        if self._config.cmd_vel_linear_axis_invert:
            linear_x *= -1.0
        if self._config.cmd_vel_angular_axis_invert:
            angular_z *= -1.0
        return (
            linear_x * self._config.cmd_vel_linear_scale,
            angular_z * self._config.cmd_vel_angular_scale,
        )

    def update_stick_for_key(self, stick_x: float, stick_y: float, key: str) -> tuple[float, float]:
        """方向キー 1 回分を stick 保持値へ反映する。"""

        key = key.lower()
        step = max(0.0, min(1.0, float(self._config.stick_step)))
        if key == self._config.key_stick_left.lower():
            stick_x -= step
        elif key == self._config.key_stick_right.lower():
            stick_x += step
        elif key == self._config.key_stick_backward.lower():
            stick_y -= step
        elif key == self._config.key_stick_forward.lower():
            stick_y += step
        stick_x = self._clamp_axis(stick_x)
        stick_y = self._clamp_axis(stick_y)
        if self._config.normalize_diagonal_stick:
            stick_x, stick_y = self._normalize_diagonal(stick_x, stick_y)
        return stick_x, stick_y

    def is_stick_key(self, key: str) -> bool:
        """指定キーが stick 操作用かを返す。"""

        key = key.lower()
        return key in {
            self._config.key_stick_forward.lower(),
            self._config.key_stick_backward.lower(),
            self._config.key_stick_left.lower(),
            self._config.key_stick_right.lower(),
        }

    @staticmethod
    def _normalize_diagonal(stick_x: float, stick_y: float) -> tuple[float, float]:
        norm = math.hypot(stick_x, stick_y)
        if norm <= 1.0:
            return stick_x, stick_y
        return stick_x / norm, stick_y / norm

    @staticmethod
    def _clamp_axis(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    @staticmethod
    def _write_axis(axes: list[float], index: int, value: float) -> None:
        if 0 <= index < len(axes):
            axes[index] = value

    @staticmethod
    def _write_button(buttons: list[int], index: int, pressed: bool) -> None:
        if 0 <= index < len(buttons):
            buttons[index] = 1 if pressed else 0
