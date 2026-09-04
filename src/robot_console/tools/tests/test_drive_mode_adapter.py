"""core/drive_mode_adapter.py の単体テスト。"""

import math
from types import SimpleNamespace

from robot_console.core.drive_mode_adapter import (
    apply_cmd_vel_autonomous_msg,
    apply_cmd_vel_msg,
    apply_drive_mode_status_msg,
    drive_mode_label,
    output_source_label,
)
from robot_console.core.snapshot_model import DriveModeStateView


def _twist(linear_x: float, angular_z: float) -> SimpleNamespace:
    return SimpleNamespace(
        linear=SimpleNamespace(x=linear_x, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=angular_z),
    )


def test_drive_mode_label_maps_msg_constants():
    assert drive_mode_label(1) == 'autonomous'
    assert drive_mode_label(2) == 'manual'
    assert drive_mode_label(99) == 'unknown'


def test_output_source_label_maps_msg_constants():
    assert output_source_label(0) == 'zero'
    assert output_source_label(1) == 'autonomous_cmd'
    assert output_source_label(2) == 'manual_cmd'
    assert output_source_label(99) == 'unknown'


def test_apply_drive_mode_status_msg_updates_mode_fields_only():
    view = DriveModeStateView(cmd_vel_linear_mps=1.5)
    msg = SimpleNamespace(mode=2, output_source=2, auto_resume_pending=True)

    updated = apply_drive_mode_status_msg(view, msg)

    assert updated.mode == 'manual'
    assert updated.output_source == 'manual_cmd'
    assert updated.auto_resume_pending is True
    assert updated.cmd_vel_linear_mps == 1.5


def test_apply_cmd_vel_msg_converts_angular_velocity_to_degrees():
    updated = apply_cmd_vel_msg(DriveModeStateView(), _twist(0.4, math.pi / 2))

    assert updated.cmd_vel_linear_mps == 0.4
    assert round(updated.cmd_vel_angular_dps, 3) == 90.0


def test_apply_cmd_vel_autonomous_msg_fills_optional_fields():
    view = DriveModeStateView()
    assert view.cmd_vel_autonomous_linear_mps is None

    updated = apply_cmd_vel_autonomous_msg(view, _twist(0.2, 0.0))

    assert updated.cmd_vel_autonomous_linear_mps == 0.2
    assert updated.cmd_vel_autonomous_angular_dps == 0.0
