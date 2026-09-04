import math

from drive_mode_manager.gui_core import DriveStatusGuiCore
from drive_mode_manager.drive_mode_core import MODE_AUTONOMOUS, MODE_MANUAL


class Status:
    mode = MODE_AUTONOMOUS
    output_source = 0
    reason = 'test'
    output_linear_x = 0.0
    output_angular_z = 0.0
    auto_resume_pending = False
    auto_resume_remaining_s = 0.0
    pending_autonomous_linear_x = 0.0
    pending_autonomous_angular_z = 0.0
    joy_available = False
    l1_pressed = False
    autonomous_cmd_alive = False


def test_regulation_state_label_mapping() -> None:
    core = DriveStatusGuiCore()
    status = Status()
    status.mode = MODE_MANUAL

    view = core.update_status(status)
    assert view.state_label == '操縦 / Manual'

    status.mode = MODE_AUTONOMOUS
    status.auto_resume_pending = True
    view = core.update_status(status)
    assert view.state_label == '自律 / Auto'


def test_aspect_ratio_fit_rect() -> None:
    width, height = DriveStatusGuiCore.fit_rect(1000.0, 1000.0)

    assert round(width / height, 6) == round(16.0 / 9.0, 6)
    assert width <= 1000.0
    assert height <= 1000.0


def test_planned_direction_text_uses_arrow_angle_thresholds() -> None:
    assert DriveStatusGuiCore.planned_direction_text(
        linear_x=1.2,
        angular_z=0.0,
        turn_preview_seconds=1.0,
        max_linear_x=0.8,
        max_angular_z=1.2,
    ) == '前進'

    assert DriveStatusGuiCore.planned_direction_text(
        linear_x=1.2,
        angular_z=0.6,
        turn_preview_seconds=1.0,
        max_linear_x=0.8,
        max_angular_z=1.2,
        angular_axis_invert=True,
    ) == '左旋回'

    assert DriveStatusGuiCore.planned_direction_text(
        linear_x=1.2,
        angular_z=-0.6,
        turn_preview_seconds=1.0,
        max_linear_x=0.8,
        max_angular_z=1.2,
        angular_axis_invert=True,
    ) == '右旋回'


def test_planned_direction_text_warns_for_sharp_turn_and_backward() -> None:
    assert DriveStatusGuiCore.planned_direction_text(
        linear_x=1.2,
        angular_z=1.5,
        turn_preview_seconds=1.0,
        max_linear_x=0.8,
        max_angular_z=1.2,
        angular_axis_invert=True,
    ) == '左旋回\n急旋回注意！'

    assert DriveStatusGuiCore.planned_direction_text(
        linear_x=-0.2,
        angular_z=0.0,
        turn_preview_seconds=1.0,
        max_linear_x=0.8,
        max_angular_z=1.2,
    ) == '後進\n後方注意！'


def test_cmd_vel_to_stick_point_uses_manual_teleop_scale() -> None:
    stick_x, stick_y = DriveStatusGuiCore.cmd_vel_to_stick_point(
        linear_x=0.6,
        angular_z=0.75,
        linear_scale=1.2,
        angular_scale=1.5,
        deadzone=0.05,
        angular_axis_invert=True,
    )

    assert stick_x == -0.5
    assert stick_y == 0.5


def test_direction_vector_from_cmd_vel_points_to_chart_dot() -> None:
    dx, dy = DriveStatusGuiCore.direction_vector_from_cmd_vel(
        linear_x=1.2,
        angular_z=0.0,
    )

    assert dx == 0.0
    assert dy == -1.0

    dx, dy = DriveStatusGuiCore.direction_vector_from_cmd_vel(
        linear_x=0.0,
        angular_z=1.5,
        angular_axis_invert=True,
    )

    assert dx == -1.0
    assert dy == -0.0

    dx, dy = DriveStatusGuiCore.direction_vector_from_cmd_vel(
        linear_x=0.6,
        angular_z=0.75,
        angular_axis_invert=True,
    )

    assert dx < 0.0
    assert round((dx ** 2 + dy ** 2) ** 0.5, 6) == 1.0


def test_cmd_vel_to_stick_point_clamps_to_chart_circle() -> None:
    stick_x, stick_y = DriveStatusGuiCore.cmd_vel_to_stick_point(
        linear_x=1.2,
        angular_z=1.5,
        angular_axis_invert=True,
    )

    assert stick_x < 0.0
    assert round((stick_x ** 2 + stick_y ** 2) ** 0.5, 6) == 1.0


def test_direction_angle_from_cmd_vel_matches_chart_coordinates() -> None:
    angle = DriveStatusGuiCore.direction_angle_from_cmd_vel(
        linear_x=0.0,
        angular_z=1.5,
        angular_axis_invert=True,
    )

    assert angle == math.radians(-90.0)
