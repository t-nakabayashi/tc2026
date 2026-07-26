import math

from drive_mode_manager.ps3_joy_sim_core import (
    Ps3JoySimConfig,
    Ps3JoySimCore,
    key_name_from_text_or_code,
)


def test_l1_and_ps_buttons_are_mapped_to_configured_indices() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig(l1_button_index=4, ps_button_index=5))

    state = core.compute({'l', 'p'})

    assert state.buttons[4] == 1
    assert state.buttons[5] == 1  # 実機モード遷移トリガ R1 (buttons[5])
    assert state.l1_pressed
    assert state.ps_pressed


def test_stick_keys_accumulate_axis_values() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig(stick_step=0.2, normalize_diagonal_stick=False))

    stick_x, stick_y = 0.0, 0.0
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'd')
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'd')
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'w')
    state = core.compute(set(), stick_x, stick_y)

    assert math.isclose(state.axes[0], -0.4)
    assert math.isclose(state.axes[1], 0.2)
    assert math.isclose(state.left_stick_x, 0.4)
    assert math.isclose(state.left_stick_y, 0.2)


def test_left_stick_x_invert_can_be_disabled() -> None:
    core = Ps3JoySimCore(
        Ps3JoySimConfig(
            invert_left_stick_x=False,
            stick_step=0.2,
            normalize_diagonal_stick=False,
        )
    )

    stick_x, stick_y = core.update_stick_for_key(0.0, 0.0, 'd')
    state = core.compute(set(), stick_x, stick_y)

    assert math.isclose(state.axes[0], 0.2)
    assert math.isclose(state.left_stick_x, 0.2)
    assert math.isclose(state.preview_angular_z, 0.3)


def test_stick_values_can_move_back_toward_neutral() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig(stick_step=0.25, normalize_diagonal_stick=False))

    stick_x, stick_y = 0.0, 0.0
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'd')
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'd')
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'a')
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'w')
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 's')
    state = core.compute(set(), stick_x, stick_y)

    assert math.isclose(state.axes[0], -0.25)
    assert math.isclose(state.axes[1], 0.0)


def test_stick_values_are_clamped() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig(stick_step=0.2, normalize_diagonal_stick=False))

    stick_x, stick_y = 0.0, 0.0
    for _ in range(10):
        stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'd')
        stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'w')
    state = core.compute(set(), stick_x, stick_y)

    assert math.isclose(state.axes[0], -1.0)
    assert math.isclose(state.axes[1], 1.0)


def test_left_stick_y_can_be_inverted() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig(invert_left_stick_y=True))

    stick_x, stick_y = core.update_stick_for_key(0.0, 0.0, 'w')
    state = core.compute(set(), stick_x, stick_y)

    assert math.isclose(state.axes[1], -0.1)
    assert math.isclose(state.left_stick_y, 0.1)


def test_normalize_diagonal_stick_limits_norm_to_one() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig(normalize_diagonal_stick=True))

    stick_x, stick_y = 0.0, 0.0
    for _ in range(10):
        stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'w')
        stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'd')
    state = core.compute(set(), stick_x, stick_y)

    assert math.isclose(math.hypot(state.left_stick_x, state.left_stick_y), 1.0)


def test_reset_equivalent_empty_state_outputs_neutral() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig())

    state = core.compute(set(), 0.0, 0.0)

    assert state.axes == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert state.buttons == (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    assert not state.l1_pressed
    assert not state.ps_pressed
    assert state.preview_linear_x == 0.0
    assert state.preview_angular_z == 0.0


def test_out_of_range_indices_do_not_change_array_lengths() -> None:
    core = Ps3JoySimCore(
        Ps3JoySimConfig(
            num_axes=2,
            num_buttons=2,
            left_stick_x_axis=10,
            left_stick_y_axis=11,
            l1_button_index=12,
            ps_button_index=13,
        )
    )

    state = core.compute({'l', 'p'}, 1.0, 1.0)

    assert state.axes == (0.0, 0.0)
    assert state.buttons == (0, 0)
    assert state.l1_pressed
    assert state.ps_pressed


def test_l1_ps_right_combination_is_represented_in_core() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig())

    stick_x, stick_y = core.update_stick_for_key(0.0, 0.0, 'd')
    state = core.compute({'l', 'p'}, stick_x, stick_y)

    assert state.buttons[4] == 1
    assert state.buttons[5] == 1
    assert math.isclose(state.axes[0], -0.1)
    assert math.isclose(state.left_stick_x, 0.1)


def test_preview_cmd_vel_matches_manual_teleop_defaults() -> None:
    core = Ps3JoySimCore(Ps3JoySimConfig(stick_step=0.2, normalize_diagonal_stick=False))

    stick_x, stick_y = 0.0, 0.0
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'd')
    stick_x, stick_y = core.update_stick_for_key(stick_x, stick_y, 'w')
    state = core.compute({'l'}, stick_x, stick_y)

    assert math.isclose(state.preview_linear_x, 0.24)
    assert math.isclose(state.preview_angular_z, -0.3)


def test_empty_qt_text_falls_back_to_key_code_for_right_input() -> None:
    assert key_name_from_text_or_code('', 0x44) == 'd'
    assert key_name_from_text_or_code('', 0x01000014) == 'd'
