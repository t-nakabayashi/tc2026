from drive_mode_manager.drive_mode_core import (
    CommandSnapshot,
    DriveModeConfig,
    DriveModeCore,
    JoySnapshot,
    MODE_AUTONOMOUS,
    MODE_MANUAL,
    SOURCE_AUTONOMOUS_CMD,
    SOURCE_MANUAL_CMD,
    SOURCE_ZERO,
)


def cmd(stamp: float, linear_x: float = 0.4, angular_z: float = 0.1) -> CommandSnapshot:
    return CommandSnapshot(linear_x=linear_x, linear_y=0.0, angular_z=angular_z, stamp_s=stamp)


def joy(stamp: float, l1: bool = False, ps: bool = False) -> JoySnapshot:
    buttons = [0] * 17
    buttons[4] = 1 if l1 else 0
    buttons[5] = 1 if ps else 0  # 実機モード遷移トリガ R1 (buttons[5])
    return JoySnapshot(buttons=buttons, stamp_s=stamp)


def test_manual_transition_requires_l1_and_ps_hold() -> None:
    core = DriveModeCore(DriveModeConfig(manual_transition_hold_s=2.0))

    out = core.update(1.0, joy(1.0, l1=True, ps=False), cmd(1.0), None)
    assert out.mode == MODE_AUTONOMOUS

    out = core.update(2.0, joy(2.0, l1=True, ps=True), cmd(2.0), None)
    assert out.mode == MODE_AUTONOMOUS

    out = core.update(4.1, joy(4.1, l1=True, ps=True), cmd(4.1), None)
    assert out.mode == MODE_MANUAL
    assert out.output_source == SOURCE_ZERO


def test_manual_to_auto_requires_l1_release_and_auto_alive() -> None:
    core = DriveModeCore(
        DriveModeConfig(
            initial_mode='manual',
            manual_to_auto_l1_released_s=1.0,
            auto_resume_delay_s=5.0,
        )
    )

    out = core.update(0.0, joy(0.0, l1=True), cmd(0.0), cmd(0.0))
    assert out.mode == MODE_MANUAL

    out = core.update(0.5, joy(0.5, l1=False), cmd(0.5), cmd(0.5))
    assert out.mode == MODE_MANUAL

    out = core.update(1.6, joy(1.6, l1=False), cmd(1.6), cmd(1.6))
    assert out.mode == MODE_AUTONOMOUS
    assert out.auto_resume_pending
    assert out.output_source == SOURCE_ZERO


def test_auto_resume_outputs_zero_until_delay_elapsed() -> None:
    core = DriveModeCore(
        DriveModeConfig(
            initial_mode='manual',
            manual_to_auto_l1_released_s=0.1,
            auto_resume_delay_s=2.0,
        )
    )

    out = core.update(0.0, joy(0.0, l1=False), cmd(0.0), cmd(0.0))
    out = core.update(0.2, joy(0.2, l1=False), cmd(0.2), cmd(0.2))
    assert out.auto_resume_pending
    assert out.output_source == SOURCE_ZERO

    out = core.update(2.3, joy(2.3, l1=False), cmd(2.3), cmd(2.3))
    assert not out.auto_resume_pending
    assert out.output_source == SOURCE_AUTONOMOUS_CMD


def test_autonomous_cmd_timeout_outputs_zero() -> None:
    core = DriveModeCore(DriveModeConfig(autonomous_cmd_timeout_s=0.5))

    out = core.update(1.0, joy(1.0), cmd(0.0), None)

    assert out.mode == MODE_AUTONOMOUS
    assert out.output_source == SOURCE_ZERO
    assert out.reason == 'autonomous_cmd_timeout'


def test_manual_outputs_manual_cmd_only_while_l1_pressed() -> None:
    core = DriveModeCore(DriveModeConfig(initial_mode='manual'))

    out = core.update(0.0, joy(0.0, l1=True), cmd(0.0), cmd(0.0, linear_x=0.2))
    assert out.output_source == SOURCE_MANUAL_CMD
    assert out.linear_x == 0.2

    out = core.update(0.1, joy(0.1, l1=False), cmd(0.1), cmd(0.1, linear_x=0.2))
    assert out.output_source == SOURCE_ZERO


def test_manual_cmd_alive_is_separate_from_nonzero_input() -> None:
    core = DriveModeCore(DriveModeConfig(initial_mode='manual'))

    out = core.update(0.0, joy(0.0, l1=True), cmd(0.0), cmd(0.0, linear_x=0.0, angular_z=0.0))

    assert out.manual_cmd_alive
    assert not out.manual_input_active
    assert out.output_source == SOURCE_MANUAL_CMD
