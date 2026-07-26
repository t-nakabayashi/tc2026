from drive_mode_manager.manual_teleop_core import ManualTeleopConfig, ManualTeleopCore


def test_manual_deadman_outputs_zero_when_l1_released() -> None:
    core = ManualTeleopCore(ManualTeleopConfig(enable_button=4))
    core.update_joy([0.8, 1.0], [0, 0, 0, 0, 0], 0.0)

    result = core.compute(0.1)

    assert result.linear_x == 0.0
    assert result.angular_z == 0.0
    assert result.reason == 'deadman_released'


def test_axis_deadzone_and_invert() -> None:
    core = ManualTeleopCore(
        ManualTeleopConfig(
            linear_axis=1,
            angular_axis=0,
            linear_scale=2.0,
            angular_scale=3.0,
            deadzone=0.1,
            linear_axis_invert=True,
            enable_button=4,
            turbo_button=-1,
        )
    )
    core.update_joy([0.05, 0.5], [0, 0, 0, 0, 1], 0.0)

    result = core.compute(0.1)

    assert result.linear_x == -1.0
    assert result.angular_z == 0.0
    assert result.enabled


def test_turbo_scales_enabled_command() -> None:
    core = ManualTeleopCore(
        ManualTeleopConfig(
            linear_scale=1.0,
            angular_scale=1.0,
            turbo_button=6,
            turbo_ratio=2.0,
        )
    )
    # buttons[4]=L1 デッドマン, buttons[6]=L2 ターボ
    core.update_joy([0.5, 0.5], [0, 0, 0, 0, 1, 0, 1], 0.0)

    result = core.compute(0.1)

    assert result.linear_x == 1.0
    assert result.angular_z == 1.0


def test_joy_timeout_outputs_zero() -> None:
    core = ManualTeleopCore(ManualTeleopConfig(joy_timeout_s=0.5))
    core.update_joy([0.0, 1.0], [0, 0, 0, 0, 1], 0.0)

    result = core.compute(0.6)

    assert result.linear_x == 0.0
    assert result.reason == 'joy_timeout'
