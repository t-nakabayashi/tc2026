"""融合表示の変換、異常入力、UIとの接続を確認する."""
import math
from types import SimpleNamespace

from robot_console.core.console_core import ConsoleCore
from robot_console.core.freshness import FreshnessLevel
from robot_console.core.business_mode import get_preset


def fusion_state(mode: str, *, has_estimate: bool = True, yaw_rad: float = 0.,
                 heading_sigma_deg: float = 0., baseline_reference_m: float = .5):
    """`tc_geo_msgs/FusionState` 相当のメッセージを組み立てる."""

    return SimpleNamespace(mode=mode, has_estimate=has_estimate, yaw_rad=yaw_rad,
                           heading_sigma_deg=heading_sigma_deg,
                           baseline_reference_m=baseline_reference_m)


def test_fusion_view_and_invalid_data() -> None:
    core = ConsoleCore()
    core.update_fusion_status(fusion_state('LIO_PRIORITY', yaw_rad=1., heading_sigma_deg=2.,
                                           baseline_reference_m=.515))
    state = core.build_snapshot().fusion_state
    assert state.mode == 'LIO_PRIORITY'
    assert 57 < state.yaw_deg < 58
    assert state.freshness == FreshnessLevel.OK
    # 非有限の基線長は表示を壊すため、直前の値を保持して無視する。
    core.update_fusion_status(fusion_state('LIO_PRIORITY', baseline_reference_m=math.nan))
    assert core.build_snapshot().fusion_state.baseline_m == .515


def test_non_finite_heading_does_not_reach_the_view() -> None:
    """姿勢だけが不正な場合もモードと基線長は表示し続ける."""

    core = ConsoleCore()
    core.update_fusion_status(fusion_state('GPS_LIO', yaw_rad=math.inf, heading_sigma_deg=1.,
                                           baseline_reference_m=.3))
    state = core.build_snapshot().fusion_state
    assert state.mode == 'GPS_LIO'
    assert state.yaw_deg is None and state.heading_sigma_deg is None
    assert state.baseline_m == .3


def test_negative_heading_sigma_is_rejected() -> None:
    core = ConsoleCore()
    core.update_fusion_status(fusion_state('GPS_LIO', yaw_rad=.1, heading_sigma_deg=-1.))
    state = core.build_snapshot().fusion_state
    assert state.yaw_deg is None and state.heading_sigma_deg is None


def test_fused_presets_only_select_one_stack() -> None:
    for mode, environment in [('実機（融合）', 'real'), ('デジタルツイン', 'simulation')]:
        preset = get_preset(mode, '自律走行')
        assert len(preset) == 1
        assert preset[0].profile_id == ('icart_recorded_route' if environment == 'real' else 'icart_fused_stack')
        assert preset[0].overrides == ({} if environment == 'real' else {'environment': environment})


def test_initial_fix_wait_has_no_invented_heading() -> None:
    core = ConsoleCore()
    core.update_fusion_status(fusion_state('WAIT_INITIAL_FIX', has_estimate=False))
    state = core.build_snapshot().fusion_state
    assert state.mode == 'WAIT_INITIAL_FIX'
    assert state.yaw_deg is None and state.heading_sigma_deg is None
    assert state.freshness == FreshnessLevel.OK


def test_gravity_wait_is_visible_without_a_heading() -> None:
    core = ConsoleCore()
    core.update_fusion_status(fusion_state('WAIT_GRAVITY_ALIGNMENT', has_estimate=False,
                                           baseline_reference_m=.15))
    state = core.build_snapshot().fusion_state
    assert state.mode == 'WAIT_GRAVITY_ALIGNMENT'
    assert state.yaw_deg is None and state.heading_sigma_deg is None
    assert state.freshness == FreshnessLevel.OK
