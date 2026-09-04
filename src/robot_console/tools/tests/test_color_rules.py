"""ui_qt/widgets/color_rules.py の単体テスト。"""

from robot_console.core.freshness import FreshnessLevel
from robot_console.ui_qt.widgets.color_rules import (
    COLOR_ERROR,
    COLOR_NOTICE,
    COLOR_OK,
    COLOR_UNKNOWN,
    COLOR_WARN,
    freshness_color,
    launch_status_color,
    phase_color,
    rtk_state_color,
    severity_color,
)
from robot_console.utils import NodeLaunchStatus


def test_freshness_color_maps_all_levels():
    assert freshness_color(FreshnessLevel.OK) == COLOR_OK
    assert freshness_color(FreshnessLevel.STALE) == COLOR_WARN
    assert freshness_color(FreshnessLevel.LOST) == COLOR_ERROR
    assert freshness_color(FreshnessLevel.UNKNOWN) == COLOR_UNKNOWN


def test_rtk_state_color_prioritizes_lost_freshness_over_state():
    assert rtk_state_color('RTK_FIX', FreshnessLevel.LOST) == COLOR_ERROR


def test_rtk_state_color_by_state_when_fresh():
    assert rtk_state_color('RTK_FIX', FreshnessLevel.OK) == COLOR_OK
    assert rtk_state_color('RTK_FLOAT', FreshnessLevel.OK) == COLOR_WARN
    assert rtk_state_color('DGPS', FreshnessLevel.OK) == COLOR_NOTICE
    assert rtk_state_color('STANDALONE', FreshnessLevel.OK) == COLOR_NOTICE
    assert rtk_state_color('UNKNOWN', FreshnessLevel.OK) == COLOR_UNKNOWN


def test_severity_color_known_and_unknown_values():
    assert severity_color('error') == COLOR_ERROR
    assert severity_color('WARN') == COLOR_WARN
    assert severity_color('notice') == COLOR_NOTICE
    assert severity_color('info') == COLOR_OK
    assert severity_color('something_else') == COLOR_UNKNOWN


def test_phase_color_known_phases():
    assert phase_color('走行中') == COLOR_OK
    assert phase_color('走行準備完了') == COLOR_NOTICE
    assert phase_color('起動確認中') == COLOR_NOTICE
    assert phase_color('終了処理中') == COLOR_NOTICE
    assert phase_color('一時停止') == COLOR_WARN
    assert phase_color('異常') == COLOR_ERROR
    assert phase_color('未起動') == COLOR_UNKNOWN


def test_phase_color_unknown_phase_defaults_to_unknown_color():
    assert phase_color('未定義フェーズ') == COLOR_UNKNOWN


def test_launch_status_color_maps_all_statuses():
    assert launch_status_color(NodeLaunchStatus.RUNNING) == COLOR_OK
    assert launch_status_color(NodeLaunchStatus.STARTING) == COLOR_NOTICE
    assert launch_status_color(NodeLaunchStatus.ERROR) == COLOR_ERROR
    assert launch_status_color(NodeLaunchStatus.STOPPING) == COLOR_WARN
    assert launch_status_color(NodeLaunchStatus.STOPPED) == COLOR_UNKNOWN
