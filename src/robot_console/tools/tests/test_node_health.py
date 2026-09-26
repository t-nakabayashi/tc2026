"""健全性の合成規則（docs/ノード健全性監視設計.md 4章・5.5節）の単体テスト。"""

import pytest

from robot_console.core.freshness import FreshnessLevel as F
from robot_console.core.launch_profile import HealthTopic
from robot_console.core.node_health import (
    DIAGNOSTIC_ERROR,
    DIAGNOSTIC_OK,
    DIAGNOSTIC_STALE,
    DIAGNOSTIC_WARN,
    DiagnosticEntry,
    aggregate_topic_freshness,
    apply_diagnostics,
    build_summary,
    combine_with_launch_status,
    select_diagnostics,
    within_grace,
)
from robot_console.utils import NodeLaunchStatus as S


@pytest.mark.parametrize('levels,grace,expected', [
    ([], False, F.UNKNOWN),
    ([F.OK, F.OK], False, F.OK),
    ([F.OK, F.LOST], False, F.STALE),
    ([F.OK, F.UNKNOWN], False, F.STALE),
    ([F.STALE], False, F.STALE),
    ([F.STALE, F.LOST], False, F.STALE),
    ([F.LOST, F.LOST], False, F.LOST),
    ([F.UNKNOWN, F.UNKNOWN], False, F.LOST),
    ([F.UNKNOWN, F.UNKNOWN], True, F.UNKNOWN),
])
def test_topic_freshness_aggregation(levels, grace, expected) -> None:
    assert aggregate_topic_freshness(levels, within_startup_grace=grace) == expected


@pytest.mark.parametrize('status,topic_health,expected', [
    (S.RUNNING, F.OK, ('RUNNING', F.OK, False)),
    (S.RUNNING, F.STALE, ('RUNNING', F.STALE, False)),
    (S.RUNNING, F.UNKNOWN, ('STARTING', F.UNKNOWN, False)),
    (S.STARTING, F.OK, ('STARTING', F.UNKNOWN, False)),
    (S.ERROR, F.OK, ('ERROR', F.LOST, False)),
    (S.STOPPED, F.OK, ('RUNNING', F.OK, True)),
    (S.STOPPED, F.STALE, ('RUNNING', F.STALE, True)),
    (S.STOPPED, F.LOST, ('STOPPED', F.UNKNOWN, False)),
    (S.STOPPED, F.UNKNOWN, ('STOPPED', F.UNKNOWN, False)),
])
def test_launch_status_and_topic_health_combination(status, topic_health, expected) -> None:
    assert combine_with_launch_status(status, topic_health) == expected


def test_running_process_without_output_is_an_error() -> None:
    """publisherはあるがデータが流れない故障を、稼働中と誤表示しない."""

    assert combine_with_launch_status(S.RUNNING, F.LOST) == ('ERROR', F.LOST, False)


def test_diagnostics_do_not_change_anything_when_absent() -> None:
    """診断の不在は異常ではない。外部パッケージは配信しないため区別できない."""

    assert apply_diagnostics('RUNNING', F.OK, []) == ('RUNNING', F.OK, '')


def test_diagnostic_error_forces_error_status() -> None:
    entries = [DiagnosticEntry('driver', 'device', DIAGNOSTIC_ERROR, '未接続')]
    assert apply_diagnostics('RUNNING', F.OK, entries) == ('ERROR', F.LOST, '未接続')


@pytest.mark.parametrize('level', [DIAGNOSTIC_WARN, DIAGNOSTIC_STALE])
def test_diagnostic_warning_degrades_health_only(level) -> None:
    entries = [DiagnosticEntry('fusion', 'quality', level, '縮退中')]
    assert apply_diagnostics('RUNNING', F.OK, entries) == ('RUNNING', F.STALE, '縮退中')


def test_diagnostic_warning_does_not_upgrade_a_worse_health() -> None:
    """既に LOST の profile を、診断のWARNで STALE へ格上げしない."""

    entries = [DiagnosticEntry('fusion', 'quality', DIAGNOSTIC_WARN, '縮退中')]
    assert apply_diagnostics('ERROR', F.LOST, entries) == ('ERROR', F.LOST, '縮退中')


def test_stopped_profile_does_not_keep_the_last_diagnostic_message() -> None:
    """診断は配信が途絶えても最後の値が残るため、停止判定時は表示しない."""

    entries = [DiagnosticEntry('mux', 'liveness', DIAGNOSTIC_OK, '稼働中')]
    assert apply_diagnostics('STOPPED', F.UNKNOWN, entries) == ('STOPPED', F.UNKNOWN, '')


def test_worst_diagnostic_message_is_selected() -> None:
    entries = [
        DiagnosticEntry('fusion', 'liveness', DIAGNOSTIC_OK, '稼働中'),
        DiagnosticEntry('fusion', 'quality', DIAGNOSTIC_ERROR, 'LIO 異常'),
    ]
    assert apply_diagnostics('RUNNING', F.OK, entries)[2] == 'LIO 異常'


def test_diagnostics_are_selected_by_node_name() -> None:
    diagnostics = {
        'a_node/liveness': DiagnosticEntry('a_node', 'liveness', DIAGNOSTIC_OK, ''),
        'b_node/liveness': DiagnosticEntry('b_node', 'liveness', DIAGNOSTIC_OK, ''),
    }
    selected = select_diagnostics(diagnostics, ['b_node'])
    assert [entry.node_name for entry in selected] == ['b_node']


def test_profile_without_health_topics_falls_back_to_process_state() -> None:
    result = build_summary(
        launch_status=S.RUNNING, topic_levels=[], diagnostics=[], within_startup_grace=False
    )
    assert result['status'] == 'RUNNING'
    assert result['health'] == F.OK


def test_stopping_is_displayed_as_stopped() -> None:
    result = build_summary(
        launch_status=S.STOPPING, topic_levels=[], diagnostics=[], within_startup_grace=False
    )
    assert result['status'] == 'STOPPED'


def test_startup_grace_window() -> None:
    assert within_grace(None, now=100.0) is False
    assert within_grace(95.0, now=100.0, grace_sec=10.0) is True
    assert within_grace(80.0, now=100.0, grace_sec=10.0) is False


@pytest.mark.parametrize('rate_hz,expected', [
    (1.0, (3.0, 10.0)),
    (10.0, (0.5, 2.0)),
    (0.5, (6.0, 20.0)),
])
def test_thresholds_are_derived_from_the_nominal_rate(rate_hz, expected) -> None:
    """高レートtopicでしきい値が過度に短くならないよう下限を設ける."""

    topic = HealthTopic(topic='/t', type='std_msgs/msg/Bool', rate_hz=rate_hz)
    assert topic.thresholds() == expected
