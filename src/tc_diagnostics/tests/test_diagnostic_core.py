"""診断メッセージ組み立ての規約適合を確認する."""
import pytest

from builtin_interfaces.msg import Time

from tc_diagnostics.diagnostic_core import (
    ASPECT_LIVENESS,
    ERROR,
    OK,
    STALE,
    WARN,
    DiagnosticNameError,
    build_array,
    build_status,
    compose_name,
    worst_level,
)


def test_name_follows_node_name_and_aspect() -> None:
    assert compose_name('route_manager_node', ASPECT_LIVENESS) == 'route_manager_node/liveness'


@pytest.mark.parametrize('node_name,aspect', [
    ('', 'liveness'),
    ('route_manager_node', ''),
    ('route/manager', 'liveness'),
    ('route_manager_node', 'live/ness'),
])
def test_invalid_names_are_rejected(node_name: str, aspect: str) -> None:
    """規約違反の name は配信前に弾く。GUI 側の突き合わせが壊れるため."""

    with pytest.raises(DiagnosticNameError):
        compose_name(node_name, aspect)


def test_status_carries_level_message_and_values() -> None:
    status = build_status('fusion_node', 'quality', WARN, '縮退中',
                          values={'mode': 'LIO_PRIORITY', 'sigma': 1.5})
    assert status.name == 'fusion_node/quality'
    assert status.level == WARN
    assert status.message == '縮退中'
    assert status.hardware_id == ''
    assert {entry.key: entry.value for entry in status.values} == {
        'mode': 'LIO_PRIORITY', 'sigma': '1.5'}


def test_hardware_id_is_only_set_when_given() -> None:
    status = build_status('driver_node', 'device', OK, '接続済み', hardware_id='/dev/ttyUSB0')
    assert status.hardware_id == '/dev/ttyUSB0'


def test_unknown_level_is_rejected() -> None:
    with pytest.raises(ValueError):
        build_status('driver_node', 'device', b'\x09', '不明')


def test_array_keeps_stamp_and_statuses() -> None:
    array = build_array(Time(sec=12, nanosec=34), [
        build_status('a_node', 'liveness', OK, '稼働中'),
        build_status('b_node', 'liveness', OK, '稼働中'),
    ])
    assert array.header.stamp.sec == 12
    assert [status.name for status in array.status] == ['a_node/liveness', 'b_node/liveness']


def test_worst_level_orders_ok_stale_warn_error() -> None:
    assert worst_level([]) == OK
    assert worst_level([OK, OK]) == OK
    assert worst_level([OK, STALE]) == STALE
    assert worst_level([STALE, WARN]) == WARN
    assert worst_level([WARN, ERROR, OK]) == ERROR
