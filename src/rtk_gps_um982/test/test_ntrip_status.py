from types import SimpleNamespace

from tc_diagnostics import OK

from rtk_gps_um982.ntrip_status import NtripStatus


def test_connection_and_crc_data_are_distinct_and_credentials_absent():
    status = NtripStatus()
    client = SimpleNamespace(total_bytes=0, last_rtcm_monotonic=None,
        transport_connected=True, connection_attempts=1, last_error='')
    args = dict(enabled=True, host='caster', port=2101, mountpoint='MOUNT')
    assert status.sample(client, now=10., **args)['state'] == 'WAITING'
    client.total_bytes = 200
    client.last_rtcm_monotonic = 11.
    result = status.sample(client, now=12., **args)
    assert result['state'] == 'RECEIVING'
    assert result['rtcm_bytes_per_s'] == 100.
    assert result['last_rtcm_age_s'] == 1.
    assert not {'user', 'password', 'connection'} & result.keys()
    assert status.sample(client, now=20., **args)['state'] == 'STALE'
    client.transport_connected = False
    client.connection_attempts = 2
    result = status.sample(client, now=21., **args)
    assert result['state'] == 'RECONNECTING' and result['reconnect_count'] == 1
    assert status.sample(None, now=22., **{**args, 'enabled':False})['state'] == 'DISABLED'
    assert status.sample(None, now=23., **{**args, 'host':''})['state'] == 'ERROR'


def test_driver_heartbeat_does_not_require_position_callback():
    import json
    from rtk_gps_um982.driver_node import Um982DriverNode
    messages, reports = [], []
    driver = SimpleNamespace(_client=SimpleNamespace(_ntrip_client=None),
        _ntrip_status=NtripStatus(),
        _ntrip_fields=dict(enabled=False, host='', port=2101, mountpoint=''),
        _pub_ntrip=SimpleNamespace(publish=messages.append),
        _NTRIP_QUALITY=Um982DriverNode._NTRIP_QUALITY,
        _diagnostics=SimpleNamespace(
            report=lambda aspect, level, text, values=None: reports.append((aspect, level, text))))
    Um982DriverNode._publish_ntrip_status(driver)
    assert json.loads(messages[0].data)['state'] == 'DISABLED'
    # NTRIP 無効は設定どおりの状態であり、異常として申告しない。
    assert reports == [('quality', OK, 'NTRIP 無効設定')]
