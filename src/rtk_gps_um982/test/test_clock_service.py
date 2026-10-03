from rtk_gps_um982.clock_service import PacketHealth
from rtk_gps_um982 import clock_service
import socket
import pytest


def feed(health,start=0.,duration=12.,mode=1,age=.002):
    for i in range(int(duration*100)):
        mono=start+i*.01
        for kind in ('lidar','imu'):
            health.observe((kind,mode,int((1700000000.+mono-age)*1e9)),1700000000.+mono,mono)
    return mono


def test_ptp_gate_requires_both_streams_and_loses_ready_on_failure():
    h=PacketHealth()
    assert not h.snapshot(0.)[0]
    end=feed(h)
    assert h.snapshot(end)[0]
    assert not h.snapshot(end+.3)[0]
    for mode,age in [(0,.002),(1,.02),(1,-.02)]:
        h=PacketHealth();end=feed(h,mode=mode,age=age)
        assert not h.snapshot(end)[0]


def test_time_backwards_has_holdoff():
    h=PacketHealth();end=feed(h)
    h.observe(('imu',1,int(1700000000.*1e9)),1700000000.+end+.01,end+.01)
    assert not h.snapshot(end+.01)[0]


def test_small_regressions_are_counted_but_cumulative_regression_is_rejected():
    h=PacketHealth();end=feed(h)
    peak=h.previous['lidar']
    for retreat in (7_000, 14_000, 100_000):
        h.observe(('lidar',1,peak-retreat),peak/1e9+.002,end+.001)
        assert h.snapshot(end+.001)[0]
    assert h.snapshot(end+.001)[1]['lidar']['minor_backwards'] == 3
    h.observe(('lidar',1,peak-100_001),peak/1e9+.002,end+.002)
    assert not h.snapshot(end+.002)[0]


@pytest.mark.parametrize('clock_ok,link,tools_ok', [(True, False, True), (False, True, True),
                                                 (True, True, False)])
def test_service_reports_clock_separately_from_ptp_prerequisites(
        tmp_path, monkeypatch, clock_ok, link, tools_ok):
    states = iter([dict(ready=False, clock_ready=clock_ok, link=link,
                       tools={'ptp4l': tools_ok})])
    def check(*args):
        try:
            return next(states)
        except StopIteration:
            raise KeyboardInterrupt
    class Socket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def bind(self, *args): pass
        def settimeout(self, *args): pass
        def setsockopt(self, *args): pass
        def recvmsg(self, *args): raise socket.timeout
    ticks = iter(range(100))
    records = []
    monkeypatch.setattr(clock_service, 'check', check)
    monkeypatch.setattr(clock_service.socket, 'socket', lambda *args: Socket())
    monkeypatch.setattr(clock_service.time, 'monotonic', lambda: next(ticks))
    monkeypatch.setattr(clock_service, 'atomic_status', lambda path, state: records.append(state))
    monkeypatch.setattr(clock_service.subprocess, 'Popen',
                        lambda *a, **kw: pytest.fail('PTP must not start without prerequisites'))
    monkeypatch.setattr(clock_service.signal, 'signal', lambda *args: None)
    with pytest.raises(KeyboardInterrupt):
        clock_service.serve('eth0', '192.0.2.1', tmp_path)
    state = records[0]
    assert state['status_version'] == 2
    assert state['clock_ready'] is clock_ok
    assert state['link_ready'] is link
    assert state['tools_ready'] is tools_ok
    assert not state['ready'] and not state['ptp_running']
