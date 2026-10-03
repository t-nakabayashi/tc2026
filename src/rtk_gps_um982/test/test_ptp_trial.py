"""PTP開始条件、異常終了時の後始末、MID-360生パケットを検証する。"""

from pathlib import Path
import socket
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from rtk_gps_um982 import ptp_trial as trial

SOURCES = '#* UM98 0 0 377 1 +2ms[+2ms] +/- 100ms'
TRACKING = 'System time     : 0.002 seconds slow of NTP time\nLeap status     : Normal\n'


def test_clock_requires_selected_fresh_gnss():
    assert trial.chrony_ready(SOURCES, TRACKING, .05)
    # filter=4 timestamps represent the sample window, not its publish time.
    assert trial.chrony_ready(SOURCES.replace('377 1', '377 6'), TRACKING, .05)
    for sources in [SOURCES.replace('#*', '#?'), SOURCES.replace('UM98', 'GPS'),
                    SOURCES.replace('377 1', '377 9'), SOURCES.replace('377 1', '377 1m')]:
        assert not trial.chrony_ready(sources, TRACKING, .05)
    assert not trial.chrony_ready(SOURCES, TRACKING.replace('Normal', 'Not synchronised'), .05)
    assert not trial.chrony_ready(SOURCES, TRACKING.replace('0.002', '37'), .05)


def test_prepare_no_overwrite_and_no_phc_servo(tmp_path):
    output = tmp_path/'session'
    trial.prepare(output, '/run/chrony/um982.sock')
    assert 'time_stamping software' in (output/'ptp4l.conf').read_text()
    assert 'ptp_minor_version 0' in (output/'ptp4l.conf').read_text()
    assert 'serverOnly 1' in (output/'ptp4l.conf').read_text()
    assert 'refid UM98' in (output/'chrony.conf').read_text()
    with pytest.raises(FileExistsError):
        trial.prepare(output, '/run/chrony/um982.sock')
    with pytest.raises(ValueError):
        trial.prepare(tmp_path/'bad', '/tmp/socket\nmakestep 1 -1')


def test_run_refuses_unready_clock(tmp_path, monkeypatch):
    monkeypatch.setattr(trial, 'check', lambda *_: {'ready': False})
    with pytest.raises(RuntimeError, match='開始条件'):
        trial.run('eth0', tmp_path/'out', 1., .05)
    assert not (tmp_path/'out').exists()


def test_run_stops_owned_process_on_clock_loss(tmp_path, monkeypatch):
    states = iter([{'ready': True}, {'ready': False}])
    monkeypatch.setattr(trial, 'check', lambda *_: next(states))
    class Process:
        stopped = False
        waited = False
        def poll(self):
            return None
        def terminate(self):
            self.stopped = True
        def wait(self, **kwargs):
            self.waited = True
    process = Process()
    monkeypatch.setattr(trial.subprocess, 'Popen', lambda *a, **kw: process)
    with pytest.raises(RuntimeError, match='喪失'):
        trial.run('eth0', tmp_path/'out', 1., .05)
    assert process.stopped and process.waited


def packet(kind=0, sync=1, stamp=1700000000000000000):
    count = 1
    payload = bytearray(36+{0:24, 1:14}[kind])
    struct.pack_into('<H', payload, 1, len(payload))
    struct.pack_into('<H', payload, 5, count)
    payload[10], payload[11] = kind, sync
    struct.pack_into('<Q', payload, 28, stamp)
    udp = struct.pack('!HHHH', 56400 if kind == 0 else 56300, 56401, 8+len(payload), 0)+payload
    ip = bytearray(20)
    ip[0], ip[9] = 0x45, 17
    struct.pack_into('!H', ip, 2, 20+len(udp))
    ip[12:16] = socket.inet_aton('192.168.1.123')
    return bytes(12)+b'\x08\x00'+ip+udp


@pytest.mark.parametrize('kind,name', [(0,'imu'), (1,'lidar')])
@pytest.mark.parametrize('sync', [0,1,2])
def test_packet_preserves_sync_type_and_epoch(kind, name, sync):
    assert trial.mid360_packet(packet(kind, sync), '192.168.1.123') == (
        name, sync, 1700000000000000000)


def test_packet_ignores_other_lidar_truncated_and_fragmented():
    frame = packet()
    assert trial.mid360_packet(frame, '192.168.1.124') is None
    for length in range(len(frame)):
        assert trial.mid360_packet(frame[:length], '192.168.1.123') is None
    damaged = bytearray(frame)
    damaged[20] = 0x20
    assert trial.mid360_packet(damaged, '192.168.1.123') is None


def test_ntp_trial_requires_selected_fresh_server_and_error_bound():
    sources = '^* 192.0.2.1 2 6 377 12 +2ms[+2ms] +/- 7ms'
    tracking = TRACKING + 'Root delay : 0.010 seconds\nRoot dispersion : 0.002 seconds\n'
    assert trial.ntp_ready(sources, tracking, .005)
    assert not trial.ntp_ready(SOURCES, tracking, .005)
    for bad in [sources.replace('^*','^?'), sources.replace('377 12','376 12'),
                sources.replace('377 12','377 129'), sources.replace('377 12','377 2m')]:
        assert not trial.ntp_ready(bad, tracking, .005)
    for bad in [tracking.replace('0.010','0.100'), tracking.replace('0.002','0.040'),
                tracking.replace('Normal','Not synchronised'), TRACKING]:
        assert not trial.ntp_ready(sources, bad, .005)


@pytest.mark.parametrize('error,ready', [(.020001, True), (.034999, True),
                                       (.035, True), (.035001, False)])
def test_ntp_estimated_error_limit_is_35_ms(error, ready):
    sources = '^* 192.0.2.1 2 7 377 12 +0ms'
    tracking = ('System time : 0 seconds\nLeap status : Normal\n'
                f'Root delay : 0 seconds\nRoot dispersion : {error} seconds\n')
    assert trial.ntp_ready(sources, tracking, .005) is ready


def test_ntp_error_budget_includes_offset_delay_and_dispersion():
    sources = '^* 192.0.2.1 2 7 377 12 +2ms'
    tracking = (TRACKING + 'Root delay : 0.040 seconds\n'
                'Root dispersion : 0.012 seconds\n')
    assert trial.ntp_ready(sources, tracking, .005)
    assert not trial.ntp_ready(sources, tracking.replace('0.012', '0.014'), .005)
    assert not trial.ntp_ready(sources, tracking.replace('0.002', '0.006'), .005)


def test_ntp_maxpoll_seven_keeps_normal_cycle_fresh_but_rejects_stale_data():
    tracking = TRACKING + 'Root delay : 0.010 seconds\nRoot dispersion : 0.002 seconds\n'
    for age in (0, 127, 128, 129, 255, 256):
        assert trial.ntp_ready(f'^* 192.0.2.1 2 7 377 {age} +2ms', tracking, .005)
    for age in (257, 653):
        assert not trial.ntp_ready(f'^* 192.0.2.1 2 7 377 {age} +2ms', tracking, .005)
