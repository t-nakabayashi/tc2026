"""EthernetのみのMID-360 PTP試験。OS設定変更は生成物の手動適用に分離する。"""

import argparse
from collections import Counter, deque
import ipaddress
import json
import math
from pathlib import Path
import re
import shutil
import signal
import socket
import struct
import subprocess
import time


PTP_CONFIG = '''[global]
# UTCを配信する試験用ソフトウェアPTP。PHCは操作しない。
time_stamping software
network_transport UDPv4
delay_mechanism E2E
domainNumber 0
# MID360 IEEE1588-2008 interoperability; linuxptp 4 defaults to minor version 1.
ptp_minor_version 0
twoStepFlag 1
BMCA noop
serverOnly 1
clockClass 248
clockAccuracy 0xFE
logSyncInterval -3
logAnnounceInterval 0
'''


def interface_name(value: str) -> str:
    if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', value):
        raise argparse.ArgumentTypeError('不正なNIC名')
    return value


def positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or not 0 < result <= 86400:
        raise argparse.ArgumentTypeError('0より大きく86400以下の有限値が必要')
    return result


def prepare(output: Path, sock: str) -> None:
    """新規ディレクトリに設定を生成する。既存OS設定へは書き込まない。"""
    if not re.fullmatch(r'/[A-Za-z0-9_./-]+', sock) or len(sock.encode()) > 100:
        raise ValueError('Unix socketは空白なしの絶対パス（100 bytes以下）で指定する')
    output.mkdir(parents=True, exist_ok=False)
    (output/'ptp4l.conf').write_text(PTP_CONFIG)
    (output/'chrony.conf').write_text(
        '# 既存chronyの設定へ取り込む断片。NMEA遅延は実測してoffsetを調整する。\n'
        f'refclock SOCK {sock} refid UM98 poll 0 filter 4 precision 0.1 offset 0.0\n')
    (output/'gnss.yaml').write_text(
        '/rtk_gps/rtk_gps_um982_node:\n  ros__parameters:\n'
        '    time_sync.enabled: true\n'
        f'    time_sync.chrony_socket: "{sock}"\n'
        '    stamp_source: gnss_utc\n    transport_delay_ms: 0\n'
        '    use_sim_time: false\n')


def command(args: list[str]) -> str:
    return subprocess.run(args, check=True, text=True, capture_output=True, timeout=5).stdout


def chrony_ready(sources: str, tracking: str, max_offset: float) -> bool:
    """UM98の選択・鮮度・時計収束を確認する。絶対精度の保証ではない。"""
    selected = False
    for line in sources.splitlines():
        fields = line.split()
        if len(fields) >= 6 and fields[:2] == ['#*', 'UM98']:
            selected = fields[5].isdigit() and int(fields[5]) <= 8
    offset = re.search(r'System time\s*:\s*([\d.eE+-]+) seconds', tracking)
    normal = re.search(r'Leap status\s*:\s*Normal\b', tracking)
    return bool(selected and offset and normal
                and math.isfinite(float(offset[1])) and abs(float(offset[1])) <= max_offset)


def ntp_ready(sources: str, tracking: str, max_offset: float) -> bool:
    """NTP trial gate: fresh selected server and estimated total error <=35 ms.

    This bound assumes the upstream clock is correct, not independent proof.
    """
    selected = False
    for line in sources.splitlines():
        fields = line.split()
        if len(fields) >= 6 and fields[0] == '^*':
            try:
                poll, reach, age = int(fields[3]), int(fields[4], 8), int(fields[5])
                selected = (reach & 1 != 0 and 0 <= age <= min(256, max(16, 2**(poll+1))))
            except ValueError:
                pass
    values = []
    for key in ('System time', 'Root delay', 'Root dispersion'):
        match = re.search(re.escape(key)+r'\s*:\s*([\d.eE+-]+) seconds', tracking)
        if not match:
            return False
        value = float(match[1])
        if not math.isfinite(value):
            return False
        values.append(value)
    offset, delay, dispersion = values
    return bool(selected and re.search(r'Leap status\s*:\s*Normal\b', tracking)
                and abs(offset) <= max_offset and delay >= 0 and dispersion >= 0
                and abs(offset)+delay/2+dispersion <= .035)


def check(interface: str, max_offset: float, clock_source: str = 'gnss') -> dict:
    """NICとchronyを読み取り、試験開始条件を返す。"""
    nic = Path('/sys/class/net')/interface
    tools = {name: shutil.which(name) is not None for name in ('ptp4l', 'chronyc', 'ethtool')}
    result = dict(interface=interface, tools=tools, link=False, clock_ready=False)
    try:
        result['link'] = (nic/'carrier').read_text().strip() == '1'
        result['timestamping'] = command(['ethtool', '-T', interface])
        result['sources'] = command(['chronyc', '-n', 'sources'])
        result['tracking'] = command(['chronyc', 'tracking'])
        validator = ntp_ready if clock_source == 'ntp' else chrony_ready
        result['clock_ready'] = validator(result['sources'], result['tracking'], max_offset)
    except (OSError, subprocess.SubprocessError) as exc:
        result['error'] = str(exc)
    result['ready'] = bool(all(tools.values()) and result['link'] and result['clock_ready'])
    return result


def run(interface: str, output: Path, duration: float, max_offset: float) -> None:
    """前提喪失で自身のPTP子プロセスを停止する。自動的な再開は行わない。"""
    state = check(interface, max_offset)
    if not state['ready']:
        raise RuntimeError('PTP開始条件を満たさない:\n'+json.dumps(state, ensure_ascii=False))
    output.mkdir(parents=True, exist_ok=False)
    cfg = output/'ptp4l.conf'
    if re.search(r'\s', str(output.resolve())):
        raise ValueError('PTP出力ディレクトリに空白は使用できない')
    cfg.write_text(PTP_CONFIG + f'uds_address {output.resolve()}/ptp4l.sock\n')
    if len(str(output.resolve()/'ptp4l.sock').encode()) > 100:
        raise ValueError('出力ディレクトリのパスが長すぎる')
    (output/'preflight.json').write_text(json.dumps(state, ensure_ascii=False, indent=2))
    proc = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    old = signal.signal(signal.SIGTERM, interrupted)
    try:
        with (output/'ptp4l.log').open('w') as log:
            proc = subprocess.Popen(['ptp4l', '-S', '-i', interface, '-f', str(cfg), '-m'],
                                    stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+duration
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(f'ptp4lが終了した: {proc.returncode}。ログを確認する')
                state = check(interface, max_offset)
                if not state['ready']:
                    raise RuntimeError('GNSS時計またはNICの開始条件を喪失。PTPを停止する')
                time.sleep(min(1., max(0., deadline-time.monotonic())))
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        signal.signal(signal.SIGTERM, old)


def mid360_packet(frame: bytes, lidar_ip: str) -> tuple[str, int, int] | None:
    """Ethernet/IPv4/UDPフレームからMID-360の種別・時刻を抽出する。"""
    if len(frame) < 14:
        return None
    offset = 14
    protocol = struct.unpack_from('!H', frame, 12)[0]
    if protocol == 0x8100 and len(frame) >= 18:
        protocol = struct.unpack_from('!H', frame, 16)[0]
        offset = 18
    if protocol != 0x0800 or len(frame) < offset+20:
        return None
    ip = frame[offset:]
    ihl = (ip[0] & 15)*4
    size = struct.unpack_from('!H', ip, 2)[0]
    if (ip[0] >> 4 != 4 or ihl < 20 or size < ihl+8 or len(ip) < size or ip[9] != 17
            or struct.unpack_from('!H', ip, 6)[0] & 0x3FFF
            or socket.inet_ntoa(ip[12:16]) != lidar_ip):
        return None
    udp = ip[ihl:size]
    port, _, length = struct.unpack_from('!HHH', udp)
    if port not in (56300, 56400) or length < 44 or length > len(udp):
        return None
    payload = udp[8:length]
    if payload[0] != 0 or struct.unpack_from('<H', payload, 1)[0] != len(payload):
        return None
    kind = 'imu' if port == 56400 else 'lidar'
    data_type = payload[10]
    count = struct.unpack_from('<H', payload, 5)[0]
    stride = {0: 24, 1: 14, 2: 8, 3: 10}.get(data_type)
    if (stride is None or count == 0 or len(payload) != 36+count*stride
            or (kind == 'imu') != (data_type == 0)):
        return None
    return kind, payload[11], struct.unpack_from('<Q', payload, 28)[0]


def monitor(interface: str, lidar_ip: str, duration: float, output: Path) -> dict:
    """ドライバのUDPポートを奪わずAF_PACKETで受動観測する。"""
    counts = {kind: Counter() for kind in ('lidar', 'imu')}
    ranges = {kind: deque(maxlen=10000) for kind in counts}
    bad_epoch = Counter()
    last = {}
    first_receive = {}
    last_receive = {}
    max_gap = {}
    backwards = Counter()
    with output.open('x') as log, socket.socket(
            socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3)) as sock:
        sock.bind((interface, 0))
        sock.settimeout(.5)
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            try:
                frame = sock.recv(65535)
            except socket.timeout:
                continue
            received = time.time()
            parsed = mid360_packet(frame, lidar_ip)
            if parsed is None:
                continue
            kind, sync, stamp = parsed
            first_receive.setdefault(kind, received)
            if kind in last_receive:
                max_gap[kind] = max(max_gap.get(kind, 0.), received-last_receive[kind])
            last_receive[kind] = received
            counts[kind][sync] += 1
            age = received-stamp/1e9
            if not -.05 <= age <= .5:
                bad_epoch[kind] += 1
            ranges[kind].append(age)
            if kind in last and stamp < last[kind]:
                backwards[kind] += 1
            last[kind] = stamp
            log.write(json.dumps(dict(kind=kind, time_type=sync, timestamp_ns=stamp,
                                      received_unix=received, receive_minus_stamp_s=age))+'\n')
    result = dict(counts={k: dict(v) for k, v in counts.items()}, backwards=dict(backwards))
    result['implausible_epoch_or_delay'] = dict(bad_epoch)
    result['ptp_seen_on_both'] = all(c[1] > 0 and sum(c.values()) == c[1] for c in counts.values())
    result['receive_minus_stamp_s'] = {}
    for kind, ages in ranges.items():
        if ages:
            ages = sorted(ages)
            result['receive_minus_stamp_s'][kind] = dict(
                min=ages[0], max=ages[-1], p50=ages[len(ages)//2],
                p99=ages[min(len(ages)-1, int(len(ages)*.99))])
    result['stream_duration_s'] = {k: last_receive[k]-v for k, v in first_receive.items()}
    result['max_receive_gap_s'] = max_gap
    result['capture_usable'] = capture_usable(result, duration)
    result['precision_verified'] = False
    output.with_suffix(output.suffix+'.summary.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2))
    return result


def capture_usable(result: dict, duration: float) -> bool:
    """PTPモードだけで成功にせず、両ストリームの連続性・時系も確認する。"""
    return bool(result['ptp_seen_on_both'] and not result['backwards']
        and not result['implausible_epoch_or_delay']
        and all(result['stream_duration_s'].get(k, 0.) >= max(1., .8*duration)
                and result['max_receive_gap_s'].get(k, float('inf')) <= .5
                for k in ('lidar', 'imu')))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--socket', default='/run/chrony/um982.sock')
    for name in ('check', 'run', 'monitor'):
        p = sub.add_parser(name)
        p.add_argument('--interface', required=True, type=interface_name)
        if name in ('check', 'run'):
            p.add_argument('--max-clock-offset', type=positive, default=.05)
        if name in ('run', 'monitor'):
            p.add_argument('--duration', type=positive, default=60.)
            p.add_argument('--output', type=Path, required=True)
        if name == 'monitor':
            p.add_argument('--lidar-ip', required=True, type=lambda value: str(ipaddress.IPv4Address(value)))
    args = parser.parse_args()
    try:
        if args.action == 'prepare':
            prepare(args.output, args.socket)
        elif args.action == 'check':
            state = check(args.interface, args.max_clock_offset)
            print(json.dumps(state, ensure_ascii=False, indent=2))
            if not state['ready']:
                raise SystemExit(2)
        elif args.action == 'run':
            run(args.interface, args.output, args.duration, args.max_clock_offset)
        else:
            result = monitor(args.interface, args.lidar_ip, args.duration, args.output)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            if not result['capture_usable']:
                raise SystemExit(2)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, str(exc)+'\n')
    except KeyboardInterrupt:
        parser.exit(130, 'PTP試験を停止した\n')


if __name__ == '__main__':
    main()
