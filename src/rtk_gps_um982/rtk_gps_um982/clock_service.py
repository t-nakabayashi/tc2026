"""NTP-selected PC -> software PTP, with a live raw-packet startup gate.
Run as the dedicated systemd service; no ROS or actuator interfaces.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import struct
import time
try:
    from .ptp_trial import PTP_CONFIG, check, mid360_packet, interface_name
except ImportError:
    from ptp_trial import PTP_CONFIG, check, mid360_packet, interface_name


class PacketHealth:
    def __init__(self):
        self.rows = {k: deque(maxlen=12000) for k in ('lidar', 'imu')}
        self.previous = {}
        self.bad_until = 0.
        self.minor_backwards = {k: 0 for k in self.rows}

    def observe(self, packet, wall, mono):
        kind, mode, stamp = packet
        age = wall-stamp/1e9
        # Compare against the high-water mark: repeated tiny regressions must
        # not accumulate into an undetected large reversal. Preserve raw stamps.
        previous = self.previous.get(kind, stamp)
        regression = previous - stamp
        backwards = regression > 100_000  # 0.1 ms; measured jitter was <=14 us.
        if 0 < regression <= 100_000:
            self.minor_backwards[kind] += 1
        self.previous[kind] = max(previous, stamp)
        if mode != 1 or not math.isfinite(age) or not -.001 <= age <= .01 or backwards:
            self.bad_until = mono+10.
        self.rows[kind].append((mono, age, mode))

    def snapshot(self, mono):
        metrics = {}
        ready = mono >= self.bad_until
        for kind, rows in self.rows.items():
            while rows and mono-rows[0][0] > 5.:
                rows.popleft()
            ages = sorted(r[1] for r in rows)
            good = (len(rows) >= 2 and rows[-1][0]-rows[0][0] >= 4.
                    and mono-rows[-1][0] <= .2 and all(r[2] == 1 for r in rows)
                    and max(b[0]-a[0] for a,b in zip(rows, list(rows)[1:])) <= .2)
            ready = ready and good
            metrics[kind] = dict(count=len(rows), ready=bool(good),
                                 minor_backwards=self.minor_backwards[kind])
            if ages:
                metrics[kind].update(min_s=ages[0], p50_s=ages[len(ages)//2],
                    p99_s=ages[min(len(ages)-1,int(len(ages)*.99))], max_s=ages[-1])
        return bool(ready), metrics


def atomic_status(path, data):
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    temp.chmod(0o644)
    temp.replace(path)


def serve(interface, lidar_ip, directory, clock_source='ntp'):
    directory.mkdir(parents=True,exist_ok=True)
    cfg=directory/'ptp4l.conf'
    cfg.write_text(PTP_CONFIG+f'uds_address {directory}/ptp4l.sock\nuds_ro_address {directory}/ptp4lro.sock\n')
    status=directory/'status.json'
    proc=None; log=None; health=PacketHealth(); next_check=0.; clock_ok=False
    def stop_signal(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop_signal)
    def stop_ptp():
        nonlocal proc,log,health
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        if log:log.close()
        proc=None;log=None;health=PacketHealth()
    try:
        with socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(3)) as sock:
            sock.bind((interface,0));sock.settimeout(.1)
            sock.setsockopt(socket.SOL_SOCKET, 35, 1)  # Linux SO_TIMESTAMPNS, kernel RX time
            while True:
                mono=time.monotonic()
                if mono >= next_check:
                    state=check(interface,.005 if clock_source == 'ntp' else .01,clock_source)
                    clock_ok=state['ready']
                    if not clock_ok:
                        stop_ptp()
                    elif proc is None:
                        log=(directory/'ptp4l.log').open('a')
                        proc=subprocess.Popen(['ptp4l','-S','-i',interface,'-f',str(cfg),'-m'],stdout=log,stderr=log)
                        health=PacketHealth()
                    elif proc.poll() is not None:
                        stop_ptp()
                    packet_ok,metrics=health.snapshot(mono)
                    atomic_status(status,dict(checked_unix=time.time(),checked_monotonic=mono,
                        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                        ready=bool(clock_ok and proc is not None and proc.poll() is None and packet_ok),
                        status_version=2,clock_source=clock_source,
                        clock_ready=state['clock_ready'],link_ready=state['link'],
                        tools_ready=all(state['tools'].values()),check_error=state.get('error'),
                        gnss_clock_ready=state['clock_ready'] if clock_source == 'gnss' else False,
                        ptp_running=proc is not None and proc.poll() is None,
                        packet_metrics=metrics,precision_verified=False,
                        note='Receive latency is not an independently measured absolute clock offset.'))
                    next_check=time.monotonic()+1.
                try:frame,ancillary,_,_=sock.recvmsg(65535,128)
                except socket.timeout:continue
                packet=mid360_packet(frame,lidar_ip)
                if packet and proc is not None:
                    received = next((struct.unpack('qq',data[:16]) for level,kind,data in ancillary
                                     if level==socket.SOL_SOCKET and kind==35 and len(data)>=16), None)
                    if received is not None:
                        health.observe(packet,received[0]+received[1]/1e9,time.monotonic())
    finally:
        stop_ptp()
        atomic_status(status,dict(checked_unix=time.time(),ready=False,reason='service stopped'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',required=True,type=interface_name)
    p.add_argument('--lidar-ip',required=True)
    p.add_argument('--directory',type=Path,default=Path('/run/icart-clock'))
    p.add_argument('--clock-source',choices=['gnss','ntp'],default='ntp')
    args=p.parse_args()
    try:serve(args.interface,args.lidar_ip,args.directory,args.clock_source)
    except KeyboardInterrupt:pass

if __name__=='__main__':main()
