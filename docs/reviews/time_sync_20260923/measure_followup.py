import json,time,socket,select,struct,subprocess,statistics
from pathlib import Path
from collections import Counter
out=Path('/home/nkb/colcon_ws/log/codex/time_sync_20260923')
# Wait for the administrator's concrete compatibility update, then settle.
deadline=time.monotonic()+300
while time.monotonic()<deadline:
    if 'ptp_minor_version 0' in Path('/run/icart-clock/ptp4l.conf').read_text():break
    time.sleep(1)
else:raise SystemExit('PTP compatibility update not applied')
time.sleep(1)
counts={k:Counter() for k in ['lidar','imu']};ages={k:[] for k in counts};last={};back=Counter();maxgap={};states=[];socks=[];events=[];previous_headers={}
try:
    for port in (56301,56401):
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.setsockopt(socket.SOL_SOCKET,35,1)
        s.bind(('192.168.1.5',port));socks.append(s)
    start=time.monotonic();end=start+60;nextstate=start
    while time.monotonic()<end:
        ready,_,_=select.select(socks,[],[],.1)
        for sock in ready:
            data,anc,_,peer=sock.recvmsg(65535,128)
            if peer[0]!='192.168.1.201' or len(data)<36:continue
            kind='imu' if sock.getsockname()[1]==56401 else 'lidar'
            mode=data[11];stamp=struct.unpack_from('<Q',data,28)[0]/1e9
            received=next((struct.unpack('qq',d[:16]) for l,t,d in anc if l==socket.SOL_SOCKET and t==35),None)
            if received is None:continue
            wall=received[0]+received[1]/1e9
            counts[kind][mode]+=1;ages[kind].append(wall-stamp)
            if kind in last:
                if stamp<last[kind]:
                    back[kind]+=1
                    events.append(dict(kind=kind,delta_ns=round((stamp-last[kind])*1e9),header=data[:28].hex(),previous_header=previous_headers[kind],received=wall))
                maxgap[kind]=max(maxgap.get(kind,0),stamp-last[kind])
            last[kind]=stamp;previous_headers[kind]=data[:28].hex()
        now=time.monotonic()
        if now>=nextstate:
            states.append(json.loads(Path('/run/icart-clock/status.json').read_text()));nextstate=now+1
finally:
    for s in socks:s.close()
summary={}
for k,values in ages.items():
    values.sort()
    summary[k]=dict(time_type_counts=dict(counts[k]),count=len(values),backwards=back[k],max_timestamp_gap_s=maxgap.get(k))
    if values:summary[k].update(min_s=values[0],median_s=statistics.median(values),p99_s=values[int(.99*(len(values)-1))],max_s=values[-1],stdev_s=statistics.pstdev(values))
result=dict(events=events,duration_s=60,packet_metrics=summary,gate_ready_samples=sum(s['ready'] for s in states),gate_samples=len(states),precision_verified=False)
(out/'ptp_60s_followup.json').write_text(json.dumps(result,indent=2)+'\n')
(out/'ptp_service_followup.json').write_text(json.dumps(states,indent=2)+'\n')
for name,cmd in [('chrony_tracking',['chronyc','tracking']),('chrony_sources',['chronyc','-n','sources','-v'])]:
    (out/(name+'.txt')).write_text(subprocess.check_output(cmd,text=True))
print(json.dumps(result,indent=2))
