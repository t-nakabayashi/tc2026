import json,time,statistics
from pathlib import Path
import rclpy
from sensor_msgs.msg import NavSatFix
rclpy.init();node=rclpy.create_node('ntp_gnss_stamp_evaluation');rows=[]
def callback(msg):
 stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
 rows.append(dict(received=time.time(),stamp=stamp,status=msg.status.status))
sub=node.create_subscription(NavSatFix,'/rtk_gps/rtk_gps_um982_node/fix',callback,10)
end=time.monotonic()+240
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.2)
node.destroy_node();rclpy.shutdown()
ages=sorted(r['received']-r['stamp'] for r in rows)
result={'count':len(rows),'backwards':sum(b['stamp']<a['stamp'] for a,b in zip(rows,rows[1:])),'samples':rows}
if ages:result.update(min_s=min(ages),median_s=statistics.median(ages),p99_s=ages[int(.99*(len(ages)-1))],max_s=max(ages))
p=Path('/home/nkb/colcon_ws/log/codex/time_sync_20260923/ntp/gnss_stamps.json');p.parent.mkdir(exist_ok=True);p.write_text(json.dumps(result,indent=2)+'\n')
print({k:v for k,v in result.items() if k!='samples'})
