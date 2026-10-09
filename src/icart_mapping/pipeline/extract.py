#!/usr/bin/env python3
"""Extract numerical observations read-only; no ROS node is started."""
import json, sqlite3, sys, time
from pathlib import Path
import numpy as np, yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from common import ROOT, CONFIG
BAGS=[Path(CONFIG['bag']['directory'])]
TOPICS={'/lio/odometry_raw':'lio','/rtk_gps/fix':'fix','/rtk_gps/rtk_status':'status','/mid360/livox/imu':'imu','/ypspur_ros/odom':'wheel'}
def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def extract(bag,idx):
 out=ROOT/f'obs_{idx}.npz'
 if out.exists(): return
 result={n:[] for n in TOPICS.values()}; extra={'source':str(bag),'tf':[],'alignment':[],'episode':[]}; classes={}
 meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
 wanted=set(TOPICS)|{'/tf_static','/lio/alignment_status','/dataset/source_episode'}; last_align=None
 for f in meta['relative_file_paths']:
  with sqlite3.connect((bag/f).as_uri()+'?mode=ro',uri=True) as db:
   topics={i:(n,t) for i,n,t in db.execute('select id,name,type from topics') if n in wanted}
   for tid,ns,blob in db.execute('select topic_id,timestamp,data from messages where topic_id in ('+','.join('?'*len(topics))+') order by timestamp',tuple(topics)):
    n,t=topics[tid]
    if t not in classes: classes[t]=get_message(t)
    m=deserialize_message(blob,classes[t])
    if n in TOPICS:
     k=TOPICS[n]; ts=stamp(m)
     if k in ['lio','wheel']:
      p,q=m.pose.pose.position,m.pose.pose.orientation
      row=[ts,p.x,p.y,p.z,q.x,q.y,q.z,q.w,ns*1e-9]
     elif k=='fix': row=[ts,m.latitude,m.longitude,m.altitude,*m.position_covariance,ns*1e-9]
     elif k=='status':row=[ts,m.rtk_state,m.num_satellites,m.hdop,m.heading_deg,m.heading_stddev_deg,m.baseline_length_m,m.correction_age_s,ns*1e-9]
     else:
      a,w=m.linear_acceleration,m.angular_velocity;row=[ts,a.x,a.y,a.z,w.x,w.y,w.z,ns*1e-9]
     result[k].append(row)
    elif n=='/tf_static':extra['tf'].append([ns,str(m)])
    elif n=='/dataset/source_episode':extra['episode'].append([ns,m.data])
    elif m.data!=last_align:extra['alignment'].append([ns,m.data]);last_align=m.data
  print('extracted',idx,f,flush=True)

 for key in ('lio','fix','status','imu'):
  if len(result[key])<2:raise ValueError('記録数不足: '+key)
 for key in ('fix','status'):
  result[key]=sorted(result[key],key=lambda row:row[0])
 np.savez_compressed(out,**{k:np.array(v) for k,v in result.items()})
 extra['counts']={k:len(v) for k,v in result.items()}
 (ROOT/f'audit_{idx}.json').write_text(json.dumps(extra,indent=2,ensure_ascii=False))
 print(extra['counts'],flush=True)
if __name__=='__main__':
 for idx,bag in enumerate(BAGS):extract(bag,idx)
