#!/usr/bin/env python3
"""Build GNSS-blind keyframes, deskewed using recorded FAST-LIO relative motion."""
import json, sqlite3, struct, sys
from pathlib import Path
import numpy as np, yaml
from scipy.spatial.transform import Rotation
from common import *
from extract import BAGS

def livox(blob):
 sec,ns,n=struct.unpack_from('<iII',blob,4);p=16+n;p=4+((p-4+7)//8)*8
 base,count=struct.unpack_from('<QI',blob,p);p+=16
 length=struct.unpack_from('<I',blob,p)[0];p+=4
 dtype=np.dtype({'names':['offset','x','y','z','intensity','tag','line'],'formats':['<u4','<f4','<f4','<f4','u1','u1','u1'],'offsets':[0,4,8,12,16,17,18],'itemsize':20})
 pts=np.frombuffer(blob+b'\0',dtype=dtype,count=length,offset=p)
 assert length==count
 return sec+ns*1e-9,pts

def main():
 (ROOT/'clouds').mkdir(exist_ok=True);frames=[];segments=[];counter=0
 # Full-course bag only. Additional run is held separately for later verification.
 for bi,bag in enumerate(BAGS[:1]):
  d=np.load(ROOT/f'obs_{bi}.npz');rows=d['lio'];imu=d['imu']
  dt=np.diff(rows[:,0]);dx=np.linalg.norm(np.diff(rows[:,1:4],axis=0),axis=1)
  breaks=np.r_[0,np.where((dt>10)|(dt<=0)|(dx>2))[0]+1,len(rows)]
  trs=[]
  for a,b in zip(breaks[:-1],breaks[1:]):
   r=rows[a:b];tr=Trajectory(r)
   # Gravity from first 20 s, excluding high angular velocity; rotate into LIO world.
   ii=(imu[:,0]>=r[0,0])&(imu[:,0]<=min(r[0,0]+20,r[-1,0]))&(np.linalg.norm(imu[:,4:7],axis=1)<.15)
   it=imu[ii][::5];world=np.einsum('nij,nj->ni',tr.at(it[:,0])[:,:3,:3],it[:,1:4])
   g=np.median(world/np.linalg.norm(world,axis=1)[:,None],axis=0);g/=np.linalg.norm(g)
   rot=Rotation.align_vectors([[0,0,1]],[g])[0].as_matrix();level=np.eye(4);level[:3,:3]=rot;level[:3,3]=-rot@r[0,1:4]
   sid=len(segments);segments.append({'id':sid,'bag':bi,'start':r[0,0],'end':r[-1,0],'rows':int(b-a),'gravity':g.tolist(),'level':level.tolist(),'gravity_samples':len(it)})
   trs.append((sid,tr,level))
  if (ROOT/'keyframes.json').exists():
   (ROOT/'segments.json').write_text(json.dumps(segments,indent=2));print('Metadata recovered',flush=True);return
  meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];last={};checked=False
  for f in meta['relative_file_paths']:
   with sqlite3.connect(f'file:{bag/f}?mode=ro',uri=True) as db:
    tid=db.execute("select id from topics where name='/mid360/livox/lidar'").fetchone()[0]
    for ns,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
     t,pts=livox(blob)
     if not checked:
      from rclpy.serialization import deserialize_message
      from livox_ros_driver2.msg import CustomMsg
      m=deserialize_message(blob,CustomMsg)
      for j in [0,len(m.points)//2,len(m.points)-1]:
       assert pts[j]['x']==m.points[j].x and pts[j]['offset']==m.points[j].offset_time and pts[j]['line']==m.points[j].line
      checked=True
     end=t+float(pts['offset'].max())*1e-9;ref=(t+end)/2
     entry=next(((sid,tr,lv) for sid,tr,lv in trs if t>=tr.t[0] and end<=tr.t[-1]),None)
     if entry is None:continue
     sid,tr,lv=entry
     ix=np.searchsorted(tr.t,[t,end]);localdt=np.diff(tr.t[max(0,ix[0]-1):min(len(tr.t),ix[1]+1)])
     if len(localdt) and max(localdt)>.3:continue
     raw=tr.at(ref);pose=lv@raw
     if sid in last:
      lt,lp=last[sid];move=np.linalg.norm(pose[:3,3]-lp[:3,3]);angle=Rotation.from_matrix(lp[:3,:3].T@pose[:3,:3]).magnitude()
      if move<1.2 and angle<np.radians(12) and ref-lt<5:continue
     xyz=np.c_[pts['x'],pts['y'],pts['z']].astype(float);r2=(xyz*xyz).sum(1)
     valid=np.isfinite(r2)&(r2>.8**2)&(r2<65**2)&np.isin(pts['tag']&0x30,[0,0x10])
     xyz=xyz[valid];tt=t+pts['offset'][valid].astype(float)*1e-9
     motion=tr.at(tt);world=np.einsum('nij,nj->ni',motion[:,:3,:3],xyz+LIDAR_T)+motion[:,:3,3]
     deskew=(world-raw[:3,3])@raw[:3,:3];cloud=np.c_[deskew,pts['intensity'][valid]]
     cloud=voxel(cloud,.15).astype(np.float32)
     if len(cloud)<1000:continue
     np.save(ROOT/'clouds'/f'{counter:05d}.npy',cloud)
     frames.append({'id':counter,'t':ref,'segment':sid,'pose':pose.tolist(),'raw':raw.tolist(),'points':len(cloud)})
     last[sid]=(ref,pose);counter+=1
   print('keyframes',f,counter,flush=True)
 (ROOT/'keyframes.json').write_text(json.dumps(frames))
 (ROOT/'segments.json').write_text(json.dumps(segments,indent=2))
 print('DONE',len(frames),'frames',len(segments),'segments',flush=True)
if __name__=='__main__':main()
