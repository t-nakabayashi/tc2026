#!/usr/bin/env python3
"""Post-hoc held-out GNSS comparison and GNSS-free map export.
GNSS is read only after graph optimization is complete; no feedback to graph.
"""
import csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from common import *

def stats(v):
 a=np.asarray(v);a=a[np.isfinite(a)]
 if not len(a):return {'n':0}
 return {'n':len(a),'rmse_m':float(np.sqrt(np.mean(a*a))),'median_m':float(np.median(a)),'p95_m':float(np.percentile(a,95)),'max_m':float(max(a))}

def main():
 fs=json.loads((ROOT/'keyframes.json').read_text());segments=json.loads((ROOT/'segments.json').read_text())
 kt=np.array([f['t'] for f in fs]);ks=np.array([f['segment'] for f in fs]);rawkf=np.array([f['raw'] for f in fs]);opt=np.load(ROOT/'optimized_poses.npy');initial=np.load(ROOT/'initial_poses.npy')
 d=np.load(ROOT/'obs_0.npz');fix=d['fix'];status=d['status'];lio=d['lio'];si=nearest(status[:,0],fix[:,0]);quality=status[si,1].astype(int)
 fresh=(abs(status[si,0]-fix[:,0])<.05);qualified=(quality==4)&fresh&(status[si,7]<2)&(status[si,2]>=10)&(status[si,3]<=2)
 origin=fix[0,1:4];gps=enu(fix[:,1:4],origin);valid=np.zeros(len(fix),bool);sid=np.full(len(fix),-1)
 pred={key:np.full((len(fix),4,4),np.nan) for key in ['lio','slam']}
 for s in segments:
  k=ks==s['id'];rr=lio[(lio[:,0]>=s['start'])&(lio[:,0]<=s['end'])];tr=Trajectory(rr)
  use=(fix[:,0]>=kt[k][0])&(fix[:,0]<=kt[k][-1]);ids=np.flatnonzero(use);at=fix[ids,0]
  ix=np.clip(np.searchsorted(tr.t,at),1,len(tr.t)-1);good=tr.t[ix]-tr.t[ix-1]<.3;ids=ids[good];at=fix[ids,0]
  valid[ids]=True;sid[ids]=s['id'];raw=tr.at(at)
  for key,poses in [('lio',initial),('slam',opt)]:
   correction=poses[k]@np.linalg.inv(rawkf[k]);c=np.tile(np.eye(4),(len(ids),1,1));c[:,:3,:3]=Slerp(kt[k],Rotation.from_matrix(correction[:,:3,:3]))(at).as_matrix()
   for j in range(3):c[:,j,3]=np.interp(at,kt[k],correction[:,j,3])
   pred[key][ids]=c@raw
 calibration=valid&qualified&(fix[:,0]<kt[0]+180)
 assert calibration.sum()>50
 heldout=valid&(fix[:,0]>=kt[0]+180)
 result={'ground_truth':False,'gnss_used_in_graph':False,'origin_llh':origin.tolist(),'altitude_datum':'Receiver-reported altitude; geoid datum not verified','calibration_duration_s':180,'calibration_count':int(calibration.sum()),'quality_counts':{str(q):int(np.sum(quality==q)) for q in np.unique(quality)},'valid_comparisons':int(valid.sum()),'excluded_for_time_or_gaps':int((~valid).sum()),'results':{},'segments':[]}
 aligned={};errors={};transforms={};postfit={};map_transform=None
 for key in pred:
  ant=pred[key][:,:3,3]+np.einsum('nij,j->ni',pred[key][:,:3,:3],IMU_ANT)
  rot,shift=fit_xy(ant[calibration,:2],gps[calibration,:2]);ap=ant[:,:2]@rot.T+shift
  error=np.linalg.norm(ap-gps[:,:2],axis=1);aligned[key]=ap;errors[key]=error
  transforms[key]={'rotation_xy':rot.tolist(),'translation_xy':shift.tolist(),'z_offset':float(np.median(gps[calibration,2]-ant[calibration,2]))}
  result['results'][key]={'calibration':stats(error[calibration]),'heldout_qualified_fix':stats(error[heldout&qualified]),'heldout_by_state':{str(q):stats(error[heldout&fresh&(quality==q)]) for q in np.unique(quality)}}
  rp={}
  for interval in [10,30,60]:
   j=nearest(fix[:,0],fix[:,0]+interval)
   u=heldout&qualified&heldout[j]&qualified[j]&(sid==sid[j])&(abs(fix[j,0]-fix[:,0]-interval)<.15)
   # Independent endpoint-displacement disagreement, no refitting per window.
   err=np.linalg.norm((ap[j]-ap)-(gps[j,:2]-gps[:,:2]),axis=1)
   rp[str(interval)]=stats(err[u])
  result['results'][key]['relative_displacement']=rp
 for s in segments:
  u=heldout&qualified&(sid==s['id']);result['segments'].append({'id':s['id'],'start_elapsed_s':s['start']-kt[0],'end_elapsed_s':s['end']-kt[0],'fix_disagreement':{key:stats(errors[key][u]) for key in pred}})
 # Distance-sampled FIX comparison avoids a long stationary period dominating the result.
 ds=np.zeros(len(fix));step=np.linalg.norm(np.diff(aligned['slam'],axis=0),axis=1)
 same=(sid[1:]==sid[:-1])&valid[1:]&valid[:-1]&(np.diff(fix[:,0])<.3)
 ds[1:]=np.where(same,step,0);arc=np.cumsum(ds);selected=[];seen=set()
 for i in np.flatnonzero(heldout&qualified):
  key=(int(sid[i]),int(arc[i]))
  if key not in seen:seen.add(key);selected.append(int(i))
 for key in pred:
  result['results'][key]['heldout_fix_per_meter']=stats(errors[key][selected])
  # Conventional globally aligned ATE is a shape diagnostic, not a held-out test.
  use=valid&qualified;rot,shift=fit_xy(aligned[key][use],gps[use,:2]);post=aligned[key]@rot.T+shift
  result['results'][key]['all_fix_rigid_alignment_diagnostic']=stats(np.linalg.norm(post[use]-gps[use,:2],axis=1))
  result['results'][key]['all_fix_by_state']={str(q):stats(np.linalg.norm(post[valid&fresh&(quality==q)]-gps[valid&fresh&(quality==q),:2],axis=1)) for q in np.unique(quality)}
  result['results'][key]['all_fix_per_meter']=stats(np.linalg.norm(post[selected]-gps[selected,:2],axis=1))
  postfit[key]=post
  if key=='slam':
   map_transform={'rotation_xy':(rot@np.array(transforms[key]['rotation_xy'])).tolist(),'translation_xy':(rot@np.array(transforms[key]['translation_xy'])+shift).tolist(),'z_offset':transforms[key]['z_offset'],'fit':'All qualified FIX, post-hoc rigid SE2 only, no graph feedback'}
  result['results'][key]['all_fix_additional_yaw_deg']=float(np.degrees(np.arctan2(rot[1,0],rot[0,0])))
 result['distance_sampled_fix_indices']=selected
 result['alignment']=transforms
 result['map_display_alignment']=map_transform
 # 0.1 s synchronization sensitivity; this is not used to tune the alignment.
 result['sync_sensitivity']={}
 for offset in [-.1,.1]:
  j=nearest(fix[:,0],fix[:,0]+offset);u=heldout&qualified&valid[j]&(sid==sid[j])&(abs(fix[j,0]-fix[:,0]-offset)<.05)
  result['sync_sensitivity'][str(offset)]=stats(np.linalg.norm(aligned['slam'][j]-gps[:,:2],axis=1)[u])
 with (ROOT/'trajectory_comparison.csv').open('w') as f:
  w=csv.writer(f);w.writerow(['stamp_utc','elapsed_s','segment','rtk_state','satellites','hdop','correction_age_s','qualified_fix','calibration','comparison_valid','gnss_e','gnss_n','lio_e','lio_n','slam_e','slam_n','lio_disagreement_m','slam_disagreement_m'])
  for i in range(len(fix)):w.writerow([fix[i,0],fix[i,0]-kt[0],sid[i],quality[i],*status[si[i],[2,3,7]],int(qualified[i]),int(calibration[i]),int(valid[i]),*gps[i,:2],*aligned['lio'][i],*aligned['slam'][i],errors['lio'][i],errors['slam'][i]])
 np.savez_compressed(ROOT/'comparison.npz',time=fix[:,0]-kt[0],gps=gps,quality=quality,qualified=qualified,heldout=heldout,calibration=calibration,valid=valid,segment=sid,slam=aligned['slam'],lio=aligned['lio'],slam_error=errors['slam'],lio_error=errors['lio'],slam_postfit=postfit['slam'],lio_postfit=postfit['lio'])
 (ROOT/'evaluation.json').write_text(json.dumps(result,indent=2))
 # Map in intrinsic gravity-aligned graph frame, and a separate ENU display export.
 allpoints=[]
 for i,p in enumerate(opt):
  xyz=np.load(ROOT/'clouds'/f'{i:05d}.npy');allpoints.append(np.c_[xyz[:,:3]@p[:3,:3].T+p[:3,3],xyz[:,3]].astype(np.float32))
  if (i+1)%300==0:print('map transforms',i+1,'/',len(opt),flush=True)
 points=voxel(np.vstack(allpoints),.15).astype(np.float32);np.save(ROOT/'map_graph_xyz_intensity.npy',points)
 n=len(points)
 header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
 with (ROOT/'map_graph.pcd').open('wb') as f:f.write(header.encode());f.write(points.tobytes())
 rot=np.array(map_transform['rotation_xy']);shift=np.array(map_transform['translation_xy']);points[:,:2]=points[:,:2]@rot.T+shift;points[:,2]+=transforms['slam']['z_offset'];np.save(ROOT/'map_enu_xyz_intensity.npy',points)
 header=f'ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n'
 with (ROOT/'map_enu.ply').open('wb') as f:f.write(header.encode());f.write(points.tobytes())
 base=opt[:,:3,3]+opt[:,:3,:3]@IMU_BASE
 base[:,:2]=base[:,:2]@rot.T+shift;base[:,2]+=transforms['slam']['z_offset']
 np.savetxt(ROOT/'trajectory_keyframes_enu.csv',np.c_[kt,ks,base],delimiter=',',header='stamp_utc,segment,east,north,up',comments='')
 distance=float(np.sum(np.linalg.norm(np.diff(base,axis=0)[np.diff(ks)==0],axis=1)))
 summary={'points':n,'voxel_m':.15,'path_length_m':distance,'bounds_enu_min':points[:,:3].min(0).tolist(),'bounds_enu_max':points[:,:3].max(0).tolist(),'gnss_constraints':0,'frames':len(opt),'coordinates':'Graph frame PCD; post-hoc all-FIX rigid ENU PLY (no feedback to graph)','observed_only':True,'free_space_not_classified':True}
 (ROOT/'map_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(result['results']),flush=True);print(json.dumps(summary),flush=True)
if __name__=='__main__':main()
