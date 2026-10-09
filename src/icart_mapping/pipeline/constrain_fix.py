"""LiDAR graph with robust horizontal FIX antenna factors from the selected bag."""
from pathlib import Path
import sys,json,time,hashlib,csv
import numpy as np
from scipy.spatial.transform import Rotation

from common import *
from initialize import align_graph
import gtsam
OLD=ROOT
F=json.loads((OLD/'keyframes.json').read_text());P=np.array([f['pose'] for f in F]);RAW=np.array([f['raw'] for f in F]);S=np.array([f['segment'] for f in F]);T=np.array([f['t'] for f in F]);N=len(F)
origin,baseline=align_graph(F)
joins=json.loads((OLD/'joins.json').read_text());loops=json.loads((OLD/'loops.json').read_text())
# Predeclared primary settings. These are engineering weights, not measured GNSS accuracy.
config={'primary_sigma_xy_m':CONFIG['sigma_xy_m'],'sensitivity_sigma_xy_m':[],'huber_k':1.345,'fix_condition':'state=4; satellites>=20; 0<HDOP<=1; 0<=correction_age<=1.5s; fix/status difference<=0.05s','minimum_between_selected_factors_s':1.,'minimum_motion_m_or_stationary_interval_s':[2.,10.],'keyframe_to_fix_max_dt_s':.15,'horizontal_only':True,'preserve_original_lidar_height_and_tilt':True,'height_tilt_sigma':[0.0001,0.0001,0.0001],'GNSS_heading_used_in_graph':False,'GNSS_height_used_in_graph':False,'antenna_lever_arm_used':True,'source_bag':CONFIG['bag']['directory'],'independent_evaluation':False,'loop_edges_unchanged':True,'odometry_noise_unchanged':True,'anchor':'LiDAR gravity/relative height retained; horizontal pose constrained by FIX and ICP','post_optimization_GNSS_refit':False,'selection_uses_localization_error':False}
(OUT/'config.json').write_text(json.dumps(config,indent=2))
# FIX candidates come only from the selected input bag.
o=np.load(OLD/'obs_0.npz');fix=o['fix'];st=o['status'];lio=o['lio'];si=nearest(st[:,0],fix[:,0]);gps=enu(fix[:,1:4],origin)
A=(st[si,1]==4)&(st[si,2]>=20)&(st[si,3]>0)&(st[si,3]<=1+1e-6)&(st[si,7]>=0)&(st[si,7]<=1.5+1e-6)&(abs(st[si,0]-fix[:,0])<=.05)&np.isfinite(fix[:,:4]).all(axis=1)
segments=json.loads((OLD/'segments.json').read_text());tr={s['id']:Trajectory(lio[(lio[:,0]>=s['start'])&(lio[:,0]<=s['end'])]) for s in segments};candidates=[];last={};obs=[]

for i in range(N):
 cand=np.flatnonzero(A&(abs(fix[:,0]-T[i])<=.15)&(fix[:,0]>=tr[S[i]].t[0])&(fix[:,0]<=tr[S[i]].t[-1]))
 if not len(cand):continue
 j=int(cand[np.argmin(abs(fix[cand,0]-T[i]))]);tm=float(fix[j,0]);rawat=tr[S[i]].at(tm);k=np.searchsorted(tr[S[i]].t,tm);k=min(max(1,k),len(tr[S[i]].t)-1)
 if tr[S[i]].t[k]-tr[S[i]].t[k-1]>.3:continue
 rr=np.linalg.inv(RAW[i])@rawat;local=rr[:3,:3]@IMU_ANT+rr[:3,3];candidate={'keyframe':i,'segment':int(S[i]),'epoch':tm,'gnss_enu':gps[j].tolist(),'antenna_in_keyframe':local.tolist(),'satellites':float(st[si[j],2]),'hdop':float(st[si[j],3]),'correction_age_s':float(st[si[j],7]),'keyframe_dt_s':float(tm-T[i])};candidates.append(candidate)
 if S[i] in last:
  prev=last[S[i]];dt=tm-prev['epoch'];dist=float(np.linalg.norm(gps[j,:2]-np.array(prev['gnss_enu'])[:2]))
  if dt<1-1e-6 or (dist<2 and dt<10-1e-6):continue
 obs.append(candidate);last[S[i]]=candidate
if len(obs)<3:raise ValueError('品質条件を満たすFIX拘束が3点未満です')
(OUT/'gnss_factors.json').write_text(json.dumps(obs,indent=2));print('FACTORS',len(obs),'of',len(candidates),'keyframe candidates','source bag',CONFIG['bag']['directory'],flush=True)

def hat(p):
 x,y,z=p;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def gps_error(key,local,measurement):
 def error(factor,values,jacobians):
  pose=values.atPose3(key);R=pose.rotation().matrix();e=pose.transformFrom(local)[:2]-measurement[:2]
  if jacobians is not None:jacobians[0]=np.asfortranarray(np.c_[-R@hat(local),R][:2])
  return e
 return error
# A world/gravity-coordinate gauge prior avoids locking the tilted IMU's x/z components together.
ref=baseline[0];anchor_sig=np.array([1e-4,1e-4,np.radians(20),20.,20.,1e-4])
def gauge_error(factor,values,jacobians):
 pose=values.atPose3(0);R=pose.rotation().matrix();R0=ref[:3,:3];rot=Rotation.from_matrix(R@R0.T).as_rotvec();e=np.r_[rot,pose.translation()-ref[:3,3]]
 if jacobians is not None:
  # Numeric derivative only for this single gauge factor, in GTSAM's local Pose3 coordinates.
  J=np.zeros((6,6));eps=1e-6
  for k in range(6):
   dx=np.zeros(6);dx[k]=eps;plus=pose.retract(dx);minus=pose.retract(-dx)
   rp=Rotation.from_matrix(plus.rotation().matrix()@R0.T).as_rotvec();rm=Rotation.from_matrix(minus.rotation().matrix()@R0.T).as_rotvec();J[:,k]=np.r_[rp-rm,plus.translation()-minus.translation()]/(2*eps)
  jacobians[0]=np.asfortranarray(J)
 return e
# Keep the original LiDAR graph height and tilt; only horizontal position/yaw respond to GNSS.
def height_tilt_error(key):
 R0=baseline[key,:3,:3];z0=baseline[key,2,3]
 def error(factor,values,jacobians):
  pose=values.atPose3(key);R=pose.rotation().matrix();w=Rotation.from_matrix(R@R0.T).as_rotvec();e=np.r_[w[:2],pose.translation()[2]-z0]
  if jacobians is not None:
   # SO(3) log differential for a right perturbation of R, expressed in world frame.
   theta=np.linalg.norm(w);W=hat(w)
   coef=1/12 if theta<1e-5 else (1/(theta*theta)-(1+np.cos(theta))/(2*theta*np.sin(theta)))
   Jr_inv=np.eye(3)+.5*W+coef*(W@W)
   J=np.zeros((3,6));J[:2,:3]=(Jr_inv@R0)[:2];J[2,3:]=R[2,:]
   jacobians[0]=np.asfortranarray(J)
  return e
 return error
# Validate height/tilt Jacobian away from the prior, including combined local rotation.
for key in [0,N//2,N-1]:
 pose=gtsam.Pose3(baseline[key]).retract(np.array([.002,-.001,.005,.1,-.05,.01]));v=gtsam.Values();v.insert(key,pose);js=[None];fun=height_tilt_error(key);fun(None,v,js);num=np.zeros((3,6))
 for k in range(6):
  dx=np.zeros(6);dx[k]=1e-6;vp=gtsam.Values();vm=gtsam.Values();vp.insert(key,pose.retract(dx));vm.insert(key,pose.retract(-dx));num[:,k]=(fun(None,vp,None)-fun(None,vm,None))/2e-6
 assert np.max(abs(num-js[0]))<1e-6

# Verify the implemented horizontal antenna-factor Jacobian before optimization.
jac_errors=[]
for v in obs[::max(1,len(obs)//8)]:
 pose=gtsam.Pose3(baseline[v['keyframe']]);local=np.array(v['antenna_in_keyframe']);R=pose.rotation().matrix();J=np.c_[-R@hat(local),R][:2];Jnum=np.zeros((2,6))
 for k in range(6):
  x=np.zeros(6);x[k]=1e-6;Jnum[:,k]=(pose.retract(x).transformFrom(local)[:2]-pose.retract(-x).transformFrom(local)[:2])/2e-6
 jac_errors.append(float(np.max(abs(J-Jnum))))
assert max(jac_errors)<1e-6

def summarize(v):
 v=np.array(v);return {'n':len(v),'mean':float(v.mean()),'p95':float(np.percentile(v,95)),'max':float(v.max()),'rmse':float(np.sqrt(np.mean(v*v)))}
def optimize(sigma):
 graph=gtsam.NonlinearFactorGraph();values=gtsam.Values()
 for i,p in enumerate(baseline):values.insert(i,gtsam.Pose3(p))
 graph.add(gtsam.CustomFactor(gtsam.noiseModel.Diagonal.Sigmas(anchor_sig),[0],gauge_error))
 for i in range(N):graph.add(gtsam.CustomFactor(gtsam.noiseModel.Isotropic.Sigma(3,1e-4),[i],height_tilt_error(i)))
 for j in range(1,N):
  if S[j]!=S[j-1]:continue
  x=np.linalg.inv(P[j-1])@P[j];dist=np.linalg.norm(x[:3,3]);dt=T[j]-T[j-1];sig=np.array([np.radians(.08+.03*dist)]*3+[.025+.015*dist]*3)*np.sqrt(max(1,dt/2))
  graph.add(gtsam.BetweenFactorPose3(j-1,j,gtsam.Pose3(x),gtsam.noiseModel.Diagonal.Sigmas(sig)))
 for e in joins+loops:
  sig=np.array([np.radians(.3)]*3+[.1]*3);noise=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),gtsam.noiseModel.Diagonal.Sigmas(sig));graph.add(gtsam.BetweenFactorPose3(e['i'],e['j'],gtsam.Pose3(np.array(e['transform'])),noise))
 for e in obs:
  model=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),gtsam.noiseModel.Isotropic.Sigma(2,sigma));graph.add(gtsam.CustomFactor(model,[e['keyframe']],gps_error(e['keyframe'],np.array(e['antenna_in_keyframe']),np.array(e['gnss_enu']))))
 pars=gtsam.LevenbergMarquardtParams();pars.setMaxIterations(100);pars.setRelativeErrorTol(1e-7);start=time.time();opt=gtsam.LevenbergMarquardtOptimizer(graph,values,pars);sol=opt.optimize();poses=np.array([sol.atPose3(i).matrix() for i in range(N)])
 residuals=[]
 for e in joins+loops:
  er=np.linalg.inv(np.array(e['transform']))@np.linalg.inv(poses[e['i']])@poses[e['j']];residuals.append({'i':e['i'],'j':e['j'],'kind':e['kind'],'translation_m':float(np.linalg.norm(er[:3,3])),'rotation_deg':float(np.degrees(Rotation.from_matrix(er[:3,:3]).magnitude()))})
 oldr=[];newr=[]
 for e in obs:
  i=e['keyframe'];a=np.array(e['antenna_in_keyframe']);g=np.array(e['gnss_enu']);oldr.append(np.linalg.norm((baseline[i,:3,:3]@a+baseline[i,:3,3]-g)[:2]));newr.append(np.linalg.norm((poses[i,:3,:3]@a+poses[i,:3,3]-g)[:2]))
 correction=np.linalg.norm(poses[:,:2,3]-baseline[:,:2,3],axis=1)
 # Relative shift after removing the best whole-trajectory rigid transform quantifies nonrigid change.
 old=baseline[:,:2,3];new=poses[:,:2,3];u,_,vt=np.linalg.svd((old-old.mean(0)).T@(new-new.mean(0)));rot=vt.T@np.diag([1,np.linalg.det(vt.T@u.T)])@u.T;shift=new.mean(0)-rot@old.mean(0);nonrigid=np.linalg.norm(old@rot.T+shift-new,axis=1)
 info={'sigma_xy_m':sigma,'nodes':N,'odometry_edges':int(N-len(np.unique(S))),'loop_edges':len(loops),'join_edges':len(joins),'gnss_factors':len(obs),'height_tilt_preservation_factors':N,'all_factors':graph.size(),'initial_cost':graph.error(values),'final_cost':graph.error(sol),'iterations':opt.iterations(),'wall_s':time.time()-start,'horizontal_keyframe_change_m':summarize(correction),'change_after_best_global_SE2_removed_m':summarize(nonrigid),'vertical_keyframe_change_m':summarize(abs(poses[:,2,3]-baseline[:,2,3])),'GNSS_factor_difference_before_m':summarize(oldr),'GNSS_factor_difference_after_m':summarize(newr),'closure_residuals':residuals,'largest_loop_translation_residual_m':max((v['translation_m'] for v in residuals if v['kind']=='loop'),default=0.)}
 print('OPTIMIZED',json.dumps(info),flush=True);np.save(OUT/f'poses_sigma_{sigma:.2f}.npy',poses);return poses,info
allinfo={};primary=None
for sig in [CONFIG['sigma_xy_m']]:
 poses,info=optimize(sig);allinfo[str(sig)]=info
 primary=poses
(OUT/'optimization.json').write_text(json.dumps(allinfo,indent=2));np.save(OUT/'optimized_poses_enu.npy',primary);np.save(OUT/'baseline_poses_enu.npy',baseline)
# Rebuild from original keyframe-local LiDAR returns, not by warping the old fused map.
clouds=[]
for i,pose in enumerate(primary):
 pts=np.load(OLD/'clouds'/f'{i:05d}.npy');clouds.append(np.c_[pts[:,:3]@pose[:3,:3].T+pose[:3,3],pts[:,3]].astype(np.float32))
 if (i+1)%500==0:print('MAP',i+1,'/',N,flush=True)
points=voxel(np.vstack(clouds),CONFIG['map_voxel_m']).astype(np.float32);np.save(OUT/'map_enu_xyz_intensity.npy',points);n=len(points)
header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
with (OUT/'map_rtk_constrained_enu.pcd').open('wb') as f:f.write(header.encode());f.write(points.tobytes())
header=f'ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n'
with (OUT/'map_rtk_constrained_enu.ply').open('wb') as f:f.write(header.encode());f.write(points.tobytes())
base=primary[:,:3,3]+primary[:,:3,:3]@IMU_BASE;np.savetxt(OUT/'trajectory_keyframes_enu.csv',np.c_[T,S,base],delimiter=',',header='timestamp,segment,east,north,up',comments='')
(OUT/'map_summary.json').write_text(json.dumps({'points':n,'voxel_m':CONFIG['map_voxel_m'],'coordinates':'ENU, fixed original origin, direct horizontal GNSS graph constraints, no posthoc alignment','origin_llh':origin,'primary_sigma_xy_m':CONFIG['sigma_xy_m'],'GNSS_factors':len(obs),'loop_factors':len(loops),'source_bag':CONFIG['bag']['directory'],'bounds_min':points[:,:3].min(0).tolist(),'bounds_max':points[:,:3].max(0).tolist(),'jacobian_max_abs_error':max(jac_errors),'map_sha256':hashlib.sha256((OUT/'map_enu_xyz_intensity.npy').read_bytes()).hexdigest()},indent=2))
print('DONE_MAP',n,flush=True)
