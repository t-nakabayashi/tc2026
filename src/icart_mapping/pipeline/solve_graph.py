#!/usr/bin/env python3
"""GNSS-blind SE(3) pose graph: FAST-LIO edges + LiDAR GICP loop closures."""
import json, sys, time
from functools import lru_cache
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from common import *
sys.path.insert(0,str(BACKEND))
import small_gicp, gtsam
F=json.loads((ROOT/'keyframes.json').read_text());P=np.array([f['pose'] for f in F]);S=np.array([f['segment'] for f in F]);T=np.array([f['t'] for f in F]);N=len(F)
@lru_cache(maxsize=200)
def submap(i):
 ids=[j for j in range(max(0,i-2),min(N,i+3)) if S[j]==S[i] and abs(T[j]-T[i])<12]
 inv=np.linalg.inv(P[i]);cloud=[]
 for j in ids:
  pts=np.load(ROOT/'clouds'/f'{j:05d}.npy')[:,:3].astype(float);x=inv@P[j];cloud.append(pts@x[:3,:3].T+x[:3,3])
 a=voxel(np.vstack(cloud),.3)
 return a,*small_gicp.preprocess_points(a,downsampling_resolution=.35,num_threads=2),cKDTree(a)

def register(i,j,init,wide=False):
 a,ta,tree,kd=submap(i);b,sb,_,kdb=submap(j)
 result=small_gicp.align(ta,sb,tree,init_T_target_source=init,max_correspondence_distance=3 if wide else 2,num_threads=2,max_iterations=60)
 x=result.T_target_source
 result=small_gicp.align(ta,sb,tree,init_T_target_source=x,max_correspondence_distance=.8,num_threads=2,max_iterations=40)
 x=result.T_target_source;trans=b@x[:3,:3].T+x[:3,3];ds=kd.query(trans)[0];dr=kdb.query((a-x[:3,3])@x[:3,:3])[0]
 eig=np.linalg.eigvalsh(result.H);ratio=float(eig[0]/max(eig[-1],1e-9))
 score={'i':int(i),'j':int(j),'converged':bool(result.converged),'overlap':float(np.mean(ds<.5)),'reverse_overlap':float(np.mean(dr<.5)),'median':float(np.median(ds)),'p90':float(np.percentile(ds,90)),'hessian_ratio':ratio,'transform':x.tolist()}
 score['good']=bool(result.converged and min(score['overlap'],score['reverse_overlap'])>.62 and score['median']<.23 and ratio>1e-6)
 return x,score

def main():
 poses=P.copy();joins=[]
 for sid in np.unique(S)[1:]:
  j=int(np.where(S==sid)[0][0]);i=j-1
  best=None
  # Recording gap is known. No position or heading from GNSS enters this registration.
  for yaw in [0,30,-30,90,-90,180]:
   init=np.eye(4);init[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
   x,score=register(i,j,init,True)
   if best is None or score['median']<best[1]['median']:best=(x,score)
   if score['good']:
    best=(x,score);break
  x,score=best;print('JOIN',json.dumps(score),flush=True)
  if not score['good']:raise RuntimeError('Cannot connect recording segments with validated LiDAR registration')
  world=poses[i]@x@np.linalg.inv(P[j]);poses[S==sid]=world@P[S==sid];score['kind']='join';joins.append(score)
 np.save(ROOT/'initial_poses.npy',poses)
 (ROOT/'joins.json').write_text(json.dumps(joins,indent=2))
 # Candidate radius uses only the stitched LIO trajectory. No RTK information.
 candidates=[];tree=cKDTree(poses[:,:3,3]);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(poses[:,:3,3],axis=0),axis=1))]
 for j in range(0,N,4):
  cand=[i for i in tree.query_ball_point(poses[j,:3,3],18) if i<j-25 and T[j]-T[i]>60 and arc[j]-arc[i]>60]
  cand.sort(key=lambda i:np.linalg.norm(poses[i,:3,3]-poses[j,:3,3]));chosen=[]
  for i in cand:
   if all(abs(i-k)>25 for k in chosen):chosen.append(i)
   if len(chosen)>=2:break
  candidates.extend((i,j) for i in chosen)
 loops=[];attempts=[]
 print('CANDIDATES',len(candidates),flush=True)
 for num,(i,j) in enumerate(candidates):
  init=np.linalg.inv(poses[i])@poses[j];x,score=register(i,j,init,True)
  delta=np.linalg.inv(init)@x;score['correction_m']=float(np.linalg.norm(delta[:3,3]));score['correction_deg']=float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()));score['kind']='loop'
  score['good']=bool(score['good'] and score['correction_m']<15 and score['correction_deg']<35)
  if score['good']:
   # Reciprocal solve checks a second optimization direction.
   y,reverse=register(j,i,np.linalg.inv(x));cycle=x@y
   score['cycle_m']=float(np.linalg.norm(cycle[:3,3]));score['cycle_deg']=float(np.degrees(Rotation.from_matrix(cycle[:3,:3]).magnitude()))
   score['good']=bool(reverse['good'] and score['cycle_m']<.15 and score['cycle_deg']<1)
  attempts.append(score)
  if score['good']:loops.append(score)
  if num%10==0:print('LOOPS',num,'/',len(candidates),'accepted',len(loops),flush=True)
 (ROOT/'loop_attempts.json').write_text(json.dumps(attempts,indent=2))
 (ROOT/'loops.json').write_text(json.dumps(loops,indent=2))
 if not loops:raise RuntimeError('No validated loop closure')
 result,optstats=optimize(poses,joins,loops)
 np.save(ROOT/'optimized_poses.npy',result)
 (ROOT/'optimization.json').write_text(json.dumps(optstats,indent=2))
 print('DONE',json.dumps(optstats),flush=True)

def optimize(initial,joins,loops):
 graph=gtsam.NonlinearFactorGraph();values=gtsam.Values()
 for i,p in enumerate(initial):values.insert(i,gtsam.Pose3(p))
 graph.add(gtsam.PriorFactorPose3(0,gtsam.Pose3(initial[0]),gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-5]*6))))
 for j in range(1,N):
  if S[j]!=S[j-1]:continue
  x=np.linalg.inv(P[j-1])@P[j];dist=np.linalg.norm(x[:3,3]);dt=T[j]-T[j-1]
  sig=np.array([np.radians(.08+.03*dist)]*3+[.025+.015*dist]*3)*np.sqrt(max(1,dt/2))
  graph.add(gtsam.BetweenFactorPose3(j-1,j,gtsam.Pose3(x),gtsam.noiseModel.Diagonal.Sigmas(sig)))
 for e in joins+loops:
  sig=np.array([np.radians(.3)]*3+[.1]*3)
  model=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),gtsam.noiseModel.Diagonal.Sigmas(sig))
  graph.add(gtsam.BetweenFactorPose3(e['i'],e['j'],gtsam.Pose3(np.array(e['transform'])),model))
 params=gtsam.LevenbergMarquardtParams();params.setMaxIterations(100);params.setRelativeErrorTol(1e-6)
 optimizer=gtsam.LevenbergMarquardtOptimizer(graph,values,params);opt=optimizer.optimize()
 result=np.array([opt.atPose3(i).matrix() for i in range(N)])
 residuals=[]
 for e in joins+loops:
  pred=np.linalg.inv(result[e['i']])@result[e['j']];delta=np.linalg.inv(np.array(e['transform']))@pred
  residuals.append({'i':e['i'],'j':e['j'],'kind':e['kind'],'translation_m':float(np.linalg.norm(delta[:3,3])),'rotation_deg':float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()))})
 correction=np.linalg.norm(result[:,:3,3]-initial[:,:3,3],axis=1)
 return result,{'nodes':N,'odometry_edges':int(N-len(np.unique(S))),'joins':len(joins),'loop_edges':len(loops),'factors':graph.size(),'gnss_factors':0,'initial_cost':graph.error(values),'final_cost':graph.error(opt),'iterations':optimizer.iterations(),'correction_max_m':float(max(correction)),'closure_residuals':residuals}
if __name__=='__main__':main()
