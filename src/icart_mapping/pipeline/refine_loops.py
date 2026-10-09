#!/usr/bin/env python3
"""One additional LiDAR-only candidate pass after pose graph optimization."""
import json
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from common import ROOT
import solve_graph as g

def main():
 poses=np.load(ROOT/'optimized_poses.npy');loops=json.loads((ROOT/'loops.json').read_text());joins=json.loads((ROOT/'joins.json').read_text());previous={(e['i'],e['j']) for e in loops};tree=cKDTree(poses[:,:3,3]);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(poses[:,:3,3],axis=0),axis=1))]
 candidates=[]
 for j in range(0,g.N,3):
  cand=[i for i in tree.query_ball_point(poses[j,:3,3],12) if i<j-25 and g.T[j]-g.T[i]>60 and arc[j]-arc[i]>60 and (i,j) not in previous]
  cand.sort(key=lambda i:np.linalg.norm(poses[i,:3,3]-poses[j,:3,3]));chosen=[]
  for i in cand:
   if all(abs(i-k)>25 for k in chosen):chosen.append(i)
   if len(chosen)>=2:break
  candidates.extend((i,j) for i in chosen)
 attempts=[];added=[]
 for n,(i,j) in enumerate(candidates):
  init=np.linalg.inv(poses[i])@poses[j];x,e=g.register(i,j,init,True);delta=np.linalg.inv(init)@x;e['kind']='loop';e['round']=2;e['correction_m']=float(np.linalg.norm(delta[:3,3]));e['correction_deg']=float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()));e['good']=bool(e['good'] and e['correction_m']<15 and e['correction_deg']<35)
  if e['good']:
   y,rev=g.register(j,i,np.linalg.inv(x));cycle=x@y;e['cycle_m']=float(np.linalg.norm(cycle[:3,3]));e['cycle_deg']=float(np.degrees(Rotation.from_matrix(cycle[:3,:3]).magnitude()));e['good']=bool(rev['good'] and e['cycle_m']<.15 and e['cycle_deg']<1)
  attempts.append(e)
  if e['good']:added.append(e)
  if n%20==0:print('ROUND2',n,'/',len(candidates),'added',len(added),flush=True)
 (ROOT/'loop_attempts_round2.json').write_text(json.dumps(attempts,indent=2))
 # Preserve the original anchor and baseline; initialization can use the refined poses.
 first=json.loads((ROOT/'optimization.json').read_text());(ROOT/'optimization_round1.json').write_text(json.dumps(first,indent=2))
 result,stats=g.optimize(poses,joins,loops+added);stats['rounds']=2;stats['first_round_loops']=len(loops);stats['additional_loops']=len(added);stats['correction_max_from_original_m']=float(np.max(np.linalg.norm(result[:,:3,3]-np.load(ROOT/'initial_poses.npy')[:,:3,3],axis=1)))
 (ROOT/'loops.json').write_text(json.dumps(loops+added,indent=2));np.save(ROOT/'optimized_poses.npy',result);(ROOT/'optimization.json').write_text(json.dumps(stats,indent=2));print('DONE',len(loops+added),'loop factors',flush=True)
if __name__=='__main__':main()
