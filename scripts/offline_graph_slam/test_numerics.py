import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from common import Trajectory,enu,fit_xy

def test_interpolation_and_forbidden_extrapolation():
 rows=np.array([[0,0,0,0,0,0,0,1],[1,2,0,0,0,0,np.sin(np.pi/4),np.cos(np.pi/4)]])
 tr=Trajectory(rows);p=tr.at(.5)
 np.testing.assert_allclose(p[:3,3],[1,0,0]);np.testing.assert_allclose(p[:3,:3]@[1,0,0],[2**-.5,2**-.5,0],atol=1e-12)
 with pytest.raises(ValueError):tr.at(-.01)
 with pytest.raises(ValueError):tr.at(1.01)

def test_enu_axes_and_rigid_alignment():
 origin=np.array([36.,140.,25.]);p=enu(np.array([origin,origin+[0,.00001,0],origin+[.00001,0,0]]),origin)
 np.testing.assert_allclose(p[0],0,atol=1e-9);assert p[1,0]>.8 and abs(p[1,1])<1e-5;assert p[2,1]>1
 rng=np.random.default_rng(12);x=rng.normal(size=(100,2))*30;r=Rotation.from_euler('z',.65).as_matrix()[:2,:2];y=x@r.T+[20,-30]
 rot,shift=fit_xy(x,y);np.testing.assert_allclose(rot,r,atol=1e-8);np.testing.assert_allclose(shift,[20,-30],atol=1e-8)

def test_pose_graph_closes_synthetic_loop_without_gnss():
 import solve_graph as g
 n=41;theta=np.linspace(0,2*np.pi,n);truth=np.tile(np.eye(4),(n,1,1));truth[:,:3,3]=np.c_[10*np.sin(theta),10*(1-np.cos(theta)),np.zeros(n)]
 drift=truth.copy();drift[:,0,3]+=np.linspace(0,1,n)
 old=(g.N,g.P,g.S,g.T)
 try:
  g.N=n;g.P=drift;g.S=np.zeros(n,int);g.T=np.arange(n,dtype=float)
  result,stats=g.optimize(drift,[],[{'i':0,'j':40,'kind':'loop','transform':np.eye(4).tolist()}])
  assert stats['gnss_factors']==0 and stats['final_cost']<stats['initial_cost']
  assert np.linalg.norm(result[-1,:3,3]-result[0,:3,3])<.2
  assert np.mean(np.linalg.norm(result[:,:3,3]-truth[:,:3,3],axis=1))<np.mean(np.linalg.norm(drift[:,:3,3]-truth[:,:3,3],axis=1))
 finally:g.N,g.P,g.S,g.T=old
