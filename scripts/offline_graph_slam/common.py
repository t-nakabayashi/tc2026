import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.optimize import least_squares
from pathlib import Path
ROOT=Path('/media/nkb/TEST/graph_slam_20261004')
RUNTIME=Path('/home/nkb/colcon_ws_experiments/glim_runtime')
LIDAR_T=np.array([-.011,-.02329,.04412])
BASE_R=Rotation.from_euler('xyz',[-.6,26.9,0],degrees=True).as_matrix()
IMU_BASE=BASE_R.T@np.array([0,0,-.414])+LIDAR_T
IMU_ANT=BASE_R.T@np.array([-.3,0,.15])+LIDAR_T
class Trajectory:
 def __init__(self,rows):
  self.rows=rows;self.t=rows[:,0];self.slerp=Slerp(self.t,Rotation.from_quat(rows[:,4:8]))
 def at(self,t):
  ts=np.atleast_1d(t)
  if np.any(ts<self.t[0]) or np.any(ts>self.t[-1]):raise ValueError('no extrapolation')
  a=np.tile(np.eye(4),(len(ts),1,1));a[:,:3,:3]=self.slerp(ts).as_matrix()
  for k in range(3):a[:,k,3]=np.interp(ts,self.t,self.rows[:,k+1])
  return a[0] if np.ndim(t)==0 else a

def voxel(x,size):
 _,i=np.unique(np.floor(x[:,:3]/size).astype(np.int32),axis=0,return_index=True)
 return x[np.sort(i)]

def enu(llh,origin):
 def ecef(a):
  lat,lon=np.radians(a[:,0]),np.radians(a[:,1]);h=a[:,2];n=6378137/np.sqrt(1-6.69437999014e-3*np.sin(lat)**2)
  return np.c_[(n+h)*np.cos(lat)*np.cos(lon),(n+h)*np.cos(lat)*np.sin(lon),(n*(1-6.69437999014e-3)+h)*np.sin(lat)]
 lat,lon=np.radians(origin[:2]);sl,cl,so,co=np.sin(lat),np.cos(lat),np.sin(lon),np.cos(lon)
 r=np.array([[-so,co,0],[-sl*co,-sl*so,cl],[cl*co,cl*so,sl]])
 return (ecef(llh)-ecef(np.array([origin]))[0])@r.T

def nearest(t,q):
 j=np.clip(np.searchsorted(t,q),1,len(t)-1)
 return np.where(abs(t[j]-q)<abs(t[j-1]-q),j,j-1)

def fit_xy(x,y):
 u,_,vt=np.linalg.svd((x-x.mean(0)).T@(y-y.mean(0)))
 r=vt.T@np.diag([1,np.linalg.det(vt.T@u.T)])@u.T
 z=[np.arctan2(r[1,0],r[0,0]),*(y.mean(0)-r@x.mean(0))]
 def rot(a):return Rotation.from_euler('z',a).as_matrix()[:2,:2]
 o=least_squares(lambda z:(x@rot(z[0]).T+z[1:]-y).ravel(),z,loss='soft_l1',f_scale=.3)
 return rot(o.x[0]),o.x[1:]
