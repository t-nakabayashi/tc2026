"""局所路面モデルと、観測された連続領域だけから求める回避幅。"""
from collections import deque
import math
import numpy as np


def unknown(reason='unknown'):
    return dict(left=0., right=0., left_reason=reason, right_reason=reason,
                review_required=True, method='local_plane_material_v1')


def surface_width(points, cell=.2, max_width=3., step=.06, margin=.35,
                  max_seed_intensity=30., material_ratio=2.5, details=False):
    """反射強度は拒否にだけ使う。未観測・粗面・段差を横断して幅を延長しない。"""
    if cell <= 0 or max_width <= 0 or step <= 0 or margin < 0:
        raise ValueError('路面判定の寸法が不正')
    p=np.asarray(points,dtype=float)
    if p.size==0:return unknown()
    if p.ndim!=2 or p.shape[1] not in (3,4):raise ValueError('XYZまたはXYZI点群が必要')
    p=p[np.isfinite(p[:,:3]).all(axis=1)]
    p=p[(p[:,0]>=-.6)&(p[:,0]<3.)&(abs(p[:,1])<max_width)&(p[:,2]>-.65)&(p[:,2]<1.5)]
    if p.shape[1]==4 and (not np.isfinite(p[:,3]).all() or np.any((p[:,3]<0)|(p[:,3]>255))):
        return unknown('material_unavailable')
    intensity=bool(p.shape[1]==4)
    seed=p[(abs(p[:,1])<.35)&(p[:,2]<.15)&(p[:,2]>-.4)]
    if intensity:seed=seed[seed[:,3]<=max_seed_intensity]
    if len(seed)<20:return unknown('no_surface_reference')
    seed=seed[::max(1,len(seed)//2000)]
    design=np.column_stack([seed[:,:2],np.ones(len(seed))])
    keep=np.ones(len(seed),bool)
    for _ in range(4):
        if keep.sum()<15:return unknown('no_surface_reference')
        if np.linalg.matrix_rank(design[keep])<3:return unknown('no_surface_reference')
        plane=np.linalg.lstsq(design[keep],seed[keep,2],rcond=None)[0]
        residual=seed[:,2]-design@plane
        noise=max(.003,float(np.median(abs(residual[keep]-np.median(residual[keep])))/.6745))
        keep=abs(residual)<max(.025,2.5*noise)
    if np.linalg.norm(plane[:2])>math.tan(math.radians(15)):
        return unknown('slope')
    if noise>.025:return unknown('rough_reference')
    # A rough/vegetated centre cannot become the template just because it was driven over.
    if np.quantile(abs(residual),.9)>max(.04,3*noise):return unknown('rough_reference')
    reference=float(np.median(seed[keep,3])) if intensity else None
    material_limit=max(30.,material_ratio*reference) if intensity else math.inf
    result=unknown();result.update(ground_noise_sigma_m=noise,plane=plane.tolist(),
        intensity_reference=reference,intensity_reject_above=material_limit if intensity else None,
        material_checked=intensity)
    footprint=p[(p[:,0]>=-.6)&(p[:,0]<.6)]
    # Three longitudinal patches; every patch in every lateral band must be observed.
    patch=np.floor((footprint[:,0]+.6)/.4).astype(int)
    bands=np.floor(abs(footprint[:,1])/cell+1e-9).astype(int)
    residual=footprint[:,2]-(footprint[:,0]*plane[0]+footprint[:,1]*plane[1]+plane[2])
    cells=[]
    for side,sign in [('left',1),('right',-1)]:
        extent=0.;reason='range_limit'
        previous=0.
        for band in range(int(max_width/cell)):
            levels=[];failure=None
            for ix in range(3):
                mask=(sign*footprint[:,1]>=0)&(bands==band)&(patch==ix)
                z=residual[mask];reason_cell='clear'
                if len(z)<3:reason_cell='unknown'
                else:
                    lo,med,hi=np.quantile(z,[.1,.5,.9])
                    # Protect thin obstacles too; do not hide them inside a broad quantile.
                    if np.sum(z>step+.05)>=2 or hi-lo>max(.04,3.5*noise):reason_cell='rough_or_obstacle'
                    elif max(abs(lo),abs(hi))>=step+min(.015,noise):reason_cell='curb_or_drop'
                    elif intensity and np.median(footprint[mask,3])>material_limit:reason_cell='material_change'
                    levels.append(med)
                cells.append([-.4+ix*.4,sign*(band+.5)*cell,reason_cell,int(len(z))])
                if reason_cell!='clear':failure=failure or reason_cell
            if failure:
                reason=failure;break
            level=float(np.median(levels))
            if abs(level-previous)>cell*math.tan(math.radians(15)):
                reason='slope';break
            previous=level;extent=(band+1)*cell
        result[side+'_observed_extent_m']=extent
        result[side]=max(0.,extent-margin)
        result[side+'_reason']=reason
    result['center_verified'] = min(result['left_observed_extent_m'], result['right_observed_extent_m']) >= margin
    if not result['center_verified']:
        result['left']=result['right']=0.
    if details:result['cells']=cells
    return result


class SurfaceWindow:
    """指定した共通座標で短時間だけ蓄積。古い地面を安全扱いしない。"""
    def __init__(self, seconds=6.):
        if not np.isfinite(seconds) or not .5 <= seconds <= 15.:
            raise ValueError('点群保持時間は0.5-15秒')
        self.seconds=seconds
        self.frames=deque()
        self.last_pose=None

    def clear(self):
        self.frames.clear();self.last_pose=None

    def update(self, stamp, world_points, base_position, yaw):
        base=np.asarray(base_position)
        if self.last_pose is not None:
            old_t,old_p=self.last_pose;dt=stamp-old_t
            if dt<=0 or dt>1. or np.linalg.norm(base-old_p)>2*dt+.15:self.clear()
        self.last_pose=(stamp,base.copy())
        points=np.asarray(world_points)
        near=np.linalg.norm(points[:,:2]-base[:2],axis=1)<5.
        self.frames.append((stamp,points[near].copy()))
        while self.frames and stamp-self.frames[0][0]>self.seconds:self.frames.popleft()
        all_points=np.concatenate([v for _,v in self.frames])
        q=all_points.copy();q[:,:3]-=base
        c,s=math.cos(yaw),math.sin(yaw)
        q[:,:2]=q[:,:2]@np.array([[c,-s],[s,c]])
        q=q[(q[:,0]>=-.6)&(q[:,0]<3.)&(abs(q[:,1])<3.)&(q[:,2]>-.65)&(q[:,2]<1.5)]
        if not len(q):return q
        # Repeated hits in the same small voxel are not independent coverage.
        voxel=np.floor(q[:,:3]/.025).astype(np.int64)
        key=(voxel[:,0]+1000)*4000000+(voxel[:,1]+1000)*2000+voxel[:,2]+1000
        _,index=np.unique(key,return_index=True)
        return q[index]
