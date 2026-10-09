#!/usr/bin/env python3
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from scipy.spatial import cKDTree
from common import *
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
plt.rcParams['font.family']=FontProperties(fname=FONT).get_name();plt.rcParams['axes.unicode_minus']=False
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#f5f7fb','axes.facecolor':'white','savefig.facecolor':'#f5f7fb'})

def main():
 out=ROOT/'figures';out.mkdir(exist_ok=True)
 d=np.load(ROOT/'comparison.npz');m=np.load(ROOT/'map_enu_xyz_intensity.npy',mmap_mode='r');traj=np.loadtxt(ROOT/'trajectory_keyframes_enu.csv',delimiter=',',skiprows=1);e=json.loads((ROOT/'evaluation.json').read_text());g=json.loads((ROOT/'optimization.json').read_text());loops=json.loads((ROOT/'loops.json').read_text());summary=json.loads((ROOT/'map_summary.json').read_text())
 stride=max(1,len(m)//650000);p=m[::stride];lo=p[:,:2].min(0);hi=p[:,:2].max(0)
 fig,ax=plt.subplots(figsize=(12,8));c=ax.scatter(p[:,0],p[:,1],c=np.clip(p[:,2],-2,15),s=.23,cmap='terrain',rasterized=True,linewidths=0,vmin=-2,vmax=15)
 ax.plot(traj[:,2],traj[:,3],color='#d83a46',lw=.9,label='RTK拘束なし グラフSLAM')
 for l in loops:
  a,b=traj[[l['i'],l['j']],2:4];ax.plot([a[0],b[0]],[a[1],b[1]],color='#006cff',alpha=.3,lw=.8)
 ax.scatter(*traj[0,2:4],s=80,c='#08a078',marker='o',label='開始');ax.scatter(*traj[-1,2:4],s=80,c='#282c3b',marker='x',label='終了')
 ax.set(aspect='equal',xlabel='東 [m]',ylabel='北 [m]',title=f'つくばチャレンジ2026 | LiDARグラフSLAM地図\n{summary["path_length_m"]/1000:.2f} km・{summary["points"]:,}点・ループ拘束 {g["loop_edges"]}本')
 ax.legend(loc='best',fontsize=10);fig.colorbar(c,ax=ax,shrink=.5,label='相対高さ [m]');fig.text(.1,.025,'地図最適化へのRTK入力は0件。ENU表示のみ全良好FIXで剛体位置合わせ。\n収録された周辺の点群地図。未観測域・動体除去・走行可能判定は含まない。',fontsize=10)
 fig.subplots_adjust(bottom=.10,top=.91);fig.savefig(out/'map_overview.png',dpi=180);plt.close(fig)
 fig,ax=plt.subplots(figsize=(12,8));valid=d['valid'];time=d['time'];
 for s in np.unique(d['segment'][valid]):
  u=valid&(d['segment']==s);ax.plot(d['lio'][u,0],d['lio'][u,1],color='#9ca5b5',lw=1,alpha=.8,zorder=1);ax.plot(d['slam'][u,0],d['slam'][u,1],color='#d72d43',lw=1.7,zorder=2)
 for q,color,label in [(4,'#0c986c','RTK FIX'),(3,'#efa839','RTK FLOAT'),(2,'#6280bf','DGPS')]:
  u=(d['quality']==q)&valid;ax.scatter(d['gps'][u,0][::5],d['gps'][u,1][::5],s=6,c=color,label=label,alpha=1,zorder=3)
 ax.plot([],[],color='#d72d43',label='ループ閉合後');ax.plot([],[],color='#9ca5b5',label='閉合前LIO（中断部は点群接続）')
 u=d['calibration'];ax.scatter(d['gps'][u,0][::5],d['gps'][u,1][::5],s=20,facecolors='none',edgecolors='#26243b',label='座標合わせ用 FIX')
 ax.set(aspect='equal',xlabel='東 [m]',ylabel='北 [m]',title='軌跡比較 | RTKはグラフ最適化に不使用');ax.legend(fontsize=9);ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'trajectory_overlay.png',dpi=170);plt.close(fig)
 fig,axs=plt.subplots(3,1,figsize=(11,11),gridspec_kw={'height_ratios':[2,1,1]})
 u=d['heldout']&d['qualified'];
 for key,color,label in [('lio','#939aab','閉合前 LIO'),('slam','#d72d43','グラフSLAM')]:
  axs[0].scatter(time[u]/60,d[key+'_error'][u],s=3,alpha=.65,c=color,label=label)
 axs[0].set(ylabel='FIXとの水平差 [m]',title='未使用RTK FIXとの比較（冒頭3分を除外）');axs[0].legend();axs[0].grid(alpha=.2)
 for key,color,label in [('lio','#939aab','閉合前'),('slam','#d72d43','閉合後')]:
  vals=np.sort(d[key+'_error'][u]);axs[1].plot(vals,np.arange(1,len(vals)+1)/len(vals)*100,c=color,label=label)
 axs[1].set(xlabel='FIXとの水平差 [m]',ylabel='累積割合 [%]');axs[1].grid(alpha=.2)
 axs[2].scatter(time/60,d['quality'],s=2,c=d['quality'],cmap='viridis',vmin=2,vmax=4);axs[2].set(yticks=[2,3,4],yticklabels=['DGPS','FLOAT','FIX'],xlabel='開始からの時間 [分]',ylabel='GNSS品質');axs[2].grid(alpha=.2)
 sm=e['results']['slam']['heldout_qualified_fix'];lm=e['results']['lio']['heldout_qualified_fix'];fig.suptitle(f'評価用FIX {sm["n"]:,}観測 | RMSE {lm["rmse_m"]:.2f} → {sm["rmse_m"]:.2f} m',fontsize=16)
 fig.text(.09,.012,'RTKとの差であり、独立した測量真値に対する絶対精度ではない。',fontsize=11);fig.tight_layout(rect=[0,.035,1,.96]);fig.savefig(out/'evaluation.png',dpi=170);plt.close(fig)
 # Conventional all-FIX rigid alignment: useful shape comparison, explicitly in-sample.
 fig,axs=plt.subplots(1,2,figsize=(13,6),gridspec_kw={'width_ratios':[1.7,1]})
 ax=axs[0];u=d['valid'];ax.plot(d['slam_postfit'][u,0],d['slam_postfit'][u,1],color='#d72d43',lw=1.8,label='LiDARグラフSLAM',zorder=2)
 for q,color,label in [(4,'#0c986c','FIX'),(3,'#efa839','FLOAT'),(2,'#6280bf','DGPS')]:
  qmask=u&(d['quality']==q);ax.scatter(d['gps'][qmask,0][::4],d['gps'][qmask,1][::4],s=6,c=color,label=label,alpha=1,zorder=3)
 ax.set(aspect='equal',xlabel='東 [m]',ylabel='北 [m]');ax.legend(fontsize=9);ax.grid(alpha=.2)
 for key,color,label in [('lio','#939aab','閉合前'),('slam','#d72d43','閉合後')]:
  use=d['valid']&d['qualified'];er=np.linalg.norm(d[key+'_postfit'][use]-d['gps'][use,:2],axis=1);v=np.sort(er);axs[1].plot(v,np.arange(1,len(v)+1)/len(v)*100,color=color,label=label)
 axs[1].set(xlabel='FIXとの水平差 [m]',ylabel='累積割合 [%]',xlim=(0,3));axs[1].legend();axs[1].grid(alpha=.2)
 a=e['results']['lio']['all_fix_rigid_alignment_diagnostic'];b=e['results']['slam']['all_fix_rigid_alignment_diagnostic'];fig.suptitle(f'軌跡形状の比較 | 全FIX剛体整列後 RMSE {a["rmse_m"]:.2f} → {b["rmse_m"]:.2f} m',fontsize=16)
 fig.text(.08,.035,'全良好FIXを座標合わせと比較に共用。独立検証・絶対精度の保証ではない。\n地図形状はLiDARのみで最適化済み。回転・並進だけを合わせ、スケールや軌跡は変形しない。',fontsize=10)
 fig.tight_layout(rect=[0,.13,1,.93]);fig.savefig(out/'shape_comparison.png',dpi=180);plt.close(fig)
 # Oblique overview and four observed-area crops.
 pp=m[::max(1,len(m)//160000)];fig=plt.figure(figsize=(12,8));ax=fig.add_subplot(111,projection='3d');ax.scatter(pp[:,0],pp[:,1],pp[:,2],c=np.clip(pp[:,2],-2,15),s=.1,cmap='terrain',linewidths=0);ax.set_box_aspect([np.ptp(pp[:,0]),np.ptp(pp[:,1]),max(30,np.ptp(pp[:,2]))]);ax.view_init(55,-65);ax.set(xlabel='東 [m]',ylabel='北 [m]',zlabel='高さ [m]',title='観測点群の立体俯瞰');fig.tight_layout();fig.savefig(out/'map_oblique.png',dpi=170);plt.close(fig)
 fig,axs=plt.subplots(2,2,figsize=(12,12))
 for k,ax in enumerate(axs.flat):
  center=traj[int((len(traj)-1)*(k+.5)/4),2:4];sel=(np.abs(m[:,0]-center[0])<45)&(np.abs(m[:,1]-center[1])<45);part=m[sel][::2];ax.scatter(part[:,0],part[:,1],c=np.clip(part[:,2],-2,15),s=.4,cmap='terrain',linewidths=0);ax.set(aspect='equal',title=f'区間拡大 {k+1}',xlabel='東 [m]',ylabel='北 [m]')
 fig.suptitle('地図の各部 | 90 m四方');fig.tight_layout();fig.savefig(out/'map_details.png',dpi=160);plt.close(fig)
 # Reference is synthetic course geometry, not surveyed truth.
 ref=np.genfromtxt('/home/nkb/colcon_ws/src/route_planner/routes/tsukuba2026_digital_twin/fixed/waypoints.csv',delimiter=',',names=True)
 llh=np.c_[ref['latitude'],ref['longitude'],np.full(len(ref),e['origin_llh'][2])];refxy=enu(llh,np.array(e['origin_llh']))[:,:2];dist=cKDTree(traj[:,2:4]).query(refxy)[0]
 coverage={'reference':'digital-twin waypoints, not surveyed ground truth','points':len(ref),'within_10m':int(np.sum(dist<10)),'within_20m':int(np.sum(dist<20)),'max_distance_m':float(max(dist)),'p95_distance_m':float(np.percentile(dist,95))};(ROOT/'coverage.json').write_text(json.dumps(coverage,indent=2));print(coverage)
if __name__=='__main__':main()
