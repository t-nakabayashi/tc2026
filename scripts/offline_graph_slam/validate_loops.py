#!/usr/bin/env python3
import json
import numpy as np
from scipy.spatial import cKDTree
from common import ROOT
from solve_graph import submap
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
plt.rcParams['font.family']=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name();plt.rcParams['axes.unicode_minus']=False

def main():
 before=np.load(ROOT/'initial_poses.npy');after=np.load(ROOT/'optimized_poses.npy');loops=json.loads((ROOT/'loops.json').read_text());rows=[]
 for e in loops:
  i,j=e['i'],e['j'];a,_,_,kd=submap(i);b,_,_,_=submap(j);row={'i':i,'j':j}
  for name,poses in [('before',before),('after',after)]:
   x=np.linalg.inv(poses[i])@poses[j];dist=kd.query(b@x[:3,:3].T+x[:3,3])[0];row[name]={'median_nn_m':float(np.median(dist)),'within_0_5m_fraction':float(np.mean(dist<.5))}
  rows.append(row)
 (ROOT/'loop_geometry.json').write_text(json.dumps(rows,indent=2))
 fig,axs=plt.subplots(2,2,figsize=(11,10))
 for line,e in enumerate([loops[2],loops[13]]):
  i,j=e['i'],e['j'];a=submap(i)[0][::2];b=submap(j)[0][::2]
  for col,(name,poses) in enumerate([('閉合前',before),('閉合後',after)]):
   center=poses[i,:3,3];aa=a@poses[i,:3,:3].T;bb=b@poses[j,:3,:3].T+poses[j,:3,3]-center;ax=axs[line,col]
   for v,color,label in [(aa,'#444c5f','先行走行'),(bb,'#ed5968','再訪走行')]:
    u=(abs(v[:,0])<25)&(abs(v[:,1])<25)&(v[:,2]>.1)&(v[:,2]<7);ax.scatter(v[u,0],v[u,1],s=.8,c=color,label=label,alpha=.6)
   ax.set(aspect='equal',xlim=(-25,25),ylim=(-25,25),title=f'{name} | キーフレーム {i} / {j}',xlabel='地図Xの相対距離 [m]',ylabel='地図Yの相対距離 [m]');ax.legend(fontsize=9)
 fig.suptitle('再訪点群の重なり | GNSS入力なし',fontsize=17);fig.tight_layout();fig.savefig(ROOT/'figures/loop_geometry.png',dpi=160);plt.close(fig)
 print('NN medians before/after',np.median([r['before']['median_nn_m'] for r in rows]),np.median([r['after']['median_nn_m'] for r in rows]))
if __name__=='__main__':main()
