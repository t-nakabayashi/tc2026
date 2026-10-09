#!/usr/bin/env python3
"""Overlay exact trajectory coordinates on GSI aerial imagery and inspect selected regions."""
import json,datetime,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
from matplotlib.path import Path as PolygonPath
from matplotlib.font_manager import FontProperties
from PIL import Image
from common import ROOT
from aerial_base import OUT,xy,llh,origin
FONT=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
plt.rcParams.update({'font.family':FONT,'font.size':12,'axes.unicode_minus':False,'figure.facecolor':'#f5f7fb','savefig.facecolor':'#f5f7fb'})
REGIONS=[
 {'id':'A','name':'市役所南側の曲線区間','bounds':[-110,15,-17,57]},
 {'id':'B','name':'市役所東側の交差・接続部','bounds':[-5,70,28,115]},
 {'id':'C','name':'連絡道路の往復区間','bounds':[95,235,27,85]},
 {'id':'D','name':'大周回の北東角','bounds':[431,505,132,207]},
 {'id':'E','name':'ホテル南側の折返し・角','bounds':[376,472,-89,-28]},
 {'id':'F','name':'ホテル北東側の接続部','bounds':[420,501,13,69]},
]
COL={4:'#19dd87',3:'#ffbd3c',2:'#529cff'};LABEL={4:'RTK FIX',3:'RTK FLOAT',2:'DGPS'}
D=np.load(ROOT/'comparison.npz');VALID=D['valid'];SLAM=D['slam_postfit'];GPS=D['gps'][:,:2];SX=xy(SLAM);GX=xy(D['gps']);ERR=np.linalg.norm(SLAM-GPS,axis=1);TM=D['time'];QUALITY=D['quality'];MAN=json.loads((OUT/'aerial_manifest.json').read_text());BG=np.array(Image.open(OUT/'aerial_background.png'));ZONE=datetime.timezone(datetime.timedelta(hours=9));T0=np.load(ROOT/'obs_0.npz')['fix'][0,0]-TM[0]
def stat(x):
 x=np.asarray(x);return {'n':len(x),'rmse_m':float(np.sqrt(np.mean(x*x))),'median_m':float(np.median(x)),'p95_m':float(np.percentile(x,95)),'max_m':float(max(x))} if len(x) else {'n':0}
def inside(a,b):return np.isfinite(a).all(1)&(a[:,0]>=b[0])&(a[:,0]<=b[1])&(a[:,1]>=b[2])&(a[:,1]<=b[3])
def fmt(t):return datetime.datetime.fromtimestamp(float(T0+t),ZONE).strftime('%H:%M:%S')
def passes(mask):
 inds=np.flatnonzero(mask)
 if not len(inds):return []
 cut=np.r_[0,np.where((np.diff(inds)>1)|(np.diff(TM[inds])>1)|(np.diff(D['segment'][inds])!=0))[0]+1,len(inds)]
 return [inds[a:b] for a,b in zip(cut[:-1],cut[1:]) if b-a>=5]
def legend():return [Line2D([],[],color='#fa3754',lw=2,label='グラフSLAM（下層）')]+[Line2D([],[],ls='',marker='o',markersize=6,color=COL[q],markeredgecolor='#34404d',markeredgewidth=.4,label=LABEL[q]) for q in [4,3,2]]
def background(ax,b):
 ax.imshow(BG,extent=MAN['extent'],origin='upper',zorder=0,interpolation='nearest');ax.set(xlim=b[:2],ylim=b[2:],aspect='equal');ax.set_xlabel('東 [m：原点基準]');ax.set_ylabel('北 [m：原点基準]');ax.tick_params(labelsize=10)
 length=100 if b[1]-b[0]>400 else 20 if b[1]-b[0]>100 else 10;x=b[0]+.05*(b[1]-b[0]);y=b[2]+.06*(b[3]-b[2]);stroke=[pe.Stroke(linewidth=5,foreground='white'),pe.Normal()]
 ax.plot([x,x+length],[y,y],c='black',lw=2,path_effects=stroke,zorder=8);ax.text(x+length/2,y+.025*(b[3]-b[2]),f'{length} m',ha='center',fontsize=10,color='black',bbox=dict(fc='white',ec='none',alpha=.85),zorder=9)
 ax.text(.98,.95,'北 ↑',transform=ax.transAxes,ha='right',color='white',fontsize=12,path_effects=[pe.withStroke(linewidth=3,foreground='black')],zorder=9)
def draw_tracks(ax,mask=VALID,size=8):
 for inds in passes(mask):ax.plot(SX[inds,0],SX[inds,1],color='#fa3754',lw=1.8,zorder=2)
 for q in [2,3,4]:
  u=mask&(QUALITY==q);ax.scatter(GX[u,0],GX[u,1],s=size,c=COL[q],edgecolors='#14263b',linewidths=.12,zorder=3)
def credit(fig,extra=''):
 fig.text(.045,.024,'出典：国土地理院 地理院タイル（2017年2月撮影）に軌跡・注記を重ねて作成。'+extra,fontsize=9,color='#465268')
 fig.text(.045,.008,'航空写真の地上画素は約0.48 m。全良好FIXで剛体整列したSLAMを表示。写真への手動位置合わせは行っていない。',fontsize=8.5,color='#465268')

def main():
 out=OUT/'figures';out.mkdir(exist_ok=True)
 manifest=MAN.copy();spec=json.loads((OUT/'photo_metadata.geojson').read_text());ll=llh(D['gps']);coverage=[]
 for f in spec['features']:
  g=f['geometry'];polys=g['coordinates'] if g['type']=='MultiPolygon' else [g['coordinates']];hit=np.zeros(len(ll),bool)
  for p in polys:
   h=PolygonPath(p[0]).contains_points(ll[:,:2])
   for hole in p[1:]:h &= ~PolygonPath(hole).contains_points(ll[:,:2])
   hit |= h
  if np.any(hit):coverage.append({'source':f['properties']['データソース'],'date':f['properties']['撮影年月'],'trajectory_observations':int(hit.sum())})
 manifest['trajectory_photo_coverage']=coverage;(OUT/'aerial_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
 fig,ax=plt.subplots(figsize=(13,8));b=[-135,535,-105,232];background(ax,b);draw_tracks(ax,size=7)
 for region in REGIONS:
  rb=region['bounds'];ax.add_patch(Rectangle((rb[0],rb[2]),rb[1]-rb[0],rb[3]-rb[2],fill=False,edgecolor='white',lw=1.5,zorder=7));ax.text(rb[0]+2,rb[3]-3,region['id'],va='top',color='#15283d',fontweight='bold',fontsize=14,bbox=dict(boxstyle='round,pad=.22',fc='white',ec='none'),zorder=8)
 ax.legend(handles=legend(),loc='upper center',bbox_to_anchor=(.5,1.10),ncol=4,fontsize=10,framealpha=1);fig.suptitle('つくば2026 | 航空写真上のSLAM・GNSS軌跡',fontsize=19,y=.98);credit(fig);fig.tight_layout(rect=[0,.055,1,.95]);fig.savefig(out/'aerial_overview.png',dpi=190);plt.close(fig)
 results=[]
 for r in REGIONS:
  b=r['bounds'];u=VALID&inside(SLAM,b);observed=u&np.isin(QUALITY,[2,3,4]);runs=passes(u)
  result={**r,'selection':'all observations whose aligned SLAM antenna lies inside the ENU rectangle','n':int(observed.sum()),'by_state':{},'all':stat(ERR[observed]),'passes':[]}
  for q in [4,3,2]:result['by_state'][LABEL[q]]={'fraction_pct':float(np.mean(QUALITY[observed]==q)*100),**stat(ERR[observed&(QUALITY==q)])}
  for inds in runs:
   result['passes'].append({'start_jst':fmt(TM[inds[0]]),'end_jst':fmt(TM[inds[-1]]),'n':len(inds),'median_m':float(np.median(ERR[inds])),'max_m':float(max(ERR[inds])),'fix_pct':float(np.mean(QUALITY[inds]==4)*100)})
  i=int(np.argmax(np.where(u,ERR,-1)));result['max_sample']={'stamp_jst':fmt(TM[i]),'elapsed_s':float(TM[i]),'quality':int(QUALITY[i]),'gps_enu':GPS[i].tolist(),'slam_enu':SLAM[i].tolist(),'difference_m':float(ERR[i])}
  results.append(result)
  fig,ax=plt.subplots(figsize=(10,9));background(ax,b);draw_tracks(ax,size=12)
  # The maximum connector is a synchronous trajectory difference, not a photograph-measured error.
  ax.plot([SX[i,0],GX[i,0]],[SX[i,1],GX[i,1]],color='#fff56e',ls='--',lw=2,zorder=6)
  center=(SX[i]+GX[i])/2;ax.annotate(f'同時刻の差 {ERR[i]:.2f} m\n{fmt(TM[i])} / {LABEL[int(QUALITY[i])]}',xy=center,xycoords='data',xytext=(.04,.94),textcoords='axes fraction',va='top',fontsize=11,color='#1c2c43',bbox=dict(boxstyle='round,pad=.45',fc='white',ec='#fff56e',alpha=.94),arrowprops=dict(arrowstyle='->',color='#fff56e',lw=1.7),zorder=10)
  fig.suptitle(f'{r["id"]} | {r["name"]}',fontsize=19,y=.98);fig.legend(handles=legend(),loc='upper center',bbox_to_anchor=(.5,.944),ncol=4,fontsize=10)
  f=result['by_state']['RTK FIX']['fraction_pct'];p95=result['all']['p95_m'];fig.text(.08,.073,f'全状態の水平差：中央値 {result["all"]["median_m"]:.2f} m / 95%点 {p95:.2f} m / 最大 {ERR[i]:.2f} m',fontsize=12)
  fig.text(.08,.051,f'FIX観測割合 {f:.1f}%（停止含む） | 差はGNSS対SLAM。航空写真を真値とした誤差ではない。',fontsize=10)
  credit(fig);fig.subplots_adjust(left=.09,right=.97,bottom=.17,top=.86);fig.savefig(out/f'detail_{r["id"]}.png',dpi=180);plt.close(fig)
  # Separate repeated visits to prevent one pass's GNSS status hiding another's.
  mainruns=[inds for inds in runs if TM[inds[-1]]-TM[inds[0]]>5]
  if len(mainruns)>1 and r['id'] in ['B','C']:
   fig,axs=plt.subplots(1,len(mainruns),figsize=(7*len(mainruns),7),squeeze=False)
   for n,(ax,inds) in enumerate(zip(axs.flat,mainruns)):
    background(ax,b);mask=np.zeros(len(VALID),bool);mask[inds]=True;draw_tracks(ax,mask,size=13);ax.set_title(f'{n+1}回目 {fmt(TM[inds[0]])}-{fmt(TM[inds[-1]])}\n中央値 {np.median(ERR[inds]):.2f} m / 最大 {max(ERR[inds]):.2f} m',fontsize=12)
   fig.suptitle(f'{r["id"]} | 同じ場所の通過を時間で分離',fontsize=18);fig.legend(handles=legend(),loc='upper center',bbox_to_anchor=(.5,.935),ncol=4,fontsize=10);credit(fig);fig.tight_layout(rect=[0,.06,1,.86]);fig.savefig(out/f'passes_{r["id"]}.png',dpi=170);plt.close(fig)
 (OUT/'regional_evaluation.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
 with (OUT/'regional_summary.csv').open('w') as f:
  w=csv.writer(f);w.writerow(['region','name','observations','fix_pct','float_pct','dgps_pct','median_difference_m','p95_difference_m','max_difference_m'])
  for r in results:w.writerow([r['id'],r['name'],r['n'],*[r['by_state'][LABEL[q]]['fraction_pct'] for q in [4,3,2]],r['all']['median_m'],r['all']['p95_m'],r['all']['max_m']])
 # Exchange format for geographic viewers; no external publication.
 features=[]
 for name,data in [('SLAM',SLAM),('GNSS',D['gps'])]:
  geo=llh(data)
  for k,inds in enumerate(passes(VALID)):
   keep=np.unique(np.r_[inds[::5],inds[-1]]);features.append({'type':'Feature','properties':{'name':name,'segment':k,'alignment':'SLAM uses all-FIX rigid alignment; no GNSS graph factors'},'geometry':{'type':'LineString','coordinates':geo[keep,:2].tolist()}})
 (OUT/'trajectories.geojson').write_text(json.dumps({'type':'FeatureCollection','features':features},ensure_ascii=False))
 # Verify ECEF/ENU inverse using the source latitude/longitude, not picture pixels.
 source=np.load(ROOT/'obs_0.npz')['fix'];back=llh(D['gps']);error=float(np.max(abs(back[:,:2]-source[:,[2,1]])));assert error<1e-8
 (OUT/'coordinate_check.json').write_text(json.dumps({'max_roundtrip_lonlat_degree':error,'photo_registration_fitted':False,'source_observations':len(source),'origin_llh':origin.tolist(),'ground_pixel_m':MAN['ground_pixel_m']},indent=2));print(json.dumps(results,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
