"""RTK拘束地図の自己位置とPS3マーカ観測から、オフラインで2D経路を作る.

waypointの置き方は次のとおりである。記録した姿勢列に対し、始点・終点・各イベント点・
記録の途切れの両端を必ず残す点とし、その間をRamer-Douglas-Peucker法で許容誤差0.06 mに
間引く。残した線分を0.75 m以下の間隔へ等分して補間する。角は6 cmの精度で保たれ、
直線部は0.75 m間隔になる。進行方位は前後0.5 mの接線から求め、終点だけは実測の姿勢を使う。

入力と出力のパスは引数で受け取る。アルゴリズムは移植前と同一で、同じ入力から同じ
出力を得る。
"""
import argparse
from pathlib import Path
import json, sys, csv, math, hashlib
import numpy as np, yaml

from geo_pose_converter.geo_core import (ProjectionConfig, enu_to_llh_on_ground,
                                         yaw_enu_rad_to_heading_deg)

_parser = argparse.ArgumentParser(description=__doc__)
_parser.add_argument('--source', type=Path, required=True,
                     help='sources/ を含む採取結果のディレクトリ')
_parser.add_argument('--map', type=Path, required=True,
                     help='map_summary.json を含むRTK拘束地図のディレクトリ')
_parser.add_argument('--output', type=Path, required=True, help='経路の書き出し先')
_args = _parser.parse_args()
S = _args.source.expanduser().resolve()
M = _args.map.expanduser().resolve()
R = _args.output.expanduser().resolve()
(R/'route/fixed').mkdir(parents=True, exist_ok=True)
(R/'sources').mkdir(parents=True, exist_ok=True)

J=json.loads((S/'sources/joy_events.json').read_text());G={g['id']:g for run in J for g in run['groups']}
origin=json.loads((M/'map_summary.json').read_text())['origin_llh'];proj=ProjectionConfig(*origin,projection_id='tsukuba2026_rtk_graph_20261005')
# Labels are per traversal, including two visits to each road crossing.
def spec(gid,name,kind,site,direction,source,note,confidence='high'):
 return dict(marker_id=gid,name=name,kind=kind,site_id=site,direction=direction,official_image=source,interpretation=note,semantic_confidence=confidence)
roadimg='stop_area1_2026-1280x764.jpg';sigimg='crosswalk_2026-768x862.jpg'
specs=[
 spec('R1-01','庁舎東・横断歩道1 往路','road_edge_stop','cityhall_crosswalk_1','outbound',roadimg,'公式図の西端横断歩道。映像に横断歩道と道路端。4走行で押下。'),
 spec('R1-02','庁舎東・横断歩道2 往路','road_edge_stop','cityhall_crosswalk_2','outbound',roadimg,'お客様駐車場2の出入口。映像と公式図が一致。3走行で押下。'),
 spec('R1-03','庁舎東・車両出入口3 往路','road_edge_stop','cityhall_driveway_3','outbound',roadimg,'職員駐車場への車両出入口。反復走行でも道路端で押下。'),
 spec('R1-04','庁舎東・車両出入口4 往路','road_edge_stop','cityhall_driveway_4','outbound',roadimg,'東側の車両出入口。反復走行でも道路端で押下。'),
 spec('R1-05','研究学園交番前 往路','road_edge_stop','koban','outbound','koban_area_2026-1280x961.jpg','公式の交番前横断歩道。押下時映像に横断車両。'),
 spec('R1-06','市役所前交差点・待機列 往路','queue_stop','signal_crossing','outbound',sigimg,'道路端の約4.5m手前。看板の横で停止後に右へ向き直り道路端へ。余分な△は同一操作に統合。'),
 spec('R1-07','市役所前交差点・道路端 往路','signal_road_edge_stop','signal_crossing','outbound',sigimg,'横断歩道を東向きに渡る直前。映像で信号・道路端を確認。'),
 spec('R2-02','駅入口南側・横断歩道 往路','road_edge_stop','station_south_crosswalk','outbound','crosswalk_area_2026-1280x915.jpg','R2-01から約4m横移動後の押し直し候補。白い横断模様の前の後の位置を採用。','medium'),
 spec('R2-03','駅入口南側・横断歩道 復路','road_edge_stop','station_south_crosswalk','return','crosswalk_area_2026-1280x915.jpg','ホテル一周後の北向き横断。映像と公式図が一致。'),
 spec('R2-04','市役所前交差点・待機列 復路','queue_stop','signal_crossing','return',sigimg,'復路の横断歩道手前で停止後、約15.6m西南西へ進み道路端で再停止。'),
 spec('R2-05','市役所前交差点・道路端 復路','signal_road_edge_stop','signal_crossing','return',sigimg,'横断歩道を西向きに渡る直前。映像に横断車両。'),
 spec('R3-01','研究学園交番前 復路','road_edge_stop','koban','return','koban_area_2026-1280x961.jpg','南向き。映像に交番標識と横断歩道。'),
 spec('R3-02','庁舎東・車両出入口4 復路','road_edge_stop','cityhall_driveway_4','return',roadimg,'西向き。往路と反対側の道路端。'),
 spec('R3-03','庁舎東・車両出入口3 復路','road_edge_stop','cityhall_driveway_3','return',roadimg,'西向き。往路と反対側の道路端。'),
 spec('R3-04','庁舎東・横断歩道2 復路','road_edge_stop','cityhall_crosswalk_2','return',roadimg,'駐車場2出口の東側から西向きに横断。'),
 spec('R3-05','庁舎東・横断歩道1 復路','road_edge_stop','cityhall_crosswalk_1','return',roadimg,'庁舎入口へ西向きに横断。'),
 spec('R3-06','ゴール停止位置','goal_stop','goal','finish','goal_area2_2026-1280x739.jpg','ゴール付近での押下。車体全体が白線を越えたことは映像下端外のため未確定。','medium')]
repeat={'R1-01':['R4-01','R5-01','R6-01'],'R1-02':['R4-02','R6-02'],'R1-03':['R6-03'],'R1-04':['R6-04']}
# User replies can be recorded here without modifying data extraction.
answers=json.loads((S/'user_review.json').read_text()) if (S/'user_review.json').exists() else {}
if answers.get('station_duplicate_confirmed'):
 specs[7]['semantic_confidence']='high';specs[7]['interpretation']+=' ユーザーが押し直しと確認。'
tracks=[]
for run in [1,2,3]:
 d=np.load(S/f'sources/track_run{run}.npz');t=d['time'];mask=np.ones(len(t),bool)
 if run==1:mask=t>=1791078230.4560535
 if run==3:mask=np.arange(len(t))<=G['R3-06']['frame']
 inds=np.flatnonzero(mask)
 tracks.append(dict(time=t[mask],xy=d['xyz'][mask,:2],yaw=d['yaw'][mask],run=np.repeat(run,len(inds)),frame=inds,segment=d['segment'][mask]))
raw={k:np.concatenate([v[k] for v in tracks]) for k in tracks[0]}
N=len(raw['time']);event_rows=[]
for i,s in enumerate(specs,1):
 g=G[s['marker_id']];idx=int(np.flatnonzero((raw['run']==g['run'])&(raw['frame']==g['frame']))[0]);xy=raw['xy'][idx]
 e=dict(id=f'E{i:02d}',**s,raw_index=idx,source_run=g['run'],source_frame=g['frame'],time=g['time'],jst=g['jst'],x=float(xy[0]),y=float(xy[1]),marker_pattern=g['pattern'],pose_sync_error_s=g['pose_dt'],recorded_speed_mps=g['speed'],camera=f"sources/camera/{g['id']}_+0s.jpg",source_position='recorded_robot_base',stop_target_offset_m=0.0,position_status='marker_position_not_surveyed_line',line_is_stop=1,signal_is_stop=0,isnot_skipnum=1,resume_policy='manual_safety_confirmation',review_required=s['semantic_confidence']!='high')
 e['signal_controlled']=e['kind']=='signal_road_edge_stop'
 e['boundary_type']='road_edge' if e['kind'] in ('road_edge_stop','signal_road_edge_stop') else 'white_stop_line' if e['kind']=='queue_stop' else 'goal_line'
 e['official_boundary_tolerance_m']={'before':1.5,'body_must_not_cross':True} if e['boundary_type']=='road_edge' else {'before':1.0,'after':.5,'body_must_not_exceed_after':True} if e['boundary_type']=='white_stop_line' else {'entire_body_must_cross':True}
 e['repeat_observations']=[dict(id=h,x=G[h]['x'],y=G[h]['y'],distance_to_primary_m=float(np.hypot(G[h]['x']-g['x'],G[h]['y']-g['y'])),camera=f'sources/camera/{h}_+0s.jpg') for h in repeat.get(g['id'],[])]
 if e['kind']=='signal_road_edge_stop':e['optional_signal_recognition_attribute']=True;e['signal_is_stop']=0 # 緑の検出だけでは横断先が空いていることを確認できない。
 if e['kind']=='goal_stop':e['resume_policy']='finish';e['line_is_stop']=0 # Native follower enters FINISHED at final ordinary WP.
 event_rows.append(e)
# Deterministic simplification between every required marker and recording boundary.
forced=sorted(set([0,N-1]+[e['raw_index'] for e in event_rows]+np.flatnonzero((np.diff(raw['time'])>.3)|(np.diff(raw['run'])!=0)).tolist()+ (np.flatnonzero((np.diff(raw['time'])>.3)|(np.diff(raw['run'])!=0))+1).tolist()))
def rdp_indices(a,tol=.06):
 keep={0,len(a)-1};stack=[(0,len(a)-1)]
 while stack:
  i,j=stack.pop()
  if j-i<2:continue
  v=a[j]-a[i];den=float(v@v);b=a[i+1:j];u=np.clip((b-a[i])@v/den,0,1) if den>1e-15 else np.zeros(len(b));dist=np.linalg.norm(b-(a[i]+u[:,None]*v),axis=1);k=int(np.argmax(dist))
  if dist[k]>tol:k+=i+1;keep.add(k);stack.extend([(i,k),(k,j)])
 return np.array(sorted(keep),int)
keep=[]
for start,end in zip(forced[:-1],forced[1:]):keep.extend((rdp_indices(raw['xy'][start:end+1])+start).tolist())
keep=np.array(sorted(set(keep)));xy=raw['xy'][keep]
# Avoid numerical repeated vertices, never drop an event or a gap anchor.
mask=np.ones(len(keep),bool)
for i in range(1,len(keep)):
 if np.linalg.norm(xy[i]-xy[i-1])<.001 and keep[i] not in forced:mask[i]=False
keep=keep[mask];xy=raw['xy'][keep]
# Densify line segments, preserving all retained corners, marker vertices and their provenance.
points=[];key_to_wp={}
for i in range(len(keep)-1):
 n=max(1,int(np.ceil(np.linalg.norm(xy[i+1]-xy[i])/.75)))
 key_to_wp[int(keep[i])]=len(points)
 for j in range(n):
  f=j/n;points.append(dict(x=float(xy[i,0]*(1-f)+xy[i+1,0]*f),y=float(xy[i,1]*(1-f)+xy[i+1,1]*f),source_time=float(raw['time'][keep[i]]*(1-f)+raw['time'][keep[i+1]]*f),source_run=int(raw['run'][keep[i]]),source_start_frame=int(raw['frame'][keep[i]]),source_end_frame=int(raw['frame'][keep[i+1]]),interpolated=bool(j),source_raw_start=int(keep[i]),source_raw_end=int(keep[i+1])))
key_to_wp[int(keep[-1])]=len(points);points.append(dict(x=float(xy[-1,0]),y=float(xy[-1,1]),source_time=float(raw['time'][-1]),source_run=3,source_start_frame=int(raw['frame'][-1]),source_end_frame=int(raw['frame'][-1]),interpolated=False,source_raw_start=N-1,source_raw_end=N-1))
p=np.array([[v['x'],v['y']] for v in points]);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
# Tangent over a 1m window is less sensitive to in-place operator steering than body yaw.
yaws=[]
for i in range(len(p)):
 l=int(np.searchsorted(arc,max(0,arc[i]-.5)));r=min(len(p)-1,int(np.searchsorted(arc,arc[i]+.5)))
 if l==r:l=max(0,i-1);r=min(len(p)-1,i+1)
 v=p[r]-p[l];yaws.append(float(np.arctan2(v[1],v[0])))
by_wp={}
for e in event_rows:
 e['waypoint_index']=key_to_wp[e['raw_index']];e['distance_from_start_m']=float(arc[e['waypoint_index']]);by_wp[e['waypoint_index']]=e
# Preserve measured final orientation; road stop heading follows the upcoming crossing trajectory.
yaws[-1]=float(raw['yaw'][-1])
fields=['label','latitude','longitude','altitude','heading_deg','right_is_open','left_is_open','line_is_stop','signal_is_stop','isnot_skipnum']
rows=[]
for i,v in enumerate(points):
 llh=enu_to_llh_on_ground(v['x'],v['y'],proj,ground_altitude=proj.origin_altitude);e=by_wp.get(i)
 rows.append(dict(label=e['id'] if e else f'WP{i:04d}',latitude=f'{llh.latitude:.11f}',longitude=f'{llh.longitude:.11f}',altitude='',heading_deg=f'{yaw_enu_rad_to_heading_deg(yaws[i]):.8f}',right_is_open=0,left_is_open=0,line_is_stop=int(e['line_is_stop']) if e else 0,signal_is_stop=int(e['signal_is_stop']) if e else 0,isnot_skipnum=1))
 v.update(index=i,label=rows[-1]['label'],yaw=yaws[i],distance_from_start_m=float(arc[i]),event_id=e['id'] if e else None,width_status='unknown',**{k:rows[-1][k] for k in fields[1:]})
for directory,values in [(R/'route',rows)]:
 with (directory/'fixed/waypoints.csv').open('w',newline='') as f:w=csv.DictWriter(f,fields);w.writeheader();w.writerows(values)
 (directory/'projection.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':vars(proj)}},sort_keys=False))
 (directory/'route_config.yaml').write_text(yaml.safe_dump({'blocks':[{'name':'tsukuba2026_recorded_full_course','segment_id':'fixed/waypoints.csv','type':'fixed'}]},sort_keys=False))
 (directory/'waypoints_enu.json').write_text(json.dumps(points,ensure_ascii=False,indent=2))
 (directory/'survey.json').write_text(json.dumps(dict(version=2,active=False,generated_offline=True,production_activated=False,validation_state='offline_candidate',projection=vars(proj),waypoints=[dict(v,reason='offline_rtk_graph_route',observation={'method':'not_evaluated','review_required':True}) for v in rows]),ensure_ascii=False,indent=2))
# Extra markers retain evidence. Missing button at repeats does not remove a required crossing.
excluded=[dict(id='X01',marker_id='R2-01',kind='duplicate_or_approach_marker',active_stop=False,superseded_by='E08',user_confirmed=bool(answers.get('station_duplicate_confirmed')),review_required=not bool(answers.get('station_duplicate_confirmed')),x=G['R2-01']['x'],y=G['R2-01']['y'],time=G['R2-01']['time'],reason='20.2秒後に同じ横断歩道の約4.1m西側で再押下。後の映像が横断位置に一致。意図が別なら独立イベントへ戻す。',camera='sources/camera/R2-01_+0s.jpg')]
g=G['R4-03'];ind_candidates=np.flatnonzero((raw['run']==1)&(raw['xy'][:,0]>60)&(raw['xy'][:,0]<110)&(raw['xy'][:,1]>40)&(raw['xy'][:,1]<80));craw=int(ind_candidates[np.argmin(np.linalg.norm(raw['xy'][ind_candidates]-[g['x'],g['y']],axis=1))]);ci=int(np.argmin(abs(np.array([v['source_time'] for v in points])-raw['time'][craw])))
confirmation=dict(id='C01',kind='confirmation_end_reference',name='確認走行区間の終了目安',marker_id='R4-03',x=points[ci]['x'],y=points[ci]['y'],observed_x=g['x'],observed_y=g['y'],snap_distance_m=float(np.linalg.norm(np.array([points[ci]['x'],points[ci]['y']])-[g['x'],g['y']])),waypoint_index=ci,distance_from_start_m=float(arc[ci]),active_stop=False,review_required=not bool(answers.get('confirmation_end_confirmed') or answers.get('confirmation_end_judge_from_map')),user_delegated_map_judgment=bool(answers.get('confirmation_end_judge_from_map')),user_confirmed=bool(answers.get('confirmation_end_confirmed')),semantic_confidence='medium',interpretation='公式の星印付近。逆順押下の約15秒後にfinishコマンド。全周ルートでは通過点。地図・映像・記録終了時刻からの推定。',camera='sources/camera/R4-03_+0s.jpg')
info=[dict(id='START',kind='start',name='記録時スタート位置',x=float(p[0,0]),y=float(p[0,1]),waypoint_index=0,distance_from_start_m=0,active_stop=False),dict(id='D20',kind='distance_20m_reference',name='開始点から経路距離20mの目安',x=float(np.interp(20,arc,p[:,0])),y=float(np.interp(20,arc,p[:,1])),waypoint_index=int(np.searchsorted(arc,20)),distance_from_start_m=20,active_stop=False,review_required=True,interpretation='公式の20m白線を測量した位置ではない。'),confirmation]
for e in event_rows+excluded+info:
 llh=enu_to_llh_on_ground(e['x'],e['y'],proj,ground_altitude=proj.origin_altitude);e.update(latitude=llh.latitude,longitude=llh.longitude)
# Optional tasks are not silently enabled by marker interpretation.
optional=dict(A=dict(enabled=False,reason='事前計測なし区域。固定計測ルートを作成しない。'),B=dict(enabled=False,road_edge_events=['E07','E11'],reason='信号属性はイベントに保持。現followerのsignal_stopはsig_recog==1だけでも再開するため基本CSVはオペレータ再開。横断先安全判断と合わせた実装確認が必要。'),D1_D2=dict(enabled=False,reason='配送先が当日指定のため固定配送点を生成しない。'),E=dict(enabled=False,reason='対象ロボットの認識機能を本ルート生成では実装しない。'))
# Duplicate detection quantifies along/lateral relation using original body direction.
a=G['R2-01'];b=G['R2-02'];delta=np.array([b['x']-a['x'],b['y']-a['y']]);fwd=np.array([np.cos(b['yaw']),np.sin(b['yaw'])]);excluded[0].update(separation_m=float(np.linalg.norm(delta)),along_m=float(delta@fwd),lateral_m=float(delta@np.array([-fwd[1],fwd[0]])),time_difference_s=b['time']-a['time'])
# All 26 groups receive an explicit disposition.
dispositions=[]
for run in J:
 for g in run['groups']:
  e=next((e for e in event_rows if e['marker_id']==g['id']),None);matched=next((e for e in event_rows if any(h['id']==g['id'] for h in e['repeat_observations'])),None)
  dispositions.append(dict(marker_id=g['id'],pattern=g['pattern'],run=g['run'],disposition='route_event' if e else 'repeat_support' if matched else 'review_duplicate' if g['id']=='R2-01' else 'confirmation_end',event_id=e['id'] if e else matched['id'] if matched else 'E08' if g['id']=='R2-01' else 'C01',x=g['x'],y=g['y'],time=g['time'],jst=g['jst']))
gaps=[]
for i in np.flatnonzero((np.diff(raw['time'])>.3)|(np.diff(raw['run'])!=0)):
 gaps.append(dict(from_run=int(raw['run'][i]),to_run=int(raw['run'][i+1]),time_start=float(raw['time'][i]),time_end=float(raw['time'][i+1]),duration_s=float(raw['time'][i+1]-raw['time'][i]),endpoint_distance_m=float(np.linalg.norm(raw['xy'][i+1]-raw['xy'][i])),x1=float(raw['xy'][i,0]),y1=float(raw['xy'][i,1]),x2=float(raw['xy'][i+1,0]),y2=float(raw['xy'][i+1,1]),method='straight_endpoint_link',limitation='間の走行は未観測。近接した停止前後の端点を接続。'))
payload=dict(schema_version=1,coordinate_system='robot base XY in ENU; geographic route on fixed origin altitude',origin_llh=origin,events=event_rows,informational_events=info,excluded_markers=excluded,optional_tasks=optional,all_marker_dispositions=dispositions,gaps=gaps)
(R/'events.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2))
with (R/'events.csv').open('w',newline='') as f:
 cols=['id','name','kind','direction','marker_id','jst','x','y','latitude','longitude','waypoint_index','distance_from_start_m','semantic_confidence','review_required'];w=csv.DictWriter(f,cols,extrasaction='ignore');w.writeheader();w.writerows(event_rows)
features=[dict(type='Feature',properties=dict(kind='route',length_m=float(arc[-1]),status='offline_candidate'),geometry=dict(type='LineString',coordinates=[[float(r['longitude']),float(r['latitude'])] for r in rows]))]
for e in event_rows+info+excluded:features.append(dict(type='Feature',properties=e,geometry=dict(type='Point',coordinates=[e['longitude'],e['latitude']])))
(R/'route_and_events.geojson').write_text(json.dumps(dict(type='FeatureCollection',features=features),ensure_ascii=False))
np.savez_compressed(R/'sources/route_geometry.npz',raw_xy=raw['xy'],raw_time=raw['time'],raw_run=raw['run'],raw_frame=raw['frame'],simplified_indices=keep,route_xy=p,route_arc=arc,route_yaw=yaws)
summary=dict(length_m=float(arc[-1]),waypoints=len(rows),source_pose_count=N,button_messages=sum(r['joy_count'] for r in J),button_rising_edges=sum(len(g['presses']) for r in J for g in r['groups']),marker_groups=len(G),course_events=len(event_rows),stop_waypoints=sum(r['line_is_stop'] for r in rows),road_edge_stops=sum(e['kind']=='road_edge_stop' for e in event_rows),queue_stops=2,signal_road_edge_stops=2,goal_events=1,repeat_supports=sum(len(e['repeat_observations']) for e in event_rows),simplification_tolerance_m=.06,max_waypoint_spacing_m=.75,source_map=str(M/'map_rtk_constrained_enu.pcd'),source_map_sha256=json.loads((M/'map_summary.json').read_text())['map_sha256'],source_map_sha256_file='map_enu_xyz_intensity.npy',gnss_used_for_map=True,gnss_used_for_route_tracking=False,full_course_source_runs=[1,2,3],repeat_validation_runs=[4,5,6],native_signal_stop_enabled=False,route_widths='unknown; zero lateral avoidance allowance',production_activated=False,review_items=['R2-01押し直しの解釈','停止目標と車体外形・到達閾値の整合','ゴール白線と車体最後端の位置'],gaps=gaps)
(R/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False,indent=2))
