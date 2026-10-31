#!/usr/bin/env python3
"""前方映像と俯瞰図を並べ、区間ごとに道幅を記入する単体HTMLを作る.

判定は利用者が行う。推定値は参考として表示するだけで、初期値には入れない。
記入結果はCSVとJSONで保存でき、走行中の経路へは自動反映しない。
"""
import argparse
import csv
import json
from pathlib import Path

KINDS = [
    ('wide', '歩道・広い（左右へ出てよい）'),
    ('narrow', '狭い・並ぶ（左右へ出ない）'),
    ('roadway', '車道隣接（コーン外へ出不可）'),
    ('hold', '判断保留（要再確認）'),
]

TEMPLATE = """<!doctype html><html lang="ja"><meta charset="utf-8">
<title>道幅の区間確認</title>
<style>
body{margin:0;font:15px system-ui;color:#102c45;background:#f4f6f8}
header{position:sticky;top:0;background:#102c45;color:#fff;padding:10px 16px;z-index:9;
display:flex;gap:16px;align-items:center;flex-wrap:wrap}
header b{font-size:17px} header button{font:14px system-ui;padding:6px 12px}
main{padding:16px;max-width:1180px;margin:0 auto}
section{background:#fff;border-radius:8px;padding:12px;margin-bottom:18px;
box-shadow:0 1px 3px rgba(0,0,0,.14);scroll-margin-top:64px}
section.done{border-left:7px solid #1b7f4b} section.cur{outline:3px solid #ffb000}
.row{display:flex;gap:14px;flex-wrap:wrap} .row img{border:1px solid #d0d6dc;border-radius:4px}
.cam{width:620px;max-width:100%} .map{height:470px}
.ctl{display:flex;gap:18px;flex-wrap:wrap;align-items:center;margin-top:10px}
.ctl label{display:flex;gap:5px;align-items:center}
input[type=number]{width:72px} .note{width:100%;margin-top:8px}
.est{color:#5b6b7b;font-size:14px} .warn{color:#c0392b}
</style>
<header><b>道幅の区間確認</b><span id="progress"></span>
<button id="save_csv">CSV保存</button><button id="save_json">JSON保存</button>
<label style="color:#fff">読込<input id="load" type="file" accept=".json"></label>
<span style="font-size:13px">j/k で移動、1-4 で種別、c で前区間の値を複製</span></header>
<main id="list"></main>
<script>
const DATA=__REVIEW_DATA__, KINDS=__KINDS__, KEY='route_width_review';
let state=JSON.parse(localStorage.getItem(KEY)||'{}'), cur=0;
const $=id=>document.getElementById(id);
function store(){localStorage.setItem(KEY,JSON.stringify(state));progress()}
function progress(){
 const n=DATA.filter(d=>state[d.target_distance_m]&&state[d.target_distance_m].kind).length;
 $('progress').textContent=`記入 ${n} / ${DATA.length} 区間`;
 DATA.forEach(d=>{const s=document.getElementById('s'+d.target_distance_m);
  s.classList.toggle('done',!!(state[d.target_distance_m]||{}).kind)})}
function build(){
 DATA.forEach((d,i)=>{
  const s=document.createElement('section');s.id='s'+d.target_distance_m;
  const e=d.left_estimate===null?'未確定':`左 ${d.left_estimate.toFixed(1)} / 右 ${d.right_estimate.toFixed(1)} m`;
  s.innerHTML=`<b>${d.target_distance_m} m 地点</b>
   <span class="est">（${d.phase||''} / 推定 ${e} / 映像は ${d.lead_m} m 手前）</span>
   <div class="row"><img class="cam" src="${d.camera}" alt="前方映像">
   <img class="map" src="${d.panel}" alt="俯瞰図"></div>
   <div class="ctl">${KINDS.map(k=>`<label><input type="radio" name="k${i}" value="${k[0]}">${k[1]}</label>`).join('')}
   <label>左 <input type="number" step="0.1" min="0" max="5" id="L${i}"> m</label>
   <label>右 <input type="number" step="0.1" min="0" max="5" id="R${i}"> m</label>
   <button data-copy="${i}">前区間を複製</button></div>
   <input class="note" id="N${i}" placeholder="備考（コーンの位置、当日と異なる物など）">`;
  $('list').append(s);
  const v=state[d.target_distance_m]||{};
  if(v.kind)s.querySelector(`input[value="${v.kind}"]`).checked=true;
  if(v.left!==undefined)$('L'+i).value=v.left;
  if(v.right!==undefined)$('R'+i).value=v.right;
  if(v.note)$('N'+i).value=v.note;
  s.addEventListener('change',()=>record(i));
  s.addEventListener('input',()=>record(i));
  s.querySelector('[data-copy]').onclick=()=>copy(i)});
 progress();focus(0)}
function record(i){
 const d=DATA[i],s=$('s'+d.target_distance_m),k=s.querySelector(`input[name="k${i}"]:checked`);
 state[d.target_distance_m]={kind:k?k.value:'',left:$('L'+i).value,right:$('R'+i).value,
  note:$('N'+i).value};store()}
function copy(i){
 if(i===0)return;const p=state[DATA[i-1].target_distance_m];if(!p)return;
 const s=$('s'+DATA[i].target_distance_m);
 if(p.kind)s.querySelector(`input[value="${p.kind}"]`).checked=true;
 $('L'+i).value=p.left;$('R'+i).value=p.right;record(i)}
function focus(i){
 cur=Math.max(0,Math.min(DATA.length-1,i));
 DATA.forEach(d=>$('s'+d.target_distance_m).classList.remove('cur'));
 const s=$('s'+DATA[cur].target_distance_m);s.classList.add('cur');
 s.scrollIntoView({block:'start',behavior:'smooth'})}
document.addEventListener('keydown',ev=>{
 if(['INPUT','TEXTAREA'].includes(ev.target.tagName))return;
 if(ev.key==='j')focus(cur+1); else if(ev.key==='k')focus(cur-1);
 else if(ev.key==='c')copy(cur);
 else if('1234'.includes(ev.key)){
  const s=$('s'+DATA[cur].target_distance_m);
  s.querySelector(`input[value="${KINDS[+ev.key-1][0]}"]`).checked=true;record(cur)}});
function download(name,text,type){
 const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type}));
 a.download=name;a.click();URL.revokeObjectURL(a.href)}
$('save_csv').onclick=()=>{
 const head='distance_m,kind,left_m,right_m,left_estimate_m,right_estimate_m,note\\n';
 const body=DATA.map(d=>{const v=state[d.target_distance_m]||{};
  return [d.target_distance_m,v.kind||'',v.left||'',v.right||'',
   d.left_estimate===null?'':d.left_estimate.toFixed(2),
   d.right_estimate===null?'':d.right_estimate.toFixed(2),
   '"'+(v.note||'').replace(/"/g,'""')+'"'].join(',')}).join('\\n');
 download('width_review.csv',head+body,'text/csv')};
$('save_json').onclick=()=>download('width_review.json',
 JSON.stringify({records:DATA,review:state},null,1),'application/json');
$('load').onchange=async e=>{try{
 const d=JSON.parse(await e.target.files[0].text());
 if(!d.review)throw Error('review が無い');state=d.review;store();location.reload()}
 catch(err){alert('読込失敗: '+err.message)}};
build();
</script></html>
"""


def load_estimates(path: Path) -> dict[int, tuple]:
    """既存の左右退避候補を距離[m]で引けるようにする.

    Args:
        path (Path): review_widths.csv.

    Returns:
        dict[int, tuple]: 距離[m]から (左, 右, 走行区分)。未確定は None.
    """
    out = {}
    for row in csv.DictReader(path.open()):
        left = float(row['left_center_offset_m']) if row['left_center_offset_m'] else None
        right = float(row['right_center_offset_m']) if row['right_center_offset_m'] else None
        out[int(round(float(row['distance_m'])))] = (left, right, row.get('travel_phase', ''))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True, help='frames.json を含むディレクトリ')
    parser.add_argument('--panels', type=Path, required=True, help='俯瞰図PNGのディレクトリ')
    parser.add_argument('--review-widths', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='出力するHTMLのパス')
    args = parser.parse_args()

    records = json.loads((args.frames/'frames.json').read_text())
    estimates = load_estimates(args.review_widths)
    base = args.output.parent
    data = []
    for record in records:
        target = int(round(record['target_distance_m']))
        left, right, phase = estimates.get(target, (None, None, ''))
        panel = args.panels/Path(record['name']).with_suffix('.png').name
        if not panel.is_file():
            raise FileNotFoundError(f'俯瞰図が無い: {panel}')
        data.append(dict(target_distance_m=target, lead_m=record['lead_m'],
                         camera=str((args.frames/record['name']).relative_to(base)),
                         panel=str(panel.relative_to(base)),
                         left_estimate=left, right_estimate=right, phase=phase))
    html = TEMPLATE.replace('__REVIEW_DATA__', json.dumps(data, ensure_ascii=False))
    html = html.replace('__KINDS__', json.dumps(KINDS, ensure_ascii=False))
    args.output.write_text(html, encoding='utf-8')
    print(f'DONE {len(data)} 区間 -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
