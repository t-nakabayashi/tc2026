#!/usr/bin/env python3
"""Create the verified aerial-overlay evaluation PDF."""
import json, sys, hashlib, shutil
from pathlib import Path
sys.path.insert(0,'/home/nkb/colcon_ws_experiments/glim_runtime/python')
from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.lib.styles import ParagraphStyle
from PIL import Image
from common import ROOT
OUT=ROOT/'aerial'; FONT='HeiseiKakuGo-W5';pdfmetrics.registerFont(UnicodeCIDFont(FONT));W,H=A4;M=34
STYLE=ParagraphStyle('body',fontName=FONT,fontSize=11,leading=17,wordWrap='CJK',textColor=colors.HexColor('#23334b'))
SMALL=ParagraphStyle('small',parent=STYLE,fontSize=9,leading=14)
NOTES={
'A':'市役所南側の曲線と広場への出入りは写真上の通路配置に概ね対応する。FIXだけでも差の中央値0.69 m、最大0.97 mが残る。全体の0.31 mというRMSEが全区間で均一に得られるわけではなく、局所的なSLAM形状差・整列残差・GNSS誤差を分離するには独立基準が必要。',
'B':'最も大きい不一致。10:50:33のDGPS位置は、同時刻のSLAMに対して北へ13.24 m、東へ1.88 m離れる。写真では市役所の建物沿いから連絡道路へ出る部分にあたる。初回通過の最大差13.38 mに対し、後の通過は2.77 m。固定的な地理位置合わせのずれだけでは説明しにくい時間依存の不一致がある。FLOATでも最大11.84 m。',
'C':'連絡道路の両通過とも、写真上の道路北側の通路帯に沿う。GNSS対SLAM差の中央値は初回0.35 m、2回目0.22 m、全FIXの95%点は0.23 m。今回の拡大箇所では整合の良い区間。ただし往復で走行位置が異なる可能性があるため、2本の軌跡を重ねるだけでループ閉合の正しさを保証しない。',
'D':'北東角の曲がり方は、写真上の道路の角と概ね対応する。FIX割合71.9%には長時間滞在が含まれる。11:03:49-11:09:53は98.4%がFIXである一方、11:02:03-11:02:40の通過はFIX 0%。この場所を常時良好な測位区間とは判定できない。',
'E':'ホテル周辺の南側の角と直線部分で、GNSSがSLAMに対して曲がり込み・蛇行する形が見える。DGPSが96.4%、同時刻差は最大4.75 m、95%点4.19 m。赤線の輪郭は写真上の通路形状と概ね整合するが、写真だけで両軌跡のどちらが真値かは確定できない。',
'F':'東側の通路からホテル周辺へ入る接続部。2回の通過とも全観測がDGPSで、差の中央値は初回1.10 m、2回目0.82 m。FIXがない区間でも形状はおおむね追従しているが、この区間にはFIXによる局所確認がない。',
}
def main():
 results=json.loads((OUT/'regional_evaluation.json').read_text());dest=OUT/'output/pdf';dest.mkdir(parents=True,exist_ok=True)
 path=dest/'tsukuba2026_aerial_evaluation.pdf';c=canvas.Canvas(str(path),pagesize=A4);c.setTitle('つくば2026 航空写真と軌跡の拡大評価');page=0
 def begin(title,sub):
  nonlocal page
  page+=1;c.setFillColor(colors.HexColor('#f5f7fb'));c.rect(0,0,W,H,fill=1,stroke=0);c.setFillColor(colors.HexColor('#192b43'));c.setFont(FONT,19);c.drawString(M,H-42,title);c.setFont(FONT,10);c.drawString(M,H-64,sub);c.setFont(FONT,8);c.drawString(M,23,'2026-10-04収録 / RTK拘束なしグラフSLAM / 全良好FIXによる剛体整列');c.drawRightString(W-M,23,str(page))
 def para(txt,y,small=False):
  p=Paragraph(txt,SMALL if small else STYLE);_,h=p.wrap(W-2*M,1000);assert y-h>40,(page,y,h);p.drawOn(c,M,y-h);return y-h-12
 def pic(name,y,width=W-2*M):
  p=OUT/'figures'/name
  with Image.open(p) as im:h=width*im.height/im.width
  c.drawImage(str(p),(W-width)/2,y-h,width,h);return y-h-14
 begin('航空写真上の軌跡と6区間の評価','赤：グラフSLAM（下層） / 緑：FIX / 黄：FLOAT / 青：DGPS')
 y=pic('aerial_overview.png',H-80)
 y=para('元の数値軌跡を緯度・経度へ変換し、国土地理院の航空写真に重ねた。SLAMのグラフ最適化にはRTK拘束を使わず、完成後の位置・方位合わせに全良好FIXを用いている。写真に合わせた手動補正やスケール変更は行っていない。',y)
 y=para('最大の不一致は市役所東側Bの13.38 m、次いでホテル南側Eの4.75 m。連絡道路Cは中央値0.26 m。これらは同時刻のGNSSとSLAMの水平差であり、航空写真から測った絶対誤差ではない。',y)
 y=para('使用した写真の撮影時期は2017年2月。2026年の現況との変化を含みうる。地上画素は約0.48 mで、写真の位置精度も独立検証していないため、0.31 mの絶対精度をこの写真で検証したとはいえない。',y)
 c.showPage()
 begin('区間別の数値比較','選択範囲は全体図のA-F / 停止を含む時刻対応観測')
 rows=[['区間','FIX割合','中央値 [m]','95%点 [m]','最大 [m]']]
 for r in results:rows.append([r['id'],f'{r["by_state"]["RTK FIX"]["fraction_pct"]:.1f}%',*[f'{r["all"][k]:.2f}' for k in ['median_m','p95_m','max_m']]])
 data=[[Paragraph(x,SMALL) for x in row] for row in rows];table=Table(data,colWidths=[60,95,110,110,110]);table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dfe8f4')),('GRID',(0,0),(-1,-1),.4,colors.HexColor('#bbc7d6')),('TOPPADDING',(0,0),(-1,-1),10),('BOTTOMPADDING',(0,0),(-1,-1),10)]));_,h=table.wrap(W-2*M,700);table.drawOn(c,M,H-88-h);y=H-105-h
 for r in results:y=para(f'{r["id"]}：{r["name"]}（{r["n"]:,}観測）',y,True)
 y=para('FIX割合は観測数の割合であり、移動距離の割合ではない。各矩形は独立した抽出範囲で、一部重複する。比較対象のSLAMアンテナ位置が矩形内にある時刻を選び、GNSS位置が外へずれても集計から除かない。',y)
 y=para('全良好FIXを整列と評価に共用しているため、独立した精度評価ではない。FIXが乏しいB・E・Fの不一致を、そのままGNSS単独の誤差と断定しない。建物との近さだけからマルチパスやFIX低下の原因も断定できない。',y)
 c.showPage()
 for r in results:
  begin(f'{r["id"]}：{r["name"]}','黄色の破線：区間内で最大となった同時刻の水平差')
  y=pic(f'detail_{r["id"]}.png',H-78,width=510)
  y=para(NOTES[r['id']],y)
  c.showPage()
 begin('通過ごとの比較と確認範囲','同じ場所でも測位状態・差の大きさは時間で変わる')
 y=pic('passes_B.png',H-78,width=465)
 y=pic('passes_C.png',y,width=465)
 y=para('市役所東側Bでは初回と後の通過で不一致の大きさが異なる。連絡道路Cでは両通過とも小さい差を維持している。通過ごとに描画を分け、別時刻のFIXがDGPSを覆い隠すことを避けている。',y,True)
 y=para('座標変換は元GNSSの緯度経度へ逆変換して照合し、最大差2.85×10^-14度以下を確認した。画像の画素に合わせた再推定はしていない。GeoJSON、集計CSV/JSON、使用タイルのURL・ハッシュ、再現スクリプトを原本とともに保存。',y,True)
 y=para('出典：国土地理院 地理院タイル（写真）。<br/><link href="https://maps.gsi.go.jp/development/ichiran.html">https://maps.gsi.go.jp/development/ichiran.html</link><br/>撮影時期はseamlessphoto_specのコース全観測を含む範囲を確認（2017年2月）。',y,True)
 c.showPage();c.save()
 rep=OUT/'reproduction';rep.mkdir(exist_ok=True)
 for name in ['aerial_base.py','aerial_evaluate.py','aerial_report.py','common.py']:shutil.copy2(Path(__file__).parent/name,rep/name)
 files=[p for p in OUT.rglob('*') if p.is_file() and 'tmp' not in p.parts and p.name!='deliverables_sha256.json']
 (OUT/'deliverables_sha256.json').write_text(json.dumps({str(p.relative_to(OUT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
 print(path)
if __name__=='__main__':main()
