#!/usr/bin/env python3
"""Create a mobile-readable Japanese PDF report from verified numerical outputs."""
import json,sys
from pathlib import Path
sys.path.insert(0,'/home/nkb/colcon_ws_experiments/glim_runtime/python')
from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import Paragraph,Table,TableStyle
from reportlab.lib.styles import ParagraphStyle
from common import ROOT
pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
FONT='HeiseiKakuGo-W5';W,H=A4;M=40
style=ParagraphStyle('body',fontName=FONT,fontSize=11,leading=17,textColor=colors.HexColor('#273148'),wordWrap='CJK')
small=ParagraphStyle('small',parent=style,fontSize=9,leading=13)

def main():
 e=json.loads((ROOT/'evaluation.json').read_text());g=json.loads((ROOT/'optimization.json').read_text());m=json.loads((ROOT/'map_summary.json').read_text());coverage=json.loads((ROOT/'coverage.json').read_text());loops=json.loads((ROOT/'loop_geometry.json').read_text());out=ROOT/'output/pdf';out.mkdir(parents=True,exist_ok=True)
 c=canvas.Canvas(str(out/'tsukuba2026_graph_slam_evaluation.pdf'),pagesize=A4);c.setTitle('つくばチャレンジ2026 RTK拘束なしグラフSLAM・軌跡評価');page=0
 def begin(title,subtitle):
  nonlocal page;page+=1;c.setFillColor(colors.HexColor('#eef2f8'));c.rect(0,0,W,H,fill=1,stroke=0);c.setFillColor(colors.HexColor('#17253b'));c.setFont(FONT,18);c.drawString(M,H-49,title);c.setFont(FONT,10);c.drawString(M,H-70,subtitle);c.setFont(FONT,8);c.drawString(M,24,'2026-10-04 / recorded FAST-LIO + small_gicp + GTSAM / offline');c.drawRightString(W-M,24,str(page))
 def para(text,y,sm=False):
  p=Paragraph(text,small if sm else style);_,h=p.wrap(W-2*M,y-40);p.drawOn(c,M,y-h);return y-h-12
 def picture(name,y,width=W-2*M,height=None):
  from PIL import Image
  path=ROOT/'figures'/name;im=Image.open(path);ih=height or width*im.height/im.width;c.drawImage(str(path),M+(W-2*M-width)/2,y-ih,width,ih);return y-ih-12
 def table(rows,y,widths=None):
  data=[[Paragraph(str(x),small) for x in row] for row in rows];t=Table(data,colWidths=widths or [(W-2*M)/len(rows[0])]*len(rows[0]));t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dce5f3')),('GRID',(0,0),(-1,-1),.4,colors.HexColor('#b8c4d7')),('VALIGN',(0,0),(-1,-1),'TOP'),('BOTTOMPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),8)]));_,h=t.wrap(W-2*M,700);t.drawOn(c,M,y-h);return y-h-12
 begin('RTK拘束なしの全コース地図','つくばチャレンジ2026 / 10月4日の実測bag')
 y=picture('map_overview.png',H-90)
 y=para(f'走行約 {m["path_length_m"]/1000:.2f} km、キーフレーム {g["nodes"]:,}枚、3D点群 {m["points"]:,}点（15 cmボクセル）。LiDARの再訪拘束 {g["loop_edges"]}本と、記録中断の接続3本を用いて最適化した。GNSS位置・方位のグラフ因子は0件。',y)
 y=para('PC上の入力は /media/nkb/TEST 内の full_course_manual_logging_merged。約53.1分の3収録を含む統合bagを用い、内部リセットを含む4区間として処理した。追加の12:36走行は地図・軌跡評価に使用していない。',y)
 y=para(f'既存デジタルツイン参照線の {coverage["points"]:,}点すべてが地図軌跡から10 m以内（最大 {coverage["max_distance_m"]:.2f} m）。これは収録範囲の確認であり、公式コースの精密測量との照合ではない。',y)
 para('建物・樹木・路面など、LiDARで観測した周辺の点群地図。建物の裏側・遮蔽域など未観測範囲の完全性は保証しない。自律走行用の自由空間・障害物グリッドは生成していない。',y,True);c.showPage()
 begin('軌跡形状はループ閉合で改善','全良好FIXとの剛体整列 / スケール変更・非剛体補正なし')
 y=picture('shape_comparison.png',H-90)
 rows=[['指標 [m]','閉合前LIO','グラフSLAM']]
 for key,label in [('rmse_m','水平差 RMSE'),('median_m','中央値'),('p95_m','95%点'),('max_m','最大')]:rows.append([label,*[f'{e["results"][k]["all_fix_rigid_alignment_diagnostic"][key]:.3f}' for k in ['lio','slam']]])
 y=table(rows,y,[225,145,145])
 y=para('良好FIX 9,487観測を使い、各軌跡の水平回転と並進をロバストに推定した。方位・位置合わせ後の差を比較する一般的な軌跡形状の評価で、地図の最適化にRTKを入力したものではない。',y)
 y=para(f'停止区間の影響を抑えた1 mごとの評価でも、307点で閉合後RMSEは {e["results"]["slam"]["all_fix_per_meter"]["rmse_m"]:.3f} m。結果が長い停止時間だけで決まっていないことを確認した。',y)
 para('全FIXを座標合わせと比較に共用しているため、独立検証ではない。独立した測量真値がないので「絶対位置精度31 cm」とは判定しない。',y);c.showPage()
 begin('未使用FIXによる評価','最初の3分のFIXだけで座標を合わせ、その後を評価')
 y=picture('evaluation.png',H-90,width=425)
 y=para('座標合わせは876 FIX観測、評価は以後の良好FIX 8,611観測。水平差RMSEは閉合前9.99 m、閉合後6.44 m。1 mごとの307点では8.23 mから5.03 mとなった。',y)
 y=para('全FIXでの整列との方位差は約0.897度。長いコースでは小さな基準方位の差が数mの横ずれになる。局所の座標合わせを広域へ適用した誤差と、SLAMの形状・ドリフトを区別する必要がある。',y)
 para('測定UTCで時刻対応し、0.3秒超のLIO欠測は内挿しない。GNSS時刻を±0.1秒ずらした感度確認ではRMSE 6.439-6.446 mで、この大きな差を単純な0.1秒の時刻ずれでは説明できない。',y,True);c.showPage()
 begin('ループ閉合とGNSS品質','点群の一致に基づいて拘束を採用')
 y=picture('loop_geometry.png',H-90,width=405)
 y=para('再訪候補は、接続したLIO軌跡の近傍から選ぶ。RTKによる候補検索・位置初期値は使用しない。GICPの収束、両方向の重なり、最近傍距離、Hessian、逆方向照合の整合性を確認し、2回の候補探索で16本を採用した。',y)
 y=para('各ループの点群最近傍距離中央値をさらに中央値で集計すると、閉合前約'+f'{__import__("numpy").median([r["before"]["median_nn_m"] for r in loops]):.2f}'+' m、閉合後約'+f'{__import__("numpy").median([r["after"]["median_nn_m"] for r in loops]):.2f}'+' m。これは採用した点群の整合度であり、独立した位置精度ではない。',y)
 para('GNSS記録30,449件の内訳はFIX 10,001件（32.85%）、FLOAT 794件（2.61%）、DGPS 19,654件（64.55%）。良好FIX条件は状態4、衛星10以上、HDOP 2以下、補正齢2秒未満、品質時刻差0.05秒未満。FIX自体の正しさを保証する条件ではない。',y,True);c.showPage()
 begin('処理条件と確認範囲','再現・解釈のための技術情報')
 y=H-95
 for txt in [
  '前段：bagに記録された /lio/odometry_raw（FAST-LIO）を再利用する。生LiDAR全フレームから移動1.2 m、回転12度、または5秒のキーフレームを抽出。生点群の点ごとの時刻とLIO姿勢内挿で歪みを補正する。生IMUから各区間の重力方向を求める。FAST-LIOそのものの全bag再実行ではない。',
  '座標：LiDAR-IMUの公称並進 [-0.011, -0.02329, 0.04412] m、取付角 [-0.6, 26.9, 0] 度を使用。主アンテナの車体後方0.30 m・LiDAR上方0.15 mをレバーアーム補正する。記録時の外部標定・時計精度は独立校正していない。',
  'グラフ：2,179姿勢、2,175相対移動因子、3中断接続、16再訪因子、1初期姿勢事前分布。SE(3)をGTSAMのLevenberg-Marquardt法で最適化。再訪・接続にはHuber損失を使用。外れ値条件・重みは処理設定であり精度保証ではない。',
  '地図：PCDはSLAM固有座標のまま保存。ENUのPLYは、完成した地図全体を良好FIXで水平回転・並進した表示版。鉛直基準は受信機の記録高度を参照するが、楕円体高・標高の区別は未確認。GNSSの鉛直拘束は使用しない。',
  '品質別の軌跡差：全FIX剛体整列後の状態別RMSEはFIX 0.321 m、FLOAT 3.363 m、DGPS 1.961 m。観測場所・時間が状態ごとに異なり、この差をGNSS単体の誤差率とは解釈しない。',
  '確認済み：CDR直接読取りをROS公式デシリアライズの先頭・中央・末尾点と照合。時刻内挿、ENU変換・剛体整列、合成閉ループ最適化の数値試験3件が成功。点群・軌跡・PDFは画像で確認。',
  '未検証：別日・別走行での自己位置推定、動体除去、誤閉合の独立正解、実機の走行可能性、絶対位置精度。長距離の地理位置合わせには、今回のような冒頭の短い区間だけに依存しない基準設定が必要。']:
  y=para(txt,y,True)
 y=para('一次資料：GTSAM Pose3 / GPSFactor資料（borglab.github.io/gtsam）、small_gicp（github.com/koide3/small_gicp）、FAST-LIO2（arxiv.org/abs/2107.06829）。実際の数値は本bagの処理結果。',y,True)
 para('保存先：/media/nkb/TEST/graph_slam_20261004。PC内の保存先であり、iPhoneから直接開けるURLではない。会話中の画像と要約で閲覧し、ファイルの受渡しには指定された共有先が必要。公開・認証変更・外部ポート公開は行っていない。',y,True);c.showPage()
 begin('各部の点群と立体俯瞰','観測した環境の閲覧版')
 y=picture('map_details.png',H-90,width=430)
 y=picture('map_oblique.png',y,width=300)
 c.save();print(out/'tsukuba2026_graph_slam_evaluation.pdf')
if __name__=='__main__':main()
