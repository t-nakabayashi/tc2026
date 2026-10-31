# waypointの回避幅を求める手順

## この文書の範囲

既存の経路（waypointリスト）に対し、各waypointの `right_is_open` / `left_is_open`
を観測から求め、経路CSVへ反映するまでの手順を示す。経路の座標・方位・停止属性は
変更しない。ROSノードの起動、速度指令、走行中の経路更新は行わない。

出力は提案値であり、測量値でも衝突余裕の保証値でもない。走行へ使う前に利用者が
確認する。

## 成果物

この手順が作るものは次の4つである。前の3つが走行と見直しに使うもの、最後が
現地の様子を伝えるものである。

| 成果物 | 置き場所 | 内容 |
|---|---|---|
| 区間定義 | `src/route_survey/params/tsukuba2026_regions.csv` | 区間の範囲・名前・境界の種類・上書き値・説明。**この手順の正本** |
| 回避幅を書き込んだ経路CSV | `src/route_survey/results/tsukuba2026_waypoints.csv` | `right_is_open` / `left_is_open` だけを更新した経路 |
| 全域の回避幅の図 | `src/route_survey/results/tsukuba2026_route_widths.png` | 全点の幅をENU俯瞰図と距離方向の帯で1枚にしたPNG |
| 区間の説明 | リポジトリ外の作業ディレクトリ | 区間ごとの写真・説明・幅をまとめたHTMLと、配布用のPDF |

**区間の説明HTMLは現地の写真を含むため、リポジトリで管理しない。** 作業ディレクトリ
へ出力し、必要な相手へ個別に渡す。ほかの3つは文字と図だけなのでリポジトリに置く。
経路CSVと図は手順の出力であり、本番として使うと決めた経路ではない。実機での確認を
経てから `src/route_planner/routes/<コース名>/` へ移す。

## 入力

手動走行のログと、そのログから作った地図、そのログから作った経路の3つを使う。
**3つが同じ走行ログに由来していること**を確認する。別のログで作った経路へ、この
手順で求めた幅を当てはめてはならない。

| 入力 | 必要なもの |
|---|---|
| 走行ログ | ROS bag。`/scan`（LaserScan）と `/usb_cam/image_raw`（Image）を含むこと |
| 記録区間の対応 | `run<N>/summary.json`。各記録区間がどのbagかを示す |
| 地図 | `optimized_poses_enu.npy`、`keyframes.json`、`hardware.yaml` |
| 路面領域 | `surface_mask.npz`（20 cm格子の路面判定） |
| 経路 | `fixed/waypoints.csv` と `waypoints_enu.json`（弧長・時刻・記録区間を持つ）。`tools/build_route.py` の出力 |

経路と地図のENU原点が一致していることを確認する。一致していなければ以降の測定は
意味を持たない。

## 前提の確認

経路の座標と、地図作成に使った姿勢が一致していることを確かめる。waypointの時刻で
地図側の軌跡を内挿して比較する。**最近傍のwaypointと比較してはならない。** waypointの
時間間隔ぶん（0.6秒程度）の差が位置の差として現れ、実際より大きなずれに見える。

一致の目安は中央値で数cmである。0.3 mを超える箇所があれば、幅の値にその誤差が乗る。

## 手順

### 1. 断面を測る

```bash
python3 src/route_survey/tools/measure_cross_sections.py \
  --waypoints <経路>/waypoints_enu.json --surface-mask <路面場>/surface_mask.npz \
  --output <出力>/cross_sections.csv --spacing 1.0
```

断面の原点は弧長で引いたwaypoint、向きは前後3 mから求めた接線の法線とする。
撮影位置から進行方位へ直線外挿すると曲線部で経路を外れる。曲率を併記するので、
左右の意味が崩れる右左折部を後段で除外できる。

### 2. URGから鉛直境界を取り出す

```bash
source /opt/ros/jazzy/setup.bash
python3 src/route_survey/tools/extract_urg_boundary.py \
  --poses <地図>/optimized_poses_enu.npy --keyframes <グラフ>/keyframes.json \
  --hardware <地図>/hardware.yaml --surface-mask <路面場>/surface_mask.npz \
  --run-cache <記録区間キャッシュ> --bag-root <bag置き場> \
  --waypoints <経路>/waypoints_enu.json --output <出力>/urg_boundary.npz \
  --scan-stride 2 --max-gap 6.0 --pose-stride 0.25 --max-range 8.0 --min-range 0.6
```

URGは床から一定の高さを水平に走査するため路面を観測せず、生垣・壁・柱・車止め・
ロードコーンのような鉛直物だけが写る。路面点群と違い、路面の微小な凹凸や
キーフレーム間の鉛直ずれの影響を受けない。

次の3つを除外しないと、軌跡上に偽の境界が出る。既定値で除外される。

- 車体前方に車体自身の構造物の反射が常時ある（`--min-range`）
- 手動走行は停止が多く、時間で間引くと停止地点の観測だけが積み上がる（`--pose-stride`）
- 遠距離のビームは車体のピッチで路面に当たる（`--max-range`）

走査の配置には地図作成と同じ最適化済みキーフレーム姿勢を使う。

手順1の出力へ鉛直境界の列を加える。

```bash
python3 src/route_survey/tools/measure_cross_sections.py \
  --waypoints <経路>/waypoints_enu.json --surface-mask <路面場>/surface_mask.npz \
  --urg-boundary <出力>/urg_boundary.npz \
  --output <出力>/cross_sections.csv --spacing 1.0
```

### 3. 確認用の映像を取り出す

```bash
python3 src/route_survey/tools/extract_route_frames.py \
  --waypoints <経路>/waypoints_enu.json \
  --run-cache <記録区間キャッシュ> --bag-root <bag置き場> \
  --output <出力>/frames --spacing 20 --lead 8
```

カメラは前方を向いており車体の真横を写さない。距離 s の断面を判断する映像には、
経路の弧長で8 m手前のコマを使う。20 m間隔で取り出すと各コマが断面の約1 m手前から
12 m先までを写し、経路全体が切れ目なく覆われる。

### 4. 区間定義を用意する

区間はコースの構造そのものなので、**つくばチャレンジのコースが変わらない限り不変**で
ある。通常の実施では `src/route_survey/params/tsukuba2026_regions.csv` をそのまま使い、
手順4から6を飛ばして手順7へ進む。

ただし**ログが変わると区間の開始・終了の距離値はわずかに動く**。経路の弧長は走行軌跡
から積算するため、同じ場所でも数m前後する。手順3で取り出したコマを境界付近で確認し、
区間の切れ目が実際の切り替わり位置と合うよう `start_m` と `end_m` だけを直す。
名前・境界の種類・上書き値は変えない。

コースが変わった年度は、次の手順4から6で区間を作り直す。

### 4-1. 環境が切り替わる位置の候補を挙げる

```bash
python3 src/route_survey/tools/detect_environment_changes.py \
  --cross-sections <出力>/cross_sections.csv --output <出力>/environment_changes.csv \
  --span 25 --min-separation 40 --distance-change 0.8 --rate-change 0.45
```

走行環境は数十mから百m超にわたり同じ性質が続く。窓と最小間隔を狭めると、出入口や
個々のコーンを拾って候補が増えすぎる。上記の値で2 km級のコースに対し40件程度になる。

### 5. 【人の判断・コース変更時のみ】候補を映像で確認し、区間を確定する

候補ごとに前後15 mのコマを並べ、環境が実際に変わっているかを見る。

判断を助ける資料として、映像と俯瞰図を同じ地点・同じ向きで並べた確認シートを作れる。

```bash
python3 src/route_survey/tools/render_width_panels.py \
  --frames <出力>/frames --surface-mask <路面場>/surface_mask.npz \
  --cross-sections <出力>/cross_sections.csv \
  --waypoints <経路>/waypoints_enu.json --output <出力>/panels

python3 src/route_survey/tools/build_width_review.py \
  --frames <出力>/frames --panels <出力>/panels \
  --review-widths <幅推定>/review_widths.csv --output <出力>/width_review.html
```

俯瞰図は進行方向を上、車体から見た左を図の左に置き、映像と左右を一致させる。
緑が路面候補、橙が段差・上方物体の候補、灰が観測はあるが路面候補でない格子、
白が未観測である。青の線が経路、赤の線が判断対象の断面を示す。格子は20 cmで、
色のない場所を走行不可と断定したものではない。

HTMLはブラウザで開き、区間ごとに種別と左右の幅を記入してCSVとJSONで取り出す。
`j`・`k`で移動、`1`〜`4`で種別、`c`で前区間の値を複製する。入力はブラウザに
自動保存し、JSONを読み込めば途中から再開できる。

この記入結果が、検証で突き合わせる「映像の判断」になる。

人や他の機体に遮られて判断できないコマが出る。手前距離を変えて取り直す。

```bash
python3 src/route_survey/tools/extract_route_frames.py \
  --waypoints <経路>/waypoints_enu.json \
  --run-cache <記録区間キャッシュ> --bag-root <bag置き場> \
  --output <出力>/frames_retry --distances <距離> --lead 4 --lead 12 --lead 16
```

`--lead` は複数指定でき、同じ断面を異なる位置から撮ったコマが並ぶ。随伴者は車体と
一定の間隔を保って歩くため、手前距離を変えると写り込む位置が変わる。手前を短くすると
断面が画面下端へ寄り、長くすると断面が小さくなる。4 mから16 mの範囲で選ぶ。

障害物として置かれたロードコーンや車止めに反応した候補は、環境が変わっていない
ので削除する。確認の結果、候補のうち3分の1程度は削除される。

確定した区間を `src/route_survey/params/tsukuba2026_regions.csv` へ書く。これが
**この手順の正本**であり、以降の出力はここから生成する。経路CSVを直接編集しない。

```csv
start_m,end_m,name,left_type,right_type,near_limit_m,left_override_m,right_override_m,description
```

`description` は区間の説明HTMLに載せる1、2文である。映像から確認した地物を書く。

### 6. 【人の判断・コース変更時のみ】区間ごとに境界の種類を決める

左右それぞれについて、映像を見て種類を与える。

| 種類 | 対象 | 測り方 |
|---|---|---|
| `continuous` | 生垣・壁・フェンス | 近い側の検出に1次式を当てはめる |
| `cone` | 路端を示すロードコーンの列 | 同じ当てはめに追加の余裕を引く |
| `discrete` | 柱・車止め・出入口で途切れる境界 | 区間内の下側5%点を使う |
| `none` | 探索範囲に境界が無い | 上限値を使う |
| `open_wide` | 障害物が多い広い区間 | 遠い側の検出から求め、上限で頭打ちにする |
| `crossing` | 横断歩道 | 0とする |

**ロードコーンは路端を示す場合と障害物として置かれる場合があり、見た目で区別できない。**
並び方で判断する。経路に沿って列をなすものは路端、単独または群で置かれたものは障害物
である。つくばチャレンジでは、市役所一帯の敷地内に路端のコーンは無い。

障害物として置かれた区間で幅を絞ると、回避できずに停止する。幅を狭める方向へ扱わない。

記録したコーンと随伴者は記録日の配置であり、走行当日の配置ではない。

### 7. 区間ごとの横移動量を求める

```bash
python3 src/route_survey/tools/estimate_region_widths.py \
  --cross-sections <出力>/cross_sections.csv --regions <出力>/regions.csv \
  --output <出力>/region_widths.csv
```

ロードコーンが路端を成す区間では、コーンの隙間から遠くの物が見えるため検出距離の
分布が二峰になる。近い峰だけを残さないと代表値が隙間の値に引かれる。近い側として
残す上限は既定1.8 mで、広場にコーンを疎に置いて通路を示す区間では `near_limit_m` で
広げる。

障害物が多い区間では逆に、近い検出が置かれた障害物自身になる。`open_wide` では
一定距離より遠い検出だけを使う。

### 8. 経路CSVへ反映する

```bash
python3 src/route_survey/tools/apply_region_widths.py \
  --waypoints-csv <経路>/fixed/waypoints.csv \
  --waypoints-enu <経路>/waypoints_enu.json \
  --region-widths <出力>/region_widths.csv \
  --output <出力>/route_with_widths/fixed/waypoints.csv --blend 3.0
```

回避幅2列だけを書き換える。入力のCSVは上書きしない。区間境界では前後に遷移をかけ、
遷移中の値が両端の大きい方を超えないようにする。

### 9. 全域の回避幅を図にする

区間ごとの値を個別に見ても全体の傾向は掴めない。全点と区間と左右幅を一枚に描く。

```bash
python3 src/route_survey/tools/render_route_widths.py \
  --waypoints-enu <経路>/waypoints_enu.json \
  --route src/route_survey/results/tsukuba2026_waypoints.csv \
  --regions src/route_survey/params/tsukuba2026_regions.csv \
  --output src/route_survey/results/tsukuba2026_route_widths.png
```

俯瞰図は回避できる範囲を実寸で塗るため、拡大すれば区間ごとの広がりをそのまま読める。
距離方向の帯では、上が左、下が右である。区間の切れ目に振った番号が下の一覧と対応する。
色と縦軸の上限は既定で入力の最大値に合わせる。`--cap` で固定できる。

意図しない値はこの図で見つける。幅0の区間が連続しすぎていないか、広い扱いの区間が
想定より長くないかを見る。

### 10. 区間の説明をまとめる

区間の名前だけでは現地の様子が分からず、後から幅を見直すときの判断材料が残らない。
区間ごとに写真・説明・幅・観測メモを1枚のHTMLにまとめる。

```bash
python3 src/route_survey/tools/build_region_guide.py \
  --regions src/route_survey/params/tsukuba2026_regions.csv \
  --frames <出力>/frames \
  --region-widths <出力>/region_widths.csv \
  --notes <出力>/width_review_claude.csv \
  --output <出力>/region_guide.html
```

写真は区間の中央に最も近いコマを選ぶ。横断歩道の手前のように区間が短くコマが入らない
場合は、最寄りのコマを区間外と明示して使う。画像はHTMLへ埋め込むので1ファイルだけで
閲覧できる。

HTMLは閲覧環境を選ぶため、配布にはPDFを使う。HTMLには印刷用の体裁を入れてあり、
ヘッドレスのブラウザでそのまま変換できる。区間が途中で改ページされることはない。

```bash
google-chrome --headless=new --disable-gpu --no-pdf-header-footer \
  --print-to-pdf=<出力の絶対パス>/region_guide.pdf \
  file://<出力の絶対パス>/region_guide.html
```

`--print-to-pdf` と `file://` は絶対パスで渡す。A4縦、33区間で28ページになる。

**HTMLもPDFもリポジトリの外へ置く。** 現地の写真を含むため、リポジトリで管理しない。

## 検証

最初の2つは必ず行う。どちらも通らなければ反映しない。残る2つは、反映した値が全域で
どうなったかと、実際の走行で何が起きるかを見るために行う。

### 幅2列以外が変わっていないこと

入力と出力のCSVを列ごとに比較し、`right_is_open` と `left_is_open` 以外の差分が
0であることを確認する。

### 映像の判断と突き合わせること

**観測だけでは危険側の誤りを検出できない。** 20 m間隔の映像から人が判断した値と、
生成した値を同じ地点で比較する。

生成値が映像の判断より大きい地点は危険側の誤りである。原因は主に次の2つで、
いずれも区間表を直して再生成する。

- 区間の端が実態とずれている。広い扱いの区間が実際より長く取られている
- 横断歩道が独立した区間になっていない

生成値が映像の判断より小さい地点は安全側だが、多い場合は区間の種類を見直す。
URGは最も近い鉛直物で走査を打ち切るため、通り抜けられるボラードや疎な植栽を
連続した境界として扱い、値を過小に見積もる。この場合は区間表の上書き列へ
利用者の判断値を与える。

### 模擬環境で走らせて確かめること

同梱のデジタルツイン（`obstacle_route_sim/maps/tsukuba2026`）へ経路を入れて走らせ、
記録から横ずれと回避の判断を集計する。

```bash
ros2 run icart_bringup run_digital_twin --output <session> --prepare-only
cp <out>/route_with_widths/fixed/waypoints.csv <session>/routes/fixed/waypoints.csv
# <session>/session.yaml の start_label と goal_label を経路のラベルへ変更する
ros2 run icart_bringup run_session --session <session>/session.yaml --environment simulation
```

`route_follower` は `line_is_stop` の waypoint で `WAITING_STOP` に入り、`manual_start`
による解除を待つ。UIを使わない場合は `manual_start` を一定間隔で送り続ける。

```bash
ros2 topic pub -r 1 /manual_start std_msgs/msg/Bool "{data: true}"
```

記録した bag は次で集計する。経路は走行時と同じ `projection.yaml` でENUへ直すため、
地図作成時のENUとは原点が違っていてよい。

```bash
python3 src/route_survey/tools/summarize_avoidance_run.py \
  --bag <session>/run/bag \
  --route <out>/route_with_widths/fixed/waypoints.csv \
  --projection <session>/projection.yaml \
  --output <out>/sim_run_width_check.csv
```

同梱地図には生垣もロードコーンも無く、経路沿いの離隔が広い。そのまま走らせると回避が
一度も起きないため、経路上へ障害物を置く。幅0・最大幅・狭い幅の区間をそれぞれ選び、
障害物の25 m手前から走らせる。

```bash
python3 src/route_survey/tools/prepare_blocked_run.py --session <session> \
  --route <out>/route_with_widths/fixed/waypoints.csv \
  --start-distance 425 --blocker-distance 450 --goal-distance 480
```

**次の走行を始める前に、前の走行の記録とセッションを完全に止める。** 同じDDS domainを
使うため、記録が残っていると後続の走行まで同じファイルへ入り、1回の走行として読めなく
なる。集計ツールは記録の途切れで切り分けるが（`--max-gap`）、途切れが無い場合は
距離帯で絞る。

障害物を置いた走行では、`_try_shift` のログで命令された横移動量を見る。**経路からの
位置そのものを回避の大きさとして読まない。** 模擬環境の車体は幅0.3 m、障害物は0.6 m角
のため、`obstacle_monitor` の前方くさび（±10°、1 m以内）から外れた時点で前進が続き、
剛体に接触したまま横へ押し出される。実機では、剛体なら接触前に人が非常停止し、草なら
押しのけて直進するため、この押し出しは起きない。

**この確認で分かるのは、値がスタックへ正しく届き、走行が破綻しないことまでである。**
同梱地図は公式コース図を図上トレースした概略経路を基準に作られ、その経路に対してだけ
左右1 mの離隔を保証している。実走ログ由来の経路とは中央0.99 m、最大7.70 mずれるため、
地図上の離隔は実際の路端と対応しない。生垣・ロードコーン・縁石・植栽帯は地図に無い。
幅の値そのものの正しさは、この確認では判定できない。

## 人の判断が必要な箇所

この手順は全自動ではない。次の箇所で人が映像を見て判断する。

1. 手順4：既存の区間定義の切れ目が、今回のログの距離値と合っているか
2. 手順5：候補が実際に環境の切り替わりかどうか（コース変更時のみ）
3. 手順6：境界の種類。特にロードコーンが路端か障害物か（コース変更時のみ）
4. 手順10：区間の説明文
5. 検証：生成値と映像の判断の突き合わせ

## この手順で分からないこと

- URGは1つの高さの水平断面しか持たない。縁石・草地・点字ブロックのような低い境界は
  観測できない。張り出した葉と壁も区別できない。
- 境界物の種類（固いか柔らかいか）は判定しない。
- **幅が縛るのは命令された横移動だけである。** 幅は `route_manager` が経路を横へ
  動かす量（`_try_shift` の `shift_d` = min(幅, `offset_step_max_m`)）と、AI助言の
  横オフセットの上限を決める。追従と局所制御の結果として出る位置そのものは縛らない。
- **幅が0の区間では横移動が一度も試みられない。** 模擬環境で経路上へ障害物を置くと
  `_try_shift` の呼び出しが0回だった。実機では前方が塞がったまま停止が続く。
  この滞留を幅の値で解消することはできない。
- 路面領域の格子は、走行した軌跡の直下でも25%が路面と判定されない。このため路面場
  単独では左右端を決められず、URGを併用する。
