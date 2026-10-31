# 本走行ルートの登録と選択課題Aの通路網を作る手順

## この文書の範囲

回避幅を書き込んだ経路CSVを `route_planner` の本走行ルートとして登録し、選択課題A
（事前計測なしエリア通過）に対応するルートを可変ブロックとして加えるまでの手順を示す。
経路の座標・方位・停止属性は変更しない。実機は動かさない。

[waypointの回避幅を求める手順](waypointの回避幅を求める手順.md)の続きにあたる。

課題A対応と非対応の2つのルートは、**同じ waypoint の行を複製せずに**定義する。複製
すると停止属性や回避幅の更新が片方にだけ入る。

## 成果物

| 成果物 | 置き場所 | 内容 |
|---|---|---|
| 分割した経路CSV | `src/route_planner/routes/tsukuba2026/fixed/part1.csv`、`part2.csv` | 連結すると `route_survey` の成果物と一致する |
| 課題A非対応の構成 | 同ディレクトリの `route_config.yaml` | 2つの固定ブロック |
| 通路網の定義 | `src/route_survey/params/tsukuba2026_task_a_graph.yaml` | ノードの緯度経度・エッジ・回避幅・通過点。**課題A側の正本** |
| 課題A対応の構成 | `routes/tsukuba2026/route_config_task_a.yaml` と `variable/` 一式 | 定義から生成する |
| 起動カード | `route_planner` と `route_manager` の `params/tsukuba2026*.yaml` | 両方に同名で置く |

`variable/` 配下と `route_config_task_a.yaml` は生成物である。手で編集せず、定義を
直して作り直す。

## 入力

| 入力 | 必要なもの |
|---|---|
| 回避幅を書き込んだ経路 | `src/route_survey/results/tsukuba2026_waypoints.csv` |
| 課題Aのエリアのノード座標 | 現地で観測した緯度経度。公式図のゲートとチェックポイントに対応づけたもの |
| 公式のエリア図 | つくばチャレンジの課題ページに載る航空写真の図 |

## 手順

### 1. 分割点を決める

課題Aのゲートへ入る位置でコースを2つに分ける。進入路の道路側の端と駐車場側の端を
結ぶ線分を用意し、**往路の線分のうちこれと交わるもの**を探して、その手前のwaypointを
分割点とする。

分割点は次を満たすこと。満たさない場合は隣のwaypointを採る。

- **停止点でないこと。** `route_planner` はブロック末尾の姿勢を直前点からの方位で
  上書きするため、停止点を分割点にすると停止時の向きが変わる
- 直線部にあること。CSVの `heading_deg` と直前点からの方位の差が小さいこと

つくば2026コースでは `WP0738`（36.08336371645, 140.07864295169）である。道路端停止
`E04` の3.3 m先にあり、`heading_deg` との差は0.18度である。

### 2. 経路CSVを分割して登録する

```bash
python3 src/route_survey/tools/split_route_csv.py \
  --source src/route_survey/results/tsukuba2026_waypoints.csv \
  --split-label WP0738 \
  --output src/route_planner/routes/tsukuba2026/fixed
```

停止点・経路の端・重複するラベルを指定した場合はツールが拒否する。

`route_config.yaml` には2つの固定ブロックを順に書く。これが課題A非対応のルートである。

### 3. 課題Aの通路網を定義する

`src/route_survey/params/tsukuba2026_task_a_graph.yaml` にノードとエッジを書く。
ノード座標は**現地の観測値を正とする**。地図サービスから求めた値は、観測値が無い間の
下書きとして扱う。

ゲートのノードは、分割点と同じ位置に置く。可変ブロックの `start` と `goal` の双方に
このノードを指定すると、進入と退出が同じ位置になる経路を組み立てられる。

エッジは `source`・`target`・説明を書く。向きは探索時に解決されるため、どちら向きで
書いてもよい。

**地図サービスから下書きを作る場合**、OpenStreetMap の `service=parking_aisle` が
使える。ただし線形は航空写真からのトレースであり、観測値と突き合わせると系統的に
ずれる。つくば市役所職員駐車場3では、OSMの原位置が観測値と中央値2.4 m（最大3.4 m）
離れており、**全体を東2.5 m・北1.2 m並進すると中央値0.85 m（最大1.8 m）まで寄った**。
向きによらない一様な並進であり、エッジごとに航空写真の通路中心へ吸着させる方法は、
駐車車両の寄り方に引かれて並進を上回らなかった。別の区画で作る場合も**一様並進を
基本とし、航空写真は通路の有無と幅の確認に使う**。

航空写真は国土地理院のシームレス写真タイル（`seamlessphoto`、ズーム18、約0.48 m/画素）
を使う。Googleマップから座標を機械的に取り出すことはしない。利用規約がコンテンツの
抽出・保存・派生データの作成を禁じている。目視確認に使うのは公式に認められている。

### 4. 回避幅を決める

定義YAMLの `right_is_open` / `left_is_open` に書く。駐車場の通路では左右1.5 mとした。
車体全幅0.6 mに対し、車体端で1 m強の横移動を許す値である。

**これは上限であり、実際の横移動量ではない。** 走行時の横移動は `obstacle_monitor` の
提案値を回避幅で打ち切った値になる。停車車両からの離隔は回避幅ではなく
`obstacle_monitor` の `robot_width_m` が決める。詳細は
[走行システムの機能と制約](../../../docs/走行システムの機能と制約.md)を参照する。

### 5. 可変ブロックを生成する

```bash
python3 src/route_survey/tools/build_task_a_route.py \
  --spec src/route_survey/params/tsukuba2026_task_a_graph.yaml \
  --output src/route_planner/routes/tsukuba2026
```

`variable/nodes.csv`、`variable/edges.csv`、`variable/segments/edge_<始点>_<終点>.csv`、
`route_config_task_a.yaml` ができる。エッジは直線とし、0.75 m以下へ等分して補間する。
進行方位は始点から終点への方位とする。固定ブロックは課題A非対応のルートと同じ
2ファイルを指す。

### 6. 起動カードを置く

`route_planner` と `route_manager` の `params/` に同名で置く。`robot_console` は両方の
`params/` を走査するため、片方だけでは選べない。課題A対応のカードでは
`route_manager` の `checkpoint_labels` にチェックポイントのノードIDを入れる。

### 7. ビルドして配置を確認する

```bash
colcon build --packages-select route_planner route_manager route_survey
```

経路CSVの構成を変えた場合は、`build/route_planner` と `install/route_planner` を消して
から再ビルドする。消さないと古いCSVが共有ディレクトリに残る。

可変ブロックの探索には `networkx` が要る。apt の `python3-networkx` を入れていない環境
では、`networkx` を持つ venv の site-packages を `PYTHONPATH` へ通してから起動する。
`ros2 launch` が起動するノードはシステムのPythonで動くため、venvを有効化しただけでは
足りない。

## 検証

`src/route_planner/tests/test_tsukuba2026_route.py` が次を検査する。

- 分割した2ファイルの連結が `route_survey` の成果物と一致すること
- 分割点が停止点でないこと
- 課題A非対応のルートが3,704点で、全点が固定区間、停止16点、点間隔0.75 m以下であること
- 課題A対応のルートが、同じ2ファイルの間へ可変ブロックを挟んで組み立つこと。課題A区間の
  回避幅が左右1.5 mで停止点を持たないこと。点間隔が保たれること

手で確かめることとして、封鎖時の再探索がある。`graph_solver.solve_variable_route()` に
エッジを1本ずつ減らして渡し、チェックポイントへ回り込めることを確認する。つくば2026の
通路網では、チェックポイントへ至る南北通路を1本外すと別の通路へ、2本外すとさらに別の
通路へ経路が移る。

## 人の判断が必要な箇所

- 分割点にするwaypoint。往路の交点から機械的に求まるが、停止点や曲線部に当たった
  場合は人が隣の点を選ぶ
- 公式図のゲートとチェックポイントを、どのノードに対応づけるか
- 通路の実在。航空写真では駐車車両に隠れて見えない通路がある。現地の目視で補う

## この手順で分からないこと

- 通路の当日の通行可否。駐車車両やロードコーンの配置は走行時にしか分からない
- 回避幅の妥当性。実機で走らせるまで確認できない
- 航空写真と観測値の対応が取れない区画での位置精度。観測値が無い場合、地図サービス
  由来の座標は数メートルの誤差を持つ
