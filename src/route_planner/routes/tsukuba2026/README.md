# つくば2026 本走行コース

2026年のつくばチャレンジ本走行コースである。選択課題Aに対応しないルートと、対応する
ルートの2つを、同じ経路CSVから定義する。

- 正本: `fixed/part1.csv`（739点）と `fixed/part2.csv`（2,965点）。LLHのみを保存し、
  高度は空欄とする。列は [route_planner README](../../README.md) のCSV仕様に従う。
  2ファイルの連結が1本の経路であり、同じ行が重複して現れることはない。
- 分割点: `WP0738`。選択課題Aのゲートへ入る位置であり、停止点ではない。課題Aに対応する
  ルートは、同じ2ファイルの間に可変ブロックを挟む別の `route_config` として定義する。
  経路CSVを複製せずに2つのルートを持てるようにするための分割である。
- 経路構成: `route_config.yaml`。2つの固定ブロックをこの順で連結する。可変ブロックを
  持たないため、滞留時の再探索はできない。
- 規模: 3,704点、経路長 約2,241 m。waypoint間隔は最大 0.75 m。
- 停止属性: `line_is_stop` が16点、`signal_is_stop` は全点0である。信号交差点の
  道路端2点（E07、E11）も `line_is_stop` だけを持ち、オペレータが再開する。
- 回避幅: `right_is_open` / `left_is_open` に区間ごとの値が入る。3,464点が
  0より大きい幅を持ち、240点は両側0（横移動を許さない）である。
- 投影: 起動カードは `geo_pose_converter` の共通設定 `params/default.yaml` を指す。
  `icart_bringup` のsessionから起動する場合は、session側の `projection_params` が
  位置推定と経路の双方へ同じ投影条件を与える。

## 由来

`route_survey` の成果物 `src/route_survey/results/tsukuba2026_waypoints.csv` を
そのまま配置したものである。成果物の作り方は
[waypointの回避幅を求める手順](../../../route_survey/docs/waypointの回避幅を求める手順.md)
にある。経路の幾何と停止属性は `src/route_survey/tools/build_route.py` が、回避幅は
`src/route_survey/tools/apply_region_widths.py` が決める。

**停止属性や幾何を変える場合は、この経路CSVを直接編集せず、生成側を直して
`route_survey` の成果物を作り直し、再度配置する。**

## 登録をやり直す

成果物を更新したときは次を実行する。`cmp` と pytest が、配置した経路と
`route_survey` の成果物が同一であることを検査する。

成果物を `WP0738` の行までと、その次の行からの2つへ分けて配置する。ヘッダは双方に置く。

```bash
python3 src/route_survey/tools/split_route_csv.py \
  --source src/route_survey/results/tsukuba2026_waypoints.csv \
  --split-label WP0738 \
  --output src/route_planner/routes/tsukuba2026/fixed
python3 src/route_survey/tools/build_task_a_route.py \
  --spec src/route_survey/params/tsukuba2026_task_a_graph.yaml \
  --output src/route_planner/routes/tsukuba2026
python3 -m pytest src/route_planner/tests/test_tsukuba2026_route.py
```

## 選択課題A対応のルート

`route_config_task_a.yaml` は、同じ `fixed/part1.csv` と `fixed/part2.csv` の間に、
市役所職員駐車場3の通路網を可変ブロックとして挟む。経路CSVは複製しない。

- 通路網: `variable/nodes.csv`（13ノード）、`variable/edges.csv`（16エッジ、総延長420 m）、
  `variable/segments/edge_<始点>_<終点>.csv`。いずれも
  `src/route_survey/tools/build_task_a_route.py` が
  [通路網の定義](../../../route_survey/params/tsukuba2026_task_a_graph.yaml)から生成する。
  手で編集せず、定義を直して作り直す。
- 進入と退出: 可変ブロックの `start` と `goal` はどちらもノード `G` で、分割点 `WP0738`
  と同じ位置にある。ロボットは歩道上のここで左折し、歩道と駐車場の境界から進入して、
  チェックポイント `O11` を通り、同じ位置へ戻る。
- 回避幅: 課題A区間は左右とも1.5 m。車体幅0.6 mに対し、車体端で1 m強の横移動を
  許す値である。通路が狭い場合は `lidar_layer_slicer` の実測空き幅が上限を縛るため、
  停車車両からの離隔はそちらの設定で決まる。停止点は持たない。進入前の安全確認は、分割点の
  3.3 m手前にある道路端停止 `E04` で行う。
- 封鎖時: 通路が塞がれて滞留した場合、`route_follower` の `/report_stuck` から
  `route_manager` が `/update_route` を呼び、`route_planner` が封鎖エッジを除いて
  再探索する。可変ブロックであるため、看板の検出に依存しない。

経路は3,997点になる。課題A区間は293点・215 mである。

**可変ブロックの探索には `networkx` が要る。** `package.xml` が `python3-networkx` を
実行依存に挙げている。apt で入れていない環境では、`networkx` を持つ venv の
site-packages を `PYTHONPATH` へ通してから起動する。見つからない場合、`/get_route` は
`networkx is required for variable route solving` で失敗し、ルートを取得できない。
課題A非対応のルートは固定ブロックだけなので影響を受けない。

## 起動する

起動カードは `route_planner` と `route_manager` の双方に同名で置く。`robot_console`
からはこのカード名で選ぶ。`route_manager` の `start_label` と `goal_label` は空であり、
全域を走る。

- 課題A非対応: `src/route_planner/params/tsukuba2026.yaml`、
  `src/route_manager/params/tsukuba2026.yaml`
- 課題A対応: `src/route_planner/params/tsukuba2026_task_a.yaml`、
  `src/route_manager/params/tsukuba2026_task_a.yaml`。`route_manager` 側は
  `checkpoint_labels` に `O11` を持つ

```bash
colcon build --packages-select route_planner route_manager
source install/setup.bash
ros2 launch route_planner route_planner.launch.py \
  param_file:=$(ros2 pkg prefix route_planner)/share/route_planner/params/tsukuba2026.yaml
ros2 launch route_manager route_manager.launch.py \
  param_file:=$(ros2 pkg prefix route_manager)/share/route_manager/params/tsukuba2026.yaml
```

CSVはビルド時に共有ディレクトリへコピーされる。通常installでCSVを更新した場合は
再ビルドし、ノードを再起動する。

## 確認した範囲

- `route_config.yaml` から3,704点が組み立てられ、全点が固定区間、停止16点、
  点間隔0.75 m以下であることを `tests/test_tsukuba2026_route.py` で確認した。
- 2ファイルの連結が `route_survey` の成果物と一致することを確認した。
- 課題A対応のルートが、同じ2ファイルの間へ可変ブロックを挟んで組み立てられること、
  課題A区間の回避幅が左右1.5 mで停止点を持たないこと、点間隔が保たれることを確認した。
- 通路の封鎖を模して、`O10`-`O11` を除くと `O9` 経由へ、さらに `O9`-`O11` も除くと
  `O12`-`O13` 経由へ再探索できることを確認した。
- 分割前の1ブロック構成と比べ、ラベル・位置・停止属性・回避幅は変わらない。分割点
  `WP0738` の姿勢だけ、ブロック末尾の再計算により0.18度変わる。

**実機では走行していない。** 回避幅の値、停止位置、信号交差点の扱いは実機走行で
確認していない。模擬環境で確認した範囲は
[回避幅の算出結果](../../../route_survey/results/README.md) にある。
