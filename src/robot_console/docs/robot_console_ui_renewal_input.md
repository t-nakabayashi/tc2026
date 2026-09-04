# robot_console UI更改 設計・実装インプット文書

## 1. 本文書の位置付け

本文書は、`tc2026` リポジトリの `src/robot_console` に対して、次期UIの設計および実装を行うチームへ渡すためのインプット文書である。

インプット先のチームメンバは、これまでの検討経緯を把握していない前提とする。そのため、本文書では、UI更改の背景、運用前提、PyQt5ローカルUIとHTML遠隔観測UIの役割分担、画面構成、現行実装からの移植方針、設計上の注意点、段階的な実装方針をまとめる。

なお、実装チームは最新のソースコードと合わせて本文書を読む前提である。本文書では、現行実装の詳細な行番号や完全なコード説明ではなく、設計・実装判断に必要な観点を中心に整理する。

対象パッケージは主に以下である。

```text
src/robot_console
```

現行の主要ファイルは以下である。

```text
src/robot_console/robot_console/robot_console_node.py
src/robot_console/robot_console/gui_core.py
src/robot_console/robot_console/ui_main.py
```

---

## 2. 背景

現行の `robot_console` は、tkinter ベースのGUIとして実装されている。機能としては、単なる表示画面にとどまらず、以下を含んでいる。

- ROS 2 topic の購読と状態表示
- ROS 2 topic への手動送信
- `route_manager` / `route_follower` / `robot_navigator` 等の状態監視
- `ros2 launch` による各ノード起動・停止
- simulator 起動切替
- パラメータファイル選択
- route_manager の起動時引数指定
- コンソールログ収集・表示
- RVizおよび可視化launchの起動
- 画像表示
- manual_start / sig_recog / road_blocked / obstacle_hint override などの手動介入

現行実装では、`robot_console_node.py` が ROS 2 ノードとして publisher / subscriber を持ち、`GuiCore` を介して tkinter UI と接続している。ROS 2 executor は tkinter mainloop と分離され、別スレッドで実行されている。この構造自体は、次期UIでも参考にできる。

一方で、`gui_core.py` と `ui_main.py` には、UI表示、状態集約、launch管理、ログ収集、画像変換、進捗計算、イベントバナー判定、走行距離・経過時間計算などが混在しており、このまま tkinter GUI を拡張し続けると保守性・拡張性に課題が出る。

そのため、次期UIでは、以下の方針を採る。

```text
1. ローカル操作UIは PyQt5 で再構成する。
2. HTMLは遠隔観測専用UIとして用意する。
3. PyQt5 UIとHTML UIは画面キャプチャ共有ではなく、共通Snapshotから別ビューとして生成する。
4. 現行GuiCoreの考え方は活かすが、UI非依存のCoreへ段階的に整理する。
5. tkinter版は移行期間中のlegacy UIとして残し、最終的にはPyQt5版を主UIにする。
```

---

## 3. 運用前提

### 3.1 ローカル操作が基本

本プロジェクトの運用では、ロボットに搭載された、またはロボット運用に用いるローカルPC上で操作することを基本とする。

走行中は、手動操作が許可されたタイミング以外では、基本的にローカルPCを操作しない想定である。したがって、走行中に表示しておく画面は、通常は「ダッシュボード」タブである。

### 3.2 HTMLは遠隔操作ではなく遠隔観測

HTML UIは、VPN経由等で外部端末から閲覧することを想定する。ただし、HTML UIは操作用ではない。

HTML UIの目的は、以下の確認である。

```text
・自己位置推定が破綻していないか
・ロボットがどこを走っているつもりか
・waypointや経路との関係が妥当か
・カメラ画像やセンサビューが更新されているか
・障害物、LiDAR、画像認識などの状態が確認できるか
・各種センサ情報が stale / lost になっていないか
```

HTML UIでは、以下を提供しない。

```text
・manual_start送信
・sig_recog送信
・road_blocked送信
・obstacle_hint override送信
・ノード起動/停止
・パラメータ選択/編集
・全コンソールログの詳細操作
```

### 3.3 RVizとの役割分担

RVizは、TF、point cloud、LaserScan、active_route、active_target、MarkerArray 等の空間可視化を担う。

`robot_console` は、運用判断・操作・観測に必要な情報を整理して表示するUIであり、RVizを完全に置き換えるものではない。

ただし、自己位置・センサ情報タブでは、OpenStreetMap上の自己位置・waypoint表示や既存のセンサビューア画像等を表示し、人間がロボットの認識状態を把握しやすくする。

---

## 4. 次期UIの全体像

次期 `robot_console` は、以下の3系統で構成する。

```text
PyQt5 ローカルUI
  - 主UI
  - ローカルPC上で操作
  - 運行状態表示、手動介入、起動設定、ログ確認、自己位置・センサ確認

HTML 遠隔観測UI
  - 補助UI
  - VPN経由等で閲覧
  - 自己位置・センサ情報を中心に閲覧専用で提供

RViz
  - 空間可視化
  - TF、point cloud、経路、target、marker等の確認
```

重要な設計方針は、PyQt5 UIとHTML UIがそれぞれ独自にROSトピックを購読して状態を組み立てないことである。共通のCore / Snapshotを用意し、それぞれが同じ状態データを異なる形式で表示する。

```text
ROS 2 topics / services
        ↓
ConsoleCore / StateStore
        ↓
Snapshot / ViewModel
        ├── PyQt5 UI
        └── HTML遠隔観測UI
```

---

## 5. PyQt5ローカルUIのタブ構成

PyQt5ローカルUIは、以下の4タブ構成を正とする。

```text
1. ダッシュボード
2. 自己位置・センサ情報
3. 起動・設定
4. コンソールログ
```

それぞれの責務は以下である。

```text
ダッシュボード:
  運行情報の詳細、イベントバナー、手動介入、トピック送信

自己位置・センサ情報:
  運行情報サマリ、OpenStreetMap上の自己位置・waypoint、画像、センサビュー、鮮度表示

起動・設定:
  ノード起動/停止、route設定、launch設定、param選択、実効パラメータ確認

コンソールログ:
  ノード別ログ、統合ログ、検索、フィルタ、tail追従、Ubuntu端末風表示
```

---

## 6. 全タブ共通領域（廃止）

**本章は `robot_console_gui_screen_function_design.md` v0.2（2026-05-28）により廃止された。** 同文書2章・13章にある通り、共通ステータスバーは設けず、運行状態の詳細確認と操作はダッシュボードタブに集約する方針へ変更されている。本章以下の記述は初期検討時の案として履歴目的でのみ残し、実装時は `robot_console_gui_screen_function_design.md` の6章（ダッシュボードタブ）・7章（自己位置・センサ情報タブ）を正とする。

### 6.1 共通ステータスバー（旧案・参考）

全タブ共通で、小型のステータスバーを配置する。

ステータスバーは「詳細判断」のためではなく、「異常や重要状態に気づくための最小表示」である。

表示例:

```text
RUNNING | WP 12/80 A-12 → A-13 | 信号 GO | 障害物 CLEAR | road OK | cmd 0.35 m/s
```

表示対象の候補:

```text
・route_manager / route_follower の代表状態
・現在 waypoint
・目標 waypoint
・信号状態
・障害物状態
・road_blocked
・cmd_vel 概要
・主要トピックの未受信/鮮度異常
```

色ルール:

```text
緑: 正常
黄: 注意・待機
赤: 停止・異常・介入必要
灰: 未受信・不明
```

### 6.2 ステータスバーとダッシュボードの棲み分け

ステータスバーとダッシュボードは同じ情報を扱う場合があるが、粒度を分ける。

```text
ステータスバー:
  今危ないかを知るための1行表示。操作ボタンは置かない。

ダッシュボード:
  なぜその状態か、次に何をすべきかを判断するための詳細表示。手動介入もここに置く。
```

### 6.3 全タブ共通操作バーの扱い

現時点では、手動介入系の操作はダッシュボードタブにまとめる。全タブ共通の下部操作バーとして常時表示するかどうかは、画面の窮屈さと誤操作リスクを考慮して慎重に判断する。

本設計の初期方針では、常時表示領域を最小化するため、手動操作は原則としてダッシュボードタブ内に配置する。

---

## 7. ダッシュボードタブ

### 7.1 役割

ダッシュボードタブは、ロボット走行中にローカルPC上で基本的に表示し続ける画面である。

このタブは、以下をまとめる。

```text
・運行情報の詳細
・イベントのバナー表示
・トピック送信などのマニュアル操作
・手動介入パネル
```

### 7.2 表示方針

ダッシュボードは「運行判断」と「必要時の手動介入」を目的とする。

自己位置推定やセンサ画像の詳細確認は、次の「自己位置・センサ情報」タブに分離する。

### 7.3 表示内容

#### 7.3.1 運行状態カード

表示候補:

```text
・route_manager 状態
・route_follower 状態
・route_version
・現在 waypoint
・目標 waypoint
・waypoint index
・route progress
・目標waypointまでの距離
・走行距離
・走行時間
・cmd_vel
```

#### 7.3.2 停止・待機状態カード

表示候補:

```text
・signal_stop_active
・line_stop_active
・WAITING_STOP 状態
・stagnation reason
・avoidance retry count
・front_blocked
・front_clearance
・left_offset
・right_offset
・road_blocked
・road_blocked の入力元
```

#### 7.3.3 イベントバナー

イベントバナーは、走行中に遠目でも認識できる大きな表示とする。

表示対象例:

```text
・manual_start: True
・信号: GO
・信号: STOP
・停止線: STOP
・道路封鎖アラート
・経路更新
・route_manager decision
・異常停止
```

色分け例:

```text
信号GO:
  青または緑

信号STOP:
  橙または赤

停止線STOP:
  紫

道路封鎖:
  赤

manual_start:
  緑

未受信・不明:
  灰
```

イベントバナーは、古いイベントをいつまで表示するかが重要である。現行実装では TTL や sticky 表示に相当する考え方があるため、次期UIでも以下を考慮する。

```text
・一定時間で消えるイベント
・STOP系のように状態解除まで残すイベント
・最終更新時刻表示
・複数イベントが重なった場合の優先順位
```

#### 7.3.4 手動介入パネル

ダッシュボードタブ内に、手動操作系をまとめる。

対象:

```text
・manual_start 送信
・sig_recog GO / STOP 送信
・road_blocked True / False 送信
・obstacle_hint override 送信/停止
・frame_image_path 送信
```

操作方針:

```text
・現在値を必ず表示する
・最終送信時刻を表示する
・入力元がGUIか外部かを表示する
・送信操作と状態表示を明確に分ける
・危険度の高い操作は確認ダイアログを検討する
・連打防止、二重送信防止を検討する
```

### 7.4 ダッシュボードに入れすぎないもの

以下はダッシュボードでは詳細表示しない。

```text
・OpenStreetMap上の詳細自己位置表示
・複数センサ画像の常時グリッド表示
・詳細なLiDARビュー
・実効パラメータ
・全ログ
```

これらは専用タブへ分離する。

---

## 8. 自己位置・センサ情報タブ

### 8.1 役割

自己位置・センサ情報タブは、ロボットが現在どこにいると推定しているか、waypointや経路との関係がどうなっているか、センサや画像がどのように見えているかを確認するための画面である。

このタブは、HTML遠隔観測UIと最も強く対応する。

### 8.2 基本構成

このタブでは、運行情報のサマリを表示したうえで、以下を常時表示する。

```text
・OpenStreetMap上に自己位置とwaypointを重畳した地図
・センサビュー
・カメラ画像
・認識結果画像
・障害物ビュー
```

表示内容は将来変わったり増えたりする可能性があるため、固定レイアウトにしすぎず、複数の表示パネルを差し替え可能な構成にする。

### 8.3 上部サマリー

ダッシュボードの詳細情報をすべて再掲するのではなく、自己位置・センサ確認に必要なサマリに絞る。

表示候補:

```text
・route state
・follower state
・現在 waypoint
・目標 waypoint
・route progress
・現在 pose
・目標までの距離
・localization freshness
・sensor freshness
```

### 8.4 OpenStreetMap地図表示

本タブの中心要素として、OpenStreetMap上に以下を重畳する。

```text
・現在自己位置
・現在姿勢 yaw
・waypoint列
・現在waypoint
・目標waypoint
・走行済み区間
・未走行区間
・必要に応じてlocal ENU原点
・必要に応じて走行軌跡
```

表示方針:

```text
・地図は閲覧用であり、編集機能は持たせない
・waypoint編集は別UIまたは別機能として扱う
・現在位置の更新鮮度を視覚的に示す
・自己位置が古い場合は警告表示する
・routeと自己位置が乖離している場合は注意表示する
```

実装候補:

```text
候補A:
  Qt WebEngine で地図HTMLを埋め込み、Leaflet等でOSMタイルとwaypointを表示する。

候補B:
  Qt側で地図タイル画像を取得/キャッシュし、QGraphicsView等でオーバーレイ描画する。

候補C:
  初期実装では既存route map画像や生成済み地図画像を表示し、後続フェーズでOSM重畳へ発展させる。
```

運用上はローカルPC前提だが、地図タイル取得のためにネットワーク接続が常に使えるとは限らない。必要に応じて、以下を検討する。

```text
・OSMタイルのローカルキャッシュ
・実験エリア周辺の事前キャッシュ
・ネットワーク未接続時の代替地図画像
・route map画像へのフォールバック
```

### 8.5 座標変換方針

OSM表示にはLLHとlocal ENUの変換が必要となる。

基本方針:

```text
・走行時のroute正本はlocal ENUとする
・OSM表示には必要に応じてLLHへ変換する
・変換規約は共通変換機能に集約する
・UI側に座標変換規約を分散させない
・UIは原則として変換済みの表示用データを受け取る
```

UI実装内で独自に地理座標変換ルールを持たないこと。将来的にLLH/ENU変換serviceまたは共通ライブラリが整備される場合は、それを利用する。

### 8.6 センサ・画像パネル

このタブには複数の表示エリアを設ける。

初期候補:

```text
・前方カメラ
・信号認識画像
・障害物検出画像
・既存 sensor_viewer
・2D LiDAR view
・Mid-360 slice view
・route map
```

ただし、表示対象は変わる可能性が高い。固定の3枚表示や4枚表示に閉じない設計にする。

推奨設計:

```text
・パネル単位で追加/削除できる
・各パネルは id / title / type / topic / image / status / updated_at を持つ
・未受信時は placeholder を表示する
・更新が古い場合は stale 表示する
・パネルの並びを設定で変更できる余地を残す
・表示対象追加時にUIコードの大改造が不要な構造にする
```

表示パネルのデータモデル例:

```yaml
sensor_panels:
  - id: front_camera
    title: 前方カメラ
    type: image
    topic: /perception/road_blockage/decision_image
    freshness: OK

  - id: signal_decision
    title: 信号認識画像
    type: image
    topic: /perception/traffic_signal/decision_image
    freshness: OK

  - id: sensor_viewer
    title: センサビュー
    type: image
    topic: /sensor_viewer
    freshness: OK
```

### 8.7 センサ鮮度表示

画像やセンサビューは、表示内容だけでなく更新鮮度が重要である。

各パネルに以下を表示する。

```text
・最終更新時刻
・現在時刻からの経過秒
・OK / STALE / LOST
```

例:

```text
Camera: OK, 0.2s ago
LiDAR view: OK, 0.1s ago
Obstacle view: STALE, 3.5s ago
GNSS pose: OK, 0.2s ago
```

鮮度判定の閾値は、topicごとに調整できるようにする。

例:

```yaml
freshness_thresholds:
  camera:
    stale_sec: 1.0
    lost_sec: 3.0
  lidar_view:
    stale_sec: 1.0
    lost_sec: 3.0
  localization:
    stale_sec: 0.5
    lost_sec: 2.0
```

### 8.8 ダッシュボードとの棲み分け

```text
ダッシュボード:
  運行状態、イベント、手動操作

自己位置・センサ情報:
  地図、自己位置、waypoint、画像、センサ、鮮度
```

このタブには手動操作を置かない。操作は原則としてダッシュボードタブに集約する。

---

## 9. 起動・設定タブ

### 9.1 役割

起動・設定タブは、走行開始前に必要な設定とノード起動をまとめる画面である。

統合対象:

```text
・ノード起動/停止
・launchファイル選択
・simulator有無
・route設定
・start_label / goal_label / checkpoint_labels
・param選択
・実効パラメータ確認
```

### 9.2 表示内容

#### 9.2.1 ノード起動状態

表示候補:

```text
・ノード名
・起動状態
・PID
・launch file
・alternate launch file
・simulator enabled
・起動/停止ボタン
・最終操作時刻
・異常終了メッセージ
```

#### 9.2.2 route設定

表示候補:

```text
・使用route
・start_label
・goal_label
・checkpoint_labels
```

#### 9.2.3 パラメータ確認

以下を見えるようにすることを目標にする。

```text
base parameter
+
launch override
=
effective parameter
```

現行では、起動時にparamファイルやroute_manager向けの起動引数を指定する。次期UIでは、起動前に「最終的に何が渡されるか」を確認できるようにする。

表示例:

```text
Route Manager
  base param: route_manager_default.yaml
  overrides:
    start_label: A-01
    goal_label: A-80
    checkpoint_labels: A-20,A-40,A-60
  effective launch args:
    start_label:=A-01
    goal_label:=A-80
    checkpoint_labels:=A-20,A-40,A-60
```

### 9.3 編集方針

パラメータファイルそのものの編集は、本画面では行わない。

本画面で行うのは、以下に限定する。

```text
・起動時に使用するparamファイルの選択
・起動時override値の入力
・実効設定の確認
```

YAMLの直接編集、保存、差分適用などは対象外とする。必要になった場合は、別機能として安全設計を行う。

### 9.4 走行中操作との分離

起動・設定タブは、走行前に使う画面である。走行中に頻繁に触る画面ではない。

手動介入操作と混在させず、走行中の操作はダッシュボードに集約する。

---

## 10. コンソールログタブ

### 10.1 役割

コンソールログタブは、ノードごとのログ確認、異常調査、開発時のデバッグを目的とする。

走行中に常時見る画面ではない。

### 10.2 表示内容

表示候補:

```text
・ノード別ログ
・全体統合ログ
・WARN / ERROR フィルタ
・検索
・tail追従 ON/OFF
・ログファイルを開く
・ログ保存先表示
```

### 10.3 表示スタイル

Ubuntu端末風の配色を採用する。

基本色:

```text
INFO:
  通常色

WARN:
  黄色

ERROR / FATAL:
  赤

DEBUG:
  グレー
```

可能であれば、ANSIエスケープシーケンスの解釈にも対応する。ただし、初期実装では `[INFO]` `[WARN]` `[ERROR]` などの文字列ベースの色分けでもよい。

### 10.4 推奨Widget

PyQt5では、ログ表示に `QPlainTextEdit` を用いることを推奨する。

理由:

```text
・大量テキスト表示に比較的向いている
・tail追従を実装しやすい
・検索/フィルタと組み合わせやすい
・等幅フォントと端末風配色を適用しやすい
```

---

## 11. HTML遠隔観測UI

### 11.1 基本方針

HTML UIは、PyQt5 UIの画面キャプチャではない。

共通Snapshotを用いて、遠隔観測に必要な情報をHTMLとして再構成する。

HTML UIは操作を提供しない。

```text
提供する:
  観測情報

提供しない:
  手動操作
  launch操作
  パラメータ編集
```

### 11.2 HTMLで主に提供する範囲

HTML UIは、PyQt5の「自己位置・センサ情報」タブに近い内容を提供する。

重点:

```text
・自己位置推定の状況
・OpenStreetMap上の自己位置とwaypoint
・画像
・センサビュー
・各情報の更新鮮度
```

### 11.3 HTMLに出す情報

#### 自己位置・経路

```text
・現在自己位置
・現在姿勢 yaw
・現在 waypoint
・目標 waypoint
・waypoint列
・route progress
・目標までの距離
・地図上の自己位置表示
```

#### センサ・画像

```text
・前方カメラ
・信号認識画像
・障害物ビュー
・既存 sensor_viewer
・LiDAR view
・必要に応じた追加画像
```

#### 鮮度・状態

```text
・GNSS / localizer 更新時刻
・LiDAR odometry 更新時刻
・camera 更新時刻
・sensor_viewer 更新時刻
・TF status
・OK / STALE / LOST 表示
```

### 11.4 HTMLに出さない情報

```text
・manual_start送信
・sig_recog送信
・road_blocked送信
・obstacle_hint override送信
・ノード起動/停止
・param選択
・param編集
・全コンソールログ
```

ログについては、自己位置・センサ状態の把握に必要な最小限の状態表示に留める。

### 11.5 HTMLレイアウト

スマホ最適化は必須ではない。

ただし、外部端末から見やすいように、シンプルな縦積みまたは2カラム程度の構成にする。

例:

```text
1. 状態サマリ
2. OSM地図
3. センサ・画像グリッド
4. 更新鮮度一覧
```

### 11.6 実装方式

初期実装では、シンプルなHTTP pollingでよい。

候補API:

```text
GET /snapshot.json
GET /map_state.json
GET /sensor_panels.json
GET /images/{panel_id}
GET /health.json
```

WebSocketは、将来必要になった場合に導入する。

---

## 12. Snapshot / ViewModel 設計方針

### 12.1 基本方針

UIはROSメッセージを直接解釈しない。

ROS topicから受け取った情報は、UI非依存のStateStore / ConsoleCoreで集約し、Snapshot / ViewModelとしてUIへ渡す。

```text
ROS 2 Node
  ↓
StateStore / ConsoleCore
  ↓
Snapshot / ViewModel
  ↓
PyQt5 UI / HTML UI
```

### 12.2 Snapshotに含めるべき情報

#### 運行状態

```text
・route_manager state
・route_follower state
・route status
・current waypoint
・target waypoint
・route progress
・event banner candidate
```

#### 手動操作状態

```text
・manual_start current value
・sig_recog current value
・road_blocked current value
・obstacle_hint override state
・last command timestamp
・input source
```

#### 自己位置・地図表示情報

```text
・current pose in ENU
・current pose in LLH
・yaw
・active target pose
・waypoint list
・route polyline
・current waypoint index
・target waypoint index
・localization freshness
```

#### センサ・画像情報

```text
・image panel entries
・topic name
・display title
・latest image reference
・last update time
・freshness status
・panel type
```

#### ログ情報

```text
・node別ログ
・latest warnings
・latest errors
・log file path
```

HTMLには全ログを出さないが、PyQt5のコンソールログタブでは使用する。

### 12.3 Snapshotの形式

Snapshotは、PyQt5 UIだけでなくHTML UIでも使えるように、JSON化しやすい構造にする。

画像本体はSnapshotに直接巨大なバイナリとして持たせるのではなく、以下のように扱うことを推奨する。

```text
Snapshot:
  panel id, title, status, timestamp, image reference

ImageStore:
  panel id → latest image bytes / QImage / encoded PNG
```

HTMLでは `/images/{panel_id}` のようなAPIで画像を取得する。

PyQt5では、ImageStoreからQImage/QPixmapとして取得する。

---

## 13. 現行tkinter実装からの移植方針

### 13.1 基本方針

全面書き直しは避ける。

現行実装には、次期UIでも活かせる構造がある。

再利用候補:

```text
・GuiCoreの状態集約の考え方
・GuiSnapshotの考え方
・command queueの考え方
・NodeLaunchProfileの考え方
・NodeLaunchManagerの起動/停止管理
・ログ収集の考え方
・画像変換処理の一部
```

ただし、現行の `GuiCore` と `ui_main.py` には責務が混在しているため、PyQt5化に先立って段階的に分離する。

### 13.2 分離すべき責務

以下をUI非依存のcoreへ分離する。

```text
・launch管理
・ログ管理
・snapshot生成
・進捗計算
・イベントバナー判定
・距離/時間計算
・画像変換
・センサ鮮度判定
・地図表示用データ生成
```

### 13.3 推奨ディレクトリ構成

**本節の構成案は初期検討時のものである。正式なディレクトリ構成は `robot_console_gui_architecture_design.md` 5章を正とする。** 同文書では、共通ステータスバー廃止に伴い `status_bar.py` を持たず、tkinter完全移行方針（同文書4章）に伴い `ui_tk/` を正式構成に含めない。

```text
src/robot_console/robot_console/
  core/
    __init__.py
    command_model.py
    snapshot_model.py
    state_store.py
    launch_profile.py
    launch_manager.py
    log_manager.py
    image_store.py
    freshness.py
    metrics.py
    localization_adapter.py
    route_adapter.py
    map_model.py

  ros/
    __init__.py
    console_node.py

  ui_qt/
    __init__.py
    main_window.py
    dashboard_tab.py
    localization_sensor_tab.py
    launch_settings_tab.py
    console_log_tab.py
    widgets/
      image_panel.py
      log_view.py
      status_card.py
      map_view.py

  web/
    __init__.py
    server.py
    static/
      index.html
      app.js
      style.css
```

既存のパッケージ構造との整合性を見ながら調整してよいが、以下の方針は守る。

```text
・Qt WidgetにROS subscriber/publisherを直接書かない
・Web serverが独自にROS topicを購読しない
・状態解釈ロジックをUIごとに重複実装しない
```

### 13.4 移植フェーズ

**本節のフェーズ計画は初期検討時のものである。以下は `robot_console_gui_architecture_design.md`・`robot_console_gui_screen_function_design.md` の内容（profile定義駆動、GPS/ypspur/simulator代替、机上確認、tkinter完全移行方針）に合わせて更新した版である。**

推奨フェーズ:

```text
Phase 1:
  Core分離 + profile定義基盤
  - launch_manager（profile定義駆動。simulator代替/alternate launchの切替を含む）
  - log_manager
  - snapshot_model（gps_state等、architecture_design.md 8章のフィールドを含む）
  - metrics
  - freshness
  - config/node_launch_profiles.yaml の初版整備
    （ypspur_ros2, rtk_gps_um982, obstacle_route_sim, route_planner/manager/follower,
      drive_mode_manager, robot_navigator, obstacle_monitor,
      road_blockage_detector, traffic_signal_recognizer,
      route_markers, target_marker, robot_console_rviz を含む）

Phase 2:
  PyQt5 UI骨格作成
  - MainWindow
  - 4タブ（共通ステータスバーは設けない）

Phase 3:
  ダッシュボードタブ実装
  - 運行状態、GPS/Poseカード、Drive/CmdVelカード
  - イベントバナー
  - Manual Opsカード（手動介入）
  - Node Healthカード

Phase 4:
  起動・設定タブ実装
  - 業務モード選択（実機/シミュレーション/机上確認）とプリセット適用
  - 起動候補ツリー、起動予定ノード一覧、ノード設定編集パネル
  - simulator代替・alternate launchトグル
  - 実効設定表示（起動内容プレビュー）

Phase 5:
  コンソールログタブ実装
  - ノード別ログ
  - 色付き表示
  - tail追従
  - 検索/フィルタ

Phase 6:
  自己位置・センサ情報タブ実装
  - OSM地図
  - waypoint重畳
  - sensor panel model
  - 画像/センサビュー
  - 鮮度表示

Phase 6.5:
  GUIデザイン・機能の確認と調整
  - Phase 2〜6でPyQt5ローカルUIの4タブ（ダッシュボード、自己位置・センサ情報、
    起動・設定、コンソールログ）が揃った時点で実施する
  - タブ横断の統一感（配色、余白、フォント、カード様式）を確認する
  - 16:9論理キャンバスのスケーリングを、各タブの実際の情報密度で確認する
  - タブ間の画面遷移（9章 画面間導線）が意図通りに機能するか確認する
  - 指摘事項をPhase 2〜6の該当タブへ反映する
  - この時点ではROS 2ノードとの実結合（`ros/console_node.py` によるConsoleCore
    ⇔ Snapshot ⇔ UIの配線）が未実施のため、確認は `ConsoleSnapshot()` 既定値
    またはテスト用の疑似Snapshotに基づくものに限られる。運用データに基づく
    確認は、ROS実結合の完了後に別途行う

Phase 7:
  HTML遠隔観測UI実装
  - Snapshot JSON
  - map_state JSON
  - sensor panel images
  - 閲覧専用ページ

Phase 8:
  tkinter版の完全撤去
  - `robot_console_gui_architecture_design.md` 4章の方針に従い、tkinter版を別UIとして残さず削除する
  - entry point、README、launch、評価ツール、設計書の参照先をPyQt5版へ統一する
```

---

## 14. PyQt5実装上の注意点

### 14.1 Qt event loop と ROS executor の分離

PyQt5では、Qtのmain threadとROS 2 executor threadを分離する。

```text
Qt main thread:
  UI描画、ユーザー操作、QTimer更新

ROS executor thread:
  ROS subscription、publication、service/action処理
```

現行 `robot_console_node.py` のように、ROS executorを別スレッドで回す構造を参考にする。

### 14.2 QWidgetにROS処理を直接書かない

禁止する構成:

```text
QWidget / QMainWindow
  └ create_subscription / create_publisher を直接持つ
```

推奨構成:

```text
ROS Node
  ↓
ConsoleCore
  ↓
Snapshot
  ↓
Qt Widget
```

### 14.3 UI更新はSnapshotベース

PyQt5 UIは、一定周期または通知ベースでSnapshotを取得して表示する。

初期実装では、200ms程度のQTimer pollingでよい。高頻度画像更新や地図更新が重い場合は、パネル単位で更新頻度を分ける。

### 14.4 大量ログの扱い

ログ表示では、全履歴をUI widgetに無制限に保持しない。

方針:

```text
・表示行数に上限を設ける
・ファイル保存はlog_managerが担う
・UIは直近ログを表示する
・検索対象をUI保持分に限定するか、ファイル検索を別途実装する
```

### 14.5 画像更新の扱い

画像は高頻度に更新される可能性がある。UIを詰まらせないよう、以下を守る。

```text
・ROS callback内で重い画像変換をしすぎない
・UI threadで重い変換をしすぎない
・必要に応じて変換済みQImage/QPixmapをImageStoreに保持する
・表示サイズに合わせた縮小はパネル側で行う
・更新頻度を制限する
```

---

## 15. HTML実装上の注意点

### 15.1 操作APIを出さない

HTML UIは遠隔観測専用である。

初期実装では、HTTP APIに操作系エンドポイントを作らない。

作らないもの:

```text
POST /manual_start
POST /sig_recog
POST /road_blocked
POST /launch
POST /stop
POST /params
```

### 15.2 読み取りAPIに限定する

候補:

```text
GET /snapshot.json
GET /map_state.json
GET /sensor_panels.json
GET /images/{panel_id}
GET /health.json
```

### 15.3 画像転送形式

画像はPNGまたはJPEGで提供する。

- センサビューや地図系はPNGが向く場合がある
- カメラ画像はJPEGが軽い場合がある
- 初期実装ではPNGで統一してもよい

### 15.4 更新方式

初期実装では polling でよい。

```text
snapshot: 0.5〜1.0秒周期
画像: 0.5〜2.0秒周期
```

WebSocketは、必要性が明確になってから導入する。

### 15.5 ネットワークとセキュリティ

VPN経由閲覧を前提とするが、以下を検討する。

```text
・bind addressをlocalhost / LAN / VPNに限定できること
・portを設定可能にすること
・操作APIを出さないこと
・必要に応じて簡易認証を追加できる余地を残すこと
```

---

## 16. OpenStreetMap表示の注意点

### 16.1 編集機能を持たせない

自己位置・センサ情報タブおよびHTML UIのOSM表示は、観測用である。

waypoint編集、route編集、地図上クリックによるroute修正などは、このUIには含めない。

### 16.2 waypoint表示

最低限、以下を表示する。

```text
・全waypointまたは表示範囲内waypoint
・現在waypoint
・目標waypoint
・route polyline
・現在自己位置
・yaw方向
```

### 16.3 表示鮮度

自己位置が stale / lost の場合は、地図上のマーカー色やステータス表示で明確に示す。

例:

```text
OK:
  通常色

STALE:
  黄色、点滅または警告表示

LOST:
  赤、最後の位置で停止表示
```

### 16.4 座標変換の責務

UI側に座標変換規約を分散させない。

UIは、Coreから以下のような表示用データを受け取ることを目標とする。

```text
・LLH済み自己位置
・LLH済みwaypoint列
・route polyline in LLH
・ENU表示が必要な場合はENUも併記
```

---

## 17. 受け入れ条件の例

### 17.1 基本UI

**本節は初期検討時の受け入れ条件案である。共通ステータスバーおよびtkinter版併存に関する記述は、`robot_console_gui_architecture_design.md`（4章: 完全移行方針）・`robot_console_gui_screen_function_design.md`（v0.2: 共通ステータスバー廃止）により置き換えられている。**

```text
・PyQt5のMainWindowが起動する
・4タブが存在する
・共通ステータスバーは設けない
・移行完了後はPyQt5版が唯一の正式entry pointであり、tkinter版は残さない（移行途中の一時的併存は許容）
```

### 17.2 ダッシュボード

```text
・route_manager / route_follower 状態が表示される
・現在WP / 目標WP / 進捗が表示される
・cmd_velが表示される
・イベントバナーが表示される
・manual_start / sig_recog / road_blocked / obstacle_hint override を送信できる
・最終送信時刻と現在値が表示される
```

### 17.3 自己位置・センサ情報

```text
・運行情報サマリが表示される
・地図上に自己位置とwaypointが表示される
・複数のセンサ/画像パネルが表示される
・各パネルに更新時刻と鮮度状態が表示される
・表示パネルの追加に大規模改修が不要な構造になっている
```

### 17.4 起動・設定

```text
・対象ノードの起動/停止ができる
・paramファイルを選択できる
・route_manager向け起動時引数を指定できる
・base parameterとoverride、実効起動引数が確認できる
```

### 17.5 コンソールログ

```text
・ノード別ログが表示される
・ログのtail追従ができる
・WARN/ERRORが色分けされる
・検索またはフィルタができる
・ログファイルを開ける、またはパスを確認できる
```

### 17.6 HTML遠隔観測UI

```text
・ブラウザから状態サマリを閲覧できる
・OSMまたは代替地図上に自己位置とwaypointが表示される
・画像/センサビューを閲覧できる
・鮮度状態が表示される
・操作系APIが存在しない
```

---

## 18. 非目標

以下は、次期 `robot_console` UI更改の初期スコープ外とする。

```text
・waypoint編集
・route編集
・地図上クリックによる経路修正
・遠隔操作UI
・HTMLからの手動介入
・パラメータYAMLの直接編集/保存
・RVizの完全代替
・全センサデータの高精細可視化
```

必要になった場合は、別途設計する。

---

## 19. 主要な設計リスク

### 19.1 UI肥大化

PyQt5化しても、全機能をMainWindowに直接実装すると、tkinter版と同じ問題が再発する。

対策:

```text
・タブごとにクラスを分ける
・共通Widgetを分ける
・CoreとUIを分離する
・Snapshot/ViewModelを中心にする
```

### 19.2 状態解釈の二重実装

PyQt5とHTMLが別々に状態解釈すると、表示不整合が起きる。

対策:

```text
・共通Snapshotを使う
・HTMLはSnapshot JSONを読む
・UI固有処理は見た目だけに限定する
```

### 19.3 地図表示の複雑化

OSM表示、タイルキャッシュ、座標変換、waypoint重畳は複雑になりやすい。

対策:

```text
・初期実装では最小機能から始める
・座標変換はCore側に閉じる
・地図表示は編集機能を持たせない
・ネットワーク無し時のフォールバックを用意する
```

### 19.4 画像更新によるUI負荷

複数画像を高頻度表示するとUIが重くなる。

対策:

```text
・表示更新頻度を制限する
・画像変換を適切に分離する
・パネルごとの更新周期を持つ
・必要以上の高解像度表示を避ける
```

---

## 20. 実装チームへの推奨初手

最初にPyQt5画面を作り始めるのではなく、以下から着手することを推奨する。

```text
1. 現行GuiCoreの責務を整理する
2. Snapshot/ViewModelに必要な項目を定義する
3. dashboard / localization_sensor / launch_settings / logs の4タブに必要なデータを洗い出す
4. sensor panelをリスト形式で表現できるモデルを作る
5. PyQt5のMainWindowと空タブだけを作る
6. 既存tkinter版と併存できるentry pointを追加する
```

想定entry point例:

```text
robot_console       # 移行期間中は既存tkinter版または互換入口
robot_console_qt    # PyQt5版
robot_console_web   # HTML遠隔観測サーバ、またはqt/core内蔵起動
```

entry point名は実装チームで最終決定してよいが、移行期間中に既存起動方法を壊さないこと。

---

## 21. 最終方針

次期 `robot_console` は、以下を正とする。

```text
PyQt5ローカルUI:
  1. ダッシュボード
  2. 自己位置・センサ情報
  3. 起動・設定
  4. コンソールログ

HTML遠隔観測UI:
  自己位置・センサ情報を中心とした閲覧専用UI

RViz:
  空間可視化専用
```

最も重要な分担は以下である。

```text
ダッシュボード:
  運行状態の詳細、イベントバナー、手動介入、トピック送信

自己位置・センサ情報:
  OSM地図、自己位置、waypoint、画像、センサ、鮮度

起動・設定:
  ノード起動、route設定、param選択、実効設定確認

コンソールログ:
  開発・異常調査用ログ表示

HTML:
  遠隔観測。操作はしない。
```

この方針により、ローカル操作ルールを守りながら、走行中の状況把握、自己位置推定の妥当性確認、センサ状態の観測、開発時の調査性を両立する。

---

## 22. 改版履歴

| 日付 | 版 | 変更概要 |
| --- | --- | --- |
| 2026-08-29 | 0.3 | 13.4節の移植フェーズに Phase 6.5「GUIデザイン・機能の確認と調整」を追加。PyQt5ローカルUIの4タブ（Phase 2〜6）が揃った時点でタブ横断の統一感・16:9スケーリング・画面間導線をまとめて確認し、Phase 2〜6の該当タブへ反映してからPhase 7（HTML遠隔観測UI）へ進む方針とした。この時点ではROS 2ノードとの実結合が未実施であることも明記した。 |
| 2026-08-29 | 0.2 | 6章（共通ステータスバー）を廃止扱いに変更し `robot_console_gui_screen_function_design.md` を正と明記。13.3節のディレクトリ構成を `robot_console_gui_architecture_design.md` 5章に合わせて更新（`status_bar.py`/`ui_tk/` を除去）。13.4節の移植フェーズをprofile定義駆動・GPS/ypspur/simulator代替・机上確認・tkinter完全撤去の方針に合わせて更新。 |
| （初版日付未記載） | 0.1 | UI更改の背景、運用前提、タブ構成、移植方針を初版として作成。 |
