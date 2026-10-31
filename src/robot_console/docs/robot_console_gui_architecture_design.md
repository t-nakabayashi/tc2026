# robot_console アーキテクチャ詳細設計書

## 1. 文書目的・対象範囲

本書は `robot_console` パッケージの構造を定める。画面の構成と操作は
[画面・機能詳細設計書](robot_console_gui_screen_function_design.md)、起動手順と公開範囲は
[README](../README.md) を正とする。

**本書の章番号はコード内のコメントから参照される。** 章の追加・削除・移動を行う場合は、
`grep -rn 'architecture_design' src/robot_console` で参照箇所を確認して併せて更新する。

正式な入口は `ui_qt_main.py` の `robot_console_qt` である。HTML 遠隔観測 UI は
`web_main.py` から単独で起動できる。

記載内容はすべて実装済みである。実機センサ一式での帯域・長時間記録は確認範囲が限られる
（16 章）。

## 2. 背景・要求・スコープ

走行システムの状態を 1 つの画面に集約し、走行前の確認と走行中の監視、ノード群の起動停止、
記録操作を行う。

責務は次の 5 点である。

- ROS の観測値を収集し、表示用のデータへ変換する。
- 画面と HTML で同じ表示データを使う。
- profile 定義に従ってノード群を起動・停止し、プロセス状態とログを集約する。
- 操作入力を ROS メッセージへ変換して配信する。
- bag 記録とオフライン地図作成のジョブを管理する。

非スコープは、走行可否の判定、自己位置推定、座標変換、経路生成である。
**画面は ROS 観測の写しであり、走行可否の判定機構を持たない。表示値だけで走行の許可を
判断しない。**

## 3. 全体構成・アーキテクチャ

Qt 表示、ROS 通信、状態集約、HTTP 配信を分離する。**画面から ROS メッセージを直接
組み立てない。**

```text
ROS topics ──→ ros/console_node.py ──→ core/console_core.py ──→ ConsoleSnapshot
                      ↑                        │                      │
                操作の publish  ←──────────────┤                      ├──→ ui_qt/（Qt）
                                               │                      └──→ web/（HTTP）
                                   core/launch_manager.py
                                               ↓
                                     ros2 launch の子プロセス
```

### 3.1 モジュール責務

| 配置 | 責務 |
| --- | --- |
| `ros/console_node.py` | ROS topic の購読・配信のみ。**ROS メッセージ型から内部型への変換はここで行う** |
| `core/console_core.py` | 状態・操作・起動管理の統合、Snapshot 生成 |
| `core/camera_overlay.py` | 生画像への信号・道路封鎖の枠重畳、保持期限、判定チップ生成 |
| `core/snapshot_model.py` | 画面・Web 共通の表示データ（8 章） |
| `core/localization_adapter.py`、`route_adapter.py`、`drive_mode_adapter.py` | 位置・経路・走行状態の表示モデル |
| `core/freshness.py` | 受信鮮度の判定（13.2 節） |
| `core/operation_phase.py` | 運行フェーズと停止理由の導出 |
| `core/event_builder.py`、`event_priority.py` | イベントの導出と優先順位 |
| `core/node_health.py` | 起動状態・鮮度・自己申告診断の合成 |
| `core/launch_manager.py`、`launch_profile.py`、`log_manager.py` | profile 起動・停止・ログ |
| `core/business_mode.py` | 業務モードと起動プリセットの対応 |
| `core/bag_recorder.py` | bag 記録プロセスと保存状態 |
| `core/image_store.py` | 画像の保持（8.2 節） |
| `core/clock_sync_status.py` | 時刻同期状態の読み取り |
| `ui_qt/` | MainWindow、7 タブ、各カード |
| `web/` | 読取専用 HTTP、JSON 整形、ブラウザ表示 |

`ConsoleCore` は **ROS メッセージ型に依存しない。** `rclpy` を初期化せずに単体試験できる。

## 4. ROS 依存と非依存の分離

| 層 | ROS 依存 | Qt 依存 |
| --- | --- | --- |
| `ros/` | あり | なし |
| `core/` | **なし** | **なし** |
| `ui_qt/` | なし | あり |
| `web/` | なし | なし |

`core/` は ROS メッセージも PyQt5 も import しない。`core/image_store.py` が画像を
`PIL.Image.Image` として保持するのもこの方針による（8.2 節）。

`ui_qt/` の QWidget は publisher も subscriber も持たない（14.2 節）。操作は
`ConsoleCore` のメソッドを呼び、`ConsoleCore` が ROS 層へ注入された publisher を呼ぶ。

## 5. パッケージ構成・ファイル配置

| パス | 用途 |
| --- | --- |
| `robot_console/ui_qt_main.py` | Qt 版の入口 |
| `robot_console/web_main.py` | HTML 遠隔観測 UI の入口 |
| `robot_console/{ros,core,ui_qt,web,utils}/` | 3 章の各層 |
| `config/node_launch_profiles.yaml` | 起動 profile 定義（9.1 節） |
| `launch/robot_console.launch.py` | Qt 版の起動 |
| `docs/` | 本書と画面・機能詳細設計書 |
| `tools/tests/` | 単体・結合試験 |
| `tools/check_qt_entry.py` | Qt 入口の起動確認 |
| `tools/headless_route_stack_eval.py`、`qt_route_stack_eval.py` | 経路スタックの評価スクリプト |

## 6. 位置・経路情報の扱い

### 6.1 座標系の正本

| 表示 | 使う topic | 座標系 |
| --- | --- | --- |
| 自己位置の地図表示 | `/localization/pose_llh` | LLH |
| 経路の地図表示 | `/active_route` の waypoint LLH | LLH |
| 目標の地図表示 | `/route/active_target_llh` | LLH |
| 数値表示の ENU 位置 | `/localization/pose_enu` | ENU |

**ENU と LLH の変換は `geo_pose_converter` に集約する。画面で独自の原点を設定しない。**
目標の緯度経度も `geo_pose_converter` の `route_geo_projector_node` が変換して配信し、
本パッケージは受け取るだけである。

### 6.2 自己位置の表示モデル（LocalizationView）

自己位置は `LocalizationStateView` として保持する。**GNSS 単独の品質と、融合が出力する
yaw・標準偏差・モードを別データとして扱う。** 両者を 1 つの「位置品質」に混ぜない。

| 区分 | 内容 |
| --- | --- |
| GNSS 単独 | RTK 状態、衛星数、HDOP、補正データ年齢、アンテナ方位、基線長 |
| 融合 | 車体 yaw、位置・方位の標準偏差、採用した運動源のモード |

**GNSS 単独方位は北基準の時計回り、融合 yaw は map 基準の反時計回りである。** 画面上で
両者を明示的に区別して表示する。

標準偏差は推定品質であり、実測誤差ではない。

### 6.3 変換の集約先

座標変換を持つのは `geo_pose_converter` だけである。本パッケージが行うのは、受け取った
LLH を地図描画用の緯度経度としてそのまま使うことと、ENU 位置を数値として表示することに
限る。

### 6.4 表示方針

`active_route` のように保持される情報（TRANSIENT_LOCAL）と、速度・センサのように鮮度が
必要な情報を区別する。**未受信の値をゼロや正常値に置換しない。** 未受信と期限切れは
区別して表示する。

## 7. 外部インタフェース仕様

### 7.1 購読トピック

ノードは**先頭にスラッシュを付けない相対名**で宣言する。launch からの remap と namespace
付与を可能にするためである。launch 引数で公開するトピック名は、operator が実際の配線
（例: `ypspur_ros2` や `robot_navigator` の `odom_topic` 設定）に合わせて上書きできる。

| 区分 | トピック |
| --- | --- |
| 経路・追従 | `route_state`、`manager_status`、`active_route`、`follower_state`、`active_target`、`route/active_target_llh` |
| 走行制御 | `drive_mode_status`、`cmd_vel`、`cmd_vel/autonomous`、`odom` |
| 自己位置 | `localization/pose_enu`、`localization/pose_llh` |
| 融合 | `/fusion/status`、`/fusion/gnss_dropout_active` |
| GNSS | `rtk_gps/rtk_status`、`rtk_gps/ntrip_status` |
| 障害物 | `obstacle_avoidance_hint` |
| 操作入力 | `manual_start`、`sig_recog`、`road_blocked` |
| 画像 | `usb_cam/image_raw`、`sensor_viewer` |
| 認識 | `perception/traffic_signal/overlay`、`perception/road_blockage/overlay` |
| 採取 | `/route_survey/status` |
| 健全性 | `/diagnostics`、profile 定義の `health_topics`（9.1 節） |

**操作入力の 3 topic は配信と購読の両方を行う。** 自身が配信した値も DDS 経由で返って
くるため、GUI 送信の結果確認と、GUI 以外（信号認識ノード、`road_blockage_detector`）
からの送信検知を同じ購読で扱う。

**認識ノードは重畳画像を配信しない。** 生画像 1 本と検出結果を受け取り、重畳は
`core/camera_overlay.py` が行う。Qt と HTML は同じ重畳画像と判定結果を表示する。

RTK/NTRIP 診断は相対名 `rtk_gps/rtk_status` と `rtk_gps/ntrip_status` で購読する。
実機・模擬・単独起動・共通起動のいずれも `/rtk_gps/` 配下の公開名を使う。

健全性監視のために、`health_topics` に挙がった topic を**生の購読として重複して購読
する**。他の用途で既に購読している topic でも、監視用の購読を別に作る。購読が無いと
配信があっても受信時刻が更新されず、誤った異常判定になるためである。重複購読は raw・
BEST_EFFORT・depth 1 であり、コストよりも判定の確実性を優先する。

### 7.2 配信トピック

| トピック | 型 | 用途 |
| --- | --- | --- |
| `manual_start` | `std_msgs/Bool` | 開始・停止点からの再開、入力解除 |
| `sig_recog` | `std_msgs/Int32` | 信号認識結果の手動入力 |
| `road_blocked` | `std_msgs/Bool` | 道路封鎖の手動入力 |
| `obstacle_avoidance_hint` | `tc_route_msgs/ObstacleAvoidanceHint` | 固定障害物ヒントの手動入力 |
| `/frame_image_path` | `std_msgs/String` | 静止画像入力のパス |
| `/route_survey/command` | `std_msgs/String` | 経路採取の開始・終了 |

**走行指令（`cmd_vel` 系）は配信しない。** 手動操縦は `drive_mode_manager` が担う。

## 8. Snapshot / ViewModel 設計

`ConsoleSnapshot` 1 個に表示用データをすべて載せ、Qt と HTML が同じものを参照する。
Snapshot は不変のデータクラスの集合であり、ROS 型も Qt 型も含まない。

ROS コールバックは観測と受信時刻を `ConsoleCore` へ渡すだけで、Snapshot は作らない。
UI 側がタイマーで Snapshot を取得する。

### 8.1 GPS 状態モデル

`GpsStateView` が GPS / Pose カードの表示データを持つ。RTK 状態、衛星数、HDOP、補正
データ年齢、方位と標準偏差、アンテナ間距離、主アンテナの緯度・経度・高度を含む。

**未受信の数値は「—」として表示し、古い FIX 表示を正常色にしない。**

### 8.2 画像参照モデル

`ImageStore` が画像を**フレームワーク非依存の `PIL.Image.Image`** として保持する。
PyQt5 向けの `QImage` / `QPixmap` への変換は UI 層（`ui_qt/widgets/image_panel.py`）が
行う。HTML 側は `ImageStore` から PNG として取り出す。

`ImageReference` が画像 1 枚分の識別子・表示名・元 topic 名・受信時刻を持つ。Snapshot は
画像の実体ではなく参照のみを持つ。

## 9. 起動管理設計

### 9.1 profile 定義駆動

起動対象は**コード内の固定リストではなく `config/node_launch_profiles.yaml` から生成
する。** profile の追加・変更でモジュールと UI の改修が不要になるよう、必要な情報は
すべて YAML 側に持たせる。

| profile の項目 | 用途 |
| --- | --- |
| `profile_id` | 識別子 |
| `category` | 起動候補ツリーの分類（9.2 節） |
| `display_name` | 画面表示名 |
| `package`、`launch_file` | 起動するもの |
| `param_argument`、`param_package`、`default_param` | 設定ファイルの引数 |
| `launch_order` | 起動順 |
| `startup_group` | 起動グループ（10 章） |
| `health_topics` | 死活監視に使う topic と公称レート |
| `diagnostic_nodes` | 自己申告診断を集めるノード名 |

`health_topics` に登録できるのは**配信様式が `stream` の topic だけ**である。`latch` と
`event` は登録しない。閾値は公称レートから導出する（13.2 節）。

### 9.2 起動カテゴリ

| category | 表示名 |
| --- | --- |
| `real_robot_base` | 実機基盤 |
| `gps_gnss` | GPS/GNSS |
| `simulation_base` | シミュレーション基盤 |
| `route_stack` | 経路計画・追従 |
| `drive_stack` | 走行制御・mux |
| `obstacle_stack` | 障害物監視 |
| `perception_stack` | 認識・判定 |
| `visualization` | 可視化 |

### 9.3 既定 profile と引数の上書き

業務モード（環境 × 走行モード）ごとに起動プリセットを持つ。実機の手動走行
（`real_survey`）と、記録ルートによる自律走行（`recorded_route` / `icp_route`）は
`icart_bringup` の共通 launch へ委譲する。

| 業務モード | 既定の自己位置 |
| --- | --- |
| 実機（ICP）/ 手動走行 | `localization_mode=icp` |
| 実機（融合）/ 手動走行 | `localization_mode=gnss` を明示 |
| 実機 / 手動走行 | `localization_mode=icp` |

**プリセットの引数変更も `ConsoleCore` へ通知する。** 表示中の方式と実際の起動引数を
一致させるためである。

**同じドライバ・走行スタックの重複起動は拒否する。** 環境の選択だけでは稼働中プロセスの
環境を切り替えない。全停止したうえで設定して再起動する。

## 10. 起動グループ

`startup_group` が同じ profile をまとめて一斉に起動・停止する。ダッシュボードの起動操作は
グループ単位で行う。

**グループの RUN 表示は、そのグループに属するプロセスが生きていることを示すだけで、
すべてのセンサが正常であることを意味しない。** プロセスの RUN 数は起動グループ数である。

## 11. パラメータ・設定仕様

launch の引数は [README](../README.md) を正とする。トピック名の引数（`odom_topic`、
`rtk_status_topic`、`ntrip_status_topic` など）は 7.1 節の方針で公開する。

GNSS を同時に起動するかは `start_gnss`（既定 false）で選ぶ。`gnss_site` と
`gnss_station`（既定「地域の既定局」）で場所と RTK 補正局を指定する。局名と地域は
`core/rtk_station_catalog.py` の基地局カタログから `ntrip.station_id` /
`station_label` / `site` に設定する。

bag の既定保存先は環境変数 `ROBOT_CONSOLE_BAG_DIR` で初期値を変更できる。既定は
`~/rosbags` である。

## 12. 処理フロー・状態遷移

### 12.1 起動時

1. `ConsoleCore` を生成する。
2. profile 定義を読み込み、`health_topics` の閾値を `FreshnessMonitor` へ登録する。
3. `ros/console_node.py::start_ros_thread()` で、Qt イベントループとは別スレッドの
   rclpy executor を起動する（14.1 節）。
4. Qt は `QTimer` で Snapshot を取得して widget を更新する。

HTML 単独起動では Qt を作らず、HTTP サーバのスレッドと rclpy executor を動かす。

### 12.2 GPS 状態更新

1. `rtk_gps/rtk_status` を受け、受信時刻を記録する。
2. 診断 JSON（`std_msgs/String`）を解釈して `GpsStateView` を更新する。
3. `rtk_gps/ntrip_status` を受け、`NtripStateView` を更新する。
4. 鮮度を判定し、未受信・期限切れの数値を正常色にしない（6.4 節）。

**GNSS の位置コールバックとは独立に診断を受け取る。** 位置が届かなくても診断は更新され、
逆もある。接続成功と補正受信を区別する。

### 12.3 位置・経路状態更新

1. `localization/pose_llh` と `pose_enu` を受け、`LocalizationStateView` を更新する。
2. `active_route` を受け、waypoint 列を `RouteView` へ変換する。**緯度経度を持つ
   waypoint だけを地図へ描く。**
3. `route_state` の現在 index を境に、走行済みと未走行を色分けする。
4. `route/active_target_llh` を受け、目標位置を更新する。
5. `follower_state`、`manager_status`、`drive_mode_status` から運行フェーズと停止理由を
   導出する。

**フォロワの走行状態（RUNNING / AVOIDING）を新鮮に受信していれば、開始操作を観測し
損ねても運行表示へ反映する。** UI が後から起動して揮発性の `manual_start` を取り逃した
場合に備える。**開始指令を推測して再送しない。**

## 13. QoS・鮮度判定・エラー処理

### 13.1 QoS

| 用途 | 設定 | 理由 |
| --- | --- | --- |
| `active_route` | RELIABLE / TRANSIENT_LOCAL / depth 1 | 起動順に依らず最新経路を受ける |
| `obstacle_avoidance_hint` | BEST_EFFORT / depth 1 | **配信元が BEST_EFFORT のため、RELIABLE 購読では QoS 非互換で接続が成立せず、hint を一切受信できない** |
| 画像・overlay | BEST_EFFORT / depth 1 | 配信元の設定が混在する（生画像は RELIABLE、overlay は BEST_EFFORT）。BEST_EFFORT 購読はどちらとも互換で、表示用途では取りこぼしを許容できる |
| 健全性監視 | BEST_EFFORT / depth 1 | 配信様式 `stream` の既定 |
| その他 | 既定プロファイル / depth 10 | — |

### 13.2 鮮度判定

キー（topic 名や profile_id）ごとに最終受信時刻を保持し、閾値と比較して 4 段階で判定
する。

| 判定 | 条件 |
| --- | --- |
| `OK` | 経過時間が `stale_sec` 以内 |
| `STALE` | `stale_sec` 超、`lost_sec` 以内 |
| `LOST` | `lost_sec` 超 |
| `UNKNOWN` | **一度も受信していない** |

既定は `stale_sec` 1.0 秒、`lost_sec` 3.0 秒である。`health_topics` の閾値は公称レートから
次式で導出する。

```
stale_sec = max(3.0 / rate_hz, 0.5)
lost_sec  = max(10.0 / rate_hz, 2.0)
```

下限を設けるのは、高レートな topic で閾値が過度に短くなり、一時的なスケジューリング遅延で
誤検出することを避けるためである。導出規則は
[トピック通信規約](../../../docs/トピック通信規約.md) 3 章に従う。

自己申告診断は 1 Hz を前提に、`stale_sec` 3.0 秒・`lost_sec` 10.0 秒で判定する。
**ROS 時刻の停止や補正に影響されないよう、受信時刻（monotonic）で監視する。**

### 13.3 健全性の合成

profile ごとに、起動状態・`health_topics` の鮮度・自己申告診断を合成する。規則は
[ノード健全性監視設計](../../../docs/ノード健全性監視設計.md) 4 章・5.5 節に従う。

topic 鮮度の集約は、全部 OK なら OK、1 つでも OK か STALE があれば STALE、すべて LOST
なら起動猶予期間内は UNKNOWN、期間外は LOST とする。**部分的な途絶を STALE として残すの
は、一部センサだけの部分故障を全滅と同じ扱いにせず画面上で区別するためである。** 起動
猶予は既定 10 秒である。

起動状態との合成は次のとおりである。

| 起動状態 | topic 鮮度 | 表示 |
| --- | --- | --- |
| ERROR | — | ERROR / LOST |
| STARTING | — | STARTING / UNKNOWN |
| RUNNING | OK | RUNNING / OK |
| RUNNING | STALE | RUNNING / STALE |
| RUNNING | UNKNOWN | STARTING / UNKNOWN |
| RUNNING | LOST | **ERROR / LOST**（プロセスは生存しているのに出力が無い） |
| STOPPED、STOPPING | OK、STALE | RUNNING（**外部起動として扱う**） |
| STOPPED、STOPPING | UNKNOWN、LOST | STOPPED / UNKNOWN |

診断による補正は、ERROR があれば ERROR / LOST、WARN または STALE があり鮮度が OK なら
STALE へ落とす。**停止と判定した profile には、診断が残っていても稼働文言を表示しない。**

### 13.4 ログとプロセス状態

子プロセスの標準出力・標準エラーをログへ集約し、終了状態を Snapshot へ反映する。
ログはファイルへも保存する。

プロセスの停止は **SIGINT → SIGTERM → SIGKILL** の順に行う。`ros2 launch` 自身へ
SIGINT を送って終了を委譲し、子プロセスへ同時送信はしない。SIGINT が二重になるためである。
待機時間は SIGINT 後 5.0 秒、SIGTERM 後 3.0 秒、SIGKILL 後 1.0 秒である。

## 14. 並行性・スレッド構成

### 14.1 スレッド分離

Qt イベントループ（`ui_qt_main.py`）または HTTP サーバスレッド（`web_main.py`）と、
rclpy executor を**別スレッドで動かす。** `start_ros_thread()` がこの分離を担う。

`ConsoleCore` は複数スレッドから呼ばれる。`FreshnessMonitor` と `LaunchManager` は
内部にロックを持つ。Snapshot は取得時点の値を複製した不変オブジェクトとして返す。

### 14.2 UI 層の制約

**`ui_qt/` の QWidget は ROS 2 の publisher / subscriber を直接持たない。**
`robot_console.core` が提供する Snapshot のみを参照する。Qt の widget 更新は UI 側で
行う。

HTML 側も Snapshot と `ImageStore` を参照し、**独自の ROS 購読を持たない。**

## 15. HTML 遠隔観測 UI

HTML は**読取専用**である。走行指令、起動停止、bag 操作、採取操作を提供しない。
`do_POST` / `do_PUT` / `do_PATCH` / `do_DELETE` は常に 405 を返す。

HTTP サーバは**既定で loopback（127.0.0.1）で待ち受け、認証機能を持たない。**
別端末への公開範囲と手順は [README](../README.md) に従う。**PC 内の URL は遠隔端末から
直接開ける共有 URL ではない。**

画像参照は `ImageStore`、JSON 変換は `web/json_codec.py` で扱う。

## 16. 確認方法と未確認範囲

### 16.1 自動確認の範囲

`tools/tests/` で観測の集約・鮮度・操作接続・起動失敗・ログ・JSON・Qt 表示を確認する。
`tools/check_qt_entry.py` で Qt 入口の起動を確認する。

### 16.2 Qt 試験の分離

Qt を初期化する試験は**別インタプリタのサブプロセスで実行する。** `tools/tests/isolated/`
に置き、`tools/tests/test_isolated_suite.py` が環境変数 `TC2026_RUN_ISOLATED` を立てて
起動する。QtWebEngine が生成するネイティブスレッドが、後続の `os.fork()` を使う試験を
デッドロックさせるためである。規則と検査はワークスペース直下の `conftest.py` を参照する。

### 16.3 ROS 結合確認

ROS を起動する試験は隔離した `ROS_DOMAIN_ID` を使い、ネットワークとデバイスを隔離する。
未受信・途絶・不正値を含めて確認する。

### 16.4 bag 記録の確認

二重開始、別 UI との競合、空き容量不足、起動失敗、停止・保存状態を確認する。記録用 relay
の周期（地図 10 秒、Sensor Viewer 1 秒）、画像データと元時刻の保持、元画像の高頻度配信、
入力停止後の重複抑止を確認する。記録後は `metadata.yaml` と必要トピックのメッセージを
確認する。

### 16.5 GNSS・基地局の確認

不正 JSON、未受信、期限切れ、Qt / HTML 表示を確認する。

### 16.6 機密フィールドの除外

**起動時の上書き入力（`LaunchProfileState.override_inputs`）は HTTP のペイロードに
含めない。** NTRIP のパスワードなど任意の値が入り得るためである。認証ユーザー名・
パスワード・サーバ応答本文は診断 JSON にも含めない。除外を `web/json_codec.py` の試験で
確認する。

### 16.7 未確認範囲

- **画面表示の試験は実機運動の検証を代替しない。**
- 実機センサ一式での帯域、保存媒体の性能、長時間保存は実構成での検証が必要である。
- 模擬 NTRIP 診断の表示と、実 GNSS の測位品質・補正受信は区別する。
- 長時間表示での画像保持のメモリ使用を確認していない。
- `/diagnostics` を配信しないノードが混在する構成での健全性表示を網羅的に確認していない。

## 17. 互換性・移行・影響範囲

購読する topic と型は `tc_route_msgs`、`tc_geo_msgs`、`rtk_gps_um982_msgs`、
`gnss_lio_fusion`、`tc_diagnostics` の定義に依存する。フィールドの変更は ROS 層の変換
処理と Snapshot のデータクラス、Qt と HTML の両表示に同時に影響する。

profile 定義のスキーマを変える場合は `config/node_launch_profiles.yaml` と
`core/launch_profile.py`、起動・設定タブを併せて更新する。

鮮度閾値の導出規則は
[トピック通信規約](../../../docs/トピック通信規約.md)、健全性の合成規則は
[ノード健全性監視設計](../../../docs/ノード健全性監視設計.md)が正である。本書はそれを
実装する側である。

**本書と画面・機能詳細設計書の章番号はコードから参照される**（1 章）。番号の変更は
コードコメントの更新を伴う。

## 18. 未決事項・今後の拡張

### 18.1 画面モデルに存在するが運用判断に使えない値

[画面・機能詳細設計書](robot_console_gui_screen_function_design.md) 11.3 節に挙げた値は、
導出が未実装または既定値のままである。実装するか、Snapshot から削るかの選択が残る。

### 18.2 HTTP サーバに認証がない

15 章のとおり、認証機能を持たない。loopback 以外へ公開する場合の保護は、
[README](../README.md) に記載した外部の手段に依存する。認証を設けるかは未決である。

### 18.3 健全性監視のための重複購読

7.1 節のとおり、`health_topics` の topic を監視用に重複して購読する。購読数と帯域が
増える一方、判定の確実性を優先している。既存購読の受信時刻を共有する仕組みへ変えるかは
未決である。
