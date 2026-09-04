# robot_console GUI改修 アーキテクチャ詳細設計書

## 1. 文書目的・対象範囲

本書は、`robot_console` のGUI改修における実装構造、ROS 2ノードとGUIの役割分担、Snapshot生成、起動管理、GPS/GNSS状態集約、HTML遠隔観測UIとの分担を定義する。

対象は `robot_console` パッケージの正式GUI実装である。既存の `tkinter` GUIは移行期間中の別UIとして残さず、PyQt5 GUIへ完全移行する。完全移行後は `robot_console` の通常entry point、README、launch、評価ツール、設計書の参照先をPyQt5版に統一する。

**実装状態:** 本書が定義する `ConsoleCore`、PyQt5 UI、HTML UIは実装済みであり、正式entry pointは PyQt5 版（`robot_console_qt`）と HTML遠隔観測UI（`robot_console_web`）である。旧 `tkinter` 版（`robot_console`）は当面コードを残すが正式UIとしては扱わず、`robot_console_詳細設計書.md` は旧UIの記録として参照する。

画面配置や業務フロー上の操作仕様は `robot_console_gui_screen_function_design.md` を正とする。本書では、画面を実現するための実装境界、データモデル、将来のLLHベース自己位置・経路への移行前提を定義する。

## 2. 背景・要求・スコープ

現行 `robot_console` は、ROS 2 topic購読、topic送信、状態集約、画像変換、launch管理、ログ収集、GUI描画が `RobotConsoleNode`、`GuiCore`、`UiMain` に密結合している。次期UIでは、PyQt5ローカル操作UIとHTML遠隔観測UIを共通状態から生成するため、UI非依存のCoreを明確に分離する。

主な要求は以下である。

- ローカルPCではPyQt5 GUIで起動設定、ノード起動、ログ確認、運行監視、手動介入を行う。
- HTML UIは遠隔観測専用とし、手動介入や起動停止操作は提供しない。
- 実機・シミュレーションで起動するノードをGUIから管理できるようにする。
- 起動管理対象は今後増減するため、UIコードへ固定リストを埋め込まない。
- GPS/GNSS関連の起動管理と受信状態表示を運行判断に含める。
- 現状は既存互換として `/localization/pose_enu`、`/active_route`、`/active_target` を購読するが、将来は `localization_fusion` の `pose_llh` とLLHベースのroute/waypoint/active_target系topicへ切り替える。
- `tkinter` GUIとの二重保守は行わず、PyQt5 GUIを正式画面として実装・検証・文書化する。

スコープ外は以下である。

- YAMLパラメータファイルのGUI直接編集と保存
- HTML UIからの操作、launch制御、topic送信
- RVizの完全置換
- `gpsd`、`chrony`、`ypspur-coordinator` などOS管理プロセスの直接制御
- `localization_fusion` やLLH route interface自体の設計

## 3. 全体構成・アーキテクチャ

次期構成は、ROS通信、状態集約、UI表示、Web配信を分離する。

```text
ROS 2 topics / services / launch processes
        ↓
RobotConsoleNode
        ↓
ConsoleCore
        ├── StateStore
        ├── CommandQueue
        ├── LaunchProfileStore
        ├── LaunchManager
        ├── LogManager
        ├── ImageStore
        ├── FreshnessMonitor
        ├── LocalizationAdapter
        ├── RouteAdapter
        └── ViewModelBuilder
        ↓
ConsoleSnapshot / ImageReference
        ├── PyQt5 Local UI
        └── HTML Observation UI
```

### 3.1 モジュール責務

| モジュール | 主責務 |
| --- | --- |
| `RobotConsoleNode` | ROS 2 publisher/subscriber/service client/timerを保持し、ROSメッセージをCoreへ渡す。GUI部品を直接参照しない。 |
| `ConsoleCore` | 状態更新、コマンド受付、Snapshot生成、起動管理、ログ管理を束ねるUI非依存のFacade。 |
| `StateStore` | ROS topicから得た最新状態、受信時刻、入力元、派生状態を保持する。 |
| `CommandQueue` | GUI操作から発生したtopic送信、launch操作、override操作をFIFOで保持する。 |
| `LaunchProfileStore` | 起動対象profileを読み込み、カテゴリ、順序、引数、health topicを管理する。 |
| `LaunchManager` | `ros2 launch` プロセス起動、停止、PID、終了コード、stdout/stderr収集を管理する。 |
| `LogManager` | profile別ログ、統合ログ、WARN/ERROR抽出、ログファイルパスを管理する。 |
| `ImageStore` | 画像topicの最新フレーム、encoded bytes、PyQt5向けQImage変換、HTML向け画像参照を管理する。 |
| `FreshnessMonitor` | topicごとの最終受信時刻から `OK / STALE / LOST / UNKNOWN` を判定する。 |
| `LocalizationAdapter` | 現行 `/localization/pose_enu` と将来の `pose_llh` を、GUI用の自己位置Viewへ正規化する。 |
| `RouteAdapter` | 現行 `/active_route`、`/active_target` と将来のLLHベースroute/waypoint topicを、GUI用のroute Viewへ正規化する。 |
| `ViewModelBuilder` | StateStore、FreshnessMonitor、Adapter群からPyQt5/HTML共通のSnapshotを生成する。 |
| PyQt5 UI | Snapshotを描画し、操作をConsoleCoreのAPIへ渡す。ROSメッセージを直接解釈しない。 |
| HTML UI | Snapshot JSONと画像APIから遠隔観測画面を描画する。操作は提供しない。 |

## 4. 完全移行方針

PyQt5 GUIへの完全移行では、`tkinter` GUIを別UIとして残さない。移行完了時の状態は以下とする。

- `robot_console` のentry pointはPyQt5 GUIを起動する。
- `robot_console_node.py`、`gui_core.py`、`ui_main.py` の責務は新構成へ移され、不要な `tkinter` 依存は削除する。
- 評価ツールはPyQt5 GUIまたはConsoleCoreのautomation hookを利用する。
- README、launch、docsはPyQt5 GUIを正式仕様として記載する。
- `tkinter` 版の画面、部品、テストは移行完了PRで削除する。

段階実装中に一時的な互換層を置くことは許容するが、main branch上の最終仕様として二重UIを維持しない。

## 5. パッケージ構成・ファイル配置

正式実装後の構成は以下を基本とする。

```text
robot_console/
  core/
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
    console_node.py
  ui_qt/
    main_window.py
    dashboard_tab.py
    localization_sensor_tab.py
    launch_settings_tab.py
    console_log_tab.py
    widgets/
  web/
    server.py
    static/
```

`ui_tk/` や `tkinter` 版entry pointは正式構成に含めない。移行途中で一時的に残る場合でも、削除対象としてissueまたはPR内で追跡する。

## 6. 位置・経路情報の移行方針

### 6.1 現行互換

現時点では、既存パッケージとの互換性のため以下を購読する。

| 情報 | 現行topic | 型 | 用途 |
| --- | --- | --- | --- |
| 自己位置 | `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | map/odom系の自己位置、目標距離、現行route map表示 |
| active target | `/active_target` | `geometry_msgs/msg/PoseStamped` | 目標waypoint方向、距離、走行状態表示 |
| active route | `/active_route` | `tc_route_msgs/msg/Route` | waypoint列、route画像、進捗表示 |
| route state | `/route_state` | `tc_route_msgs/msg/RouteState` | current index、route version、進捗 |

この互換は、画面仕様の正本ではなく、移行期間中の入力sourceである。UIやCoreの内部Viewは、将来のLLH入力へ差し替えられるようtopic型へ密結合させない。

### 6.2 将来前提

将来は、自己位置は `localization_fusion` パッケージの `pose_llh` を正とする。topic名は実装時の `localization_fusion` 設計に従うが、`robot_console` 側では `topics.localization.pose_llh` として設定可能にする。既定候補は `/localization_fusion/pose_llh` とする。

route、waypoint、active targetも同様に、現行のmap/ENU/PoseStamped前提からLLHベースのtopicへ切り替える。topic名と型は将来のroute系interface確定後に合わせるが、`robot_console` 側では以下の抽象Viewを維持する。

| View | 必須情報 |
| --- | --- |
| `LocalizationView` | source、LLH、現行互換pose、yaw、covariance、freshness、frame_id、timestamp |
| `RouteView` | route id、version、waypoint列、current waypoint、target waypoint、走行済み/未走行区間、座標種別 |
| `TargetView` | target id、LLH、現行互換pose、距離、bearing、freshness |
| `MapOverlayView` | 地図表示用polyline、marker、軌跡、座標系メタデータ |

### 6.3 Adapterの責務

`LocalizationAdapter` は以下を行う。

- `/localization/pose_enu` を現行互換sourceとして `LocalizationView` へ変換する。
- 将来の `pose_llh` を正sourceとして `LocalizationView` へ変換する。
- 両方を受信している場合、設定に従って正sourceと比較sourceを分ける。
- 自己位置の鮮度、座標種別、信頼度をViewへ付与する。

`RouteAdapter` は以下を行う。

- 現行 `/active_route` と `/active_target` をroute/target Viewへ変換する。
- 将来のLLH route/target topicをroute/target Viewへ変換する。
- UIに座標変換規約を持たせず、地図描画に必要なoverlay情報を生成する。

### 6.3.1 ENU⇔LLH変換の集約先

地図表示（PyQt5 `MapView` / HTML観測UI）は緯度経度を必要とするが、ENU⇔LLH変換は
`robot_console` では行わない。変換は `geo_pose_converter`（`route_geo_projector_node`）へ
集約し、`robot_console` は変換済みのLLH topicを購読するだけとする。

| 用途 | 購読するtopic | 配信元 |
| --- | --- | --- |
| 自己位置 | `/localization/pose_llh` | `geo_pose_converter` |
| 目標waypoint | `/route/active_target_llh` | `geo_pose_converter` |
| route waypoint列 | `/active_route` の `waypoints[].geo_pose` | `route_planner`（route file由来） |

この方針の根拠は以下である。

- `tc_route_msgs/ActiveTargetLlh` は「GUI・HTML UI・ログ向け」と定義されており、GUIがLLHを
  受け取る前提で設計されている（制御用のENU目標は `/active_target` のまま維持する）。
- 投影パラメータの解釈（原点、`map_yaw_offset_rad`、datum）を複数パッケージへ分散させると、
  投影設定の変更時に不整合が生じる。

したがって地図表示を行う構成では `geo_pose_converter` profileの起動が前提となる。同profileは
業務モードプリセット（自律走行系）へ含める。GNSS実機を伴わない構成では
`enable_geo_pose_converter:=false` とし、ENU→LLH変換を行う `route_geo_projector_node` のみ起動する。

### 6.4 移行時の表示方針

- 現行sourceのみの場合、画面には `Localization source: pose_enu` と表示する。
- `pose_llh` が有効な場合、画面には `Localization source: localization_fusion pose_llh` と表示する。
- 両sourceがある場合、自己位置・センサ情報タブで差分を表示する。
- route/targetがLLHへ移行した後も、ダッシュボードの運行状態表示は同じViewを参照するため画面構成を変更しない。

## 7. 外部インタフェース仕様

### 7.1 購読トピック

| トピック | 型 | 状態 | 用途 |
| --- | --- | --- | --- |
| `/route_state` | `tc_route_msgs/msg/RouteState` | 現行 | route_manager状態、進捗、route version表示 |
| `/manager_status` | `tc_route_msgs/msg/ManagerStatus` | 現行 | route_manager FSM、再計画理由、異常状態表示 |
| `/active_route` | `tc_route_msgs/msg/Route` | 現行互換 | waypoint列、route map、現在/目標waypoint表示 |
| `/active_target` | `geometry_msgs/msg/PoseStamped` | 現行互換 | 目標姿勢、自己位置との距離計算 |
| `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 現行互換 | 自己位置、地図表示、localization鮮度 |
| `/localization_fusion/pose_llh` | TBD | 将来正本 | LLHベース自己位置、地図表示、自己位置品質表示 |
| LLH route topic | TBD | 将来正本 | LLH waypoint列、route overlay、進捗表示 |
| LLH active target topic | TBD | 将来正本 | LLH target、距離、bearing表示 |
| `/mission_info` | `tc_route_msgs/msg/MissionInfo` | 現行 | mission概要、route設定確認 |
| `/follower_state` | `tc_route_msgs/msg/FollowerState` | 現行 | route_follower状態、停止理由、追従進捗表示 |
| `/odom` | `nav_msgs/msg/Odometry` | 現行 | 速度・自己位置補助、シミュレーション状態確認 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 現行 | 最終速度指令表示 |
| `/cmd_vel/autonomous` | `geometry_msgs/msg/Twist` | 現行 | 自律速度指令とmux後速度の比較 |
| `/drive_mode_status` | `tc_route_msgs/msg/DriveModeStatus` | 現行 | 自律/手動モード、mux状態、手動介入状態 |
| `/obstacle_avoidance_hint` | `tc_route_msgs/msg/ObstacleAvoidanceHint` | 現行 | 障害物状態、固定override表示 |
| `/sensor_viewer` | `sensor_msgs/msg/Image` | 現行 | 障害物/センサビュー表示 |
| `/perception/road_blockage/decision_image` | `sensor_msgs/msg/Image` | 現行 | 道路封鎖判定画像表示 |
| `/perception/traffic_signal/decision_image` | `sensor_msgs/msg/Image` | 現行 | 信号認識画像表示 |
| `/route/active_target_llh` | `tc_route_msgs/msg/ActiveTargetLlh` | 現行 | 地図表示用の目標LLH、目標距離・bearing |
| `/manual_start` | `std_msgs/msg/Bool` | 現行 | 手動開始状態、送信結果確認 |
| `/sig_recog` | `std_msgs/msg/Int32` | 現行 | 信号GO/STOP状態、送信結果確認 |
| `/road_blocked` | `std_msgs/msg/Bool` | 現行 | 道路封鎖状態、入力元表示 |
| `/rtk_gps/fix` | `sensor_msgs/msg/NavSatFix` | 現行 | 緯度経度高度、GPS fix鮮度、位置共分散表示 |
| `/rtk_gps/heading` | `sensor_msgs/msg/Imu` | 現行 | デュアルアンテナheading鮮度、姿勢補助 |
| `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/msg/RtkStatus` | 現行 | RTK種別、衛星数、HDOP、補正age、RTCM受信量表示 |

### 7.2 発行トピック

| トピック | 型 | 送信契機 |
| --- | --- | --- |
| `/manual_start` | `std_msgs/msg/Bool` | ダッシュボードのmanual_start送信 |
| `/sig_recog` | `std_msgs/msg/Int32` | ダッシュボードの信号GO/STOP送信 |
| `/road_blocked` | `std_msgs/msg/Bool` | ダッシュボードの道路封鎖状態送信 |
| `/obstacle_avoidance_hint` | `tc_route_msgs/msg/ObstacleAvoidanceHint` | 障害物hint固定送出開始/停止 |
| `/frame_image_path` | `std_msgs/msg/String` | 静止画入力確認用の画像パス送信 |

## 8. Snapshot / ViewModel設計

UIはROSメッセージ型を直接参照せず、以下のSnapshotを読む。

```text
ConsoleSnapshot
  timestamp
  operation_state
  gps_state
  localization_state
  route_state
  target_state
  follower_state
  obstacle_state
  drive_mode_state
  sensor_panels
  event_banners
  manual_controls
  launch_profiles
  logs
  health
```

共通ステータスバー用の専用Viewは持たない。ダッシュボードは詳細な `operation_state` と関連Viewを表示し、自己位置・センサ情報タブは必要なサマリだけを同じSnapshotから表示する。

### 8.1 GPS状態モデル

`GpsStateView` は以下を保持する。

| field | 内容 |
| --- | --- |
| `rtk_state` | `UNKNOWN / STANDALONE / DGPS / RTK_FLOAT / RTK_FIX` |
| `rtk_state_raw` | `RtkStatus.rtk_state_raw` |
| `num_satellites` | 衛星数 |
| `hdop` | 水平精度劣化率 |
| `correction_age_s` | RTCM補正age |
| `rtcm_bytes_received` | RTCM累計受信bytes |
| `heading_deg` | 真北基準CWのheading |
| `heading_stddev_deg` | heading標準偏差 |
| `baseline_length_m` | デュアルアンテナbaseline |
| `latitude` / `longitude` / `altitude` | 参照用の緯度経度高度 |
| `fix_freshness` | `/rtk_gps/fix` の鮮度 |
| `heading_freshness` | `/rtk_gps/heading` の鮮度 |
| `status_freshness` | `/rtk_gps/rtk_status` の鮮度 |
| `display_level` | `NORMAL / NOTICE / WARN / ERROR / UNKNOWN` |

### 8.2 画像参照モデル

Snapshotには画像本体を直接含めず、`ImageReference` を含める。

```text
ImageReference
  panel_id
  title
  topic
  image_id
  width
  height
  updated_at
  freshness
```

PyQt5 UIは `ImageStore.get_qimage(image_id)`、HTML UIは `GET /images/{panel_id}` で画像を取得する。

## 9. 起動管理設計

### 9.1 profile定義駆動

起動管理対象はコード内の固定リストではなく、profile定義から生成する。profileはUI生成に必要な情報を持つ。

```yaml
profiles:
  - profile_id: rtk_gps_um982
    category: gps_gnss
    display_name: RTK GPS UM982
    package: rtk_gps_um982
    launch_file: rtk_gps_um982.launch.py
    param_argument: config
    default_param: config/default.yaml
    launch_order: 20
    startup_group: real_robot_base
    health_topics:
      - /rtk_gps/fix
      - /rtk_gps/heading
      - /rtk_gps/rtk_status
```

profile追加時に必要なUI変更は原則不要とし、カテゴリ別カード、引数入力欄、ログ欄、health表示はprofile定義から自動生成する。

profile定義は、実運用launchに加えて以下の代替launchを保持できる。

| フィールド | 用途 |
| --- | --- |
| `alternate_launch_file` | 同一profile内で切り替え可能な別実装launch（例: `road_blockage_detector`のPyTorch版YOLO） |
| `simulator_package` / `simulator_launch_file` | 実センサ・実機基盤（GPS、ypspur、Gazebo）を使わずに疑似データで単体確認するための代替launch |

`simulator_launch_file`を持つprofileは、起動・設定タブのノード設定編集パネルに「Simulator代替を使用」トグルを持つ。トグルON時は`launch_file`の代わりに`simulator_package`/`simulator_launch_file`を起動する。この仕組みは、実機・Gazeboのどちらも使わない机上確認（10章参照）を成立させるために必須である。

```yaml
  - profile_id: robot_navigator
    category: drive_stack
    package: robot_navigator
    launch_file: robot_navigator.launch.py
    simulator_package: robot_navigator
    simulator_launch_file: robot_simulator.launch.py
    startup_group: desktop_check
```

`yolo_detector`は`road_blockage_detector`/`traffic_signal_recognizer`のlaunch内で直接Nodeとして起動され、単体のlaunchファイルとしては呼び出されないため、独立したprofileとしては扱わない。ただし`yolo_detector/camera_simulator_node.launch.py`は、`road_blockage_detector`・`traffic_signal_recognizer`・（将来の実カメラ導入時の）該当profileの`simulator_launch_file`として利用する。

### 9.2 起動カテゴリ

| category | 用途 | 代表profile |
| --- | --- | --- |
| `real_robot_base` | 実機基盤 | `ypspur_ros2` |
| `simulation_base` | Gazebo/疑似センサ基盤 | `obstacle_route_sim` |
| `gps_gnss` | GPS/GNSS受信 | `rtk_gps_um982` |
| `route_stack` | 経路計画・追従 | `route_planner`, `route_manager`, `route_follower` |
| `drive_stack` | 走行制御・mux | `drive_mode_manager`, `robot_navigator` |
| `obstacle_stack` | 障害物監視 | `obstacle_monitor` |
| `perception_stack` | 認識・判定 | `road_blockage_detector`, `traffic_signal_recognizer` |
| `visualization` | RViz、経路・目標のMarker表示 | `robot_console_rviz`, `route_markers`, `target_marker` |

### 9.3 既定profile

| profile_id | package | launch | 主な引数 |
| --- | --- | --- | --- |
| `ypspur_ros2` | `ypspur_ros2` | `ypspur_ros2.launch.py` | `config`, `cmd_vel_topic`, `odom_topic`, `start_coordinator`, `coordinator_device`, `coordinator_param` |
| `rtk_gps_um982` | `rtk_gps_um982` | `rtk_gps_um982.launch.py` | `config` |
| `obstacle_route_sim` | `obstacle_route_sim` | `sim_obstacle_route.launch.py` | `road_type`, `road_width`, `enable_pylons`, `pylon_seed`, `start_gazebo_gui`, `use_sim_time` |
| `route_planner` | `route_planner` | `route_planner.launch.py` | `param_file` |
| `route_manager` | `route_manager` | `route_manager.launch.py` | `param_file`, `start_label`, `goal_label`, `checkpoint_labels` |
| `route_follower` | `route_follower` | `route_follower.launch.py` | `param_file` |
| `drive_mode_manager` | `drive_mode_manager` | `drive_mode_manager.launch.py` | `start_gui`, `joy_input` |
| `robot_navigator` | `robot_navigator` | `robot_navigator.launch.py`（simulator代替: `robot_simulator.launch.py`） | `param_file`, `cmd_vel_topic`, `odom_topic` |
| `obstacle_monitor` | `obstacle_monitor` | `obstacle_monitor.launch.py`（simulator代替: `laser_scan_simulator.launch.py`） | `param_file` |
| `road_blockage_detector` | `road_blockage_detector` | `road_blockage_perception.launch.py`（alternate launch: `road_blockage_perception_yolo.launch.py`、simulator代替: `yolo_detector/camera_simulator_node.launch.py`） | `detector_param_file` |
| `traffic_signal_recognizer` | `traffic_signal_recognizer` | `traffic_signal_perception.launch.py`（simulator代替: `yolo_detector/camera_simulator_node.launch.py`） | `recognizer_param_file` |
| `route_markers` | `route_manager` | `active_route_marker.launch.py` | `active_route_topic`, `marker_topic` |
| `target_marker` | `route_follower` | `active_target_marker.launch.py` | `active_target_topic`, `marker_topic` |
| `robot_console_rviz` | `robot_console` | `robot_console_rviz.launch.py` | なし（固定config `rviz/robot_console_view.rviz`） |

`rtk_gps_um982_msgs` はmsg定義パッケージであり、起動対象profileには含めない。

## 10. 起動グループ

実機自律走行の基本グループ:

```text
rtk_gps_um982
ypspur_ros2
drive_mode_manager
route_planner
route_manager
route_follower
obstacle_monitor
robot_navigator
road_blockage_detector
traffic_signal_recognizer
```

実機手動走行の基本グループ:

```text
ypspur_ros2
drive_mode_manager
```

シミュレーション自律走行の基本グループ:

```text
obstacle_route_sim
drive_mode_manager
route_planner
route_manager
route_follower
obstacle_monitor
robot_navigator
```

シミュレーション手動走行の基本グループ:

```text
obstacle_route_sim
drive_mode_manager
```

机上確認（実センサ・Gazebo無し）の基本グループ:

```text
route_planner
route_manager
route_follower
drive_mode_manager (joy_input=ps3_joy_sim)
robot_navigator (simulator代替使用)
obstacle_monitor (simulator代替使用)
road_blockage_detector (simulator代替使用)
traffic_signal_recognizer (simulator代替使用)
```

机上確認は、`ypspur_ros2`・`rtk_gps_um982`・`obstacle_route_sim`のいずれも起動せず、各profileのsimulator代替launchのみで自己位置・LaserScan・カメラ画像を疑似生成して確認する環境である。

上記いずれのグループにも`robot_console_rviz`、`route_markers`、`target_marker`は含めていない。可視化系profileは業務モードによらず任意追加可能なオプション扱いとし、必要な業務でユーザーが個別に起動候補ツリーから追加する。

## 11. パラメータ・設定仕様

`robot_console` 側の主要設定は以下とする。

| 設定 | 既定値 | 内容 |
| --- | --- | --- |
| `ui.refresh_period_ms` | `200` | PyQt5 UIがSnapshotを取得する周期 |
| `freshness.default.stale_sec` | `1.0` | 通常topicのSTALE閾値 |
| `freshness.default.lost_sec` | `3.0` | 通常topicのLOST閾値 |
| `freshness.gps.status.stale_sec` | `3.0` | `/rtk_gps/rtk_status` のSTALE閾値 |
| `freshness.gps.status.lost_sec` | `10.0` | `/rtk_gps/rtk_status` のLOST閾値 |
| `topics.localization.compat_pose_enu` | `/localization/pose_enu` | 現行互換自己位置topic |
| `topics.localization.pose_llh` | `/localization_fusion/pose_llh` | 将来正本のLLH自己位置topic候補 |
| `topics.gps.fix` | `/rtk_gps/fix` | GPS fix topic |
| `topics.gps.heading` | `/rtk_gps/heading` | GPS heading topic |
| `topics.gps.status` | `/rtk_gps/rtk_status` | RTK status topic |
| `launch.profile_file` | `config/node_launch_profiles.yaml` | 起動profile定義 |
| `logs.buffer.max_lines` | `2000` | profile別ログ保持行数 |

## 12. 処理フロー・状態遷移

### 12.1 起動時

1. `RobotConsoleNode` が起動する。
2. `ConsoleCore` がprofile定義とUI設定を読み込む。
3. `RobotConsoleNode` が設定済みtopicを購読し、publisherを生成する。
4. PyQt5 UIが起動し、200ms周期でSnapshotを取得する。
5. HTML遠隔観測UIを有効化した場合、HTTP serverがSnapshot JSONと画像APIを提供する。

### 12.2 GPS状態更新

1. `/rtk_gps/rtk_status` を受信する。
2. `StateStore` がRTK種別、衛星数、HDOP、補正age、RTCM bytes、緯度経度高度を更新する。
3. `FreshnessMonitor` がstatus/fix/headingの鮮度を更新する。
4. `ViewModelBuilder` が `GpsStateView` を生成する。
5. PyQt5ダッシュボード、自己位置・センサ情報タブ、HTML観測UIに反映する。

### 12.3 位置・経路状態更新

1. 現行互換では `/localization/pose_enu`、`/active_route`、`/active_target` を受信する。
2. `LocalizationAdapter` と `RouteAdapter` がUI用Viewへ変換する。
3. 将来 `pose_llh` とLLH route topicが有効な場合は、それらを正sourceとしてViewを生成する。
4. ダッシュボードは運行判断に必要な距離、waypoint、進捗のみを表示する。
5. 自己位置・センサ情報タブとHTML UIは地図overlay、GPS、localization品質を表示する。

## 13. QoS・並行性・タイミング設計

- 購読QoSは配信側ノードのQoSに合わせる。QoS非互換の購読は接続自体が成立せず無言で受信ゼロになるため、以下を購読側の既定とする。
  - `/active_route`: route_managerがTransient Local（ラッチ）で配信するため、`RELIABLE` / `TRANSIENT_LOCAL` / `depth=1` で購読する（VOLATILE購読では起動順によって初回Routeを取り逃す）。
  - `/obstacle_avoidance_hint`: obstacle_monitorがBEST_EFFORTで配信するため、`BEST_EFFORT` で購読する。
  - 画像topic（`/sensor_viewer`、各 `decision_image`）: 配信側のreliabilityがノードごとに異なるため、双方と互換な `BEST_EFFORT` で一律購読する（表示用途であり取りこぼしを許容する）。
  - 上記以外のストリーム系topicは `RELIABLE` / `VOLATILE` / `depth=10` を既定とする。
- ROS callbackはCoreのスレッド安全APIへ状態を投入し、GUI部品を直接更新しない。
- PyQt5 GUIはQt main thread上でのみwidgetを更新する。
- Snapshot生成時はStateStoreを短時間lockし、UI側では読み取り専用データとして扱う。
- 画像はImageStoreに保持し、Snapshotには参照情報だけを含める。
- GPS/GNSS topicは受信周期が10Hz程度のため、GUI表示は5Hz以下へ間引いてよい。
- 将来の `pose_llh` と現行 `/localization/pose_enu` を同時購読する期間は、sourceごとの鮮度を別々に保持する。
- `odom` は実機（`ypspur_ros2` 既定: `/odom`）とシミュレーション評価構成（`node_launch_profiles.yaml` の `robot_navigator` プロファイル既定: `/ypspur_ros/odom`）とで既定トピック名が異なるため、`launch/robot_console.launch.py` は `odom_topic` launch引数でremapできるようにする（`ros2 run` 経由で起動する場合は `--ros-args -r odom:=<実際のtopic>` で同様に上書きできる）。`drive_mode_status` / `cmd_vel` / `cmd_vel/autonomous` も同様にlaunch引数でremapできるようにし、他の購読topicと同じ扱いとする。

## 14. エラー処理・ログ・診断

| 異常 | 表示・処理 |
| --- | --- |
| GPS未受信 | ダッシュボードGPSカードを灰または赤にし、HTML UIにも `GPS LOST` を表示する。 |
| RTKがFixからFloat/Standaloneへ低下 | GPSカードで警告表示し、ログへ状態遷移を残す。 |
| RTCM bytesが増えない | GPS詳細表示に `RTCM no update` を表示する。 |
| correction ageが閾値超過 | `RTK correction stale` と表示する。 |
| `/localization/pose_enu` 未受信 | 現行互換source lostとして表示する。将来 `pose_llh` が有効なら正source表示を継続する。 |
| `pose_llh` 未受信 | LLH source lostとして表示し、設定により `/localization/pose_enu` 互換表示へfallbackする。 |
| launch開始失敗 | profileカードをERRORにし、stderr先頭行とログファイルへの参照を出す。 |
| profile定義不正 | 起動・設定タブ上部にprofile validation errorを表示し、該当カードを無効化する。 |

## 15. HTML遠隔観測UI

HTML UIは観測専用で、PyQt5 UIと同じSnapshotを利用する。

提供API候補:

| API | 内容 |
| --- | --- |
| `GET /snapshot.json` | 状態サマリ、GPS状態、localization、route、sensor panel、health |
| `GET /images/{panel_id}` | 最新画像 |
| `GET /health.json` | topic鮮度、profile稼働状態 |

HTML UIには操作APIを提供しない。自己位置は現行 `/localization/pose_enu` 互換表示から開始し、将来は `pose_llh` とLLH route/targetを主表示へ切り替える。

## 16. テスト計画・受け入れ条件

設計実装時の確認項目は以下である。

- `ConsoleCore` はROSなしでSnapshot生成単体テストができる。
- GPS未受信、RTK_FIX、RTK_FLOAT、STANDALONE、補正age超過、RTCM bytes停止を単体テストで確認する。
- `/localization/pose_enu` sourceと将来 `pose_llh` sourceをAdapterで切り替えられる。
- profile定義に新規ノードを追加した場合、起動・設定タブとログタブに自動表示される。
- `rtk_gps_um982` profileの `config:=...` が正しいlaunch引数として生成される。
- HTML Snapshot JSONに操作APIや秘匿値が含まれない。
- PyQt5 UIはROS callback threadから直接widget更新しない。
- `tkinter` 依存が正式entry pointから除去されている。

## 17. 互換性・移行・影響範囲

- 既存topic名は現行互換sourceとして維持し、launch remapまたは設定で変更可能にする。
- 自己位置とroute/targetはAdapter経由で扱い、将来のLLH topic移行時に画面コードを大きく変更しない。
- 既存 `NodeLaunchProfile` は新profile定義へ移行するが、profile_idは可能な限り維持する。
- `rtk_gps_um982` の追加により、`robot_console` は `rtk_gps_um982_msgs` を実行時依存に追加する必要がある。
- `tkinter` GUIは正式移行後に削除するため、既存のtkinter向けautomationはPyQt5/Core向けへ更新する。

## 18. 未決事項・今後の拡張

- `localization_fusion/pose_llh` の正確なtopic名と型は、`localization_fusion` 実装時に確定する。
- LLHベースroute/waypoint/active_targetのtopic名と型は、route系interface拡張時に確定する。
- `gpsd`、`chrony` の状態をGUIへ表示するかは、OS管理方法が固まった後に別profileまたはhealth checkとして検討する。
- 複数GPS受信機を扱う場合は、GPS profileを複数インスタンス化できるよう `instance_id` を追加する。

## 19. 改版履歴

| 日付 | 版 | 変更概要 |
| --- | --- | --- |
| 2026-08-29 | 0.3 | simulator代替launch（`robot_simulator` / `laser_scan_simulator` / `camera_simulator_node`）と可視化profile（`route_markers` / `target_marker`）をprofile定義・起動グループへ追加。机上確認（実センサ・Gazebo無し）の起動グループを新設。 |
| 2026-05-28 | 0.2 | tkinterを別UIとして残さない完全移行方針、`localization_fusion/pose_llh` とLLH route/targetへの将来移行前提を反映。 |
| 2026-05-27 | 0.1 | UI改修向けアーキテクチャ、GPS/GNSS起動管理、profile定義駆動方針を初版として作成。 |
