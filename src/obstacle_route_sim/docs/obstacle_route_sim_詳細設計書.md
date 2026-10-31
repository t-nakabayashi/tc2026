# obstacle_route_sim 詳細設計書

## 1. 目的・スコープ

本書は `obstacle_route_sim` パッケージの設計を定める。対象は Ubuntu 24.04、ROS 2 Jazzy、
Gazebo Harmonic、`ros_gz` を前提とする障害物付き経路追従検証環境である。

道路コース、差動二輪ロボット、2D LiDAR、3D LiDAR、odometry、fake localization pose、
waypoint CSV 生成、pylon 障害物配置を提供し、既存の `route_planner`、`route_manager`、
`route_follower`、`robot_navigator`、`obstacle_monitor`、`drive_mode_manager` を Gazebo 上で
接続できる状態にする。

Gazebo Classic、`gazebo_ros_pkgs`、`gazebo_msgs/srv/SpawnEntity`、Classic plugin は
**対象から除外し、** Harmonic の SDF system plugin と `ros_gz_bridge` で置き換える。

非スコープは、自己位置推定精度の再現と Livox Mid-360 の非反復走査特性の厳密な再現である。
`fake_pose_enu.py` は Gazebo 上の robot モデル真値 pose を `/localization/pose_enu` として
配信し、**localization 誤差がない前提で route stack を評価する。**

関連する検証構成は個別の文書を正とする。

| 文書 | 内容 |
| --- | --- |
| [試験環境](terrain3dとi-Cart miniの試験環境.md) | terrain3d の生成・車体前提 |
| [全域モデル](つくば2026の地理モデル.md) | 地理入力 |
| [FAST-LIO入力](FAST-LIOの模擬入力と確認方法.md) | LiDAR/IMU 接続 |
| [地図評価](FAST-LIO点群地図の保存と評価.md) | 推定地図の整合 |
| [誤差モデル](FAST-LIOのセンサ誤差モデル.md) | LIO への雑音・欠測・bias 注入 |
| [地理地図と模擬GNSS](地理地図と模擬GNSS.md) | 模擬 GNSS |
| [比較方法](地理ウェイポイント・FLOAT・FAST-LIOの比較方法.md) | 精度比較の手順 |
| [LiDAR取付角](LiDAR取付角と地面観測.md) | 取付角と地面観測 |

## 2. 構成とファイル配置

`sim_obstacle_route.launch.py` は Gazebo 単体確認用の起動入口である。launch は道路種別と
道幅から world template を選び、robot include と seed 指定 pylon include を追加した一時
world を生成し、`ros_gz_sim` で起動する。

`gazebo_obstacle_route_stack.launch.py` は統合確認用 launch であり、Gazebo、bridge、
fake localization pose、TF、`route_planner`、`route_manager`、`route_follower`、
`obstacle_monitor`、`robot_navigator`、`ps3_joy_sim_node`、`manual_teleop_node`、
`drive_cmd_mux_node`、任意の drive status GUI を同時起動する。

| 要素 | 役割 |
| --- | --- |
| `worlds/templates/*.world` | Gazebo Harmonic の base world |
| `models/robot/model.sdf` | DiffDrive、2D LiDAR、3D LiDAR、フロントカメラを持つ差動二輪ロボット |
| `models/pylon/model.sdf` | 静的 pylon 障害物 |
| `scripts/road_geometry.py` | 道路中心線と幅キーの共通定義 |
| `scripts/generate_pylon_world.py` | 起動前 world 生成 |
| `scripts/generate_waypoints.py` | `route_planner` 互換 waypoint CSV 生成 |
| `tools/generate_route_assets.py` | 各 world 用 route CSV と `route_planner` / `route_manager` params 生成 |
| `launch/gazebo_obstacle_route_stack.launch.py` | Gazebo と route stack の統合確認 launch |
| `scripts/fake_pose_enu.py` | Gazebo 真値 pose から `/localization/pose_enu` 互換 pose を生成 |
| `scripts/odom_tf_broadcaster.py` | bridged odom から `odom -> base_link` TF を publish |

`tools/` には評価スクリプト（`evaluate_terrain_trial.py`、`review_fusion_trial.py` など）を
置く。模擬 GPS と評価ツールは本パッケージ内に置き、`gnss_lio_fusion` には置かない。
**生成物は Git に追加しない。**

## 3. 外部インタフェース仕様

既存 route stack との接続を優先し、ROS 側 topic は互換名を基本とする。

### 3.1 トピックと bridge の方向

| Topic | Type | 方向 | 内容 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 -> Gazebo | DiffDrive 速度指令 |
| `/ypspur_ros/odom` | `nav_msgs/msg/Odometry` | Gazebo -> ROS 2 | Gazebo wheel odometry |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> ROS 2 | 2D LiDAR |
| `/mid360/livox/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo -> ROS 2 | Gazebo native GPU LiDAR 点群 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS 2 | simulation time |
| `/gazebo/dynamic_pose_info` | `tf2_msgs/msg/TFMessage` | Gazebo -> ROS 2 | Gazebo dynamic entity pose info |
| `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | ROS 2 | Gazebo 真値に基づく fake localization pose |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS 2 | frame tree |

**bridge は topic 所有者を明確にするため方向指定を使う。** `/cmd_vel` は ROS 2 -> Gazebo
のみ、`/clock`、`/ypspur_ros/odom`、`/scan`、`/mid360/livox/lidar/points`、
`/world/<world_name>/dynamic_pose/info` は Gazebo -> ROS 2 のみとする。

Gazebo dynamic pose info は ROS 側で `/gazebo/dynamic_pose_info` へ remap し、
`fake_pose_enu.py` が先頭の robot model pose を抽出して `/localization/pose_enu` を
publish する。

**3D LiDAR は Gazebo の sensor base topic ではなく `/mid360/livox/lidar/points` を
`gz.msgs.PointCloudPacked` として bridge する**（7.3 節）。

`robot_navigator` を統合する場合は、既存方針どおり `robot_navigator` の出力を
`/cmd_vel/autonomous` とし、`drive_cmd_mux_node` が最終 `/cmd_vel` を publish する。

### 3.2 画面・可視化

Gazebo GUI の起動は `start_gazebo_gui` で選ぶ。drive status GUI の起動は
`start_drive_status_gui` で選ぶ。いずれも確認用であり、走行判断には関与しない。

GUI 確認では、車体上面の支持台へ MID-360 が載り、**センサだけが浮いて見えないこと**を
確認する（9.2 節）。

## 4. 設定の所在

`sim_obstacle_route.launch.py` の launch 引数は次のとおりである。

| 引数 | 既定値 | 内容 |
| --- | --- | --- |
| `road_type` | `crank` | `straight`, `crank`, `scurve` |
| `road_width` | `5.0` | `straight` は 2/3/5m、`crank` と `scurve` は 3/5m |
| `enable_pylons` | `true` | pylon include 生成の有無 |
| `pylon_seed` | `0` | pylon 配置の再現 seed |
| `use_sim_time` | `true` | ROS ノードの simulation time 使用 |
| `spawn_robot` | `true` | robot include 生成の有無 |
| `robot_x`, `robot_y`, `robot_z` | `1.0`, `0.0`, `0.16` | robot 初期 pose |
| `generated_world_dir` | `/tmp/obstacle_route_sim` | 生成 world 出力先 |
| `start_gazebo_gui` | `true` | Gazebo GUI 起動の有無 |

`gazebo_obstacle_route_stack.launch.py` は上記に加えて次を持つ。

| 引数 | 既定値 | 内容 |
| --- | --- | --- |
| `start_drive_status_gui` | `true` | drive status GUI 起動の有無 |
| `generated_route_dir` | `<generated_world_dir>/routes` | 生成 waypoint CSV の出力先 |
| `generated_config_dir` | `<generated_world_dir>/config` | 生成 route config の出力先 |
| `waypoint_step` | `3.0` | 生成 waypoint の間隔 [m] |
| `enable_route_blocker` | `true` | 経路中心線上の決定的 blocker pylon 生成の有無 |
| `route_blocker_distance` | `8.0` | blocker pylon を置く経路始点からの距離 [m] |

### 4.1 挙動に影響しない引数

`keep_generated_world` と `use_compat_remap` は**宣言されるが、挙動の切替には使われない。**
トピック名は `/scan`、`/cmd_vel`、`/ypspur_ros/odom` へ直接 bridge する。

### 4.2 simulation time の扱い

**route stack 側の ROS ノードは `use_sim_time=false` 固定で起動する。** Gazebo 側の
`use_sim_time` とは別であり、launch 引数では切り替えられない。

統合確認 launch では、Gazebo / bridge / fake localization pose は simulation time を扱う
一方、route stack 側は**初期 route request timer の停止を避けるため既定で wall time と
する。**

## 5. データモデル・内部状態

道路中心線は `road_geometry.py` を正とし、道路モデル生成、waypoint 生成、pylon 配置で
共用する。

waypoint CSV は `route_planner` の現行 parser が扱う列に合わせ、**simulation では
`latitude` と `longitude` を空欄、`node=-1` とする。**

`tools/generate_route_assets.py` は、既定で次の 7 world 分の route 資産を生成する。

| route id | road_type | road_width | waypoints | start_label | goal_label |
| --- | --- | --- | --- | --- | --- |
| `straight_w2` | `straight` | `2.0` | 21 | `0` | `20` |
| `straight_w3` | `straight` | `3.0` | 21 | `0` | `20` |
| `straight_w5` | `straight` | `5.0` | 21 | `0` | `20` |
| `scurve_w3` | `scurve` | `3.0` | 28 | `0` | `27` |
| `scurve_w5` | `scurve` | `5.0` | 28 | `0` | `27` |
| `crank_w3` | `crank` | `3.0` | 33 | `0` | `32` |
| `crank_w5` | `crank` | `5.0` | 33 | `0` | `32` |

生成物は `route_planner/routes/obstacle_route_sim/<route_id>/`、
`route_planner/params/obstacle_route_<route_id>.yaml`、
`route_manager/params/obstacle_route_<route_id>.yaml` に配置する。

**`route_planner` 側の path は package share からの相対 path とし、リポジトリ固有の絶対
path を params に持たせない。** `robot_console` は各 package の `params` を起動カードの
候補として自動検出するため、`obstacle_route_<route_id>.yaml` を `route_planner` と
`route_manager` の両方で選択すれば、該当 world の start-to-goal route を使用できる。

## 6. 処理フローと異常時の挙動

### 6.1 起動時の world 生成

1. 道路種別と道幅から world template を選ぶ。
2. robot include と pylon include を追加した一時 world を生成する。
3. `ros_gz_sim` で Gazebo を起動する。
4. bridge を方向指定で張る（3.1 節）。
5. 統合確認 launch では、`route_data_dir` 配下へ Gazebo 道路中心線と一致する
   `route_config.yaml` と CSV を生成し、`route_planner` と `route_manager` へ一時
   parameter YAML として渡す。

**pylon は Gazebo 起動後に service spawn せず、起動前に world へ `<include>` を追加する。**

### 6.2 TF の構成

```text
map
└── odom
    └── base_link
        ├── laser
        └── mid360_frame
```

| 変換 | 生成元 |
| --- | --- |
| `map -> odom` | static zero transform |
| `odom -> base_link` | `/ypspur_ros/odom` を `odom_tf_broadcaster.py` が TF 化 |
| `base_link -> laser`、`base_link -> mid360_frame` | SDF link pose と同じ値を static TF |

**`/localization/pose_enu` は TF ではなく** Gazebo dynamic pose info の先頭 robot model
pose から生成し、**covariance は 0 とする。**

### 6.3 異常時の挙動

| 事象 | 挙動 |
| --- | --- |
| world 生成時に base world が不正 | **例外を出して launch を失敗させる** |
| Gazebo dynamic pose info に対象 index が無い | 初回と一定周期で warn ログを出す |
| Gazebo や bridge の runtime エラー | 各プロセスの標準出力で確認する |
| 3D LiDAR の PointCloud2 が出ない | bridge topic と型を最初に確認する（7.3 節） |

`fake_pose_enu.py` と `odom_tf_broadcaster.py` は起動時に購読・配信 topic を info ログへ
出す。

## 7. 判定ロジックと閾値

### 7.1 pylon の配置

**同一 seed では同一配置となり、原点 5 m 以内には配置しない。**

### 7.2 決定的な blocker

ランダム pylon は障害物回避イベントを必ず発火させるものではない。そのため統合確認 launch は
既定で `enable_route_blocker=true` とし、**経路中心線上の `route_blocker_distance` に
pylon を 1 本追加する。**

これにより `/obstacle_avoidance_hint.front_blocked`、`route_follower` の `AVOIDING` 遷移、
障害物通過後の `RUNNING` 復帰を**再現性を持って**確認できる。

### 7.3 GPU LiDAR の bridge 対象

**GPU LiDAR の sensor base topic は scan 相当であり、点群は `.../points` topic に
`gz.msgs.PointCloudPacked` として出る。** したがって bridge は
`/mid360/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked` とする。

world template では `<render_engine>ogre2</render_engine>` を明示し、**headless/GPU 権限差で
レンダリング backend が変わっても sensor 出力が不安定になりにくい構成**とする。

フロントカメラは実機と同じ `/usb_cam/image_raw` を `sensor_msgs/msg/Image` で bridge する。
AI 助言の入力に使うため、画素数は実機の設定（`hardware.yaml` の `camera`）と揃える。

### 7.4 センサ取付

センサの取付は実機の `icart_bringup/params/hardware.yaml` を正とし、模擬側をそれに合わせる。
MID-360 は床上 0.562 m、前下がり 26.9°、ロール -0.6° とする。前下がり取付でなければ、
停止判定が起きる 0.75〜1.5 m で床上 0.35 m や 0.50 m の層を観測できず、水平スライスを
前提とした回避幅の算出が成立しない。topURG は床上 0.25 m とし、MID-360 より低く置く。

**SDF の link pose は model frame（床面）基準、静的 TF は base_link（車軸中心、床上
0.15 m）基準である。** 両者を取り違えると層の切り出し高さが車輪半径の分ずれる。
一致は `tests/test_robot_mount.py` が検査する。

カメラの取付位置は実機で未計測であり、模擬側の値は仮の配置である。

## 8. 並行性・周期・起動・依存関係

### 8.1 起動構成

| launch | 用途 |
| --- | --- |
| `sim_obstacle_route.launch.py` | Gazebo 単体確認 |
| `gazebo_obstacle_route_stack.launch.py` | Gazebo と route stack の統合確認 |

**通常の実機／模擬共通起動は `icart_bringup` が担当する。** 本パッケージの launch は検証用
である。

GNSS 単独・FAST-LIO 比較・融合走行は異なる構成で、`--fusion` 指定時に融合を制御へ使う。

### 8.2 依存関係

本パッケージは `ament_cmake` とし、Python script は `install(PROGRAMS)` で
`lib/obstacle_route_sim` へ配置する。Gazebo 連携は `ros_gz_sim` と `ros_gz_bridge` に
依存する。**Classic 系依存は追加しない。**

## 9. 確認方法と未確認範囲

### 9.1 自動確認

自動テストでは ROS / Gazebo 実行に依存しない `road_geometry.py`、waypoint CSV 生成、
pylon world 生成を確認する。

| 受け入れ条件 |
| --- |
| `pytest src/obstacle_route_sim/tests` が成功する |
| `/usr/bin/python3 -m pytest src/obstacle_route_sim/tests src/robot_console/tools/tests src/route_planner/tests` が成功する |
| `colcon build --symlink-install --packages-select obstacle_route_sim` が成功する |
| `colcon build --packages-select obstacle_route_sim` が成功する |
| 上記 2 形式を route stack の 6 パッケージと併せて実行しても成功する |

### 9.2 手動で確認する範囲

- ローカル ROS 実行確認時に Gazebo world が起動し、`/scan`、
  `/mid360/livox/lidar/points`、`/ypspur_ros/odom`、`/localization/pose_enu`、TF が
  確認できる
- `gazebo_obstacle_route_stack.launch.py` で `/active_route`、`/active_target`、
  `/cmd_vel/autonomous`、`/cmd_vel`、`/drive_mode_status` が確認でき、**`/cmd_vel` の
  publisher が `drive_cmd_mux_node` のみである**
- GUI 確認時に車体上面の支持台へ MID-360 が載り、センサだけが浮いて見えない
- 決定的 blocker による `AVOIDING` 遷移と `RUNNING` 復帰（7.2 節）

`--record-lio-map` で登録済み点群を分割保存する。共通デジタルツインは歩行者の模擬と
開始地点の FIX 区域も設定できる。

### 9.3 未確認範囲

- **基本の sim_obstacle_route は真値由来 pose による経路試験であり、融合精度の試験では
  ない。**
- **模擬 Mid-360 は規則格子点群を使い、非反復走査・点内時刻・材質反射を完全には再現
  しない。**
- 自己位置推定精度と Livox Mid-360 の走査特性は非スコープである（1 章）。
- シナリオとセンサ設定は使用する launch・trial・session で確認する。本書では個別の
  シナリオ値を定めない。

## 10. 影響範囲・未決事項

### 10.1 影響範囲

生成する route 資産は `route_planner` と `route_manager` の `routes/` と `params/` へ
書き込む（5 章）。両パッケージのパラメータ仕様を変える場合は
`tools/generate_route_assets.py` を併せて更新する。

waypoint CSV の列仕様は `route_planner` の parser に従う。列を変える場合は
`route_survey` の出力形式と併せて確認する。

`/localization/pose_enu` の配信元は本パッケージの fake pose である。**融合構成では
`gnss_lio_fusion` が配信元になるため、同時に起動しない。**

`/cmd_vel` の publisher を `drive_cmd_mux_node` のみに保つ前提は統合確認 launch の構成に
依存する。

### 10.2 未決事項

| 件 | 選択肢 | blocker |
| --- | --- | --- |
| 4.1 節: 宣言されるが使われない launch 引数が 2 個ある | 機能を実装する／引数を削る | ではない |
| 4.2 節: route stack 側の `use_sim_time` を切り替えられない | 引数で切り替えられるようにする／wall time 固定を前提として明示する（現状） | ではない。初期 route request timer の停止を避けるための選択である |
| 9.3 節: 模擬 Mid-360 が走査特性を再現しない | 非反復走査の模擬を実装する／非スコープとして維持する（現状） | ではない |
