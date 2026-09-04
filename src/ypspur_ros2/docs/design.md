# ypspur_ros2 設計書

## 1. 目的・スコープ

[openspur/yp-spur](https://github.com/openspur/yp-spur) (T-Frog Project の差動駆動ロボット
制御ライブラリ) を ROS 2 Jazzy から扱うためのドライバパッケージ。

`geometry_msgs/Twist` の `/cmd_vel` 1 本でロボットを走らせ、`nav_msgs/Odometry`
を `/odom` に publish する。

**スコープ内**
- `/cmd_vel` → `YPSpur_vel(v, w)` 変換
- `YPSpur_get_pos` / `YPSpur_get_vel` を用いた `/odom` 配信
- cmd_vel タイムアウト時の自動停止 (デッドマンスイッチ)
- yp-spur 本体の取り込み + Issue #245 ワークアラウンドパッチ適用

**スコープ外 (今回は実装しない)**
- TF (`odom→base_link`) の broadcast
- `joint_states` 配信
- `ypspur-coordinator` プロセスの自動起動・監視 (ユーザが手動起動)
- 高度な走行制御 (`YPSpur_circle`, `YPSpur_line` 等の経路指令)
- Lifecycle node 化

## 2. yp-spur 本体の取り込み

openspur/yp-spur を `third_party/yp-spur/` に **git submodule** として取り込み、
ビルド時に `build/` 配下へコピーしたうえで `add_subdirectory()` で同時ビルドする。

### 2.1 Issue #245 への対処

Linux kernel 6.x 環境 (Ubuntu 22.04+) で `tcflush(fd, TCOFLUSH)` が出力バッファ
だけでなく入力バッファも flush してしまうため、`odometry_receive_loop()` の
`read()` が 0 を返し続ける問題に対し、報告されたワークアラウンドを適用する。

パッチ内容 (`third_party/patches/0001-fix-tcflush-kernel-6.x.patch`):

```diff
--- a/src/serial.c
+++ b/src/serial.c
@@ -533,7 +533,7 @@ int encode_write(char* data, int len)
   {
     return -1;
   }
-  serial_flush_out();
+  // Workaround for Linux kernel 6.x: tcflush(TCOFLUSH) also flushes
+  // input buffer, breaking odometry_receive_loop. See issue #245.
+  // serial_flush_out();

   return 0;
 }
```

### 2.2 パッチ適用方法

CMake から `third_party/yp-spur` をビルドディレクトリへコピーし、コピー先に対して
`git apply --check` で適用可否を判定し、未適用なら `git apply`。
submodule 自体は upstream を指したまま変更しない。
コピー先はリポジトリの `build/` 配下にあるため、`GIT_CEILING_DIRECTORIES` を指定して
親リポジトリの ignore 設定に影響されないようにする。

```cmake
file(COPY ${YPSPUR_DIR}/ DESTINATION ${YPSPUR_BUILD_DIR}
  PATTERN ".git" EXCLUDE
)
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env
    GIT_CEILING_DIRECTORIES=${WORKSPACE_ROOT}
    git apply --check ${PATCH_FILE}
  WORKING_DIRECTORY ${YPSPUR_BUILD_DIR}
  RESULT_VARIABLE can_apply
  OUTPUT_QUIET ERROR_QUIET
)
if(can_apply EQUAL 0)
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      GIT_CEILING_DIRECTORIES=${WORKSPACE_ROOT}
      git apply ${PATCH_FILE}
    WORKING_DIRECTORY ${YPSPUR_BUILD_DIR}
  )
endif()
```

将来 upstream にマージされたら patch を削除する。

## 3. パッケージ構成

```
src/ypspur_ros2/
├── package.xml
├── CMakeLists.txt
├── README.md
├── src/
│   └── ypspur_node.cpp            # メインノード (rclcpp)
├── launch/
│   └── ypspur_ros2.launch.py
├── config/
│   └── default.yaml               # パラメータ既定値
├── docs/
│   └── design.md                  # 本書
└── third_party/
    ├── COLCON_IGNORE              # colcon にスキャンさせない
    ├── patches/
    │   └── 0001-fix-tcflush-kernel-6.x.patch
    └── yp-spur/                   # submodule
```

## 4. 動作アーキテクチャ

```
   ┌──────────────────────────┐
   │  /cmd_vel (Twist)        │──┐
   └──────────────────────────┘  │
                                  ▼
   ┌────────────────────────────────────────┐
   │           ypspur_node (rclcpp)         │
   │                                         │
   │  on_cmd_vel():                          │
   │    YPSpur_vel(twist.lin.x, twist.ang.z) │
   │                                         │
   │  timer(50Hz):                           │
   │    YPSpur_get_pos(CS_BS, &x, &y, &th)   │
   │    YPSpur_get_vel(&v, &w)               │
   │    publish Odometry                     │
   │                                         │
   │  timeout watchdog:                      │
   │    if t_since_cmd_vel > timeout         │
   │      YPSpur_vel(0, 0)                   │
   └────────────────────────────────────────┘
                       │
                       │ libypspur (IPC: msgqueue / socket)
                       ▼
   ┌────────────────────────────────────────┐
   │   ypspur-coordinator (別プロセス)        │
   │   (ユーザが事前に起動)                    │
   └────────────────────────────────────────┘
                       │ serial
                       ▼
                   Motor Driver
```

`ypspur-coordinator` の起動はユーザ責任。例:

```bash
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param --without-device-watchdog
```

## 5. ノード設計

### 5.1 サブスクライバ

| Topic       | Type                       | QoS       | 処理                                   |
| ----------- | -------------------------- | --------- | -------------------------------------- |
| `cmd_vel`   | `geometry_msgs/msg/Twist`  | KeepLast 10 | `YPSpur_vel(msg.linear.x, msg.angular.z)` を呼び、最終受信時刻を更新 |

### 5.2 パブリッシャ

| Topic       | Type                          | 頻度        | 内容                                 |
| ----------- | ----------------------------- | ----------- | ------------------------------------ |
| `odom`      | `nav_msgs/msg/Odometry`       | パラメータ (既定 50Hz) | pose, twist; orientation は yaw のみ (roll/pitch=0) |

### 5.3 ROS パラメータ

| パラメータ              | 型      | 既定値        | 説明                                                     |
| ----------------------- | ------- | ------------- | -------------------------------------------------------- |
| `cmd_vel_timeout_s`     | double  | `0.5`         | これ以上 `cmd_vel` が無ければ自動で `YPSpur_vel(0, 0)`   |
| `odom_publish_hz`       | double  | `50.0`        | `/odom` 配信レート                                       |
| `odom_frame_id`         | string  | `odom`        | Odometry header.frame_id                                 |
| `base_frame_id`         | string  | `base_link`   | Odometry child_frame_id                                  |
| `coordinate_system`     | int     | `0` (CS_BS)   | `YPSpur_get_pos` に渡す座標系。0=BS, 1=GL, 2=LC          |
| `ipc.use_socket`        | bool    | `false`       | true なら `YPSpur_init_socket()`、false なら `YPSpur_init()` |
| `ipc.ip`                | string  | `127.0.0.1`   | socket モード時のホスト                                  |
| `ipc.port`              | int     | `54321`       | socket モード時のポート                                  |
| `velocity_max.linear`   | double  | `1.0`         | 受信した linear.x のクリップ閾値 (m/s, ±対称)            |
| `velocity_max.angular`  | double  | `1.5`         | 受信した angular.z のクリップ閾値 (rad/s, ±対称)         |

### 5.4 起動シーケンス

1. パラメータ宣言・取得
2. `YPSpur_init()` / `YPSpur_init_socket()` を呼ぶ。失敗時はノードを `FATAL` で終了
3. publisher / subscriber を生成
4. `wall_timer` で odom 配信タイマーを起動
5. `wall_timer` で deadman watchdog を起動 (cmd_vel 受信時刻を監視)

### 5.5 終了シーケンス

`rclcpp::shutdown()` の前に:
1. `YPSpur_vel(0, 0)` で停止
2. `YPSpur_free()` で IPC を切断

## 6. Odometry 配信仕様

```cpp
auto t_now = this->now();
double x, y, th;
double t_pose = YPSpur_get_pos(CS_BS, &x, &y, &th);
double v, w;
double t_vel  = YPSpur_get_vel(&v, &w);

nav_msgs::msg::Odometry odom;
odom.header.stamp = t_now;
odom.header.frame_id = odom_frame_id_;
odom.child_frame_id  = base_frame_id_;
odom.pose.pose.position.x = x;
odom.pose.pose.position.y = y;
odom.pose.pose.position.z = 0.0;
// yaw-only quaternion
odom.pose.pose.orientation.z = std::sin(th * 0.5);
odom.pose.pose.orientation.w = std::cos(th * 0.5);
odom.twist.twist.linear.x  = v;
odom.twist.twist.angular.z = w;
// 共分散は対角に大きめの値を入れる (現状は粗い値、要 calibration)
odom_pub_->publish(odom);
```

`YPSpur_get_pos` の戻り値は **取得時刻** (秒) なので、本来は header.stamp に
これを使う方が正確だが、今回は ROS 時計で十分とする。将来 §7 で言及。

## 7. 将来拡張

- TF (`odom→base_link`) broadcast (パラメータで切替)
- `joint_states` 配信 (`YP_get_wheel_vel()` から積分)
- `ypspur-coordinator` のサブプロセス管理化 (`launch` から起動 → 終了時 kill)
- Stamp ソースを `YPSpur_get_pos()` の返却時刻に切替 (system clock との対応に注意)
- `YPSpur_set_accel` / `YPSpur_set_angaccel` 等の加速度上限パラメータ化
- Lifecycle node 化 (Nav2 と同様の状態遷移)
- ホットプラグ/再接続 (coordinator 落ち時の自動 reinit)

## 8. 受け入れ条件 (Definition of Done)

- [ ] `colcon build` がエラー無く通る (yp-spur 本体も同時にビルドされる)
- [ ] `cmake` の出力に「yp-spur patch applied」または「already applied」が出る
- [ ] `ypspur-coordinator` 起動済の状態で `ros2 launch ypspur_ros2 ypspur_ros2.launch.py`
      が起動でき、`ros2 topic pub /cmd_vel ...` で実機が反応する
- [ ] `ros2 topic echo /ypspur_ros/odom` で姿勢/速度が出る（launch既定のremap先）
- [ ] cmd_vel を止めると `cmd_vel_timeout_s` 後にロボットが停止する

## 9. 依存関係

```xml
<!-- package.xml -->
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2</depend>             <!-- yaw → quaternion 用 -->
<!-- yp-spur 本体は third_party submodule で内包 -->
```

## 10. ビルド & 起動

```bash
cd ~/colcon_ws
git submodule update --init --recursive
colcon build --packages-select ypspur_ros2
source install/setup.bash

# 端末 1: coordinator
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param

# 端末 2: ROS ノード
ros2 launch ypspur_ros2 ypspur_ros2.launch.py

# 端末 3: 動かしてみる
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```
