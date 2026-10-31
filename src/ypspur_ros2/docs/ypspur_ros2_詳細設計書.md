# ypspur_ros2 詳細設計書

## 1. 目的・スコープ

[openspur/yp-spur](https://github.com/openspur/yp-spur) (T-Frog Project の差動駆動ロボット
制御ライブラリ) を ROS 2 Jazzy から扱うためのドライバパッケージ。

`geometry_msgs/Twist` の `/cmd_vel` 1 本でロボットを走らせ、`nav_msgs/Odometry`
をlaunch既定の `/ypspur_ros/odom` にpublishする（ノード内の相対名は`odom`）。

**スコープ内**
- `/cmd_vel` → `YPSpur_vel(v, w)` 変換
- `YPSpur_get_pos` / `YPSpur_get_vel` を用いた車輪Odometry配信
- coordinatorの実効上限確認、加減速の切替、`/motion_limits`の継続配信
- cmd_vel タイムアウト時の自動停止 (デッドマンスイッチ)
- yp-spur 本体の取り込み + Issue #245 ワークアラウンドパッチ適用

**提供しない機能**
- TF (`odom→base_link`) の broadcast
- `joint_states` 配信
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

パッチ適用先はビルド用コピーであり、submoduleを直接変更しない。

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
│   └── ypspur_ros2_詳細設計書.md                  # 本書
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

`start_coordinator:=true`ではlaunchがcoordinatorを起動し、2秒後に車輪ノードを開始する。
既定のfalseでは外部起動したcoordinatorへ接続する。別端末での起動例:

```bash
ypspur-coordinator -d /dev/serial/by-id/usb-T-frog_project_T-frog_Driver-if00 -p "$(ros2 pkg prefix ypspur_ros2)/share/ypspur_ros2/config/icart-middle.param" --without-device-watchdog
```

## 5. ノード設計

### 5.1 サブスクライバ

| Topic       | Type                       | QoS       | 処理                                   |
| ----------- | -------------------------- | --------- | -------------------------------------- |
| `cmd_vel`   | `geometry_msgs/msg/Twist`  | KeepLast 10 | 有限値を検査し、上限でclipした目標速度と単調時計の受信時刻を更新する |

### 5.2 パブリッシャ

| Topic       | Type                          | 頻度        | 内容                                 |
| ----------- | ----------------------------- | ----------- | ------------------------------------ |
| `odom`      | `nav_msgs/msg/Odometry`       | パラメータ (既定 50Hz) | pose, twist; orientation は yaw のみ (roll/pitch=0) |
| `motion_limits` | `tc_route_msgs/msg/MotionLimits` | 正常な20 ms制御周期ごと | 実効の線/角速度上限、線加速・線減速・角加速上限。RELIABLE / VOLATILE / depth=1 |

launchは`odom`を`/ypspur_ros/odom`へremapする。`motion_limits`はnamespaceなしで
`/motion_limits`となり、navigatorはこの絶対名を購読する。

### 5.3 ROS パラメータ

| パラメータ              | 型      | 既定値        | 説明                                                     |
| ----------------------- | ------- | ------------- | -------------------------------------------------------- |
| `cmd_vel_timeout_s`     | double  | `0.5`         | これ以上 `cmd_vel` が無ければ自動で `YPSpur_vel(0, 0)`   |
| `odom_publish_hz`       | double  | `50.0`        | 車輪Odometry配信レート                                   |
| `odom_frame_id`         | string  | `odom`        | Odometry header.frame_id                                 |
| `base_frame_id`         | string  | `base_link`   | Odometry child_frame_id                                  |
| `coordinate_system`     | int     | `2` (CS_GL)   | 0=BS, 1=SP, 2=GL, 3=LC, 4=FS, 5=BL                     |
| `ipc.use_socket`        | bool    | `false`       | true なら `YPSpur_init_socket()`、false なら `YPSpur_init()` |
| `ipc.ip`                | string  | `127.0.0.1`   | socket モード時のホスト                                  |
| `ipc.port`              | int     | `54321`       | socket モード時のポート                                  |
| `velocity_max.linear`   | double  | `1.0`         | 受信した linear.x のクリップ閾値 (m/s, ±対称)            |
| `velocity_max.angular`  | double  | `1.0`         | 受信した angular.z のクリップ閾値 (rad/s, ±対称)         |
| `acceleration_max.linear` | double | `0.7` | 加速時の線加速度 [m/s²] |
| `deceleration_max.linear` | double | `1.5` | 制動時の線減速度 [m/s²] |
| `acceleration_max.angular` | double | `1.5` | 角加速度 [rad/s²] |

速度・加減速の上限はread-onlyパラメータであり、正の有限値を要求する。

### 5.4 起動シーケンス

1. パラメータ宣言・取得
2. `YPSpur_initex(ipc.key)` / `YPSpur_init_socket()`で接続する。失敗時は終了する
3. ゼロ速度を送り、coordinatorのMAX_VEL / MAX_W / MAX_ACC_V / MAX_ACC_Wを読み、要求値が上限以内か確認する
4. 速度・線減速度・角加速度を初期設定する。取得・設定失敗時は起動を中止する
5. publisher / subscriber、20 ms制御タイマー、odom配信タイマー、100 ms期限監視タイマーを開始する

制御周期ごとに`YP_get_vref`の線速度参照と要求値から加速/減速を選ぶ。
減速時は目標を下げてから減速度を設定し、加速時は加速度を設定してから目標を上げる。
反転時はゼロへ制動する。正常に適用した場合だけ`motion_limits`を発行する。
取得・設定失敗時はゼロを送り、この周期のheartbeatを出さない。
cmd_vel途絶は単調時計で判定し、`cmd_vel_timeout_s`超でゼロ速度と線減速度を設定する。

### 5.5 終了シーケンス

`rclcpp::shutdown()` の前に:
1. `YPSpur_vel(0, 0)` で停止
2. `YPSpur_free()` で IPC を切断

## 6. Odometry 配信仕様

```cpp
auto t_now = this->now();
double x, y, th;
double t_pose = YPSpur_get_pos(static_cast<YPSpur_cs>(coordinate_system_), &x, &y, &th);
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

`YPSpur_get_pos`の戻り値は取得時刻だが、odomのheader.stampにはROSの現在時刻を使う。
取得時刻とROS時計の対応を推定する処理はない。

## 7. 制約

TF・joint_states・Lifecycle・coordinatorへの自動再接続は提供しない。
coordinatorはlaunchのstart_coordinator=trueで起動できる。
起動待ち時間は準備完了の保証ではなく、IPCや能力照合が失敗した場合は起動を中止する。

## 8. 受け入れ条件 (Definition of Done)

- `colcon build` がエラー無く通る (yp-spur 本体も同時にビルドされる)
- `cmake` の出力に「yp-spur patch applied」または「already applied」が出る
- `ypspur-coordinator` 起動済の状態で `ros2 launch ypspur_ros2 ypspur_ros2.launch.py`
      が起動でき、`ros2 topic pub /cmd_vel ...` で実機が反応する
- `ros2 topic echo /ypspur_ros/odom` で姿勢/速度が出る（launch既定のremap先）
- cmd_vel を止めると `cmd_vel_timeout_s` 後にロボットが停止する

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
ypspur-coordinator -d /dev/serial/by-id/usb-T-frog_project_T-frog_Driver-if00 -p "$(ros2 pkg prefix ypspur_ros2)/share/ypspur_ros2/config/icart-middle.param"

# 端末 2: ROS ノード
ros2 launch ypspur_ros2 ypspur_ros2.launch.py

# 端末 3: 動かしてみる
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```
