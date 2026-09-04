# ypspur_ros2

[openspur/yp-spur](https://github.com/openspur/yp-spur) を ROS 2 (Jazzy) でラップし、
`/cmd_vel` で差動駆動ロボットを制御するパッケージ。

## 主要トピック

| Topic | Type | 方向 | 説明 |
| --- | --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | sub | `YPSpur_vel(linear.x, angular.z)` を呼ぶ。既定では `drive_mode_manager` が publish する `/cmd_vel` と接続する |
| `odom` | `nav_msgs/msg/Odometry` | pub | `YPSpur_get_pos` / `YPSpur_get_vel` を 50Hz で配信 |

`ypspur_node` は相対 topic `cmd_vel` を購読し、相対 topic `odom` を publish します。launch の既定では `odom` を `/ypspur_ros/odom` へ remap します（`robot_navigator`、`obstacle_route_sim` の Gazebo bridge、`robot_simulator` がいずれもこの topic 名を前提とするため）。namespace なしで `drive_mode_manager.launch.py` と同時起動した場合、`drive_cmd_mux_node` の最終出力 `/cmd_vel` がそのまま `ypspur_node` に入ります。topic 名を変える場合は launch 引数 `cmd_vel_topic` と `odom_topic` を指定します。

## Issue #245 への対処

`yp-spur` 本体は Linux kernel 6.x 環境で
[issue #245](https://github.com/openspur/yp-spur/issues/245) (tcflush が入力バッファも flush する問題)
の影響を受けます。**Ubuntu 22.04 / 24.04 では必ずパッチが必要** で、
本パッケージはこれを CMake からビルド用コピーへ **自動適用** します
([`third_party/patches/0001-fix-tcflush-kernel-6.x.patch`](third_party/patches/0001-fix-tcflush-kernel-6.x.patch))。
`third_party/yp-spur` の submodule 本体は変更しません。

`colcon build` 時に
```
-- Applying yp-spur patch (0001-fix-tcflush-kernel-6.x)
```
もしくは
```
-- yp-spur patch already applied (or not needed)
```
が表示されます。

## 必要環境

- Ubuntu 24.04 (22.04 でも可)
- ROS 2 Jazzy
- yp-spur 用パラメータファイル (`robot.param`) — 自分のロボット用のもの

## ビルド

```bash
cd ~/colcon_ws
git submodule update --init --recursive
colcon build --packages-select ypspur_ros2
source install/setup.bash
```

`ypspur-coordinator`, `ypspur-free`, `ypspur-interpreter` も install/ypspur_ros2/bin/
に配置されるので、sourceしたあと PATH 経由で利用可能です。

## 起動

### 1. ypspur-coordinator を起動 (別端末)

```bash
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param
```

ロボットによっては `--without-device-watchdog` 等のオプションが必要。詳細は yp-spur
本体ドキュメントを参照。

### 2. ROS ノードを起動

coordinator を別端末で手動起動している場合は、従来通り `ypspur_node` だけを起動します。

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py
```

`ypspur-coordinator` も launch から同時起動する場合は、`start_coordinator:=true` と robot parameter file を指定します。この場合、`ypspur_node` は coordinator 初期化待ちとして 2 秒遅延起動します。

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py \
  start_coordinator:=true \
  coordinator_device:=/dev/ttyACM0 \
  coordinator_param:=<robot_param_file>
```

`drive_mode_manager` と組み合わせる場合、既定では `drive_mode_manager` の `/cmd_vel` と `ypspur_node` の `cmd_vel` が接続されます。明示する場合は以下のようにします。

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py cmd_vel_topic:=/cmd_vel odom_topic:=/ypspur_ros/odom
```

### 3. 動作確認

```bash
# 前進 0.1 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# odometry を監視
ros2 topic echo /ypspur_ros/odom
```

cmd_vel が `cmd_vel_timeout_s` (既定 0.5 秒) 入らないとロボットは自動停止します。

## パラメータ

| パラメータ              | 型      | 既定値      | 説明                                                |
| ----------------------- | ------- | ----------- | --------------------------------------------------- |
| `cmd_vel_timeout_s`     | double  | `0.5`       | 自動停止までのタイムアウト                          |
| `odom_publish_hz`       | double  | `50.0`      | `/odom` 配信レート                                  |
| `odom_frame_id`         | string  | `odom`      | Odometry header.frame_id                            |
| `base_frame_id`         | string  | `base_link` | Odometry child_frame_id                             |
| `coordinate_system`     | int     | `2` (CS_GL) | 0=BS, 1=SP, 2=GL, 3=LC, 4=FS, 5=BL                  |
| `ipc.use_socket`        | bool    | `false`     | true なら TCP 経由、false なら local msgqueue       |
| `ipc.ip`                | string  | `127.0.0.1` | socket モード時のホスト                             |
| `ipc.port`              | int     | `54321`     | socket モード時のポート                             |
| `velocity_max.linear`   | double  | `1.0`       | 受信 linear.x のクリップ閾値 (m/s, ±対称)           |
| `velocity_max.angular`  | double  | `1.5`       | 受信 angular.z のクリップ閾値 (rad/s, ±対称)        |
| launch `cmd_vel_topic` | string | `cmd_vel` | `cmd_vel` の remap 先。`drive_mode_manager` と接続する場合は既定または `/cmd_vel` |
| launch `odom_topic` | string | `/ypspur_ros/odom` | `odom` の remap 先。robot_navigator 等が前提とする既定値 |
| launch `start_coordinator` | bool | `false` | true の場合、launch から `ypspur-coordinator` を同時起動 |
| launch `coordinator_device` | string | `/dev/ttyACM0` | `ypspur-coordinator -d` に渡す device path |
| launch `coordinator_param` | string | `""` | `ypspur-coordinator -p` に渡す robot parameter file path |

## トラブルシューティング

### `YPSpur_init failed. Is ypspur-coordinator running?`

ypspur-coordinator が起動していないか、別ユーザで起動している (msgqueue が見えない)。
同じユーザで先に coordinator を起動してから ROS ノードを起動する。

### Coordinator がモータドライバを認識しない (Ubuntu 22.04+)

- `dmesg | grep ttyACM` でデバイスが見えているか確認
- ModemManager が `ttyACM*` を掴んでしまうことがある。`sudo systemctl stop ModemManager`
- パッチが適用されていない場合 (本パッケージ経由でビルドしていない場合) は Issue #245 の症状
  (接続拒否・異音・負荷で coordinator が落ちる) が出る

### odom が出ているのに位置が動かない

- coordinator のパラメータファイルでホイール直径やトレッド幅が合っていない
- `coordinate_system` がロボットの初期化方法と合っていない

## 設計

詳細は [`docs/design.md`](docs/design.md) を参照。

## ライセンス

MIT (yp-spur 本体も MIT)
