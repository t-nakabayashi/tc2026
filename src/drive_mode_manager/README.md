# drive_mode_manager

`drive_mode_manager` は、自律走行指令と手動走行指令を切り替え、最終的な `/cmd_vel` を
出力するパッケージです。PS3 controller 相当の `/joy` 入力を使った手動走行、走行モード
状態の publish、専用 GUI による状態表示、開発用 Joy simulator を提供します。

## 主な機能

- `/joy` から手動走行用の `/cmd_vel/manual` を生成する。
- `/cmd_vel/autonomous` と `/cmd_vel/manual` を走行モードに応じて mux し、最終 `/cmd_vel` を publish する。
- L1 / PS button の操作で自律走行と手動走行を切り替える。
- `/drive_mode_status` で走行モード、入力状態、出力元、復帰待ち状態を publish する。
- `drive_status_gui_node` で走行モード、cmd、RTK、waypoint 状態を専用 GUI 表示する。
- 開発用に `ps3_joy_sim_node` でキーボード入力から `/joy` を publish する。

## 起動方法

### 走行モード管理を起動

```bash
ros2 launch drive_mode_manager drive_mode_manager.launch.py
```

この launch は以下のノードを起動します。

| ノード | 役割 |
| --- | --- |
| `joy_node` | 実 controller から `/joy` を publish する。既定の Joy 入力源 |
| `ps3_joy_sim_node` | `joy_input:=ps3_joy_sim` のとき、キーボード入力から `/joy` を publish する |
| `manual_teleop_node` | `/joy` から `/cmd_vel/manual` を生成する |
| `drive_cmd_mux_node` | `/cmd_vel/autonomous` と `/cmd_vel/manual` を切り替えて `/cmd_vel` を出力する |
| `drive_status_gui_node` | `/drive_mode_status` などを購読して専用 GUI を表示する |

既定では実 controller 用の `joy_node` を同時に起動します。開発用 simulator を Joy 入力源にする場合は以下を使います。

```bash
ros2 launch drive_mode_manager drive_mode_manager.launch.py joy_input:=ps3_joy_sim
```

GUI を起動しない場合は以下を使います。

```bash
ros2 launch drive_mode_manager drive_mode_manager.launch.py start_gui:=false
```

### 手動走行のみを行う場合

実機で手動走行だけを行う場合は、`ypspur-coordinator`、`ypspur_ros2`、`drive_mode_manager`
を起動します。`drive_mode_manager.launch.py` は既定で `joy_node` も同時起動します。
`drive_mode_manager` が `/cmd_vel` を出力し、`ypspur_ros2` が `cmd_vel_topic:=/cmd_vel` でその速度指令を車体へ渡します。

coordinator を別端末で手動起動する場合:

端末 1:

```bash
ypspur-coordinator -d /dev/serial/by-id/usb-T-frog_project_T-frog_Driver-if00 -p "$(ros2 pkg prefix ypspur_ros2)/share/ypspur_ros2/config/icart-middle.param"
```

端末 2:

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py cmd_vel_topic:=/cmd_vel
```

端末 3:

```bash
ros2 launch drive_mode_manager drive_mode_manager.launch.py
```

coordinator も `ypspur_ros2.launch.py` から起動する場合:

```bash
# 端末 1: coordinator と ypspur_node を起動
ros2 launch ypspur_ros2 ypspur_ros2.launch.py \
  start_coordinator:=true \
  coordinator_device:=/dev/serial/by-id/usb-T-frog_project_T-frog_Driver-if00 \
  coordinator_param:=<robot_param_file> \
  cmd_vel_topic:=/cmd_vel

# 端末 2: joy_node を起動し、Joy 入力から最終 /cmd_vel を publish
ros2 launch drive_mode_manager drive_mode_manager.launch.py
```

起動直後の既定モードは `autonomous` です。手動走行へ切り替えるには、Joy 入力で L1 と PS button
を `manual_transition_hold_s` 秒以上同時押しします。手動走行中は L1 を押している間だけ
`/cmd_vel/manual` が有効になります。自律走行へ戻す場合は L1 を離し、
`manual_to_auto_l1_released_s` 経過後に `/cmd_vel/autonomous` が有効なら復帰待ち状態へ移ります。

### 開発用 Joy simulator を起動

実 controller の代わりに、キーボード入力から `/joy` を publish する場合に使います。
通常は `drive_mode_manager.launch.py` の `joy_input` を切り替えて同時起動します。

```bash
ros2 launch drive_mode_manager drive_mode_manager.launch.py joy_input:=ps3_joy_sim
```

Joy simulator だけを単独起動する場合は以下を使います。

```bash
ros2 launch drive_mode_manager ps3_joy_sim.launch.py
```

既定 key bind は以下です。

| キー | 入力 |
| --- | --- |
| `w` / `s` | 左 stick 前後 |
| `a` / `d` | 左 stick 左右 |
| `l` | L1 button |
| `p` | PS button |
| `space` | 入力 reset |

`w`/`s`/`a`/`d` は押すたびに左 stick の保持値へ `0.1` ずつ加算または減算されます。GUI には現在の stick 値に対して `manual_teleop_node` の既定 scale と deadzone を適用した予測 `cmd_vel` の `v` と `w` を表示します。`space` は stick を neutral に戻します。

## 外部インタフェース

以下は単体起動の既定名です。共通起動のmuxは自律入力を `/cmd_vel/fusion_limited` へremapします。
実機UM982の状態の完全名は `/rtk_gps/rtk_status` であり、
単体の `drive_status_gui_node` を接続する場合も入力名を合わせてください。

### Subscriber

| Topic | 型 | 使用ノード | 説明 |
| --- | --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | `manual_teleop_node`, `drive_cmd_mux_node` | 手動走行入力、走行モード切替入力 |
| `/cmd_vel/autonomous` | `geometry_msgs/Twist` | `drive_cmd_mux_node`, `drive_status_gui_node` | 自律走行側の速度指令 |
| `/cmd_vel/manual` | `geometry_msgs/Twist` | `drive_cmd_mux_node` | 手動走行側の速度指令 |
| `/drive_mode_status` | `tc_route_msgs/DriveModeStatus` | `drive_status_gui_node` | 走行モード状態 |
| `/follower_state` | `tc_route_msgs/FollowerState` | `drive_status_gui_node` | waypoint 表示 |
| `/manager_status` | `tc_route_msgs/ManagerStatus` | `drive_status_gui_node` | route manager 状態表示 |
| `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/RtkStatus` | `drive_status_gui_node` | RTK 状態表示 |

### Publisher

| Topic | 型 | 使用ノード | 説明 |
| --- | --- | --- | --- |
| `/cmd_vel/manual` | `geometry_msgs/Twist` | `manual_teleop_node` | Joy 入力から生成した手動走行指令 |
| `/cmd_vel` | `geometry_msgs/Twist` | `drive_cmd_mux_node` | 車体へ渡す最終速度指令 |
| `/drive_mode_status` | `tc_route_msgs/DriveModeStatus` | `drive_cmd_mux_node` | 現在の走行モードと mux 状態 |
| `/joy` | `sensor_msgs/Joy` | `ps3_joy_sim_node` | 開発用 Joy 入力 |

## パラメータ

既定値は [`params/default.yaml`](params/default.yaml) で管理します。

| ノード | 主なパラメータ |
| --- | --- |
| `manual_teleop_node` | `linear_axis`, `angular_axis`, `linear_scale`, `angular_scale`, `enable_button`, `joy_timeout_s`, `publish_rate_hz` |
| `drive_cmd_mux_node` | `initial_mode`, `manual_transition_trigger`, `manual_transition_hold_s`, `manual_to_auto_l1_released_s`, `auto_resume_delay_s`, `autonomous_cmd_timeout_s`, `manual_cmd_timeout_s`, `l1_button_index`, `ps_button_index` |
| `drive_status_gui_node` | `main_display_ratio`, `direction_linear_scale`, `direction_angular_scale`, `direction_deadzone`, `direction_linear_axis_invert`, `direction_angular_axis_invert`, `turn_preview_seconds` |
| `ps3_joy_sim_node` | `joy_topic`, `publish_rate_hz`, `num_axes`, `num_buttons`, `invert_left_stick_x`, key bind 関連パラメータ |

## 依存関係

ROS 2 依存は `package.xml` で管理します。実 controller 入力には `joy` package の
`joy_node` を使います。GUI は apt / rosdep で導入する `python3-pyqt5` を使います。pip で追加導入する `drive_mode_manager` 固有の Python
パッケージはありません。

## ビルドと確認

```bash
colcon build --symlink-install --packages-select tc_route_msgs drive_mode_manager
colcon build --packages-select tc_route_msgs drive_mode_manager
pytest src/drive_mode_manager/tests
```

GUI 表示、Joy 入力、実機走行の確認はローカル環境で行います。実機を動かす場合は、
`ypspur-coordinator` と `ypspur_ros2` の起動状態、非常停止、周囲の安全を確認してから操作してください。

## 設計

詳細は [`docs/drive_mode_manager_詳細設計書.md`](docs/drive_mode_manager_詳細設計書.md) と
[`docs/ps3_joy_sim_設計書.md`](docs/ps3_joy_sim_設計書.md) を参照してください。

`drive_cmd_mux_node.allow_auto_resume`（既定true）をfalseにすると、
manualモードからのL1解放による自律復帰を禁止する。採取用共通launchで使用する。
手動中のL1 deadmanと入力timeoutは維持し、通常の自律起動の既定動作は変えない。

### その場旋回の操作

前後速度と角速度は独立。L1を押しながらスティックを横へ倒すとその場旋回する。
旋回入力がある場合、前後軸の微小入力は `in_place_turn_deadzone`（既定0.12）まで
ゼロとする。直進時の微速操作は`deadzone`を使い、大きな前後入力との旋回は維持する。

実機共通起動はPSボタン10、単体設定・Joy simulatorは16を使います。
採取起動ではallow_auto_resume=falseで手動を保持します。
