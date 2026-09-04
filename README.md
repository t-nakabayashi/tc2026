# tc2026

ROS 2 Jazzy ワークスペース。

## パッケージ一覧

### センサ / アクチュエータドライバ
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/rtk_gps_um982`](src/rtk_gps_um982/README.md)          | Unicore UM982 RTK GNSS ドライバ。NavSatFix / Imu / RtkStatus を配信  |
| [`src/rtk_gps_um982_msgs`](src/rtk_gps_um982_msgs/)         | 上記用カスタム msg (`RtkStatus`)                                     |
| [`src/ypspur_ros2`](src/ypspur_ros2/README.md)              | yp-spur ベースの差動駆動ロボット制御。`/cmd_vel` で動かす            |

### 経路計画・追従
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/route_planner`](src/route_planner/README.md)          | YAML / CSV から経路を生成し `/get_route`・`/update_route` を提供。可変ブロックの再計画にも対応 |
| [`src/route_manager`](src/route_manager/README.md)          | `route_planner` のサービスを呼び出し `/active_route` を配信、滞留報告から再計画を統括する FSM |
| [`src/route_follower`](src/route_follower/README.md)        | `/active_route` を追従し、現在の目標 Pose を `/active_target` として配信。滞留検知で `/report_stuck` を発行 |
| [`src/tc_route_msgs`](src/tc_route_msgs/README.md)                | 経路・走行系で共有する msg / srv 定義 (`Route`, `RouteState`, `ReportStuck` ほか) |

### 座標変換・地理情報
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/tc_geo_msgs`](src/tc_geo_msgs/)                       | LLH位置、品質、地図投影条件を共有する msg 定義                       |
| [`src/geo_pose_converter`](src/geo_pose_converter/README.md) | LLH/ENU相互変換、経路の地理座標投影、OSM経路表示を提供               |

### 走行制御・障害物
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/robot_navigator`](src/robot_navigator/README.md)      | `/active_target` を追従して `/cmd_vel` を出力する時間最適制御ノード。試験用 `robot_simulator` も同梱 |
| [`src/drive_mode_manager`](src/drive_mode_manager/README.md) | 自律走行指令と手動走行指令を切り替え、最終 `/cmd_vel` と走行モード状態を配信 |
| [`src/obstacle_monitor`](src/obstacle_monitor/README.md)    | `/scan` を解析して `/obstacle_avoidance_hint` を配信。`/sensor_viewer` への可視化も提供 |

### シミュレーション
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/obstacle_route_sim`](src/obstacle_route_sim/README.md) | Gazebo Harmonic 上で道路 world、差動二輪ロボット、LiDAR、pylon 障害物を起動し、route stack の結合動作確認を行う |

### 認識・監視
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/yolo_detector`](src/yolo_detector/README.md)          | USB カメラ画像を YOLO (PyTorch / NCNN) で物体検出し、検出画像・`Detection2DArray` を配信 |
| [`src/traffic_signal_recognizer`](src/traffic_signal_recognizer/README.md) | YOLO検出結果から信号のGO/STOPを判定し、判定画像を配信 |
| [`src/road_blockage_detector`](src/road_blockage_detector/README.md) | YOLO検出結果と自己位置から道路封鎖を判定し、判定画像を配信 |
| [`src/robot_console`](src/robot_console/README.md)          | 走行状態・障害物回避・経路進捗・ノード起動を一画面で監視する PyQt5 GUI ダッシュボードとHTML遠隔観測UI |

ワークスペース横断の仕様書は [`docs/`](docs/)、パッケージ固有の設計書は各パッケージ
配下の `docs/` を参照。

## 開発状態

- `robot_console` の正式UIは PyQt5 版（`robot_console_qt`）である。遠隔観測用の
  HTML UI（`robot_console_web`）も同じ `ConsoleCore` の状態を表示する。
  旧 tkinter 版（`robot_console`）は当面残すが、正式UIとしては扱わない。
- `localization_fusion` 実装前の暫定構成では、GNSS入力がある場合に
  `geo_pose_converter` のENU出力を `/localization/pose_enu` として使用する。
- 走行制御はENU、OSM・GUI表示はLLHを使用し、`geo_pose_converter` が両者を変換する。

## 必要環境

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`
- Python 3
- (パッケージごとの追加要件は各 README を参照)

## Claude Code スキル設定

本リポジトリでは、GUI・UI 実装時に Anthropic 公式の `frontend-design` スキルを
共通利用する。`.claude/settings.json` (Git 管理対象) で
`frontend-design@claude-plugins-official` を有効化しているため、リポジトリを
Claude Code で開いて信頼 (trust) すると、このプラグインが自動的に有効化候補として
認識される。初回のみ、各自の環境で以下を実行してインストールする。

```bash
claude plugin install frontend-design@claude-plugins-official
```

インストール後は Claude が GUI デザイン作業時に自動でこのスキルを参照する。
手動で呼び出す場合は `/frontend-design:frontend-design` のように実行する。

## Codex ローカル実行設定

Codex app / CLI / IDE Extension で `ros2 run`, `ros2 launch`, `ros2 topic` などを含む
ローカル環境の動作確認を行う場合は、`~/.codex/config.toml` に以下を追記する。

```toml
approval_policy = "on-request"
sandbox_mode = "workspace-write"

[sandbox_workspace_write]
network_access = true
```

この設定は、Codex がワークスペース内で ROS 2 ノードや確認用コマンドを実行するためのもの。
実機 driver や実ロボットを動かす確認は、各手順で明示された場合を除き実行しない。

## Python 依存モジュール

Python パッケージ群で使用する pip 依存モジュールは、[`requirements.txt`](requirements.txt) にまとめている。
対象は `obstacle_monitor`, `robot_console`, `robot_navigator`, `route_follower`,
`route_manager`, `route_planner`, `obstacle_route_sim`, `yolo_detector` と、それらが利用する
`tc_route_msgs`。`drive_mode_manager` の GUI 依存である `python3-pyqt5` と、
`obstacle_route_sim` の Gazebo / ros_gz 依存は pip ではなく apt / rosdep で導入する。

ROS 2 の環境を読み込んだうえで、ワークスペース直下で以下を実行する。

```bash
python3 -m pip install -r requirements.txt
```

## ビルド

```bash
git clone --recursive https://github.com/t-nakabayashi/tc2026.git ~/colcon_ws
# 既存 clone の場合
# git submodule update --init --recursive

cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

開発時は `--symlink-install` 付きのビルドを推奨する。Python ソース、launch、config、
route、map、waypoint などの install 対象ファイルが `install/` 配下へ symlink されるため、
既存ファイルの内容変更を再ビルドなしで反映しやすい。
新規ファイル追加、ファイル名変更、install 対象の変更を行った場合は再ビルドする。

選択的にビルドする場合:

```bash
colcon build --symlink-install --packages-select rtk_gps_um982_msgs rtk_gps_um982
colcon build --symlink-install --packages-select ypspur_ros2
```

## 起動例

代表的な運用単位の起動手順を示す。各ノードの詳細な引数、topic、GUI 操作、
route 設定は各パッケージ README を参照する。

### 実機

実機では `ypspur-coordinator` と `ypspur_ros2` が `/cmd_vel` を車体へ渡す。
`drive_mode_manager` または `robot_console` から起動した走行 stack が最終 `/cmd_vel` を publish する。

#### 手動走行のみ

手動走行だけを行う場合は、`drive_mode_manager` が Joy 入力から最終 `/cmd_vel` を publish し、
`ypspur_ros2` が `/cmd_vel` を車体へ渡す構成にする。

coordinator を別端末で手動起動する場合:

```bash
# 端末 1: yp-spur coordinator
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param

# 端末 2: /cmd_vel を購読して車体へ速度指令を渡す
ros2 launch ypspur_ros2 ypspur_ros2.launch.py cmd_vel_topic:=/cmd_vel

# 端末 3: joy_node を起動し、Joy 入力から最終 /cmd_vel を publish
ros2 launch drive_mode_manager drive_mode_manager.launch.py
```

coordinator も `ypspur_ros2.launch.py` から起動する場合:

```bash
# 端末 1: coordinator と ypspur_node を起動
ros2 launch ypspur_ros2 ypspur_ros2.launch.py \
  start_coordinator:=true \
  coordinator_device:=/dev/ttyACM0 \
  coordinator_param:=<robot_param_file> \
  cmd_vel_topic:=/cmd_vel

# 端末 2: joy_node を起動し、Joy 入力から最終 /cmd_vel を publish
ros2 launch drive_mode_manager drive_mode_manager.launch.py
```

起動直後は `drive_mode_manager` の既定モードが `autonomous` のため、Joy 入力で L1 と PS button
を長押しして手動走行へ切り替える。GUI が不要な端末では
`ros2 launch drive_mode_manager drive_mode_manager.launch.py start_gui:=false` を使う。
開発用 Joy simulator を使う場合は `joy_input:=ps3_joy_sim` を追加する。

#### 自律走行 / 手動走行

自律走行と手動走行を切り替えて運用する場合は、先に実機 driver 側を起動し、
別端末で `robot_console` を起動する。`robot_console` から route stack と
`drive_mode_manager` を起動し、必要に応じて Joy 入力で手動介入する。

coordinator を別端末で手動起動する場合:

```bash
# 端末 1: yp-spur coordinator
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param

# 端末 2: /cmd_vel を購読して車体へ速度指令を渡す
ros2 launch ypspur_ros2 ypspur_ros2.launch.py cmd_vel_topic:=/cmd_vel

# 端末 3: 運用 GUI ダッシュボード
ros2 launch robot_console robot_console.launch.py
```

coordinator も `ypspur_ros2.launch.py` から起動する場合:

```bash
# 端末 1: coordinator と ypspur_node を起動
ros2 launch ypspur_ros2 ypspur_ros2.launch.py \
  start_coordinator:=true \
  coordinator_device:=/dev/ttyACM0 \
  coordinator_param:=<robot_param_file> \
  cmd_vel_topic:=/cmd_vel

# 端末 2: 運用 GUI ダッシュボード
ros2 launch robot_console robot_console.launch.py
```

`robot_console` からは、`route_planner`、`route_manager`、`route_follower`、
`obstacle_monitor`、`drive_mode_manager`、`robot_navigator` などを起動する。
`robot_navigator` は自律走行指令を `/cmd_vel/autonomous` へ publish し、
`drive_mode_manager` が自律走行指令と Joy 由来の手動走行指令を切り替えて最終 `/cmd_vel` を publish する。
実機運用時の起動カード、パラメータ、操作手順の詳細は
[`src/robot_console/README.md`](src/robot_console/README.md) を参照する。

### シミュレーション

#### 障害物回避シミュレーション

Gazebo GUI 付きで道路 world、robot、bridge、fake localization pose、TF を起動する。
`obstacle_route_sim` は Gazebo 上の真値 pose から `/localization/pose_enu` を配信するため、経路追従側は
自己位置推定誤差なしの前提で結合確認できる。

```bash
# 端末 1: Gazebo world と robot を起動
ros2 launch obstacle_route_sim sim_obstacle_route.launch.py \
  road_type:=straight \
  road_width:=5.0 \
  enable_pylons:=false \
  start_gazebo_gui:=true
```

pylon 障害物ありで起動する場合:

```bash
# 端末 1: pylon 障害物ありで Gazebo world と robot を起動
ros2 launch obstacle_route_sim sim_obstacle_route.launch.py \
  road_type:=crank \
  road_width:=5.0 \
  enable_pylons:=true \
  pylon_seed:=0 \
  start_gazebo_gui:=true
```

Gazebo 起動後、別端末で `robot_console` を起動する。

```bash
# 端末 2: 運用 GUI ダッシュボード
ros2 launch robot_console robot_console.launch.py
```

`robot_console` からは、`route_planner`、`route_manager`、`route_follower`、
`obstacle_monitor`、`drive_mode_manager`、`robot_navigator` を起動する。
pylon ありで障害物回避を確認する場合は、`obstacle_monitor` も起動し、LiDAR 入力から
`/obstacle_avoidance_hint` を publish する構成にする。route id、goal label、
起動カードの選択、GUI 自動操作による確認手順の詳細は
[`src/obstacle_route_sim/README.md`](src/obstacle_route_sim/README.md) を参照する。

## 外部依存 (submodule)

ビルド前に `git submodule update --init --recursive` が必要。

- `src/rtk_gps_um982/third_party/UM982-RTK-GPS-Library` (MIT)
- `src/ypspur_ros2/third_party/yp-spur` (MIT) — Issue #245 のパッチを CMake が自動適用

## ライセンス

各パッケージは MIT。本リポジトリ全体としてのライセンスは [`LICENSE`](LICENSE) を参照。
