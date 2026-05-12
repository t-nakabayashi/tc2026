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
| [`src/route_msgs`](src/route_msgs/README.md)                | 経路・走行系で共有する msg / srv 定義 (`Route`, `RouteState`, `ReportStuck` ほか) |

### 走行制御・障害物
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/robot_navigator`](src/robot_navigator/README.md)      | `/active_target` を追従して `/cmd_vel` を出力する時間最適制御ノード。試験用 `robot_simulator` も同梱 |
| [`src/obstacle_monitor`](src/obstacle_monitor/README.md)    | `/scan` を解析して `/obstacle_avoidance_hint` を配信。`/sensor_viewer` への可視化も提供 |

### 認識・監視
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/yolo_detector`](src/yolo_detector/README.md)          | USB カメラ画像を YOLO (PyTorch / NCNN) で物体検出し、検出画像・`Detection2DArray` を配信 |
| [`src/robot_console`](src/robot_console/README.md)          | 走行状態・障害物回避・経路進捗・ノード起動を一画面で監視する tkinter GUI ダッシュボード |

ワークスペース横断の仕様書は [`docs/`](docs/)、パッケージ固有の設計書は各パッケージ
配下の `docs/design.md` を参照。

## 必要環境

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- (パッケージごとの追加要件は各 README を参照)

## Python 依存モジュール

Python パッケージ群で使用する pip 依存モジュールは、[`requirements.txt`](requirements.txt) にまとめている。
対象は `obstacle_monitor`, `robot_console`, `robot_navigator`, `route_follower`,
`route_manager`, `route_planner`, `yolo_detector` と、それらが利用する
`route_msgs`。

ROS 2 の環境を読み込んだうえで、ワークスペース直下で以下を実行する。

```bash
python3 -m pip install -r requirements.txt
```

## ビルド

本リポジトリ自体を colcon ワークスペースとして利用します（直下に `src/` が
含まれているため、別途 `src/` を作成する必要はありません）。

### 1. ソースの取得

```bash
# 新規取得 (submodule もまとめて clone)
git clone --recursive https://github.com/t-nakabayashi/tc2026.git ~/colcon_ws
```

`--recursive` を付け忘れた／既に clone 済みの場合は、リポジトリ内で
submodule のみ取得し直してください。

```bash
cd ~/colcon_ws
git submodule update --init --recursive
```

### 2. 依存関係の導入とビルド

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
python3 -m pip install -r requirements.txt
colcon build
source install/setup.bash
```

選択的にビルドする場合:

```bash
colcon build --packages-select rtk_gps_um982_msgs rtk_gps_um982
colcon build --packages-select ypspur_ros2
```

## 起動例

各パッケージごとに代表的な launch を抜粋。詳細な引数・トピック名は各パッケージ README を参照。

### センサ / アクチュエータドライバ
```bash
# RTK GPS ドライバ
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py

# yp-spur ロボット制御 (別端末で ypspur-coordinator も起動)
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param
ros2 launch ypspur_ros2 ypspur_ros2.launch.py
```

### 経路計画・追従
```bash
# 経路生成サービス (route_planner)
ros2 launch route_planner route_planner.launch.py

# 経路管理 FSM (route_manager)
ros2 launch route_manager route_manager.launch.py \
  start_label:=START goal_label:=GOAL \
  checkpoint_labels:="P1,P2"

# 経路追従 (route_follower)
ros2 launch route_follower route_follower.launch.py \
  arrival_threshold:=0.6 \
  control_rate_hz:=20.0 \
  start_immediately:=true
```

### 走行制御・障害物
```bash
# /active_target を追従して /cmd_vel を出力
ros2 launch robot_navigator robot_navigator.launch.py \
  obstacle_hint_topic:=/obstacle_avoidance_hint cmd_vel_topic:=/cmd_vel

# 障害物監視 (LiDAR 入力 → /obstacle_avoidance_hint)
ros2 launch obstacle_monitor obstacle_monitor.launch.py \
  scan_topic:=/scan hint_topic:=/obstacle_avoidance_hint
```

### 認識・GUI
```bash
# YOLO 物体検出 (NCNN 版を推奨)
ros2 launch yolo_detector yolo_ncnn_node.launch.py

# 運用 GUI ダッシュボード
ros2 launch robot_console robot_console.launch.py
```

## 外部依存 (submodule)

ビルド前に `git submodule update --init --recursive` が必要。

- `src/rtk_gps_um982/third_party/UM982-RTK-GPS-Library` (MIT)
- `src/ypspur_ros2/third_party/yp-spur` (MIT) — Issue #245 のパッチを CMake が自動適用

## ライセンス

各パッケージは MIT。本リポジトリ全体としてのライセンスは [`LICENSE`](LICENSE) を参照。
