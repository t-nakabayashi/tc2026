# tc2026

ROS 2 Jazzy ワークスペース。初めて読む場合は[システム構成](docs/次期システム_アーキテクチャ検討レポート.md)から参照する。

## つくばチャレンジ2026の公式情報

[走行システムの機能と制約](docs/reviews/tc2026_software_20260922/README.md)

[公式ルール・参加条件・日程の参照ガイド](docs/references/tsukuba_challenge_2026/README.md)
に、出典URL、未公表事項、公式ページ間の記載差、実装確認事項をまとめています。
大会条件に関係する設計・設定・走行準備では、この資料とリンク先の最新公式情報を参照してください。

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
| [`src/gnss_lio_fusion`](src/gnss_lio_fusion/README.md) | GNSSと水平化したFAST-LIOを融合し、車輪odomによる退避と自律速度制限を行う |
| [`src/route_survey`](src/route_survey/README.md) | 手動走行の融合位置からLLH経路を採取し、観測した路面の横移動余裕を記録・編集する |
| [`src/icart_bringup`](src/icart_bringup/README.md) | 実機・デジタルツイン・手動採取・保存経路走行の共通セッションを準備・起動する |

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
| [`src/tc_perception_msgs`](src/tc_perception_msgs/README.md) | 検出矩形・判定状態を表示側へ渡す認識結果の共通メッセージ |
| [`src/yolo_detector`](src/yolo_detector/README.md)          | USB カメラ画像を YOLO (PyTorch / NCNN) で物体検出し、検出画像・`Detection2DArray` を配信 |
| [`src/traffic_signal_recognizer`](src/traffic_signal_recognizer/README.md) | YOLO検出結果から信号のGO/STOPを判定し、検出矩形と判定データを配信 |
| [`src/road_blockage_detector`](src/road_blockage_detector/README.md) | YOLO検出結果と自己位置から道路封鎖を判定し、検出矩形と判定データを配信 |
| [`src/robot_console`](src/robot_console/README.md)          | 走行状態・障害物回避・経路進捗・ノード起動を一画面で監視する PyQt5 GUI ダッシュボードとHTML遠隔観測UI |

ワークスペース横断の仕様書は [`docs/`](docs/)、パッケージ固有の設計書は各パッケージ
配下の `docs/` を参照。

## ハードウェアパーツ

実機の取付部品のCAD、3Dプリント用STL、組立説明は
[`hardware_parts/`](hardware_parts/README.md) に部品単位で配置する。
ROSパッケージは `src/`、センサのメーカー資料は `docs/references/` で管理する。

## 開発状態

- `robot_console` の正式UIは PyQt5 版（`robot_console_qt`）である。遠隔観測用の
  HTML UI（`robot_console_web`）も同じ `ConsoleCore` の状態を表示する。
- 共通起動では `gnss_lio_fusion` が `/localization/pose_enu` を配信する。
  FAST-LIOの `/lio/odometry_raw` を重力方向に水平化し、`/lio/odometry` を融合・採取で共用する。
  GNSS単独の変換結果は `/gnss/pose_enu` に分離し、融合出力と競合させない。
  GNSS単独や真値poseを使う単体試験構成もあるため、共通起動と同時に起動しない。
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
Ubuntu 24.04ではROSのaptパッケージを参照できるvenvを作成し、その中へpip依存を導入する。
`<venv>` は開発者が選んだ仮想環境のパスとする。

```bash
python3 -m venv --system-site-packages <venv>
source <venv>/bin/activate
python3 -m pip install -r requirements.txt
```

## ビルド

```bash
git clone --recursive https://github.com/t-nakabayashi/tc2026.git ~/colcon_ws
# 既存 clone の場合
# git submodule update --init --recursive

cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
# 上記で準備したvenvを有効化する
source <venv>/bin/activate
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

センサ一式・時刻同期監視・融合・経路採取・保存済み経路での走行は
[`icart_bringup`](src/icart_bringup/README.md) の共通起動を使う。
操作画面は次のコマンドで起動する。

```bash
ros2 launch robot_console robot_console.launch.py
```

GUIで実機モードを選び、手動採取または保存経路走行のセッションを準備する。
[操作手順](src/robot_console/README.md)と
[実機設定・時計・Joyの割当](src/icart_bringup/docs/実機ハードウェア統合.md)を確認する。
ハードウェア設定の正本は `src/icart_bringup/params/hardware.yaml` である。
生成済みセッションには設定のコピーが入るため、ソース変更後はセッションを再生成する。

共通起動では同期成立後に車輪・FAST-LIO・融合・走行制御が起動する。
`robot_navigator` の `/cmd_vel/autonomous` を融合側の速度制限と
`drive_mode_manager` の自律／手動切替に通し、最終 `/cmd_vel` を車輪へ渡す。
手動採取は手動モードを固定し、L1を押して操作する。R1はGNSS途絶模擬、ターボ割当は無効。

個別の診断起動は [車輪ドライバ](src/ypspur_ros2/README.md)、
[手動切替](src/drive_mode_manager/README.md)、[GNSS](src/rtk_gps_um982/README.md)を参照する。
共通起動と同じドライバや `/cmd_vel` の配信元を二重に起動しない。

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
- `src/FAST_LIO` (GPL-2.0) — ROS 2 Jazzy 用の固定 revision。内部の ikd-Tree も再帰取得する
- `src/livox_ros_driver2` (MIT) — FAST-LIO の Livox メッセージと実機ドライバ
- `src/livox_sdk2_vendor/third_party/Livox-SDK2` — SDK 本体と同梱依存のライセンスは上流 `LICENSE.txt` を参照

SDK は [`livox_sdk2_vendor`](src/livox_sdk2_vendor/README.md) が workspace 内へビルドする。
ルートの `colcon.meta` が driver より先に SDK を構築する順序を指定するため、
colcon はワークスペースのルートから実行する。

## GNSS/LIO・デジタルツイン・経路採取

つくば2026の完成済み試験地図をGitに同梱している。
[同梱地図の入口](src/obstacle_route_sim/maps/tsukuba2026/README.md)には、
ネット接続不要の3D閲覧版と静止画像がある。再帰clone後、通常のROS依存導入・ビルドを行えば、
地図のダウンロードや再生成なしで次の1コマンドから模擬確認を始められる。

```bash
source install/setup.bash
ros2 run icart_bringup run_digital_twin --output log/digital_twin/session01 --start-ui
```

`preview.html` の閲覧だけならROS環境は不要。シミュレーションの実行にはROS/Gazebo等が必要。

- [GNSS/LIO 融合](src/gnss_lio_fusion/README.md): 品質判定、時刻同期、方位推定
- [共通起動](src/icart_bringup/README.md): 実機・模擬環境の選択とセッション準備
- [経路採取・編集](src/route_survey/README.md): 手動走行から LLH 経路を保存
- [シミュレーション](src/obstacle_route_sim/README.md): 地形・センサ生成と評価
- [検証用経路](src/route_planner/routes/tsukuba2026_digital_twin/README.md): 未測量の試験データ

全体テストは `python3 scripts/run_pytest.py -v` で実行する。対象は `pytest.ini` の
`testpaths` から取得する。ROS 環境とビルド済み `install/setup.bash` を読み込み、
有効な venv で実行する。GUI とその他を別の Python プロセスで実行し、DDS 初期化後の
fork による停止を避ける。各グループが600秒を超えた場合は失敗とする。
単一パッケージは `python3 -m pytest src/<package>/tests`（robot_console は
`src/robot_console/tools/tests`）で実行できる。
GUI テストには Qt WebEngine と pytest-forked、地形生成には Node.js が必要。
地理地形の生成には `src/obstacle_route_sim/tools/terrain3d` で `npm ci` も実行する。
通常 install のツールを使う場合は、
`npm ci --prefix "$(ros2 pkg prefix obstacle_route_sim)/lib/obstacle_route_sim/terrain3d"`
で実行先へ依存を導入する。`node_modules` は Git と colcon の配布対象から除外する。

## ライセンス

リポジトリ全体のライセンスは [`LICENSE`](LICENSE) を参照。
各パッケージの宣言は `package.xml`、外部コードは各 submodule のライセンスに従う。

実機用の一括設定・稲城／つくばのRTK局選択は
[実機ハードウェア統合](src/icart_bringup/docs/実機ハードウェア統合.md)を参照。
MID-360の実機取付角・アンテナ位置、採取時の手動固定を含む。
現行の実機設定は `src/icart_bringup/params/hardware.yaml` を正とし、
既存sessionの設定コピーへソースの変更が自動反映されるとは扱わない。
