# drive_mode_manager 詳細設計書

作成日: 2026-05-19

## 1. 文書目的・対象範囲

本書は、`drive_mode_manager` パッケージで実装する自律走行・手動走行切替機能、
手動速度指令生成、専用状態表示 GUI の詳細設計を定義する。

対象ノードは以下である。

| ノード | 実装予定ファイル | 役割 |
| --- | --- | --- |
| `manual_teleop_node` | `drive_mode_manager/manual_teleop_node.py` | `/joy` から手動 `/cmd_vel/manual` を生成する |
| `drive_cmd_mux_node` | `drive_mode_manager/drive_cmd_mux_node.py` | 自律 cmd と手動 cmd を排他的に選択し、最終 `/cmd_vel` を publish する |
| `drive_status_gui_node` | `drive_mode_manager/drive_status_gui_node.py` | 走行状態、出力 cmd、自律復帰カウントダウンを常時表示する |

本書は設計中仕様を確定するための文書であり、現時点ではパッケージ骨格と本設計書のみを
配置する。実装時は本書に従い、ROS 非依存ロジックを `drive_mode_core.py` と
`manual_teleop_core.py` に分離する。

参照した入力は、添付された自律走行・手動走行切替方式の検討資料、tc2025 ROS 1 資産の
`scripts/joystick_teleop.py`、`scripts/total_gui.py`、`launch/1_init_robot_tc2024_joystick.launch`、
および tc2026 の既存 `robot_navigator`, `ypspur_ros2`, `robot_console`, `tc_route_msgs` の設計である。

## 2. 背景・要求・スコープ

tc2025 の `joystick_teleop.py` は、L1 相当のデッドマン入力が押されている間は手動速度指令を
`ypspur_ros/cmd_vel` へ出し、デッドマン OFF の間は外部 `cmd_vel` を ypspur 側へ通す構造であった。
この方式は単純で扱いやすい一方、自律・手動の運用状態、実出力ソース、自律復帰タイミングを
明示する interface が不足していた。

tc2026 では、`robot_navigator` が `/active_target` から自律速度指令を生成し、`ypspur_ros2` が
最終 `/cmd_vel` を受けて車体を駆動する。自律 cmd と手動 cmd が同じ `/cmd_vel` へ直接 publish
する構成は、出力元が不明確になり、安全判断と GUI 表示が難しくなる。

本パッケージの責務は以下である。

- `robot_navigator` の出力を `/cmd_vel/autonomous` へ分離する。
- PS3 コントローラー由来の手動入力を `/cmd_vel/manual` へ分離する。
- `drive_cmd_mux_node` が最終 `/cmd_vel` を唯一 publish する。
- 走行状態を `AUTONOMOUS` / `MANUAL` の 2 状態として管理する。
- 実出力ソースを `ZERO` / `AUTONOMOUS_CMD` / `MANUAL_CMD` として別途公開する。
- `AUTONOMOUS -> MANUAL` は誤操作を避ける複合操作でのみ成立させる。
- `MANUAL -> AUTONOMOUS` は L1 入力なしの継続と自律 cmd 有効性により自動復帰する。
- 自律復帰直後は 5 秒間ゼロ速度を出力し、復帰予定 cmd を GUI に表示する。

スコープ外は以下である。

- 物理緊急停止回路の代替。
- `robot_navigator` の追従アルゴリズム変更。
- `route_follower` の信号停止・手動開始イベント仕様変更。
- GPS/LiDAR 融合ローカライザーそのものの実装。
- `robot_console` 全体画面への統合表示。

## 3. 全体構成・アーキテクチャ

速度指令の流れは以下とする。

```text
robot_navigator
  -> /cmd_vel/autonomous

manual_teleop_node
  -> /cmd_vel/manual

drive_cmd_mux_node
  -> /cmd_vel

ypspur_ros2
  <- /cmd_vel
```

`drive_cmd_mux_node` は `/joy` も購読し、L1/PS ボタン状態、入力鮮度、長押し時間を
状態遷移判定に使う。`manual_teleop_node` も `/joy` を購読するが、こちらは stick 軸から
手動 Twist を生成するだけであり、最終出力の採否は `drive_cmd_mux_node` が決定する。

GUI は `/drive_mode_status` と `/cmd_vel` を主入力とする。GPS/RTK 状態や waypoint 情報は
補助表示であり、購読できない場合でも走行切替表示は継続する。

## 4. パッケージ構成・ファイル配置

推奨構成は以下である。

```text
src/drive_mode_manager/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── drive_mode_manager
├── drive_mode_manager/
│   ├── __init__.py
│   ├── drive_cmd_mux_node.py
│   ├── drive_mode_core.py
│   ├── manual_teleop_node.py
│   ├── manual_teleop_core.py
│   ├── drive_status_gui_node.py
│   └── gui_core.py
├── launch/
│   └── drive_mode_manager.launch.py
├── params/
│   └── default.yaml
├── docs/
│   └── drive_mode_manager_詳細設計書.md
└── tests/
    ├── test_drive_mode_core.py
    └── test_manual_teleop_core.py
```

現時点で作成するのは、ROS 2 パッケージとして認識できる最小骨格と本詳細設計書である。
`launch/`, `params/`, `tests/` は実装フェーズで追加する。

`DriveModeStatus.msg` は、GUI やログ解析など他パッケージからも参照される共有 interface である。
実装時は `tc_route_msgs/msg/DriveModeStatus.msg` として追加する方針を基本とする。

## 5. 外部インタフェース仕様

### 5.1 `manual_teleop_node`

| 方向 | Topic | Type | QoS | 内容 |
| --- | --- | --- | --- | --- |
| Subscribe | `/joy` | `sensor_msgs/msg/Joy` | Reliable / Volatile / depth 10 | PS3 コントローラー入力 |
| Publish | `/cmd_vel/manual` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 手動速度指令 |

`manual_teleop_node` は L1 押下中のみ stick 軸を Twist に変換する。L1 非押下、入力 timeout、
軸数不足、NaN/Inf 検出時はゼロ Twist を publish する。

### 5.2 `drive_cmd_mux_node`

| 方向 | Topic | Type | QoS | 内容 |
| --- | --- | --- | --- | --- |
| Subscribe | `/cmd_vel/autonomous` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 自律速度指令 |
| Subscribe | `/cmd_vel/manual` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 手動速度指令 |
| Subscribe | `/joy` | `sensor_msgs/msg/Joy` | Reliable / Volatile / depth 10 | 状態遷移判定用 controller 入力 |
| Publish | `/cmd_vel` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | `ypspur_ros2` へ渡す最終速度指令 |
| Publish | `/drive_mode_status` | `tc_route_msgs/msg/DriveModeStatus` | Reliable / Volatile / depth 10 | 走行状態と出力状態 |

`/cmd_vel` は本ノードだけが publish する。`robot_navigator` は launch remap で
`/cmd_vel/autonomous` へ出力させる。

### 5.3 `drive_status_gui_node`

| 方向 | Topic | Type | QoS | 内容 |
| --- | --- | --- | --- | --- |
| Subscribe | `/drive_mode_status` | `tc_route_msgs/msg/DriveModeStatus` | Reliable / Volatile / depth 10 | 走行切替状態 |
| Subscribe | `/cmd_vel` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 最終出力の補助確認 |
| Subscribe | `/cmd_vel/autonomous` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 復帰予定 cmd の補助確認 |
| Subscribe | `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/msg/RtkStatus` | Reliable / Volatile / depth 10 | 任意の RTK 表示 |
| Subscribe | `/follower_state` | `tc_route_msgs/msg/FollowerState` | Reliable / Volatile / depth 10 | 任意の次 waypoint 表示 |
| Subscribe | `/manager_status` | `tc_route_msgs/msg/ManagerStatus` | Reliable / Volatile / depth 10 | 任意の自律走行状態表示 |

GUI は表示専用とし、状態遷移コマンドや速度指令を publish しない。

## 6. パラメータ・設定仕様

### 6.1 `drive_cmd_mux_node`

| パラメータ | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `initial_mode` | string | `autonomous` | 起動直後の走行状態。`autonomous` または `manual` |
| `manual_transition_trigger` | string | `l1_ps_button_hold` | 手動遷移トリガ種別 |
| `manual_transition_hold_s` | double | `2.0` | L1 + PS 長押し判定時間 |
| `manual_to_auto_l1_released_s` | double | `1.0` | L1 入力なし継続判定時間 |
| `auto_resume_delay_s` | double | `5.0` | 自律復帰後にゼロ出力する猶予時間 |
| `autonomous_cmd_timeout_s` | double | `0.5` | 自律 cmd 有効期限 |
| `manual_cmd_timeout_s` | double | `0.3` | 手動 cmd 有効期限 |
| `joy_timeout_s` | double | `0.5` | `/joy` 入力有効期限 |
| `publish_rate_hz` | double | `20.0` | `/cmd_vel` と status の publish 周期 |
| `l1_button_index` | int | `4` | PS3 L1 ボタン index |
| `ps_button_index` | int | `16` | PS ボタン index。実機確認後に調整する |

`ps_button_index` は controller driver により変わる可能性がある。PS ボタンが `/joy` で安定して
取得できない場合は、`manual_transition_trigger` を `l1_start_button_hold` などに変更できる
構成にする。

### 6.2 `manual_teleop_node`

| パラメータ | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `linear_axis` | int | `1` | 左 stick 縦軸 |
| `angular_axis` | int | `0` | 左 stick 横軸 |
| `linear_y_axis` | int | `-1` | 横移動軸。差動二輪では未使用 |
| `linear_scale` | double | `1.2` | 直進速度倍率 |
| `angular_scale` | double | `1.5` | 角速度倍率 |
| `linear_y_scale` | double | `0.5` | 横移動速度倍率 |
| `deadzone` | double | `0.05` | stick deadzone |
| `linear_axis_invert` | bool | `false` | 縦軸反転 |
| `angular_axis_invert` | bool | `false` | 横軸反転 |
| `enable_button` | int | `4` | L1 デッドマンボタン |
| `turbo_button` | int | `5` | R1 turbo ボタン |
| `turbo_ratio` | double | `1.5` | turbo 倍率 |
| `joy_timeout_s` | double | `0.5` | 入力 timeout |
| `publish_rate_hz` | double | `20.0` | `/cmd_vel/manual` publish 周期 |

tc2025 の `joystick_teleop.py` と同じ軸・ボタン・倍率を初期値として採用する。ただし、
自律 cmd の passthrough は本ノードでは行わない。

### 6.3 `drive_status_gui_node`

| パラメータ | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `main_display_ratio` | double | `0.70` | GUI 主表示帯の高さ比率。許容範囲は 0.60 から 0.80 |
| `turn_preview_seconds` | double | `1.0` | 互換用。現行の復帰方向テキストと矢印角度判定では使用しない |
| `direction_linear_scale` | double | `1.2` | 進行方向矢印用に `cmd_vel.linear.x` から stick 縦軸を逆算する scale |
| `direction_angular_scale` | double | `1.5` | 進行方向矢印用に `cmd_vel.angular.z` から stick 横軸を逆算する scale |
| `direction_deadzone` | double | `0.05` | 逆算した stick 座標へ適用する deadzone |
| `direction_linear_axis_invert` | bool | `false` | 進行方向矢印用の並進軸符号反転 |
| `direction_angular_axis_invert` | bool | `true` | 進行方向矢印用の角速度軸符号反転。`angular.z > 0` を tc2025 実機・ypspur_ros2 の左旋回表示へ合わせる |
| `manual_to_auto_l1_released_s` | double | `1.0` | 手動中に L1 を離してから自律復帰条件成立までの表示用しきい値 |
| `max_autonomous_resume_linear_x` | double | `0.8` | 互換用。現行の復帰方向テキストでは角度判定を優先する |
| `max_autonomous_resume_angular_z` | double | `1.2` | 互換用。現行の復帰方向テキストでは角度判定を優先する |

## 7. データモデル・内部状態

### 7.1 走行状態

走行状態は以下の 2 状態だけとする。

| 状態 | 意味 |
| --- | --- |
| `AUTONOMOUS` | 自律走行系を採用する状態 |
| `MANUAL` | 手動走行入力を採用可能な状態 |

### 7.2 実出力ソース

実出力ソースは、GUI とログで現在の `/cmd_vel` の由来を明示するために持つ。

| ソース | 意味 |
| --- | --- |
| `ZERO` | ゼロ Twist を出力中 |
| `AUTONOMOUS_CMD` | `/cmd_vel/autonomous` を出力中 |
| `MANUAL_CMD` | `/cmd_vel/manual` を出力中 |

`AUTONOMOUS` 状態でも、自律復帰カウントダウン中や自律 cmd timeout 時は `ZERO` になる。

### 7.3 `DriveModeStatus.msg` 案

```text
builtin_interfaces/Time stamp

uint8 MODE_AUTONOMOUS=1
uint8 MODE_MANUAL=2
uint8 mode

uint8 SOURCE_ZERO=0
uint8 SOURCE_AUTONOMOUS_CMD=1
uint8 SOURCE_MANUAL_CMD=2
uint8 output_source

bool joy_available
bool l1_pressed
bool ps_button_pressed
float32 ps_hold_progress_s

bool manual_input_active
bool manual_cmd_alive
bool autonomous_cmd_alive

bool auto_resume_pending
float32 auto_resume_remaining_s

float32 pending_autonomous_linear_x
float32 pending_autonomous_angular_z

float32 output_linear_x
float32 output_angular_z

string reason
```

`stamp` は `std_msgs/Header` ではなく単独 `builtin_interfaces/Time` とし、表示用 status として
frame_id を持たない。将来 frame_id が必要になった場合は `Header` への変更ではなく別 msg を検討する。

## 8. 処理フロー・状態遷移

### 8.1 `AUTONOMOUS -> MANUAL`

以下の条件が継続した場合に `MANUAL` へ遷移する。

```text
joy_available
and l1_pressed
and ps_button_pressed
and hold_time >= manual_transition_hold_s
```

L1 単独、PS 単独、stick 操作単独では遷移しない。遷移直後は `/cmd_vel` をゼロにし、
次周期から L1 押下中かつ手動 cmd 有効時のみ `/cmd_vel/manual` を採用する。

### 8.2 `MANUAL -> AUTONOMOUS`

以下の条件が成立した場合に `AUTONOMOUS` へ遷移する。

```text
l1_pressed == false が manual_to_auto_l1_released_s 以上継続
and autonomous_cmd_alive
```

`/joy` が timeout した場合は L1 入力なしとして扱う。ただし即座に自律 cmd を出力せず、
`auto_resume_delay_s` のカウントダウンを必ず経由する。

### 8.3 出力選択

`AUTONOMOUS` 状態の選択ルールは以下である。

| 条件 | 出力 | `output_source` |
| --- | --- | --- |
| 自律復帰カウントダウン中 | ゼロ Twist | `ZERO` |
| カウントダウン外かつ自律 cmd 有効 | 最新 `/cmd_vel/autonomous` | `AUTONOMOUS_CMD` |
| 自律 cmd timeout | ゼロ Twist | `ZERO` |

`MANUAL` 状態の選択ルールは以下である。

| 条件 | 出力 | `output_source` |
| --- | --- | --- |
| L1 押下中かつ手動 cmd 有効 | 最新 `/cmd_vel/manual` | `MANUAL_CMD` |
| L1 押下中かつ手動 cmd timeout | ゼロ Twist | `ZERO` |
| L1 非押下 | ゼロ Twist | `ZERO` |
| `/joy` timeout | ゼロ Twist | `ZERO` |

## 9. 主要アルゴリズム・判定ロジック

### 9.1 `JoyState` 正規化

`/joy` callback では、ボタン配列長を確認して L1/PS の押下状態を抽出する。index が範囲外の場合は
押下なしとして扱い、`reason` に `joy_button_index_out_of_range` を設定する。

入力鮮度は `last_joy_time` と node clock の差で判定する。`joy_timeout_s` を超えた場合は
`joy_available=false` とし、L1/PS は false 扱いにする。

### 9.2 `Twist` 有効性判定

`Twist` は以下を満たす場合だけ有効とする。

- 最新受信時刻から timeout を超えていない。
- `linear.x`, `linear.y`, `angular.z` が NaN/Inf ではない。
- 設計上使わない成分が非ゼロでも即異常にはしないが、最終出力では `linear.x`, `angular.z` を主表示とする。

### 9.3 自律復帰予定 cmd 表示

自律復帰カウントダウン中は、最新 `/cmd_vel/autonomous` の `linear.x` と `angular.z` を
`pending_autonomous_*` に格納する。復帰予定 cmd が timeout した場合は、
`auto_resume_pending=true` のまま出力はゼロを維持し、`reason=autonomous_cmd_stale` とする。

## 10. QoS・並行性・タイミング設計

`drive_cmd_mux_node` は 20Hz timer で状態更新、出力選択、status publish を行う。
購読 callback は最新値と受信時刻の更新だけを行い、状態遷移は timer に集約する。

QoS は速度指令系を Reliable / Volatile / depth 1 とする。速度指令は最新値が重要であり、
古い queue を溜めない。`/drive_mode_status` は GUI 表示向けに depth 10 とする。

`drive_status_gui_node` は Qt の GUI スレッドと rclpy executor スレッドを分ける。
ROS callback から Qt widget や scene item を直接更新せず、`gui_core.py` の dataclass snapshot を
lock 付きで更新する。GUI スレッドは `QTimer` により 100ms 周期で snapshot を読み取り、
描画へ反映する。

## 11. 起動・終了・launch 設計

`drive_mode_manager.launch.py` は以下を起動する。

| ノード | 既定起動 | 備考 |
| --- | --- | --- |
| `joy_node` | `joy_input=joy_node` のとき起動 | 実 controller から `/joy` を publish する既定の入力源 |
| `ps3_joy_sim_node` | `joy_input=ps3_joy_sim` のとき起動 | 開発用にキーボード入力から `/joy` を publish する。`joy_node` とは同時起動しない |
| `manual_teleop_node` | 常時 | `/joy` から `/cmd_vel/manual` を生成する。mux と同時起動する |
| `drive_cmd_mux_node` | 常時 | `/cmd_vel/autonomous` と `/cmd_vel/manual` を選択し、最終 `/cmd_vel` を publish する |
| `drive_status_gui_node` | `start_gui` で制御 | 実機運用時は別画面表示を推奨。headless 評価では起動しない |

`tc2026_system_bringup` から実機 profile を起動する場合は、`robot_navigator` の
`cmd_vel_topic` を `/cmd_vel/autonomous` に remap し、`ypspur_ros2` は `/cmd_vel` を購読する。

終了時、`drive_cmd_mux_node` はゼロ Twist を publish してから shutdown する。
ただし、最終停止は `ypspur_ros2` の timeout と物理停止系にも依存するため、本ノード単独を
安全停止の唯一手段とはしない。

## 12. エラー処理・ログ・診断

| 事象 | 処理 | ログ |
| --- | --- | --- |
| `/joy` timeout | L1/PS 非押下扱い、出力ゼロ、MANUAL では自律復帰判定対象 | `warn` を throttle |
| PS ボタン index 不正 | 手動遷移は成立させない | 起動時 `warn`、status reason |
| 自律 cmd timeout | `AUTONOMOUS` でも出力ゼロ | `warn` を throttle |
| 手動 cmd timeout | `MANUAL` でも出力ゼロ | `warn` を throttle |
| NaN/Inf Twist | 該当 cmd を無効扱い | `error` を throttle |
| status publish 失敗 | 次周期で再試行 | `warn` |

通常の状態遷移は `info`、周期的な状態詳細は `debug` とする。

## 13. UI・可視化仕様

専用 GUI は `robot_console` とは別ウィンドウとし、運用者が常時確認できる表示にする。
つくばチャレンジ2025「ロボット仕様条件」の状態表示に準拠し、主表示では状態に応じて
以下の色と文字を必ず表示する。

| 競技上の状態表示 | 背景色 | 表示文字 | 対応する内部状態 |
| --- | --- | --- | --- |
| 自律走行 | 緑色 | `自律 / Auto` | `mode=MODE_AUTONOMOUS`。自律復帰カウントダウン中を含む |
| マニュアル走行 | 黄色 | `操縦 / Manual` | `mode=MODE_MANUAL` |

GUI 内部では `AUTONOMOUS` / `MANUAL` を状態名として扱うが、画面の主表示には競技規定の
`自律 / Auto` と `操縦 / Manual` を使う。自律復帰カウントダウン中は、実出力が `ZERO` でも
競技上の状態表示は `自律 / Auto` とし、残り秒数や復帰予定 cmd は補助情報として表示する。

画面は状態に応じて以下の 3 種類へ切り替える。

| 画面 | 表示条件 | 主目的 |
| --- | --- | --- |
| マニュアル走行画面 | `mode=MODE_MANUAL` | 規定状態表示 `操縦 / Manual` を主表示し、L1 デッドマンと手動 cmd の採否を補助表示する |
| 自律走行画面（自律復帰カウントダウン中） | `mode=MODE_AUTONOMOUS` かつ `auto_resume_pending=true` | 規定状態表示 `自律 / Auto` を主表示し、残り秒数と復帰予定 cmd を補助表示する |
| 自律走行画面（通常） | `mode=MODE_AUTONOMOUS` かつ `auto_resume_pending=false` | 規定状態表示 `自律 / Auto` を主表示し、自律 cmd が実際に出力されているか、または停止理由を補助表示する |

全画面で共通して、ウィンドウ幅いっぱいの上部領域を規定状態表示の主表示帯とし、背景色と文字を
最も大きく表示する。主表示帯の高さは画面高に対する比率で指定し、既定値を 70% とする。
残りの補助表示エリアには、画面種別に依存しない共通スロットを定義し、状態ごとに表示内容を
差し替える。表示領域は 16:9 の論理キャンバスとして固定し、ウィンドウサイズに合わせて
全体を等比拡縮する。スクロールは使わない。

### 13.1 共通レイアウト

論理キャンバスの幅を `W`、高さを `H` とする。既定レイアウトは以下である。

| 領域 | 位置・サイズ | 役割 |
| --- | --- | --- |
| 主表示帯 | `x=0`, `y=0`, `w=W`, `h=0.70H` | 規定状態表示。`自律 / Auto` または `操縦 / Manual` を最大表示する |
| 補助表示エリア | `x=0`, `y=0.70H`, `w=W`, `h=0.30H` | 状態判断を助ける補助情報を固定スロットで表示する |
| 補助左スロット | 補助表示エリア左 | 通常時は最終出力 cmd、自律復帰カウントダウン中は復帰予定 cmd を表示する |
| 補助中央スロット | 補助表示エリア中央 | 画面固有の最重要補助情報を表示する |
| 補助右スロット | 補助表示エリア右 | 通常時は GNSS/RTK 状態、自律復帰カウントダウン中は復帰方向と警告文を表示する |
| 主表示帯右下ラベル | 主表示帯内の右下 | mux の採用元、自律 cmd 入力有無、手動 cmd 入力有無を小さく表示する |

主表示帯の高さ比率は `main_display_ratio` パラメータで変更可能とし、許容範囲は 0.60 から 0.80 とする。
既定値 0.70 は、規定状態表示を遠目に認識しやすくしつつ、補助情報を下部に 1 行分確保するための値である。

補助左スロットには、通常時は最終出力 `/cmd_vel` を表示する。自律復帰カウントダウン中は、
現在出力がゼロであることは主表示帯右下と中央カウントダウンで示し、補助左スロットには
復帰時に採用される予定の `/cmd_vel/autonomous` を `復帰時Cmd` として表示する。並進速度は
`m/s` の数値で表示する。角速度 `angular.z` は数値より直感的な把握を優先し、`manual_teleop_node` と同じ scale で `cmd_vel` から stick 座標を逆算して
矢印で表示する。

| 値 | 表示 |
| --- | --- |
| `stick_y = linear.x / direction_linear_scale` | 円チャート上の縦座標 |
| `stick_x = -angular.z / direction_angular_scale` | 円チャート上の横座標。既定の `direction_angular_axis_invert=true` により実ロボット旋回方向を表示する |
| `hypot(stick_x, stick_y) > 1.0` | ノルム 1.0 に正規化 |

矢印は、GUI 上の円チャート中心から逆算 stick 座標へ向かう方向に描画する。描画時は方向ベクトルを単位化し、stick 座標と中心点との距離で矢印長が変化しないようにする。`linear.x=0` かつ `angular.z=0` の場合は角度が定義できないため、上方向を表示する。

`output_source=ZERO` または `linear.x=0` かつ `angular.z=0` の場合は、並進速度 `0.00 m/s` と
停止表示を出す。角速度の数値は通常表示しないが、debug 表示を有効にした場合のみ小さく併記する。

補助右スロットは、マニュアル走行画面と通常の自律走行画面で GNSS/RTK 情報を共通表示する。
表示項目は `/rtk_gps/rtk_status` を主入力とし、RTK state、衛星数、heading を短く並べる。
GNSS/RTK は走行モードに関係なく自律復帰可否や自己位置信頼性の判断に使うため、
マニュアル走行中も隠さない。自律復帰カウントダウン画面では復帰方向と警告文の確認を優先し、
補助右スロットは GNSS/RTK ではなく復帰予定 cmd から算出した方向表示に切り替える。

### 13.2 マニュアル走行画面

マニュアル走行画面では、規定状態表示として黄色背景の `操縦 / Manual` を最上段に大きく表示する。
そのうえで、オペレーターが「今、手動入力で動かせるか」「L1 を離すとどうなるか」を即時に判断できる
補助情報を配置する。

| 領域 | 表示内容 | 表示方法 |
| --- | --- | --- |
| 主表示帯 | `操縦 / Manual` | 黄色背景、画面幅いっぱい、高さは既定で画面高の 70% |
| 主表示帯の右下 | mux 状態 | 出力 cmd の採用元、自律 cmd 入力有無、手動 cmd 入力有無を表示する |
| 補助左スロット | 最終出力 cmd | 並進速度を `m/s` 数値、角速度を旋回矢印で表示する |
| 補助中央スロット | 手動入力と自律復帰条件 | 自律復帰可否、L1 ON/OFF、L1 OFF 経過秒数 / 判定しきい値を表示する |
| 補助右スロット | GNSS/RTK 状態 | 自律走行通常時と同じく、RTK state、衛星数、heading を表示する |

`MANUAL` 中に L1 が押されていない場合、`output_source` は `ZERO` であり、補助中央には
`L1: OFF` と `L1 OFF時間: <elapsed> / <threshold> 秒` を出す。L1 押下中かつ手動 cmd が
有効な場合のみ `MANUAL_CMD` として進行方向を表示する。補助中央スロットの自律復帰条件では、
`autonomous_cmd_alive` を `自律復帰: 有効/無効` として表示する。L1 押下中は
`L1: ON` とし、L1 OFF 経過秒数は復帰判定中だけ表示する。

### 13.3 自律走行画面（自律復帰カウントダウン中）

自律復帰カウントダウン中も、規定状態表示としては自律走行であるため、緑色背景の
`自律 / Auto` を最上段に大きく表示する。カウントダウン残り時間は重要な補助情報だが、
状態表示そのものを置き換えない。復帰直後に急に動いたように見えないよう、現在出力と
復帰予定 cmd を分けて表示する。

| 領域 | 表示内容 | 表示方法 |
| --- | --- | --- |
| 主表示帯 | `自律 / Auto` | 緑色背景、画面幅いっぱい、高さは既定で画面高の 70% |
| 主表示帯の右下 | mux 状態 | 出力 cmd は `ZERO`、自律 cmd 入力有無、手動 cmd 入力有無を表示する |
| 補助左スロット | 復帰時 cmd | 復帰時に採用予定の自律 cmd の並進速度を `m/s` 数値、角速度を旋回矢印で表示する |
| 補助中央スロット | `auto_resume_remaining_s` | `自律走行開始まで` と残り秒数を大きく表示する |
| 補助右スロット | 復帰方向 | 予定 cmd から `前進` / `左旋回` / `右旋回` / `後進` を表示し、必要に応じて警告文を併記する |

復帰予定方向の文言は、進行方向矢印と同じ逆算 stick 座標から求めた角度で判定する。
矢印の上方向を 0 度、右方向を正、左方向を負とし、`abs(angle) <= 15deg` は `前進`、
`abs(angle) <= 90deg` は `右旋回` または `左旋回`、`abs(angle) > 90deg` は `後進` とする。
`abs(angle) >= 45deg` かつ `abs(angle) <= 90deg` の場合は 2 行目に `急旋回注意！` を表示し、
後進の場合は 2 行目に `後方注意！` を表示する。
初期実装では GUI 警告に留め、mux 側での復帰保留は将来拡張とする。

### 13.4 自律走行画面（通常）

通常の自律走行画面では、規定状態表示として緑色背景の `自律 / Auto` を最上段に大きく表示する。
通常時の視認性を優先し、最終 `/cmd_vel` と自律走行系の状態を安定して表示する。

| 領域 | 表示内容 | 表示方法 |
| --- | --- | --- |
| 主表示帯 | `自律 / Auto` | 緑色背景、画面幅いっぱい、高さは既定で画面高の 70% |
| 主表示帯の右下 | mux 状態 | 出力 cmd の採用元、自律 cmd 入力有無、手動 cmd 入力有無を表示する |
| 補助左スロット | 最終出力 cmd | 並進速度を `m/s` 数値、角速度を旋回矢印で表示する |
| 補助中央スロット | 経路進捗 | `FollowerState.active_waypoint_label` と `FollowerState.state` を表示する |
| 補助右スロット | GNSS/RTK 状態 | RTK state、衛星数、heading を表示する |

`mode=AUTONOMOUS` で `output_source=ZERO` の場合は、自律 cmd timeout、起動直後、復帰保留などの
停止理由を mux 状態や中央表示で明確にする。

通常の自律走行画面の補助中央スロットでは、`tc_route_msgs/msg/FollowerState` を主入力として使う。
`active_waypoint_label` を `WP: <label>`、`state` を `状態: <state>` として表示する。
`FollowerState` がまだ届いていない場合は、`ManagerStatus.state` と `ManagerStatus.last_cause` を
補助表示として使う。距離や詳細な状態語彙の変換は初期 UI には含めず、必要になった時点で
`route_follower` 側の状態語彙拡張または GUI 側の表示変換表として追加する。

### 13.5 GUI 実装ライブラリ方針

GUI 実装は、`PyQt5` の `Qt Widgets` と `QGraphicsView` / `QGraphicsScene` を第一候補とする。
理由は以下である。

- 16:9 の固定論理キャンバスを `QGraphicsScene` として定義し、`QGraphicsView.fitInView()` で
  ウィンドウサイズに合わせて等比拡縮しやすい。
- scroll bar を無効化しても、scene 全体を `KeepAspectRatio` で常に表示できる。
- `QPainter` による速度矢印、バー、円弧ゲージ、警告枠などの描画が容易である。
- ROS callback thread は lock 付きの表示 snapshot を更新し、GUI thread は `QTimer` で定期的に snapshot を読み取って再描画できる。
- stylesheet により、3 画面の色、余白、文字サイズの設計を統一しやすい。

実装構成は以下を推奨する。

| 要素 | 推奨ライブラリ / クラス | 用途 |
| --- | --- | --- |
| GUI framework | `PyQt5.QtWidgets` | メインウィンドウ、描画 view、イベントループ |
| 固定比率描画 | `QGraphicsView`, `QGraphicsScene` | 16:9 論理キャンバスをウィンドウへ等比拡縮する |
| カスタム描画 | `QGraphicsItem` または `QPainter` | 速度矢印、ゲージ、警告枠、状態帯を描く |
| ROS 連携 | `rclpy` + `MultiThreadedExecutor` + `threading.Thread` + `QTimer` | ROS executor と GUI thread を分離し、GUI thread から snapshot を読む |
| 状態保持 | `dataclasses` | GUI 表示用 snapshot を不変に近い形で渡す |

論理キャンバスは 1600x900 または 1920x1080 とし、全 UI 要素をこの座標系で配置する。
ウィンドウ resize 時は view の viewport に対して `fitInView(sceneRect, KeepAspectRatio)` を実行し、
縦横どちらかに余白が出る場合は背景色で letterbox 表示にする。文字、線幅、余白は scene 座標に
紐づけるため、ウィンドウサイズを変えても画面全体が同じ比率で拡縮される。

`tkinter` は既存 `robot_console` と同じ技術で軽量に実装できるが、固定アスペクト比の全体拡縮、
カスタム描画、警告画面の表現を安定させるには Qt の方が適している。そのため、実装時は
`package.xml` の GUI 依存を `python3-pyqt5` へ更新する。`pyqtgraph` は時系列グラフを表示する
場合のみ追加候補とし、初期 UI では必須にしない。

## 14. 依存関係・ビルド設定

`drive_mode_manager` は `ament_python` パッケージとする。

直接依存は以下である。

| 依存 | 用途 |
| --- | --- |
| `rclpy` | ROS 2 Python node |
| `geometry_msgs` | `Twist` |
| `sensor_msgs` | `Joy` |
| `std_msgs` | 補助 topic |
| `builtin_interfaces` | `DriveModeStatus.stamp` |
| `tc_route_msgs` | `DriveModeStatus`, `FollowerState`, `ManagerStatus` |
| `rtk_gps_um982_msgs` | GUI の RTK 状態表示 |
| `python3-pyqt5` | 専用 GUI。16:9 論理キャンバス、等比拡縮、Qt event loop と ROS executor thread の分離 |

`DriveModeStatus.msg` を `tc_route_msgs` に追加する実装時は、`tc_route_msgs/CMakeLists.txt` に msg を追加し、
`drive_mode_manager/package.xml` の `tc_route_msgs` 依存を維持する。

## 15. テスト計画・受け入れ条件

優先して ROS 非依存コアの pytest を整備する。

| テスト | 対象 | 観点 |
| --- | --- | --- |
| `test_manual_transition_requires_l1_and_ps_hold` | `drive_mode_core.py` | L1 単独や stick 単独で手動遷移しない |
| `test_manual_to_auto_requires_l1_release_and_auto_alive` | `drive_mode_core.py` | L1 押下中の意図的停止では自律復帰しない |
| `test_auto_resume_outputs_zero_until_delay_elapsed` | `drive_mode_core.py` | カウントダウン中はゼロ出力 |
| `test_autonomous_cmd_timeout_outputs_zero` | `drive_mode_core.py` | 自律 cmd timeout 時にゼロ出力 |
| `test_manual_deadman_outputs_zero_when_l1_released` | `manual_teleop_core.py` | L1 非押下でゼロ Twist |
| `test_axis_deadzone_and_invert` | `manual_teleop_core.py` | deadzone、反転、倍率 |
| `test_regulation_state_label_mapping` | `gui_core.py` | `MODE_MANUAL` は `操縦 / Manual`、`MODE_AUTONOMOUS` はカウントダウン中も `自律 / Auto` と表示する |
| `test_aspect_ratio_fit_rect` | `gui_core.py` | ウィンドウサイズ変更時に 16:9 全体表示を維持する |
| `test_planned_direction_text_uses_arrow_angle_thresholds` | `gui_core.py` | 復帰予定方向を進行方向矢印と同じ角度しきい値で表示する |
| `test_planned_direction_text_warns_for_sharp_turn_and_backward` | `gui_core.py` | 急旋回および後進時の 2 行目警告文を表示する |

受け入れ条件は以下である。

- `pytest src/drive_mode_manager/tests` が成功する。
- `colcon build --symlink-install --packages-select tc_route_msgs drive_mode_manager` が成功する。
- `colcon build --packages-select tc_route_msgs drive_mode_manager` が成功する。
- 実機 `/joy` で L1 と PS ボタン index を確認し、PS が不安定な場合は代替 trigger を決める。
- `DriveStatusGuiCore.fit_rect()` の単体テストで 16:9 表示領域の算出を確認する。
- 画面を使う GUI 表示確認、controller 入力、実機駆動は自動テスト外の未確認事項として扱う。

## 16. 互換性・移行・影響範囲

`robot_navigator` は既存の `cmd_vel_topic` launch 引数で `/cmd_vel/autonomous` へ変更するため、
ノード内部 API の変更は不要である。

`ypspur_ros2` は引き続き `/cmd_vel` を購読するため変更不要である。ただし、実機 bringup では
`/cmd_vel` の publish 元が `drive_cmd_mux_node` だけになるよう、他ノードの remap を確認する。

`robot_console` は既存どおり `/cmd_vel` を表示できる。ただし、切替状態と復帰カウントダウンは
専用 GUI を正本表示とし、`robot_console` へ統合する場合も `/drive_mode_status` の購読表示に留める。

`tc_route_msgs` に `DriveModeStatus.msg` を追加するため、interface package の再ビルドが必要になる。
既存 msg/srv の field は変更しない。

tc2025 の `joystick_teleop.py` から継承するのは、Joy 軸変換、deadzone、L1 デッドマン、
timeout 停止、turbo 倍率である。自律 cmd passthrough と waypoint flag publish は移植しない。

## 17. 未決事項・今後の拡張

- PS ボタンが `/joy` で安定して取得できるか。実機 controller で確認する。
- PS ボタンが使えない場合の代替複合操作を決める。
- `manual_transition_hold_s=2.0`、`manual_to_auto_l1_released_s=1.0`、`auto_resume_delay_s=5.0` の
  実運用値を確認する。
- 自律復帰予定速度が閾値を超えた場合、GUI 表示だけにするか、mux 側で復帰を保留するかを決める。
- GPS/RTK、次 waypoint、自律走行状態の表示元を `/drive_mode_status` に含めるか、
  既存 status topic の購読表示に留めるかを決める。
- 専用 GUI をどの PC・画面で表示するかを運用手順で決める。
- 画面を使った GUI の実表示確認で、文字の重なり、警告文、RTK/waypoint 表示の視認性を確認する。

## 18. 改版履歴

| 版 | 日付 | 変更概要 |
| --- | --- | --- |
| 0.1 | 2026-05-19 | 初版。添付検討資料と tc2025 ROS 1 資産をもとに、`drive_mode_manager` の責務、topic、状態遷移、GUI、テスト計画を整理した |
| 0.2 | 2026-05-20 | レギュレーションの状態表示に準拠し、マニュアル走行、自律復帰カウントダウン中、自律走行通常の 3 画面構成と、PyQt5/Qt Graphics View による 16:9 等比拡縮 UI 方針を追記した |
| 0.3 | 2026-05-21 | 実装後の UI 仕様に合わせ、mux 状態表示、復帰時 cmd 表示、復帰方向警告、RTK/waypoint 表示、GUI パラメータ所属、GUI 非依存テスト項目を更新した |

## 19. `ps3_joy_sim_node` 追補

`ps3_joy_sim_node` は、PS3 controller が手元にない開発環境で `joy` topic を模擬する補助ノードである。正式設計は `docs/ps3_joy_sim_設計書.md` を正とし、本詳細設計書では `drive_mode_manager` 全体との接続境界を整理する。

### 19.1 責務と接続

`ps3_joy_sim_node` は相対 topic `joy` に `sensor_msgs/msg/Joy` を publish する。`manual_teleop_node` と `drive_cmd_mux_node` は実 controller の `joy_node` と同じ下流 interface として扱う。

本ノードは `/cmd_vel`、`/cmd_vel/manual`、`/drive_mode_status` を publish しない。速度指令生成は `manual_teleop_node`、最終出力選択は `drive_cmd_mux_node` に集約する。

### 19.2 追加ファイル

| ファイル | 役割 |
| --- | --- |
| `drive_mode_manager/ps3_joy_sim_core.py` | 押下キー集合と stick 保持値から Joy 配列および予測 cmd_vel 表示値を生成する ROS 非依存ロジック |
| `drive_mode_manager/ps3_joy_sim_node.py` | PyQt5 GUI と `rclpy` publisher を持つ ROS 2 ノード |
| `launch/ps3_joy_sim.launch.py` | simulator 単独起動用 launch |
| `tests/test_ps3_joy_sim_core.py` | index、符号、累積 stick、予測 cmd_vel、同時押し、reset 相当の単体テスト |

### 19.3 パラメータ

既定値は現行 `manual_teleop_node` と `drive_cmd_mux_node` に合わせ、`left_stick_x_axis=0`、`left_stick_y_axis=1`、`l1_button_index=4`、`ps_button_index=16`、`stick_step=0.1` とする。GUI の点を円内に保つため `normalize_diagonal_stick=true` とする。キーボード左右入力から publish する Joy 横軸は tc2025 実機 Joy 互換のため `invert_left_stick_x=true` を既定とし、画面上の stick 点はキー入力方向のまま表示する。`cmd_vel_linear_scale=1.2`、`cmd_vel_angular_scale=1.5`、`cmd_vel_deadzone=0.05` により、publish する Joy 軸で `manual_teleop_node` が出す想定の `v` と `w` を GUI に表示する。`joy_topic` は相対 `joy`、`publish_rate_hz` は 20.0Hz とする。

### 19.4 起動方針

`ps3_joy_sim_node` は `drive_mode_manager.launch.py joy_input:=ps3_joy_sim` で、`joy_node` の代替入力源として同時起動できる。既定は `joy_input:=joy_node` とし、実 controller 用の `joy_node` を起動する。`ps3_joy_sim.launch.py` は simulator 単独確認用に残す。本物の `joy_node` と同じ `joy` topic へ同時 publish しない。

### 19.5 テスト計画

`test_ps3_joy_sim_core.py` で L1/PS index、`w/s/a/d` の累積軸更新、Y 軸反転、斜め入力正規化、予測 `cmd_vel`、reset 相当、index 範囲外時の配列長維持を確認する。GUI 目視確認と ROS 2 topic 結合確認は `ros2-local-run` スキルに従い、実機 driver と `ypspur_ros2` を起動しない範囲で実施する。

| 版 | 日付 | 変更概要 |
| --- | --- | --- |
| 0.8 | 2026-05-22 | 進行方向矢印の長さ一定化と復帰方向テキストの角度判定仕様を設計書へ反映した |
| 0.7 | 2026-05-22 | 復帰方向テキストを進行方向矢印と同じ逆算 stick 角度ベースへ変更した |
| 0.6 | 2026-05-22 | `drive_status_gui_node` の進行方向矢印を `cmd_vel` から逆算した stick 座標方向へ変更した |
| 0.5 | 2026-05-22 | `ps3_joy_sim_node` の累積 stick 入力、予測 `cmd_vel` 表示、斜め正規化既定有効を反映した |
| 0.4 | 2026-05-21 | 開発用 `ps3_joy_sim_node` の責務、interface、パラメータ、launch、単体テストを追補した |
