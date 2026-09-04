# 結合動作確認例（route stack 回帰）

結合動作確認は、ローカル環境で安全に実施できるシミュレーション手順として整理する。
現時点では、「既存 route stack 回帰用の簡易構成」のみを対象とする。今後、
GNSS/LiDAR 入力模擬込み構成や Gazebo 障害物回避・ルート復帰検証構成などを
実装・確認する場合は、本章へ同じ粒度で手順を追加する。

## 既存 route stack 回帰用の簡易構成

目的は、既存 route stack が `route_planner -> route_manager -> route_follower ->
robot_navigator -> robot_simulator` の接続で進行し、指定した goal label へ到達できることを
確認することである。この構成では `rtk_gps_um982` と `ypspur_ros2` は起動しない。

### 対象ノード

- `robot_console`
- `route_planner`
- `route_manager`
- `route_follower`
- `robot_navigator`
- `robot_navigator` 付属の `robot_simulator`

### 実施ツール

この構成の確認では、手書きの inline Python ではなく以下の正式ツールを使う。

- GUI あり、ローカルデスクトップまたは X11 転送ありの環境：
  `src/robot_console/tools/gui_route_stack_eval.py`
- GUI なし、または `DISPLAY` が利用できない環境：
  `src/robot_console/tools/headless_route_stack_eval.py`

GUI あり評価では `UiMain` を実際に生成し、画面座標クリックではなく automation hook
経由で `Combobox`, `Entry`, `Checkbutton`, `Button` 相当の操作を行う。

実際の実行履歴では、`DISPLAY` (X11 転送) が利用できるケースが大半で、GUI あり評価
(`gui_route_stack_eval.py`) が headless 評価より圧倒的に多く使われていた。「ローカル環境=
基本 headless」と決めつけず、まず SKILL.md の DISPLAY 判定手順に従って GUI ありが使えるか
確認し、使えるなら GUI あり評価を優先する。

### 評価条件のたたき台

以下は route/label を指定しなかった場合のたたき台であり、そのまま無条件に採用するもの
ではない。どの route（`tsukuba` か `obstacle_route_sim` の world か）、どの start/goal
label で確認するかは、対象の変更内容を踏まえてユーザーに確認してから決める
（`ros2-local-run` の SKILL.md「検証段階の決定とユーザー合意」を参照）。

| 項目 | 値 |
| --- | --- |
| `route_planner` parameter | `tsukuba.yaml` |
| `route_manager` parameter | `tsukuba.yaml` |
| `route_manager` Start Label | `10` |
| `route_manager` Goal Label | `30` |
| `route_follower` parameter | `default.yaml` |
| `robot_navigator` parameter | `default.yaml` |
| `robot_navigator` Simulator | 有効 |
| 起動後入力 | `manual_start=True` |

`route_follower` の `default.yaml` は `start_immediately: false` のため、起動後に
`manual_start=True` を送る。`obstacle_route_sim` の道路 world を使う評価では
`route_planner`/`route_manager` のパラメータをその world 用のもの（例:
`obstacle_route_crank_w5.yaml`）に差し替える。

### 監視条件

- `/route_state` (`tc_route_msgs/msg/RouteState`) を監視する。
- `/active_route` (`tc_route_msgs/msg/Route`) を監視し、waypoint 数と start / goal label を確認する。
- `/follower_state` (`tc_route_msgs/msg/FollowerState`) を監視する。
- `/cmd_vel` (`geometry_msgs/msg/Twist`) を監視し、走行中に出力されることを確認する。
- `/manual_start` (`std_msgs/msg/Bool`) を監視し、`True` が送信されたことを確認する。
- `/route_state.current_label == <goal_label>` になったら goal 到達とみなす。
- 到達後は 10 秒待機してから停止処理を行う。
- 待機には上限時間を設ける。既定では 180〜320 秒程度を上限とする（world や goal までの
  距離に応じて調整する）。
- 停止後に `ros2 node list`, `ros2 topic list`, `pgrep -af` で残存ノード・
  プロセスを確認する。

評価ツールは実行中、`[monitor] 5.0s counts route_state=0 active_route=0 follower_state=0
cmd_vel=0 ...` のような形式で各 topic の受信件数を定期的に stdout へ出す。成功したかどうかは、
この進捗ログのカウントが単調に増えているか、`route_state.current_label` が goal label に
到達したかで判断する。カウントが 0 のまま変化しない場合は、該当ノードが未起動または
param 不一致の可能性が高い。

### ログ出力先

この統合評価を実行する場合は、実行前に `run_id` を決め、
`log/codex/<run_id>/` 配下へログを集約する。`run_id` は単純なタイムスタンプだけでなく、
`<timestamp>_headless_existing_llh_enu` のように確認目的が分かるサフィックスを付けると、
後からログを追いやすい。

```bash
run_id=$(date +%Y%m%d_%H%M%S)_<目的が分かる短い説明>
mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
```

評価ツールが `robot_console` を Python から直接生成する場合は、
`--console-log-directory log/codex/<run_id>/robot_console` を指定し、
`robot_console` 管理の子プロセス stdout/stderr も `log/codex/<run_id>/robot_console`
配下へ保存する。保存済み ROS ログは `ROS_LOG_DIR` を参照する。

### GUI あり実施例

ローカルデスクトップまたは X11 転送ありの環境では GUI あり評価を使う。

```bash
source install/setup.bash
run_id=$(date +%Y%m%d_%H%M%S)
mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
python3 src/robot_console/tools/gui_route_stack_eval.py \
  --start-label 10 \
  --goal-label 30 \
  --timeout-sec 180 \
  --post-goal-wait-sec 10 \
  --console-log-directory "log/codex/${run_id}/robot_console" \
  --verify-log-open-buttons
```

`obstacle_route_sim` の world を使う評価では、`--route-planner-param` /
`--route-manager-param` / `--launch-order` / `--no-simulator` /
`--show-drive-status-gui` などを組み合わせて指定する実例もある。

```bash
source install/setup.bash
run_id=$(date +%Y%m%d_%H%M%S)
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
python3 src/robot_console/tools/gui_route_stack_eval.py \
  --route-planner-param obstacle_route_crank_w5.yaml \
  --route-manager-param obstacle_route_crank_w5.yaml \
  --start-label 0 --goal-label 32 \
  --timeout-sec 320 --post-goal-wait-sec 8 --startup-wait-sec 3 --stop-timeout-sec 25 \
  --no-simulator --show-drive-status-gui \
  --launch-order route_planner,route_manager,route_follower,obstacle_monitor,drive_mode_manager,robot_navigator \
  --console-log-directory "log/codex/${run_id}/robot_console"
```

### GUI なし実施例

純粋な CLI / SSH 環境、または `DISPLAY` が利用できない環境では headless 評価を使う。

```bash
source install/setup.bash
run_id=$(date +%Y%m%d_%H%M%S)_headless
mkdir -p "log/codex/${run_id}/ros"
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
python3 src/robot_console/tools/headless_route_stack_eval.py \
  --start-label 10 \
  --goal-label 30 \
  --timeout-sec 180 \
  --post-goal-wait-sec 10 \
  --console-log-directory "log/codex/${run_id}/robot_console" \
  2>&1 | tee "log/codex/${run_id}/headless_route_stack_eval.log"; exit ${PIPESTATUS[0]}
```

`tee` でログを保存しつつ `PIPESTATUS` で本来の終了コードを拾う点が実運用上のポイントである。

### 残存確認例

```bash
source install/setup.bash
ros2 node list
ros2 topic list
pgrep -af "robot_console|route_planner|route_manager|route_follower|robot_navigator|robot_simulator|ros2 launch" || true
```

停止処理でノード・プロセスが残った場合のエスカレーション手順は
`references/troubleshooting.md` を参照する。

### 成功条件

- `route_manager` が `/route_state` を publish する。
- `/active_route` の start label が `"10"`、goal label が `"30"` である。
- `/manual_start` に `True` が publish される。
- `/cmd_vel` が走行中に publish される。
- `/route_state.current_label` が `"10"` から進行し、最終的に `"30"` に到達する。
- `"30"` 到達後に 10 秒待機できる。
- 停止処理後に対象ノード・対象プロセスが残っていない。

### 報告例

```text
/active_route waypoints=21 start='10' goal='30' version=100
/route_state current_label='30' status=2 version=100
goal label '30' reached by /route_state
stop state: route_planner status=STOPPED pid=None sim_pid=None error=''
stop state: route_manager status=STOPPED pid=None sim_pid=None error=''
stop state: route_follower status=STOPPED pid=None sim_pid=None error=''
stop state: robot_navigator status=STOPPED pid=None sim_pid=None error=''
```
