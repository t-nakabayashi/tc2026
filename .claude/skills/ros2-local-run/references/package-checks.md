# パッケージ単体の実行確認例

`ros2-local-run` スキルの SKILL.md に記載した「共通の確認パターン」を、代表的なパッケージに
適用した具体例。実際のパラメータや topic 名は対象パッケージの README、launch、詳細設計書を
確認してから調整する。

## `route_planner`

`route_planner` は route CSV/YAML から経路を生成し、`/get_route` と `/update_route`
service を提供する。単体確認では、launch 後に `GetRoute` service を呼び、
返却 route の waypoint 数、start/goal label、成功可否を確認する。

```bash
source install/setup.bash
timeout 30s ros2 launch route_planner route_planner.launch.py \
  param_file:="$(ros2 pkg prefix route_planner)/share/route_planner/params/tsukuba.yaml"
```

別プロセスで以下を実行する。

```bash
source install/setup.bash
ros2 service call /get_route tc_route_msgs/srv/GetRoute \
  "{start_label: '10', goal_label: '50', checkpoint_labels: []}"
```

## `route_manager`

`route_manager` は `route_planner` の service を入力として `/active_route`,
`/route_state`, `/manager_status`, `/mission_info` を publish する。
単体寄りに確認する場合でも、実用上は `route_planner` と組み合わせて確認する。

```bash
source install/setup.bash
ros2 launch route_planner route_planner.launch.py \
  param_file:="$(ros2 pkg prefix route_planner)/share/route_planner/params/tsukuba.yaml"
```

別プロセスで以下を起動する。

```bash
source install/setup.bash
ros2 launch route_manager route_manager.launch.py \
  param_file:="$(ros2 pkg prefix route_manager)/share/route_manager/params/tsukuba.yaml" \
  start_label:=10 \
  goal_label:=50
```

出力確認例:

```bash
source install/setup.bash
timeout 10s ros2 topic echo --once /route_state tc_route_msgs/msg/RouteState
timeout 10s ros2 topic echo --once /active_route tc_route_msgs/msg/Route
```

## `route_follower`

`route_follower` は `/active_route`, `/localization/pose_enu`, `/manual_start`, `/sig_recog`,
`/road_blocked`, `/obstacle_avoidance_hint` を入力として、`/active_target`,
`/follower_state`, `/recog_flag` を出力する。単体確認では、短い `Route` と
`PoseWithCovarianceStamped` を仮想入力として与え、`/active_target` と
`/follower_state` が出ることを確認する。

実 route を使う場合は、`route_planner` と `route_manager` を併用し、
`robot_simulator` またはテスト用 publisher で `/localization/pose_enu` を与える。

出力確認例:

```bash
source install/setup.bash
timeout 10s ros2 topic echo --once /active_target geometry_msgs/msg/PoseStamped
timeout 10s ros2 topic echo --once /follower_state tc_route_msgs/msg/FollowerState
```

## `robot_navigator`

`robot_navigator` は `/active_target`, `/localization/pose_enu`, `/odom` などを入力として
`/cmd_vel` を publish する。単体確認では、仮想 pose と target を与え、
`/cmd_vel` が publish されることを確認する。実機 driver には接続しない。

```bash
source install/setup.bash
ros2 launch robot_navigator robot_navigator.launch.py
```

入力例:

```bash
source install/setup.bash
ros2 topic pub --once /active_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
ros2 topic pub --once /localization/pose_enu geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
ros2 topic pub --once /odom nav_msgs/msg/Odometry \
  "{header: {frame_id: 'odom'}, child_frame_id: 'base_link', pose: {pose: {orientation: {w: 1.0}}}}"
```

出力確認例:

```bash
source install/setup.bash
timeout 10s ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist
```

過去の実行例では、`tf2_echo` で座標変換の妥当性を合わせて確認していた。

```bash
source install/setup.bash
timeout 10s ros2 run tf2_ros tf2_echo map base_link
```

## `robot_console`

`robot_console` は tkinter GUI を持つため、ローカル環境で `DISPLAY` が利用できる場合に
GUI 起動確認を行う。画面座標クリックは使わず、`UiMain` の automation hook を通して
`Combobox`, `Entry`, `Checkbutton`, `Button` 相当の操作を行う。

短時間起動確認:

```bash
source install/setup.bash
timeout 10s ros2 run robot_console robot_console
```

GUI 操作を伴う route stack 評価では、手書きの inline Python ではなく
`src/robot_console/tools/gui_route_stack_eval.py` を使う。`DISPLAY` が利用できない場合は、
GUI あり評価は実施せず、`src/robot_console/tools/headless_route_stack_eval.py` による
headless 評価へ切り替える。DISPLAY の判定手順は `ros2-local-run` の SKILL.md、具体的な
実行例は `references/integration-check.md` を参照する。

確認後は `ros2 node list`, `ros2 topic list`, `pgrep -af` で残存ノード・
プロセスがないことを確認する。

次期 GUI（PyQt5 ローカル UI / HTML 遠隔観測 UI、`src/robot_console/docs/` の設計書参照）の
実装が進んだ場合、HTML 遠隔観測 UI はブラウザで見るだけの構成になるため、Claude Browser
ツールや `run` スキル、`.claude/launch.json` での起動確認が有効な手段になる。この場合も
安全規則（実機・`ypspur` を起動しない）は変わらない。
