---
name: ros2-local-run
description: ローカル環境で ros2 run / ros2 launch / ros2 topic / ros2 service / ros2 action / ros2 interface などの ROS 2 実行確認を行うときの、実行可否の原則、安全規則 (実機・ypspur を起動しない等)、ログ配置、GUI/headless の判定、ノード単体/route stack結合/Gazeboシミュレーションの3段階の動作確認手順、トラブルシュートを定義する。ユーザーがローカルでの動作確認・シミュレーション確認・robot_console GUI の起動確認を明示的に依頼したとき、pytest だけでは確認できない「ROS 2 ノードとして実際に動かしての確認」が必要なとき、どこまでのテストを実施すべきか検討してユーザーに提示したいとき、実機や実センサを動かす操作を求められて可否を判断するときは必ずこのスキルを参照する。
---

# 実行可否の原則

- Claude Code は原則として `ros2 run`, `ros2 launch`, `ros2 topic`, `ros2 service`,
  `ros2 action`, `ros2 interface` などの ROS 2 実行確認を行わない。
- 実機、センサ、GUI、Gazebo、外部デバイス、長時間起動ノードに依存する確認も原則として行わない。
- ローカル環境でユーザーから ROS 2 実行確認を明示された場合のみ、以下の規則に従って
  `ros2` コマンドを実行してよい。
- 必要な動作確認が `colcon build` と pytest（`build-and-test` スキル参照）で代替できない
  場合は、未確認事項として作業報告に明記する。pytest はノードを起動しないため、ノード
  レベルの動作確認そのものは代替できない。
- README やドキュメントに利用者向けの実行手順を書く場合は、リポジトリ外の絶対パスを使わない。

# 検証段階の決定とユーザー合意

「ROS 2 ノードとして動かして確認する」には、確認したい変更の範囲に応じて性質の異なる
3 段階がある。どの段階まで必要かはコード変更の内容次第であり、機械的に全段階を実行する
ものでも、常に 1 段階で済ませてよいものでもない。

1. **ノード単体の動作確認**: 対象ノードを実際に起動し、疑似的な topic/service を与えて
   手動でインタラクティブに入出力を確認する（`ros2 topic pub` / `echo` など）。手順は
   `references/package-checks.md`。確認を使い捨てにせず自動テストとして残したい場合
   （バグ修正の回帰確認など）は、手動確認ではなく pytest ベースの自動テスト化を検討する。
   その場合の作法は `build-and-test` スキルを参照する（本スキルの対象外）。
2. **複数ノードを組み合わせたシステムとしての確認（route stack 結合）**:
   `route_planner -> route_manager -> route_follower -> robot_navigator` などを実際に
   同時起動し、GUI が使える環境では GUI あり、SSH などの CLI 環境では headless で確認する。
   手順は `references/integration-check.md`。
3. **Gazebo シミュレータを使った動作確認**: `obstacle_route_sim` で道路 world・LiDAR・
   pylon 障害物を再現し、障害物回避・ルート復帰など実センサ相当の入力が絡む挙動を確認する。
   手順は `references/gazebo-simulation-check.md`。

対応方針:

- 着手前に、変更内容から見てどの段階が必要かを検討し、対象パッケージ・段階・
  （2, 3 段階を行う場合は）評価条件の候補をユーザーに提示して合意を得てから実行する。
  「pytest が通ったので単体確認は不要」「route_follower の変更なので結合確認まで必要」
  「LiDAR 入力に関わる変更なので Gazebo 確認も必要」のように、判断理由を添えて提示する。
- 迷う場合は段階を絞りすぎず、必要そうな段階を提示した上でユーザーに取捨選択してもらう。
- 2, 3 段階の実行条件（使用する route/world、start/goal label、pylon の有無など）は、
  `references/integration-check.md` や `references/gazebo-simulation-check.md` に載っている
  値を既定の**たたき台**として提示し、そのまま無条件に採用しない。ユーザーが指定した
  条件、または変更内容に照らして妥当な条件を確認してから実行する。

# Local ROS 2 Run Policy

- ローカル環境で、ユーザーが ROS 2 実行確認を明示した場合のみ `ros2` コマンドを実行してよい。
- 実行前に、対象パッケージ、起動するノード、投入する仮想入力、監視する出力、
  終了条件を簡潔に説明する。
- 実行前に `source install/setup.bash` を行う。未ビルドまたは install が古い可能性がある場合は、
  先に必要パッケージを `colcon build --symlink-install --packages-select <package_name>` で確認する。
- 長時間起動するノード、GUI、シミュレーションは `timeout`、監視スクリプト、または明示的な停止処理を用意する。
- ROS 2 実行確認を行う場合は、原則としてワークスペース直下の `log/codex/`
  配下に実行ごとのログディレクトリを作成する。例：`log/codex/<YYYYMMDD_HHMMSS>/`。
  このディレクトリ名は元々 Codex 向けの慣習だが、既存ツール・READMEがこの配置を前提にしているため、
  実行主体が Claude Code であっても同じ配置を踏襲する。目的が分かるサフィックスを付けると
  後から追いやすい（例: `log/codex/20260601_120000_headless_existing_llh_enu/`）。
- 実行確認時は、少なくとも `ROS_LOG_DIR` を `log/codex/<run_id>/ros` に設定する。
  `robot_console` 管理の子プロセス stdout/stderr を保存する場合は、
  `console_log_directory` 相当を `log/codex/<run_id>/robot_console` に設定する。
- 実行結果の報告では、実行したコマンド、投入した入力、確認した出力、成功/失敗、
  ログ出力先、未確認事項、停止処理の結果を明記する。
- 確認後は `ros2 node list`, `ros2 topic list`, `pgrep -af` などで、意図しないプロセスや
  ノードが残っていないことを確認する。停止処理でプロセスが残った場合のエスカレーション手順は
  `references/troubleshooting.md` を参照する。

# Safety Rules

- `ypspur_ros2` を起動してはならない。
- `ypspur-coordinator` を起動してはならない。
- 実機のロボットを実際に動かす操作を行ってはならない。
- `/cmd_vel` を実機 driver に接続する構成を起動してはならない。
- 実施してよいのは、各ノードの単体動作確認と、シミュレーションによる結合動作確認に限る。
- `rtk_gps_um982`、実センサ、実カメラ、LiDAR、Gazebo、RViz、外部デバイスを使う確認は、
  ユーザーが明示した場合のみ行い、必要な前提と未確認範囲を報告する。

上記 4 項目（`ypspur_ros2`/`ypspur-coordinator`/実機走行/`/cmd_vel`実機接続）は
ユーザーが明示的に指示した場合でも行わない無条件の禁止事項であり、それ以外（実センサ等）は
ユーザーが明示した場合のみ行う条件付き許可である。この 2 段階を混同しない。

# GUI か headless かの判定

`robot_console` や評価ツール (`gui_route_stack_eval.py` / `headless_route_stack_eval.py`) は
GUI の有無で使い分ける。「ローカル環境では基本 headless」と決めつけず、まず環境を確認する。

```bash
printf 'DISPLAY=%s WAYLAND_DISPLAY=%s XDG_SESSION_TYPE=%s\n' \
  "$DISPLAY" "$WAYLAND_DISPLAY" "$XDG_SESSION_TYPE"
```

`DISPLAY`（または `WAYLAND_DISPLAY`）が利用できる場合は GUI あり評価を優先する。実際の
運用でも GUI あり評価の方が headless より多く使われていた。利用できない場合のみ headless
評価に切り替える。判定に迷う場合や描画確認が絡む場合の詳細は
`references/troubleshooting.md` を参照する。

# 詳細手順（references/）

このファイルは常に参照される方針・安全規則のみを持つ。段階別の具体的なコマンド例は
分量が多いため、以下の参照ファイルに分けている。上記「検証段階の決定」で選んだ段階に
応じて必要になった時点で読む。

- `references/package-checks.md`（段階 1）: `route_planner` / `route_manager` /
  `route_follower` / `robot_navigator` / `robot_console` の単体動作確認例（共通の確認
  パターンを含む）。
- `references/integration-check.md`（段階 2）: `route_planner -> route_manager ->
  route_follower -> robot_navigator -> robot_simulator` の route stack 結合回帰確認の
  手順、評価条件のたたき台、成功条件、ログ配置、GUI/headless それぞれの実施例。
- `references/gazebo-simulation-check.md`（段階 3）: `obstacle_route_sim` を使った
  Gazebo シミュレーション確認（道路 world、LiDAR、pylon 障害物）の進め方と、
  `robot_console` からの結合確認への接続方法。
- `references/troubleshooting.md`: ビルド反映漏れ、GUI 評価可否判定、Gazebo/GPU 描画確認、
  停止処理のエスカレーションなど、実際の実行確認で繰り返し発生した詰まりどころと対処。
