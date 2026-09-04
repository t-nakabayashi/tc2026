# Gazebo シミュレーションを使った動作確認（段階 3）

道路 world・LiDAR・pylon 障害物など、CLI での疑似 topic 入力だけでは再現しにくい入力が
絡む変更（`obstacle_monitor` の障害物判定、`drive_mode_manager` の回避行動、経路復帰、
LiDAR 由来の入力を使う認識系との結合など）を確認する段階。`obstacle_route_sim` パッケージが
Gazebo Harmonic 上に道路 world・差動二輪ロボット・LiDAR・pylon 障害物を用意する。

具体的な起動コマンド、対応 world 一覧（`road_type` / `road_width` / waypoint 数 /
start・goal label）、`robot_console` からの結合確認手順、GUI 自動操作による確認手順、
過去の確認済み結果は **`src/obstacle_route_sim/README.md` を正とする**。本ファイルでは
その README にない、スキル運用上の判断点だけを補足する。

## 着手前に確認すること

- どの world（`straight` / `scurve` / `crank`）、どの `road_width`、pylon の有無・
  `pylon_seed`、`route_blocker` の要否を確認するかは、README の表にある値を候補として
  提示しつつ、対象の変更内容に照らして妥当か、あるいはユーザーが別の条件を意図していないかを
  確認してから実行する。既定値をそのまま採用してよいとは限らない。
- `robot_console` からの結合確認（実 GUI）にするか、`gui_route_stack_eval.py` /
  `headless_route_stack_eval.py` を使うかは、`ros2-local-run` の SKILL.md にある
  GUI/headless 判定に従う。

## 安全規則との関係

- Gazebo 確認自体は `ros2-local-run` の Safety Rules で「ユーザーが明示した場合のみ行う」
  に分類される。無条件禁止の実機操作（`ypspur_ros2` 起動、実機走行、`/cmd_vel` の実機
  driver 接続）とは異なるが、無条件で実行してよいわけではない。
- `obstacle_route_sim` の world はシミュレーションであり、実機・実センサではない。実機の
  `rtk_gps_um982` や `ypspur_ros2` と組み合わせて起動しない。

## GPU / 描画まわりの注意

- Gazebo GUI 付き確認は GPU デバイス権限が必要になる場合がある。`src/obstacle_route_sim/README.md`
  の注意事項どおり、`render` / `video` group 追加後はログアウト・ログインしてから確認するのが
  正しい対処である。ログイン仕切り直しができないセッションでの一時的な回避策や、
  それでも描画が出ない場合の切り分けは `references/troubleshooting.md` を参照する。
- `LIBGL_ALWAYS_SOFTWARE=1` は Gazebo/Ogre2 の安定性を落とす場合があるため、
  `obstacle_route_sim` では既定で使用しない。安易に付けない。

## ログ・停止

- ログ配置（`log/codex/<run_id>/`）、停止処理後の残存確認は `ros2-local-run` の
  SKILL.md および `references/integration-check.md` の作法をそのまま適用する。
- 停止時に `SIGINT` 由来の `Traceback` が表示されることがあるが、profile が `STOPPED` に
  遷移し新しい crash report が出ていなければ、停止処理として問題ないと扱ってよい
  （`src/obstacle_route_sim/README.md` 記載の既知事象）。
