# robot_navigator パッケージ README

## 概要
`robot_navigator` は `/active_target`（`PoseStamped`）を追従する時間最適制御を実装し、
`cmd_vel_topic` で指定した `Twist` トピックへ自律速度指令を出力する移動体ナビゲーションノードです。
単体起動では `/cmd_vel`、統合起動では `drive_mode_manager` の mux に渡す `/cmd_vel/autonomous` を
既定出力として扱います。障害物距離を
`ObstacleAvoidanceHint` または `LaserScan` から取得して減速・停止を行い、進行方向を
`visualization_msgs/Marker` で可視化します。試験用に `/cmd_vel` を受け取って自己位置を
配信する `robot_simulator` ノード（同パッケージ内）も併載しています。

## 主な機能
- `/ypspur_ros/odom`・`/localization/pose_enu`・`/active_target` を監視し、線速度・角速度の時間最適解を近似計算。
- `obstacle_distance_mode` に応じて `/scan` もしくは `/obstacle_avoidance_hint` から前方距離を取得し、
  `min_obstacle_distance` を残すため、現在速度・直前指令・制動減速度・遅延から前進速度を制限する。
- 角速度は PID（`kp=0.65`, `ki=0.001`, `kd=0.02`）で生成し、角度誤差に応じて線速度をスケール。
- `/direction_marker` に進行方向を示す矢印 Marker を Publish。
- `log_csv_path` が書き込み可能な場合、制御内部状態を CSV として逐次出力。
- 必要なトピックが揃わない場合は `cmd_vel_topic` にゼロを出力し、WARN ログを一定周期で発行。

## 起動方法
### launch を用いた起動
```bash
ros2 launch robot_navigator robot_navigator.launch.py \
  obstacle_hint_topic:=/obstacle_avoidance_hint cmd_vel_topic:=/cmd_vel
```
- 単体 launch では `cmd_vel_topic` の既定値は `/cmd_vel` です。
- launch 引数で入出力トピックをリマップ可能。`param_file` で任意の YAML を指定できます。
- `obstacle_distance_mode` を `scan` に設定すると `/scan` を購読し、`hint` の場合は
  `/obstacle_avoidance_hint` を使用します。
- 既定ではドライバの `/motion_limits` が必要です。未受信・0.5秒以上の途絶、
  古いstamp、設定と整合しない制動能力ではゼロ速度を出します。
  このトピックを持たないシミュレータと組み合わせる場合だけ `require_motion_limits:=false` を指定します。


### 統合 launch を用いた起動
```bash
ros2 launch robot_navigator all.launch.py use_drive_mode_manager:=true
```
- `all.launch.py` では `drive_mode_manager` の `drive_cmd_mux_node` を併用し、
  `robot_navigator` の自律速度指令は既定で `/cmd_vel/autonomous` へ出力します。
- 最終的な `/cmd_vel` は `drive_cmd_mux_node` が publish するため、実機 bringup では
  他ノードが直接 `/cmd_vel` を publish しない構成にしてください。
- `use_drive_mode_manager:=false` を指定した場合は mux を起動せず、`cmd_vel_topic` の値に従って
  `robot_navigator` が直接 Twist を publish します。

### 実行ファイルを直接起動
```bash
ros2 run robot_navigator robot_navigator
```
- 動作確認には同梱の `robot_simulator`（`ros2 run robot_navigator robot_simulator`）と組み合わせて
  `/cmd_vel` の挙動を確認できます。

## 外部インタフェース
### Subscriber
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/ypspur_ros/odom` | `nav_msgs/Odometry` | launch既定の現在速度入力。ノード内の相対名`odom`をremapする。 | RELIABLE / VOLATILE / depth=10 |
| `/motion_limits` | `tc_route_msgs/MotionLimits` | `require_motion_limits=true`のとき、ドライバの実効上限と鮮度を確認する。 | RELIABLE / VOLATILE / depth=1 |
| `/localization/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | 現在姿勢（位置・ヨー角）を取得。launch の `pose_enu_topic` で `/localization/pose_enu` などへ変更可能。 | RELIABLE / VOLATILE / depth=10 |
| `/active_target` | `geometry_msgs/PoseStamped` | 追従対象の目標姿勢。 | RELIABLE / VOLATILE / depth=10 |
| `/obstacle_avoidance_hint` | `tc_route_msgs/ObstacleAvoidanceHint` | `obstacle_distance_mode=hint` のとき使用。 | BEST_EFFORT / VOLATILE / depth=1 |
| `/scan` | `sensor_msgs/LaserScan` | `obstacle_distance_mode=scan` のとき使用（SensorDataQoS）。 | BEST_EFFORT / VOLATILE / depth=1 |
| `/localization/pose_enu_glitch_trigger` | `std_msgs/Bool` | `robot_simulator` が停止中に受信すると `pose_topic` へ単発オフセットを加算。 | RELIABLE / VOLATILE / depth=10 |

### Publisher
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/cmd_vel` または `/cmd_vel/autonomous` | `geometry_msgs/Twist` | `cmd_vel_topic` で指定した速度指令。統合起動時は `/cmd_vel/autonomous` を `drive_mode_manager` へ渡す。 | RELIABLE / VOLATILE / depth=10 |
| `/direction_marker` | `visualization_msgs/Marker` | 進行方向矢印。`marker_frame` で指定したフレームに出力。 | RELIABLE / VOLATILE / depth=1 |

> サービス・アクションは提供しません。

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `max_vel` | double | `1.0` | 線速度の上限 [m/s]。
| `max_w` | double | `1.0` | 角速度の上限 [rad/s]。
| `max_acc_v` | double | `0.7` | 線加速度上限 [m/s^2]。
| `max_acc_w` | double | `0.6` | ドライバとの整合確認に使用する角加速度要求値 [rad/s^2]。ドライバの能力がこれ以上であることを確認する。PID出力自体の変化率制限ではない。
| `max_decel_v` | double | `1.5` | 制動距離の計算で想定する線減速度 [m/s^2]。
| `braking_delay_sec` | double | `0.2` | 通信・driver遅延の設計予算 [s]。制御1周期を別途加算する。
| `require_motion_limits` | bool | `true` | ドライバの実効制限の受信を必須にする。
| `pos_tol` | double | `0.5` | 位置許容誤差 [m]。近接時に角度誤差処理へ切替。一度入ると新しい目標の受信まで解除しない。
| `pos_tol_exit_margin` | double | `0.2` | 離脱判定の余裕として宣言するが、現在の制御では参照しない。
| `ang_tol` | double | `0.25` | 角度許容誤差 [rad]。
| `control_rate_hz` | double | `20.0` | 制御ループ周期 [Hz]。
| `robot_width` | double | `0.6` | 障害物距離評価に用いるロボット幅 [m]。
| `safety_distance` | double | `0.8` | 互換用。現在の制動判定では使用しない。
| `min_obstacle_distance` | double | `0.5` | 停止を指示する距離 [m]。
| `obst_max_dist` | double | `5.0` | 障害物距離として採用する最大値 [m]。
| `obstacle_distance_mode` | string | `"hint"` | `hint` または `scan`。距離取得ソースを切替。
| `marker_frame` | string | `"map"` | Marker の出力フレーム。
| `log_csv_path` | string | `/tmp/control_log.csv` | 同梱YAMLのCSVログ出力先。ノード直接起動時の既定は`~/control_log.csv`。
| `pose_timeout_sec` | double | `1.0` | 自己位置の受信期限 [s]。
| `odom_timeout_sec` | double | `1.0` | odom の受信期限 [s]。
| `obstacle_timeout_sec` | double | `0.0` | 障害物距離の受信期限 [s]。0 で監視無効。
| `road_block_hold_sec` | double | `5.0` | `/road_blocked` による停止の保持時間 [s]。

`max_vel`・`max_w`・`max_acc_v`・`max_acc_w`・`max_decel_v`・`braking_delay_sec`・
`require_motion_limits` は読み取り専用として宣言し、起動後に変更できない。
角速度 PID のゲインはパラメータを持たず、実装内の固定値である。

## 動作確認手順
1. `robot_simulator`（当パッケージ内）を起動し、最終 `/cmd_vel` を受け取って `pose_topic`（既定 `/localization/pose_enu`）・`/ypspur_ros/odom` を配信させる。
2. `route_follower` もしくは手動で `/active_target` を Publish し、目標指令を入力する。
3. `obstacle_monitor` を起動して `/obstacle_avoidance_hint` を供給するか、`obstacle_distance_mode:=scan`
   として `/scan` を直接購読させる。
4. 同梱`robot_simulator`は`MotionLimits`を継続配信するため、`require_motion_limits:=true`で
   `robot_navigator`をlaunchする。単体構成では `/cmd_vel`、統合構成では `/cmd_vel/autonomous` と
   `/drive_mode_status`、最終 `/cmd_vel`、`/direction_marker` の挙動、CSV ログの内容を確認する。

## デバッグのヒント
- CSV ログが生成されない場合は `log_csv_path` のディレクトリ権限を確認してください。
- `obstacle_distance_mode` が `scan` で距離が常に `None` となる場合は `/scan` の FOV がロボット幅帯を
  カバーしているか確認します。
- WARN ログのスロットルが頻発する場合は、入力トピックの QoS やリマップ設定を再確認してください。
- `robot_simulator` の `pose_topic` に付与されるノイズはガウス分布（位置 `pose_noise_std_m`、ヨー角 `yaw_noise_std_deg`）を
  各 publish ごとにサンプリングして加算する実装です。標準偏差を 1.0 に設定すれば平均的には 1m 規模の
  ばらつきが常時発生しますが、「停止中にごくまれに 1m だけジャンプする」ような突発外れ値を再現する
  自動発火はありません。単発のステップ状誤差には次節の実装済み外部トリガを使います。

## 自己位置外れ値を模擬するための実装（外部トリガ方式）
`robot_simulator`は外部トリガ入力で単発の自己位置オフセットを付与します。確率分布による自動発火は行わず、
テストスクリプトや人間操作で明示的に発火させることで再現性を確保します。

- **トリガトピック**: `/localization/pose_enu_glitch_trigger`（`std_msgs/Bool`）。`data=true` を受信した瞬間に 1 回
  だけオフセットを予約し、停止状態で `glitch_wait_after_stop_sec`（既定 5 秒）の待機後に
  `pose_topic` へ反映します。一度適用されたオフセットは `data=false` を受信するまで同じ値のまま
  付与され続けます。走行中にトリガを受けた場合は「停止するまで予約」を保持し、停止と
  クールダウンを満たしたタイミングから待機を開始します。連続発火を防ぐため、
  `glitch_cooldown_sec` 経過まで後続トリガは無視します。false を送れば予約・適用中の外れ値を
  即時クリアできます。
- **停止条件の併用**: `cmd_vel` の線速度・角速度が閾値（`glitch_linear_stop_threshold`、
  `glitch_angular_stop_threshold`）未満のときだけトリガを有効化し、走行中の意図しない外れ値挿入を
  防ぎます。
- **オフセット生成**: 既存のガウスノイズとは別に、`glitch_radius_min_m`〜`glitch_radius_max_m`
  の一様分布から半径をサンプリングし、方向一様の 2D オフセットとヨー角オフセット
  （`glitch_yaw_min_deg`〜`glitch_yaw_max_deg` の一様分布を正負ランダム符号で付与）を単発で
  加算します。既定値は位置 2〜3m、ヨー角 60〜90 度の跳びが必ず発生します。
- **共分散更新**: 外れ値付与時は共分散の下限を位置 `glitch_cov_floor_m2`、ヨー角
  `glitch_yaw_cov_floor_deg2` に引き上げ、自己位置信頼度低下を表現します。通常 publish 時は設定された
  共分散を使用します。
- **ログとデバッグ**: 受信値・適用オフセット・クールダウン残り秒数を INFO で記録し、false トリガ
  など無視した条件も DEBUG/INFO で把握できます。


## 入力途絶時の停止

`pose_timeout_sec`、`odom_timeout_sec` はそれぞれ既定 1.0 秒である。未受信または受信間隔が
閾値以上の場合、制御タイマーでゼロ速度を発行する。両入力の受信が復帰すると追従を再開する。

`obstacle_timeout_sec`（既定0:監視無効）を正の秒数にすると、
選択したscan/hint入力の未受信・途絶時に速度をゼロにする。
icart_bringupの共通実機profileは1秒を設定する。手動速度系には適用しない。

監視の判定時刻、stampの鮮度を検証しないこと、別途対策が必要な事象は
[robot_navigator 詳細設計書](docs/robot_navigator_詳細設計書.md) 7.6 節を参照する。

## 関連文書

制御則、障害物距離入力の扱い、制約は
[robot_navigator 詳細設計書](docs/robot_navigator_詳細設計書.md) を参照する。
