# route_follower パッケージ README

## 概要
`route_follower` は `route_manager` から配信される `/active_route` を追従し、
現在のターゲット Pose を `/active_target` に Publish する経路追従ノードです。ROS 2
ラッパ層 (`route_follower_node.py`) とロジック中核 (`follower_core.py`) を分離し、滞留検知と
`/report_stuck` サービス呼び出しを統合しています。

## 主な機能
- `/active_route` 受信時に `FollowerCore` へ適用し、`start_immediately` 設定に応じて自動開始。
- `/localization/pose_enu`・`/obstacle_avoidance_hint`・`/sig_recog`・`/manual_start` を購読し、追従状態や滞留判定に利用。
- `FollowerCore.tick()` の結果を `/active_target` と `/follower_state` へ出力し、滞留統計も Publish。
- 滞留検知時に `/report_stuck` を呼び出し、`decision_code` や `offset_hint` を反映して走行継続・スキップ・失敗を判定。
- 1/`resend_interval_sec` [Hz] でターゲット Pose を再送し、制御系へ冪等な指示を提供。

## 起動方法
```bash
ros2 launch route_follower route_follower.launch.py \
  arrival_threshold:=0.6 \
  control_rate_hz:=20.0 \
  resend_interval_sec:=1.0 \
  start_immediately:=true \
  target_frame:=map
```

- launch 引数はノードパラメータへそのまま反映される。
- `param_file` で YAML (`params/default.yaml`) を差し替えることも可能。

## 外部インタフェース
### 購読トピック
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_route` | `tc_route_msgs/Route` | 追従対象ルート。`Route.version` と `start_index` を `FollowerCore` に転送。 | RELIABLE / TRANSIENT_LOCAL |
| `/localization/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | 現在姿勢。2D yaw を算出して `FollowerCore` へ投入。 | RELIABLE / VOLATILE |
| `/obstacle_avoidance_hint` | `tc_route_msgs/ObstacleAvoidanceHint` | 障害物回避ヒント。最新値を統計化し滞留判定に利用。 | BEST_EFFORT / VOLATILE |
| `/road_blocked` | `std_msgs/Bool` | 道路封鎖入力。Coreで最新状態を保持する。 | RELIABLE / VOLATILE |
| `/manual_start` | `std_msgs/Bool` | TRUE 受信で手動再開。 | RELIABLE / VOLATILE |
| `/sig_recog` | `std_msgs/Int32` | 信号認識結果（1=GO, 2=NOGO 等）。 | RELIABLE / VOLATILE |

### 配信トピック
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_target` | `geometry_msgs/PoseStamped` | 現在向かうべきターゲット Pose。`target_frame` を frame_id に設定。 | RELIABLE / VOLATILE |
| `/recog_flag` | `std_msgs/Int32` | 信号認識の実行制御。 | RELIABLE / VOLATILE |
| `/follower_state` | `tc_route_msgs/FollowerState` | 状態・route_version・滞留統計を Publish。 | RELIABLE / VOLATILE |

### サービスクライアント
| 名称 | 型 | 説明 |
|------|----|------|
| `/report_stuck` | `tc_route_msgs/srv/ReportStuck` | 滞留判定時に呼び出し。現在 index やオフセット履歴を送信し、再計画方針を受け取る。 |

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `arrival_threshold` | double | `0.6` | Waypoint 到達判定距離[m]。 |
| `control_rate_hz` | double | `20.0` | `FollowerCore.tick()` を呼ぶ周期[Hz]。0 以下の場合は 20Hz に補正。 |
| `resend_interval_sec` | double | `1.0` | `/active_target` 再送周期[s]。0 以下の場合は 1.0 秒に補正。 |
| `start_immediately` | bool | `true`（YAML は `false`） | ルート受信直後に自動で RUNNING へ遷移するか。 |
| `target_frame` | string | `"map"` | Publish する Pose の frame_id。 |
| `avoid_back_offset_m` | double | `0.5`（YAML は `0.75`） | 回避起点を進行方向後方へ下げる距離[m]。負の場合は 0.0 に補正。 |
| `road_blocked_confirmation_sec` | double | `5.0` | `/road_blocked` の封鎖継続を確認する時間[s]。負の場合は 0.0 に補正。 |

既定値のうち `start_immediately` と `avoid_back_offset_m` は、ノードが宣言する値と
`params/default.yaml` の値が異なる。launch は既定で YAML を読み込むため `ros2 launch`
では YAML の値が有効になり、パラメータを与えない `ros2 run` では宣言側の値が有効になる。

## 状態管理・処理フロー
### Follower 状態遷移
`FollowerCore` は以下の状態を持ち、`/follower_state.state` に文字列として出力します。

| 状態 | 説明 |
|------|------|
| `IDLE` | ルート未適用。`start_immediately=false` で manual_start 待ち。 |
| `RUNNING` | 通常追従中。 |
| `WAITING_STOP` | 停止属性のある点に到達して待機。manual_startで再開し、信号停止はsig_recog=1でも再開。 |
| `STAGNATION_DETECTED` | 滞留検知直後の経過状態。同一周期内で必ず他の状態へ移るため `/follower_state` には現れない。 |
| `AVOIDING` | 回避サブゴール（横オフセットなど）を実行中。 |
| `WAITING_REROUTE` | 再計画待ち。新しい `/active_route` の適用で再開する。 |
| `FINISHED` | 最終 Waypoint へ到達。 |
| `ERROR` | 経路未適用、再計画待ちの期限超過、再計画失敗の通知。自己復帰しない。 |

### `/report_stuck` 呼び出しフロー
1. `FollowerCore` が滞留を検知すると `WAITING_REROUTE` に遷移し、ノード側で `/report_stuck` を非同期呼び出しする。応答待ちの重複呼び出しは 1 件に抑える。
2. リクエストには `route_version`・`current_index`・`current_wp_label`・現在 Pose・滞留理由・ヒント統計を含める。
3. 応答のうち挙動へ反映するのは `DECISION_FAILED` のみで、`notify_reroute_failed()` を呼び出し `ERROR` へ遷移してオペレータ介入を要求する。
4. それ以外の `decision_code` と `offset_hint` はログに出すだけで追従判断に使わない。再開は `route_manager` が新しい `/active_route` を配信することで成立する。

## 動作確認手順
1. `route_manager` を起動し、`/active_route` と `/report_stuck` が利用可能な状態にする。
2. 本ノードを起動後、`ros2 topic echo /follower_state` で状態遷移を監視する。
3. `/manual_start` に `std_msgs/Bool data:true` を Publish し、`WAITING_STOP` から再開できることを確認する。
4. `/report_stuck` を `ros2 service call` 等で模擬し、`decision_code` に応じたログと状態変化を確認する。

## デバッグのヒント
- `/active_route` の `header.frame_id` は検査しない。`/active_target` の `frame_id` は購読内容に依らず `target_frame` で配信する。
- `/obstacle_avoidance_hint` が 5 秒窓で 5 サンプル未満の場合は統計が成立せず、`front_blocked` は False のままになる。滞留時は理由 `no_hint` で再計画要求となる。
- `/report_stuck` が利用できない状態で滞留するとリトライ待ちとなり、ログに "not ready" が出力される。サービス起動順を確認する。30 秒で `ERROR` へ移る。
- 同じ目標の周期再送では `header.stamp` を据え置く。新しい指示との区別に使える。


## 停止・回避の制約

WAITING_STOPやFINISHEDは位置・状態の判定であり、実測速度ゼロの確認結果ではありません。
目標の再送停止だけでnavigatorの保持目標は消えません。manual_start=falseも走行停止ではありません。
停止・ERROR・終了と下位の指令失効の結合を別途確認する必要があります。
回避目標の最低オフセットが小さい許可幅を上回る場合があり、左右幅を厳密な境界保証として使えません。
