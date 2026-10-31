# tc_route_msgs パッケージ README

## 概要
`tc_route_msgs` は経路計画・走行系で共通利用するメッセージ／サービス型を定義する
インタフェース専用パッケージです。ルートバージョン管理、滞留理由（`reason_code` / `reason_detail`）、
経路封鎖判定用フラグなどを定義し、
`route_manager`・`route_planner`・`route_follower`・`obstacle_monitor`・`robot_navigator` など
複数ノード間のデータ交換を統一します。

## 主な機能
- 経路データ (`Route`・`Waypoint`) と状態通知 (`RouteState`・`ManagerStatus`) のメッセージ定義。
- ミッション設定 (`MissionInfo`) やフォロワ状態 (`FollowerState`) の共有メッセージ定義。
- `/get_route`・`/update_route`・`/report_stuck` 向けサービス定義。
- 障害物回避支援情報 (`ObstacleAvoidanceHint`) の定義。
- AI エージェントの回避助言 (`AvoidanceAdvice`) と、その要求用 action (`RequestAvoidanceAdvice`) の定義。

## 起動方法
本パッケージはノードを持たず、`colcon build` により他パッケージと同時にビルドされます。
インタフェースの確認には以下を利用してください。
```bash
ros2 interface show tc_route_msgs/msg/Route
ros2 interface show tc_route_msgs/srv/ReportStuck
ros2 interface show tc_route_msgs/action/RequestAvoidanceAdvice
```

## 外部インタフェース
本パッケージはトピックやサービスサーバを提供しませんが、以下のメッセージ／サービス型を
他パッケージが使用します。

### メッセージ定義
| 名称 | 概要 | 主な利用ノード |
|------|------|---------------|
| `Route` | 経路全体。`version`・`route_image`・`Waypoint[]` を保持。 | `route_manager`, `route_planner` |
| `Waypoint` | 個別経路点。`label`・`pose`・停止/スキップフラグを含む。 | `route_planner`, `route_follower` |
| `RouteState` | 進行中ルートの概要（現在 index・status）。 | `route_manager`, 監視 UI |
| `ManagerStatus` | FSM 状態と再計画判断 (`decision`) を通知。 | `route_manager`, `route_follower` |
| `MissionInfo` | start/goal/チェックポイントの情報。 | `route_manager`, ダッシュボード |
| `FollowerState` | フォロワの状態・滞留統計を報告。 | `route_follower`, ログ収集 |
| `ObstacleAvoidanceHint` | 前方閉塞・左右オフセットのヒント。 | `obstacle_monitor`, `robot_navigator`, `route_follower` |
| `DriveModeStatus` | 自律/手動モード、入力鮮度、出力元、復帰待ち状態。 | `drive_mode_manager`, `robot_console` |
| `ActiveTargetLlh` | 現在の目標のLLH表示情報。 | `geo_pose_converter`, `robot_console` |
| `MotionLimits` | ドライバが確認した速度・加減速度上限とstamp。 | `ypspur_ros2`, `robot_navigator` |
| `AvoidanceAdvice` | AI エージェントが返した回避方向・オフセット・待機時間。 | `avoidance_advisor`, `route_follower` |

### サービス定義
| 名称 | 概要 | リクエスト主要項目 | 主な利用ノード |
|------|------|------------------|---------------|
| `GetRoute` | 初期ルートを取得。応答で `Route` を返す。 | `start_label`・`goal_label`・`checkpoint_labels`。 | `route_manager` → `route_planner` |
| `UpdateRoute` | 滞留時の部分ルート差し替え。 | `prev_index`・`current_index`・`route_version`・`reason`。 | `route_manager` ↔ `route_planner` |
| `ReportStuck` | 滞留報告と再計画結果返却。 | `route_version`・`current_index`・`reason_code`・`reason_detail`・`avoid_trial_count`。 | `route_follower` → `route_manager` |

### action 定義
| 名称 | 概要 | goal 主要項目 | 主な利用ノード |
|------|------|--------------|---------------|
| `RequestAvoidanceAdvice` | 停止中に回避の助言を要求する。result で `AvoidanceAdvice` と採否を返す。 | `waypoint_left_open_m`・`waypoint_right_open_m`・`front_clearance_m`・`min_offset_m`・`max_offset_m`。 | `route_follower` → `avoidance_advisor` |

## パラメータ
- 本パッケージが公開するパラメータはありません。

## 状態管理・処理フロー
- `Route.version` は `route_manager` が `(major % 100) * 100 + (minor % 100)` で符号化し、`/active_route` 配信の世代管理に利用します。
- `RouteState.status` は `STATUS_IDLE`～`STATUS_ERROR` の定数で表します。manager内部FSMの状態文字列とは表現が異なります。
- `ReportStuck.Response.decision_code` は `route_manager` の再計画・シフト・スキップ判断に一致し、
  `offset_hint` が `route_follower` への横オフセット指示となります。
- `ObstacleAvoidanceHint.front_clearance_m` は `robot_navigator`・`route_follower` の減速／停止条件に利用されます。
- `AvoidanceAdvice` は提案値です。`action=ACTION_UNKNOWN`、`advice_valid=false`、確信度不足のいずれでも `route_follower` は従来の回避へ退避します。採用する場合も waypoint の開放幅・実測空き幅・大域上限でクランプした範囲内に限ります。
- `RequestAvoidanceAdvice` は待機上限を client 側が持ちます。cancel された goal に対して server は起動中の外部プロセスを終了します。
- `MotionLimits`は`header`と、線/角速度上限、線加速度、線減速度、角加速度を保持します。
  既定の`/motion_limits`はRELIABLE / VOLATILE / depth=1の継続配信で、latchedな設定通知ではありません。
  `robot_navigator`はドライバの能力が要求値以上で、stampと受信がともに新しい場合だけ採用します。
  詳細は[ナビゲータの制動と入力監視](../robot_navigator/README.md)を参照してください。

## 動作確認手順
1. `colcon build` 後、`ros2 interface show tc_route_msgs/msg/ManagerStatus` などで定義内容を確認する。
2. `ros2 interface proto tc_route_msgs/msg/Route` を用いてテストデータを生成し、Publish/Service 呼び出しの型整合を確認する。
3. `route_manager`・`route_planner`・`route_follower` 間で通信ログを収集し、定義どおりのフィールドが送受信されているか検証する。

## デバッグのヒント
- `ros2 interface show` で実際の定義を確認し、生成コードと送受信側の型を揃えてください。
- メッセージ拡張時は `route_manager`・`route_planner` など関連パッケージの README と docs を同時に更新すること。
- `front_clearance_m=inf` は送信側の点数不足でも生じます。観測正常を意味する値ではないため、入力scanと前方抽出条件を確認します。
