# route_manager パッケージ README

## 概要
`route_manager` は `route_planner` が提供する `/get_route`・`/update_route` サービスを利用し、
経路データの取得・配信・状態管理を統括する経路管理ノードです。ROS 2 ノード層
(`route_manager_node.py`) とコアロジック (`manager_core.py` / `manager_fsm.py`) を分割し、
非同期 FSM で経路要求と再計画を制御します。

## 主な機能
- 起動直後に `/get_route` を呼び出し、初期ルートと PNG 画像を取得。
- `/active_route` を **TRANSIENT_LOCAL** QoS で配信し、再起動後も最新ルートを取得可能にする。
- `/route_state`・`/manager_status` で FSM 状態・再計画判断・ルートバージョンを通知。
- `/mission_info` に start / goal / checkpoint の要求条件を Publish。
- `/report_stuck` で滞留報告を受け取り、`/update_route`・shift・skip を段階的に判断。
- 再計画結果を `ReportStuck.Response` の `decision_code`・`offset_hint`・`note` へ反映。

## 起動方法
### launch を用いた起動
```bash
ros2 launch route_manager route_manager.launch.py \
  start_label:=START \
  goal_label:=GOAL \
  checkpoint_labels:="['P1','P2']"
```

- `start_label` / `goal_label` はルートをスライスするラベル（未指定時は先頭〜末尾）。
- `checkpoint_labels` は YAML 既定値に追加で通過させたいラベルを配列で指定。
- 起動直後は `/get_route` 接続を待ち、接続後に FSM から初期要求が送出される。

## 外部インタフェース
### Publisher
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_route` | `tc_route_msgs/Route` | 現在有効なルート。`Route.version` は `(major % 100) * 100 + (minor % 100)` で管理。 | RELIABLE / TRANSIENT_LOCAL / depth=1 |
| `/route_state` | `tc_route_msgs/RouteState` | 現在 index・ラベル・ステータスを通知。 | RELIABLE / VOLATILE / depth=10 |
| `/mission_info` | `tc_route_msgs/MissionInfo` | 現在の start / goal / checkpoint を配信。 | RELIABLE / TRANSIENT_LOCAL / depth=1 |
| `/manager_status` | `tc_route_msgs/ManagerStatus` | FSM 状態と再計画判断結果を通知。 | RELIABLE / VOLATILE / depth=10 |

### Service Client
| 名称 | 型 | 説明 |
|------|----|------|
| `/get_route` | `tc_route_msgs/srv/GetRoute` | 初期ルート取得。plannerのmajor版を受け取り、managerのmajor/minor表現へ変換。 |
| `/update_route` | `tc_route_msgs/srv/UpdateRoute` | 滞留時の再計画。成功時に部分ルートを差し替え、`Route.version` を進める。 |

### Service Server
| 名称 | 型 | 説明 |
|------|----|------|
| `/report_stuck` | `tc_route_msgs/srv/ReportStuck` | `route_follower` からの滞留通報を受け付け、再計画方針と `offset_hint` を返す。 |

### Subscription

`/follower_state`（tc_route_msgs/FollowerState）を購読し、世代を照合して現在index・label・完了状態を反映します。
滞留の詳細は `/report_stuck` 要求から取得します。

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `start_label` | string | `""` | 初期ルート要求時の開始ラベル。空文字で先頭から開始。 |
| `goal_label` | string | `""` | 初期ルート要求時の終了ラベル。空文字で末尾まで。 |
| `checkpoint_labels` | string[] | `[]` | 追加で通過させたいチェックポイント一覧。 |
| `planner_timeout_sec` | double | `5.0` | `/update_route` 呼び出し時のサービス待機タイムアウト。 |
| `planner_retry_count` | int | `2` | 読み込むだけで、再試行処理を持たない。 |
| `planner_connect_timeout_sec` | double | `10.0` | `/get_route` 接続待ちの打ち切り時間。 |
| `state_publish_rate_hz` | double | `1.0` | タイマー周期の計算に使うが、そのコールバックは何もしない。状態の再送は 1 Hz 固定。 |
| `image_encoding_check` | bool | `false` | 読み込むだけで、画像の符号化を検査しない。 |
| `report_stuck_timeout_sec` | double | `5.0` | 読み込むだけで、滞留報告の処理に時間制限を適用しない。 |
| `offset_step_max_m` | double | `1.0` | Shift 回避で適用する横方向オフセットの上限値[m]。 |

再計画の時間制限は状態機械側の 5.0 秒で固定されており、ROS パラメータを持たない。

## 状態管理・処理フロー
### FSM 状態遷移
`manager_fsm.py` に実装された FSM は以下の状態を持ちます。FSM の状態名はログにのみ現れます。

| FSM 状態 | 説明 |
|----------|------|
| `IDLE` | 起動直後・ルート未取得 |
| `REQUESTING` | `/get_route` 応答待ち |
| `ACTIVE` | ルート配信中 |
| `WAITING_REROUTE` | 滞留中で再計画実行中 |
| `UPDATING` | 明示的な `/update_route` 応答待ち |
| `COMPLETED` | ルート完走（follower から完了通報） |
| `ERROR` | サービス失敗・タイムアウト・例外・再計画の全段階失敗 |

### 配信する状態

`/manager_status.state` と `/route_state.status` は FSM の状態名ではなく、Core が設定する
公開状態文字列から作ります。実際に渡されるのは次の 4 種です。

| 公開状態 | 設定する契機 | `/route_state.status` |
|----------|--------------|------------------------|
| `idle` | 初期値 | `STATUS_IDLE` |
| `running` | 経路の受理、shift / skip / 再配信の成立 | `STATUS_RUNNING` |
| `holding` | 再計画の全段階が失敗 | `STATUS_HOLDING` |
| `completed` | follower の完了通知 | `STATUS_COMPLETED` |

`/manager_status` には上表の `state` と `decision`（none / replan_first / update / shift /
skip / road_blocked_fixed / failed）を出力します。

再計画が全段階失敗した場合、公開状態は `holding` になる一方で FSM は `ERROR` へ遷移し、
以降の `/report_stuck` を受け付けません。詳細は
[詳細設計](docs/route_manager_詳細設計書.md)の 6.4 節を参照してください。

> 走行開始タイミングの制御を `route_follower` の `start_immediately` パラメータに統一し、
> `route_manager` は起動直後に必ず初期ルートを要求します。`start_immediately=false` の場合は
> `/manual_start` メッセージを受信した時点で `route_follower` が走行を開始します。

### `/report_stuck` に対する再計画フロー
1. Core が滞留報告を受理し、ルートバージョン・現在 index・滞留理由を検証する。
2. 通常の可変区間では `/update_route` を試す。固定区間ではplanner更新を省略する。
3. `/update_route` が失敗した場合は、現在 waypoint 自体を `offset_step_max_m` を上限として横へずらした経路を生成する（shift）。滞留理由が `front_blocked`・`avoidance_failed`・列挙外の場合のみ試す。
4. それでも走行不能な場合は現在 index を次へ進めた経路を再配信する（skip）。waypoint の配列は変えず、1 つの経路につき 1 回まで、`not_skip` の点では行わない。
5. すべて失敗した場合は `decision_code=FAILED` とし、`note` に原因を格納する。
6. `offset_hint` には shift の横ずれ量（m、左は負）を設定する。follower はこの値を記録するだけで、追従判断には使わない。

`ROAD_BLOCKED` は別分岐で、固定区間は現行経路を再配信し、可変区間はplanner再計画だけを試します。
この場合はshift／skipを行いません。[詳細設計](docs/route_manager_詳細設計書.md)を参照してください。

## 動作確認手順
1. `route_planner` を起動し、`/get_route` `/update_route` サービスが利用可能であることを確認する。
2. 上記コマンドで `route_manager` を起動し、初期ルート取得後に `/active_route` が Publish されることを確認する。
3. 隔離した模擬環境で、実際のroute_version・index・labelに合わせて `/report_stuck` を呼び出す。
   ```bash
   ros2 service call /report_stuck tc_route_msgs/srv/ReportStuck \
     "{route_version: 100, current_index: 5, current_wp_label: 'W5',
        current_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}}},
        reason_code: 1, reason_detail: 'front_blocked', avoid_trial_count: 2, last_hint_blocked: true,
        last_applied_offset_m: 0.5}"
   ```
   応答の `decision_code` や `offset_hint` を `/manager_status` と合わせて確認する。
4. `/route_state` や `/manager_status` を `ros2 topic echo` で監視し、FSM 状態遷移が意図どおりであることを確認する。

## デバッグのヒント
- `/manager_status` の `decision` が `skip` の場合、Core が現在 index を次へ進めた経路を `minor` 版を上げて再配信している。waypoint は削除されない。
- `/update_route` が連続で失敗する場合は、`route_planner` 側の閉塞ノード指定や YAML 設定を確認する。現在 index が 0 のときは前の点が無いため要求自体を出さない。
- `/get_route` が未起動のまま `planner_connect_timeout_sec` を超えると、error ログを出して初期要求を行わない。FSM は `IDLE` のままで `ERROR` にはならない。
- 本ノードは `MultiThreadedExecutor` を前提とする。単一スレッドで動かすと滞留報告の処理中にプランナ応答を受け取れずデッドロックする。
