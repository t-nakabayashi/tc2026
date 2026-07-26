# follower_state フィールド整理メモ

## 2025-10-26 時点の反映内容
- `current_index` / `current_waypoint_label` / `current_label` の重複を解消し、`active_waypoint_index`・`active_waypoint_label` へ統一した。
- `next_waypoint_label` を廃止し、監視UIからも次ラベル表示を撤去した。
- 距離系フィールドを `active_target_distance_m`（現在地〜アクティブターゲット距離）と `segment_length_m`（前区間長）へ整理し、命名に単位サフィックスを付与した。
- 障害物ヒント統計の名称を `front_blocked` / `left_offset_m` / `right_offset_m` / `front_clearance_m` に揃え、中央値を意味するサフィックスを除去した。
- `current_pose` フィールドを削除し、姿勢は従来通り `/localization/pose_enu` から取得する前提とした。

## FollowerState.msg のフィールド一覧

| セクション | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| 進捗情報 | `route_version` | int32 | ルートバージョン番号 |
|  | `state` | string | `FollowerStatus` 名称 |
|  | `active_waypoint_index` | int32 | 現在追従しているウェイポイントのインデックス |
|  | `active_waypoint_label` | string | 現在追従ウェイポイントのラベル |
| 距離情報 | `active_target_distance_m` | float32 | 現在地とアクティブターゲットのユークリッド距離[m] |
|  | `segment_length_m` | float32 | 直前ウェイポイントから現在ウェイポイントまでの距離[m] |
| 回避・滞留 | `avoidance_attempt_count` | int32 | 現在ウェイポイントでの回避試行回数 |
|  | `last_stagnation_reason` | string | 直近の滞留理由ラベル |
| 障害物ヒント | `front_blocked` | bool | Hint 多数決による前方遮蔽判定 |
|  | `front_clearance_m` | float32 | Hint から得た前方余裕距離[m] |
|  | `left_offset_m` / `right_offset_m` | float32 | Hint サンプルの左右オフセット中央値[m] |

## 連動した実装更新
- `route_follower/follower_core.py`: 状態ディクショナリのキーを新フィールドへ合わせ、区間長を `segment_length_m` として算出するように変更した。
- `route_follower/route_follower_node.py`: Core 出力から現在地とターゲットの距離を算出し、`active_target_distance_m` へ設定。ログ出力も新しいインデックス名称を使用する。
- `robot_console/gui_core.py`: `FollowerStateView` を新フィールドへ追従させ、ターゲット距離カードの分母に `segment_length_m` を適用する。フォールバック距離は `active_target_distance_m` を使用する。
- `robot_console/ui_main.py`: 次ウェイポイントのカードを削除し、表示ラベルを `active_waypoint_*` 系へ差し替えた。
- `robot_console/tools/mock_ui.py` / `tools/tests/test_robot_console_gui_core.py`: テスト・モックデータを新フィールドへ更新し、進捗カード・区間長表示を再現。

## 今後の確認事項
- `segment_length_m` が 0.0 の場合の扱い（初期化直後や単独ウェイポイント）を GUI 側でどう表現するか運用確認する。
- RouteManager との整合（再計画通知や stuck レポートの表示項目）が追加で必要か確認する。
