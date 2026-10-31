# tc_route_msgs LLH拡張 詳細設計書

## 1. 目的・スコープ

本書は `tc_route_msgs` の LLH 拡張仕様を定める。対象は `Waypoint.msg`、`Route.msg`、
`ActiveTargetLlh.msg` である。

**`/active_route` を経路の正本とし、走行用 ENU と表示・保存用 LLH を同じメッセージに
保持する。** 表示用目標は `/route/active_target_llh`、**制御用目標は `/active_target`**
を使う。

`tc_route_msgs` は route stack の正式 interface package である。LLH 拡張では `tc_geo_msgs`
に依存し、route 固有 message から共通地理 message を参照する。各 field の意味は
[tc_geo_msgs 詳細設計書](../../tc_geo_msgs/docs/tc_geo_msgs_詳細設計書.md)を併せて参照する。

本書は interface package を対象とするため、実行時の処理・パラメータを持たない章がある。
該当しない章にはその旨と理由を記載する。

## 2. 構成とファイル配置

```text
src/tc_route_msgs/
├── msg/
│   ├── Waypoint.msg
│   ├── Route.msg
│   └── ActiveTargetLlh.msg
├── CMakeLists.txt
├── package.xml
└── docs/
    └── tc_route_msgs_llh拡張_詳細設計書.md
```

| ファイル | 内容 |
| --- | --- |
| `msg/Waypoint.msg` | ENU pose 有効フラグ、LLH pose、LLH 由来種別を定義 |
| `msg/Route.msg` | route id、frame id、投影条件を定義 |
| `msg/ActiveTargetLlh.msg` | active target の LLH 派生情報を定義 |
| `package.xml` | `tc_geo_msgs` 依存を定義 |
| `CMakeLists.txt` | `tc_geo_msgs` を rosidl 依存へ追加 |

本パッケージは LLH 拡張以外の route stack 用 message（`FollowerState`、`RouteState`、
`ManagerStatus`、`DriveModeStatus`、`MotionLimits`、`ObstacleAvoidanceHint`、`MissionInfo`）
と service も持つ。**本書の対象は LLH 拡張の 3 つに限る。** 他の message の意味は各 `.msg`
のコメントを正とする。

## 3. 外部インタフェース仕様

| Topic / field | 型 | 方向 | QoS | 意味 |
| --- | --- | --- | --- | --- |
| `/active_route` | `tc_route_msgs/msg/Route` | `route_manager` publish | RELIABLE / TRANSIENT_LOCAL / depth 1 | route 正本。ENU pose と LLH pose を含む |
| `/route/active_target_llh` | `tc_route_msgs/msg/ActiveTargetLlh` | `geo_pose_converter` publish | RELIABLE / VOLATILE / depth 10 | 表示・ログ用 active target LLH |
| `Waypoint.pose` | `geometry_msgs/Pose` | field | — | 走行制御用 map ENU pose |
| `Waypoint.geo_pose` | `tc_geo_msgs/GeoPose` | field | — | 表示・ログ用 LLH pose |
| `Route.projection` | `tc_geo_msgs/MapProjection` | field | — | route ENU と LLH の変換条件 |

**`/active_route` を transient local にするのは、購読側の起動順に依らず最新経路を受け取れる
ようにするためである。** `/route/active_target_llh` は active target と自己位置の更新に
追従する揮発性 stream とし、**古い target を後続 subscriber に残さない。**

### 3.1 画面・可視化

GUI と HTML UI は `/active_route` の `Waypoint.geo_pose` と `/route/active_target_llh` を
参照する。**走行制御用の `/active_target` は ENU pose のまま維持し、表示用 LLH と混同
しない。**

## 4. 設定の所在

**該当なし。** `tc_route_msgs` 自体は parameter を持たない。

`Route.projection` の値は `route_planner` と `geo_pose_converter` の parameter で管理する。

## 5. データモデル・内部状態

**本章が本書の実体である。**

### 5.1 `Waypoint.msg`

`pose` は走行制御用の位置・姿勢を保持する。`has_pose_enu` は pose が走行制御用に有効か
どうかを表す。**通常の route waypoint では `true` とする。**

`geo_pose` は LLH pose を保持する。

| 状況 | field の値 |
| --- | --- |
| route CSV に LLH がある | `has_geo_pose=true`、`geo_pose_source=GEO_SOURCE_ROUTE_FILE` |
| LLH がない | `has_geo_pose=false` |

**LLH がない waypoint では、GUI は projection による推定表示と区別する。** 地図描画は
`has_geo_pose=true` の点のみを描く（
[robot_console 画面・機能詳細設計書](../../robot_console/docs/robot_console_gui_screen_function_design.md)
7.3 節）。

### 5.2 `Route.msg`

| field | 意味 |
| --- | --- |
| `route_id` | route 設定を識別する文字列 |
| `map_frame_id` | ENU pose の frame |
| `earth_frame_id` | LLH pose の frame |
| `projection` | `/active_route` の ENU pose と LLH pose を対応付ける条件 |

### 5.3 `ActiveTargetLlh.msg`

`target_index` と `target_label` は follower の active waypoint と対応する。

`target_pose` は**route waypoint の LLH があればそれを優先し、なければ active target ENU
pose を projection で LLH に変換した値**を使う（7 章）。

**`distance_m` と `bearing_deg` は現在自己位置から target への目安であり、制御には使わない。**

## 6. 処理フローと異常時の挙動

message package のため runtime 処理とログを持たない。本章は**利用側がこの interface を
どの順に埋めるか**を定める。

1. `route_planner` が route CSV を読み込み、`Waypoint.pose` と `Waypoint.geo_pose` を埋める。
2. `route_manager` が `/active_route` を publish する。この際、**LLH field と projection を
   落とさず再配信する。**
3. `route_follower` は **ENU pose のみを参照して** `/active_target` を publish する。
4. `route_geo_projector` が `/active_route`、`/active_target`、自己位置 ENU を購読し、
   `/route/active_target_llh` を publish する。

**`has_geo_pose=false` の waypoint は異常ではなく、projection 変換可能な fallback として
扱う。**

値の妥当性検証は message 利用側で実施する。定義側で範囲を強制する仕組みはない。

## 7. 判定ロジックと閾値

**LLH 変換は message package では実装しない。**

`route_geo_projector` の優先順位は次のとおりである。

1. route waypoint に `has_geo_pose=true` の LLH がある場合、**それを active target LLH の
   正本として使う。**
2. なければ `Route.projection` または node parameter の projection を使って active target
   ENU pose を LLH に変換する。

実装は
[geo_pose_converter 詳細設計書](../../geo_pose_converter/docs/geo_pose_converter_詳細設計書.md)
6.3 節を正とする。

## 8. 並行性・周期・起動・依存関係

並行性・周期・launch は**該当なし。** `tc_route_msgs` 自体は launch を持たない。関連 node の
launch は `route_planner`、`route_manager`、`geo_pose_converter` 側に置く。

`tc_route_msgs` は `tc_geo_msgs` に build / exec 依存する。`CMakeLists.txt` の
`rosidl_generate_interfaces()` に `tc_geo_msgs` を依存として追加する。

## 9. 確認方法と未確認範囲

### 9.1 自動確認

- `tc_route_msgs` が `tc_geo_msgs` と共に build できる。
- `route_planner` が route CSV の LLH を `Waypoint.geo_pose` に保持する
  （`route_planner/tests/test_route_builder_llh.py`）。

### 9.2 手動で確認する範囲

- `route_manager` の local shift / skip / reissue で **LLH field が失われないこと**
- `route_follower` と `robot_navigator` の ENU 制御 topic が動作すること
- `has_geo_pose=false` の waypoint が地図描画から除かれること

### 9.3 未確認範囲

- **LLH field の保持を機械的に検査する試験は `route_planner` 側のみである。**
  `route_manager` のローカル更新での保持は手動確認に依存する。
- `distance_m` と `bearing_deg` が制御に使われていないことを機械的に保証していない。

## 10. 影響範囲・未決事項

### 10.1 影響範囲

`Waypoint` と `Route` は route stack の interface である。**定義を変更した場合は、利用する
パッケージも再ビルドする。**

利用側は `route_planner`、`route_manager`、`route_follower`、`robot_navigator`、
`geo_pose_converter`、`route_survey`、`obstacle_route_sim`、`robot_console` である。

`tc_geo_msgs` の定義変更は本パッケージを経由して route stack 全体へ波及する。

### 10.2 未決事項

| 件 | 選択肢 | blocker |
| --- | --- | --- |
| 9.3 節: ローカル更新での LLH 保持に自動試験が無い | `route_manager` へ試験を追加する／手動確認に留める（現状） | ではない |
| 2 章: 本書の対象が LLH 拡張の 3 message に限られる | 他の message も本書へ含める／各 `.msg` のコメントを正とする（現状） | ではない |
