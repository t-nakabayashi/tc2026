# tc_route_msgs LLH拡張 詳細設計書

## 1. 文書目的・対象範囲

本書は `tc_route_msgs` の LLH 拡張仕様を定義する。対象は `Waypoint.msg`、`Route.msg`、`ActiveTargetLlh.msg` である。今回の実装 phase では package 名を `tc_route_msgs` に統一し、旧 package 名への alias や fallback は残さない。

## 2. 背景・要求・スコープ

route CSV には LLH 情報を持つ waypoint が存在する一方、従来の `/active_route` は ENU pose のみを公開していた。そのため、GUI や HTML UI は route の地理座標を正本 topic から取得できず、別変換や別ファイル参照が必要になっていた。

本拡張では `/active_route` を route 正本として、走行制御に使う ENU pose と、表示・ログ・将来編集に使う LLH pose を同時に保持する。`/active_target` は走行制御用の ENU pose として維持し、LLH 表示用には `/route/active_target_llh` を追加する。

## 3. 全体構成・アーキテクチャ

`tc_route_msgs` は route stack の正式 interface package である。LLH 拡張では `tc_geo_msgs` に依存し、route 固有 message から共通地理 message を参照する。

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

## 4. パッケージ構成・ファイル配置

| ファイル | 変更内容 |
| --- | --- |
| `msg/Waypoint.msg` | ENU pose 有効フラグ、LLH pose、LLH 由来種別を追加 |
| `msg/Route.msg` | route id、frame id、投影条件を追加 |
| `msg/ActiveTargetLlh.msg` | active target の LLH 派生情報を追加 |
| `package.xml` | `tc_geo_msgs` 依存を追加 |
| `CMakeLists.txt` | `tc_geo_msgs` を rosidl 依存へ追加 |

## 5. 外部インタフェース仕様

| Topic / field | 型 | 方向 | QoS | 意味 |
| --- | --- | --- | --- | --- |
| `/active_route` | `tc_route_msgs/msg/Route` | `route_manager` publish | `RELIABLE / TRANSIENT_LOCAL / depth=1` | route 正本。ENU pose と LLH pose を含む |
| `/route/active_target_llh` | `tc_route_msgs/msg/ActiveTargetLlh` | `geo_pose_converter` publish | `RELIABLE / VOLATILE / depth=10` | 表示・ログ用 active target LLH |
| `Waypoint.pose` | `geometry_msgs/Pose` | field | - | 走行制御用 map ENU pose |
| `Waypoint.geo_pose` | `tc_geo_msgs/GeoPose` | field | - | 表示・ログ用 LLH pose |
| `Route.projection` | `tc_geo_msgs/MapProjection` | field | - | route ENU と LLH の変換条件 |

## 6. パラメータ・設定仕様

`tc_route_msgs` 自体は parameter を持たない。`Route.projection` の値は `route_planner` と `geo_pose_converter` の parameter で管理する。

## 7. データモデル・内部状態

### `Waypoint.msg`

`pose` は既存互換のため維持する。`has_pose_enu` は pose が走行制御用に有効かどうかを表す。通常の route waypoint では `true` とする。

`geo_pose` は LLH pose を保持する。route CSV に LLH がある場合は `has_geo_pose=true` とし、`geo_pose_source=GEO_SOURCE_ROUTE_FILE` とする。LLH がない waypoint では `has_geo_pose=false` とし、GUI は projection による推定表示と区別する。

### `Route.msg`

`route_id` は route 設定を識別する文字列である。`map_frame_id` は ENU pose の frame、`earth_frame_id` は LLH pose の frame を表す。`projection` は `/active_route` の ENU pose と LLH pose を対応付ける条件である。

### `ActiveTargetLlh.msg`

`target_index` と `target_label` は follower の active waypoint と対応する。`target_pose` は route waypoint の LLH があればそれを優先し、なければ active target ENU pose を projection で LLH に変換した値を使う。`distance_m` と `bearing_deg` は現在自己位置から target への目安であり、制御には使わない。

## 8. 処理フロー・状態遷移

1. `route_planner` が route CSV を読み込み、`Waypoint.pose` と `Waypoint.geo_pose` を埋める。
2. `route_manager` が `/active_route` を publish する。この際、LLH field と projection を落とさず再配信する。
3. `route_follower` は既存どおり ENU pose のみを参照して `/active_target` を publish する。
4. `route_geo_projector` が `/active_route`、`/active_target`、自己位置 LLH を購読し、`/route/active_target_llh` を publish する。

## 9. 主要アルゴリズム・判定ロジック

LLH 変換は message package では実装しない。`route_geo_projector` は route waypoint に `has_geo_pose=true` の LLH がある場合、それを active target LLH の正本として使う。なければ `Route.projection` または node parameter の projection を使って active target ENU pose を LLH に変換する。

## 10. QoS・並行性・タイミング設計

`/active_route` は既存運用どおり transient local とする。`/route/active_target_llh` は active target と自己位置の更新に追従する揮発性 stream とし、古い target を後続 subscriber に残さない。

## 11. 起動・終了・launch 設計

`tc_route_msgs` 自体は launch を持たない。関連 node の launch は `route_planner`、`route_manager`、`geo_pose_converter` 側に置く。

## 12. エラー処理・ログ・診断

message package のためログは持たない。`has_geo_pose=false` の waypoint は異常ではなく、projection 変換可能な fallback として扱う。

## 13. UI・可視化仕様

GUI と HTML UI は `/active_route` の `Waypoint.geo_pose` と `/route/active_target_llh` を参照する。走行制御用の `/active_target` は ENU pose のまま維持し、表示用 LLH と混同しない。

## 14. 依存関係・ビルド設定

`tc_route_msgs` は `tc_geo_msgs` に build / exec 依存する。`CMakeLists.txt` の `rosidl_generate_interfaces()` に `tc_geo_msgs` を依存として追加する。

## 15. テスト計画・受け入れ条件

- `tc_route_msgs` が `tc_geo_msgs` と共に build できる。
- `route_planner` が route CSV の LLH を `Waypoint.geo_pose` に保持する。
- `route_manager` の local shift / skip / reissue で LLH field が失われない。
- `route_follower` と `robot_navigator` の ENU 制御 topic は従来どおり動作する。

## 16. 互換性・移行・影響範囲

既存 `Waypoint` と `Route` に field を追加するため、生成済み message を利用する downstream package は再ビルドが必要である。topic 名は維持するが、package 名は `route_msgs` から `tc_route_msgs` へ変更したため、downstream package は import、`package.xml`、`CMakeLists.txt` または `setup.py` を更新する必要がある。旧 package 名への互換 alias は提供しない。

## 17. 未決事項・今後の拡張

- 旧 package 名への alias や fallback は設けない。
- route editor 実装時に `geo_pose_source` の手動編集値と route file 値の扱いを追加定義する。

## 18. 改版履歴

| 日付 | 版 | 変更概要 |
| --- | --- | --- |
| 2026-05-30 | 1.1 | package 名を `tc_route_msgs` に完全移行し、旧 package 名を廃止 |
| 2026-05-28 | 1.0 | 初版。LLH 拡張仕様を定義 |
