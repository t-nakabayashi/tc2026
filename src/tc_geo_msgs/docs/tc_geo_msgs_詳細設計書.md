# tc_geo_msgs 詳細設計書

## 1. 目的・スコープ

本書は `tc_geo_msgs` パッケージの詳細設計を定める。対象は LLH 系座標、LLH pose、測位品質、
地図投影条件を表す ROS 2 interface である。

`tc_geo_msgs` は `tc_route_msgs`、`geo_pose_converter`、`gnss_lio_fusion`、経路採取と GUI で
共有する地理系 interface である。GUI、HTML 遠隔観測 UI、ログ、経路編集が同じ地理座標を
参照するため、**route に依存しない地理系 interface として独立させる。**

**本パッケージの責務は message 定義のみである。** 座標変換、GNSS 受信、route 生成、
localization fusion は責務外とする。

本書は interface package を対象とするため、実行時の処理・パラメータ・QoS を持たない章が
ある。該当しない章にはその旨と理由を記載する。

## 2. 構成とファイル配置

`ament_cmake` の interface package として構成する。`std_msgs/Header` を用いて時刻と frame を
保持し、**地理情報の意味を field 名とコメントで固定する。**

```text
src/tc_geo_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── GeoPoint.msg
│   ├── GeoPose.msg
│   ├── GeoPoseWithQuality.msg
│   ├── MapProjection.msg
│   └── FusionState.msg
└── docs/
    └── tc_geo_msgs_詳細設計書.md
```

| ファイル | 役割 |
| --- | --- |
| `msg/GeoPoint.msg` | WGS84 の latitude / longitude / altitude を表す |
| `msg/GeoPose.msg` | LLH point と heading / ENU yaw を持つ pose を表す |
| `msg/GeoPoseWithQuality.msg` | GNSS または fusion 後 pose の品質情報を表す |
| `msg/MapProjection.msg` | LLH と map ENU の変換条件を表す |
| `msg/FusionState.msg` | 融合ノードの状態を表す |

## 3. 外部インタフェース仕様

**本パッケージは topic を直接 publish / subscribe しない。** 各 message は次の用途で使用
する。

| Message | 主な利用 topic / field | 用途 |
| --- | --- | --- |
| `GeoPoint` | `GeoPose.point` | LLH 位置 |
| `GeoPose` | `Waypoint.geo_pose`, `ActiveTargetLlh.target_pose` | LLH pose |
| `GeoPoseWithQuality` | `/localization/pose_llh`, `/gnss/pose_llh` | 自己位置 LLH と品質 |
| `MapProjection` | `Route.projection`, `/geo/map_projection` | map ENU と LLH の投影条件 |
| `FusionState` | `/fusion/status` の構造化部分 | 融合ノードの状態 |

**QoS は利用 topic ごとに publish / subscribe する各ノードで定義する。** 本パッケージでは
定めない。

### 3.1 画面・可視化

**本パッケージは UI を持たない。** GUI は `GeoPoseWithQuality` と `ActiveTargetLlh` を参照
し、GPS 受信状態、自己位置、active target を表示する。

## 4. 設定の所在

**該当なし。** message package のため ROS parameter を持たない。

投影原点などの値は `MapProjection` を publish するノード側の parameter で管理する。
正本は `geo_pose_converter` の parameter YAML である（
[geo_pose_converter 詳細設計書](../../geo_pose_converter/docs/geo_pose_converter_詳細設計書.md)
4.1 節）。

## 5. データモデル・内部状態

**本章が本書の実体である。** 各 field の意味・単位・符号規約を定める。

### 5.1 `GeoPoint`

| field | 型 | 単位 | 意味 |
| --- | --- | --- | --- |
| `latitude` | `float64` | deg | WGS84 緯度。**北緯を正とする** |
| `longitude` | `float64` | deg | WGS84 経度。**東経を正とする** |
| `altitude` | `float64` | m | WGS84 楕円体高または入力データの高度 |
| `has_altitude` | `bool` | - | altitude が有効かどうか |

**走行用の pose は 2 次元として扱うため、`has_altitude=false` で配信される場合がある。**
受け取り側は高度を無条件に使わない。

### 5.2 `GeoPose`

**`heading_deg` は真北 0 度、時計回り正とする。`yaw_enu_rad` は ENU 座標系の yaw で、
東 0 rad、反時計回り正とする。**

両方を持つのは、運用者表示では heading が自然で、制御・変換処理では ENU yaw が扱いやすい
ためである。**2 つの値の整合は配信側の責務である。**

### 5.3 `GeoPoseWithQuality`

| field 群 | 意味 |
| --- | --- |
| `source` | GNSS、fusion、route file などの由来 |
| `fix_quality` | unknown / standalone / DGPS / RTK float / RTK fix |
| `fusion_status` | fusion ノードが pose を採用可能か |

**品質フィールドが実体を持つかは配信側による。** `route_geo_projector_node` が配信する
`/localization/pose_llh` は固定値を入れる（
[geo_pose_converter 詳細設計書](../../geo_pose_converter/docs/geo_pose_converter_詳細設計書.md)
7.4 節）。

### 5.4 `MapProjection`

`projection_type=PROJECTION_LOCAL_TANGENT_PLANE` を標準とする。

| field | 意味 |
| --- | --- |
| `origin_latitude`、`origin_longitude`、`origin_altitude` | map frame の原点に対応する LLH |
| `map_yaw_offset_rad` | **ENU east/north 軸から map x/y 軸への回転角** |

## 6. 処理フローと異常時の挙動

**該当なし。** message package のため runtime 処理とログを持たない。

**値の妥当性検証は message 利用側で実施する。** 定義側で範囲を強制する仕組みはない。

## 7. 判定ロジックと閾値

**該当なし。** 座標変換アルゴリズムは `geo_pose_converter` の `geo_core.py` に置く（
[geo_pose_converter 詳細設計書](../../geo_pose_converter/docs/geo_pose_converter_詳細設計書.md)
7 章）。

## 8. 並行性・周期・起動・依存関係

並行性・周期・launch は**該当なし。** 本パッケージは実行時の構成要素を持たない。

`std_msgs` と `rosidl_default_generators` に依存する。runtime では
`rosidl_default_runtime` を export する。

## 9. 確認方法と未確認範囲

### 9.1 自動確認

- `colcon build --packages-select tc_geo_msgs` が成功する。
- downstream の `tc_route_msgs` と `geo_pose_converter` が同時に build できる。

### 9.2 手動で確認する範囲

- `ros2 interface show tc_geo_msgs/msg/GeoPoseWithQuality` で生成された定義を確認する。
- 符号規約（5.1 節、5.2 節）が配信側の実装と一致していること。

### 9.3 未確認範囲

- **field の意味が全利用側で一致していることを機械的に検査していない。** 符号規約と単位は
  コメントと本書の記載に依存する。
- 品質フィールドの値域が実際の配信値を網羅しているかを確認していない。

## 10. 影響範囲・未決事項

### 10.1 影響範囲

`tc_route_msgs/Waypoint` と `tc_route_msgs/Route` は本パッケージの message を参照する。
**interface 定義を変更する場合は、利用する route stack も再ビルドする。**

利用側は `tc_route_msgs`、`geo_pose_converter`、`gnss_lio_fusion`、`route_survey`、
`robot_console` である。field の追加・削除・改名はこれら全体に影響する。

### 10.2 未決事項

| 件 | 選択肢 | blocker |
| --- | --- | --- |
| 5.2 節: heading と ENU yaw の整合を定義側で強制できない | 片方を正として他方を派生とする／現状どおり配信側の責務とする | ではない |
| 5.3 節: 品質フィールドが配信側によって実体を持たない | 未評価を表す値を定義する／現状を維持する | ではない |
