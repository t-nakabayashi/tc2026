# LLH系トピックメッセージ仕様検討レポート

作成日: 2026-05-28

## 改版履歴

| 版 | 日付 | 変更概要 |
| --- | --- | --- |
| 0.4 | 2026-05-29 | Phase 2 の `/localization/pose_enu` 移行に合わせ、旧 `/amcl_pose` 前提を廃止した |
| 0.5 | 2026-05-30 | route interface package を `tc_route_msgs` に完全移行し、旧 package 名を廃止した |
| 0.3 | 2026-05-28 | 初期実装phaseでは package 改名を次phaseへ送る方針を追記した |
| 0.2 | 2026-05-28 | route正本はENUとLLHの両方を持つべきという方針へ更新し、LLH route別topicを原則不要に見直した |
| 0.1 | 2026-05-28 | 全体アーキテクチャ検討レポートと現状実装を踏まえ、LLH系topic/msg仕様案を整理した |

## 1. 目的

本レポートは、次期 `robot_console`、将来の `localization_fusion`、地図表示、経路観測、ログ解析で使用するLLH系topicとメッセージ仕様を検討するものである。

本版では、route interface package を `tc_route_msgs` に完全移行した前提に立ち、routeのあるべき姿を再整理する。結論として、route全体の正本である `/active_route` は、走行制御用のENU poseと地理表示・編集用のLLH poseを同時に保持するべきである。一方、走行制御中に逐次変わる `/active_target` と自己位置 topic `/localization/pose_enu` は、ENUを正本として維持する。

今回の実装phaseでは旧 package 名を残さず、依存パッケージの import / package.xml / launch / README を `tc_route_msgs` に一括移行する。

検討対象は以下である。

- `tc_route_msgs/Route` と `tc_route_msgs/Waypoint` の実装仕様、および今後の拡張仕様
- `localization_fusion` または周辺adapterがpublishする自己位置LLH topic
- 走行制御用 `/active_target` からGUI表示用LLH targetを導く方法
- `robot_console` の自己位置・センサ情報タブおよびHTML遠隔観測UIが購読すべき正本interface
- 既存 `rtk_gps_um982`、`route_planner`、`route_manager`、`route_follower`、`robot_navigator` との整合

本レポートでは実装そのものは行わない。後続実装時に追加・変更すべきmsg、topic、変換責務、既存実装の修正点を具体化する。

## 2. 参照した文書・実装

主に以下を確認した。

| 種別 | 対象 | 確認した内容 |
| --- | --- | --- |
| 全体方針 | `docs/次期システム_アーキテクチャ検討レポート.md` | 走行用routeはlocal ENU、LLHは地図表示・ログ・外部連携・ルート編集で使う方針。旧記述ではLLH routeを別topic化する想定が残っている |
| GPS driver | `src/rtk_gps_um982` | `/rtk_gps/fix`、`/rtk_gps/heading`、`/rtk_gps/rtk_status` をpublish。headingは既にREP-103 ENU quaternionへ変換済み |
| GPS msg | `src/rtk_gps_um982_msgs/msg/RtkStatus.msg` | RTK state、衛星数、HDOP、raw heading、heading標準偏差、baseline、correction age、LLHを保持 |
| route msg | `src/tc_route_msgs/msg/Route.msg`, `Waypoint.msg` | 現行routeは `header.frame_id="map"` のENU座標を前提とする。Waypoint msgにはLLH fieldがない |
| route生成 | `src/route_planner/route_planner/route_builder.py`, `route_planner.py` | CSVから `latitude` / `longitude` を読み込むが、現行 `WaypointRecord -> tc_route_msgs/Waypoint` 変換時にLLHはmsgへ渡らない |
| route管理 | `src/route_manager/route_manager/route_manager_node.py`, `manager_core.py` | `/active_route` は `tc_route_msgs/Route` をTRANSIENT_LOCALでpublishする。内部 `WaypointLite` はindexやLLH metadataを保持しない |
| 追従・制御 | `src/route_follower`, `src/robot_navigator` | `/active_route`、`/active_target`、`/localization/pose_enu` は `map` frameのENU poseとして扱われる。走行判断はENU poseを使う |
| GUI設計 | `src/robot_console/docs/robot_console_gui_architecture_design.md` | 走行系自己位置は `/localization/pose_enu`、表示系自己位置は `pose_llh` とLLH route/targetへ切り替える方針 |

## 3. 現状実装の整理

### 3.1 走行制御系はENU前提で成立している

現行route stackは、以下のtopic契約で成立している。

| Topic | Type | 座標系 | 主な利用者 |
| --- | --- | --- | --- |
| `/active_route` | `tc_route_msgs/msg/Route` | `map` frame local ENU | `route_follower`, `route_manager` marker, GUI |
| `/active_target` | `geometry_msgs/msg/PoseStamped` | `map` frame local ENU | `robot_navigator`, `obstacle_monitor`, GUI |
| `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `map` frame local ENU | `route_follower`, `robot_navigator`, `obstacle_monitor`, GUI |

`route_follower` と `robot_navigator` は、到達判定、障害物回避、速度指令生成を平面距離・yawに基づいて行う。ここへLLHを直接入れると、距離計算、方位計算、許容誤差、障害物回避subgoal生成の全てを再設計する必要がある。したがって、走行制御の正本をLLHへ置き換えるべきではない。

### 3.2 route CSVの正本は用途ごとにLLHまたはENUの一方にする

実コース用 route CSV は `latitude` / `longitude` / `heading_deg` を正本とし、走行に必要な ENU pose は `route_planner` が起動時に `geo_pose_converter.geo_core` のライブラリで生成する。simulation / Gazebo 用 route CSV は LLH を持たない ENU-only CSV とし、`x,y,z,q1..q4` をそのまま使用する。

LLH と ENU を同一 CSV に併記することは移行・検証目的では許容するが、二重正本にはしない。併記時は warning を出し、LLH を正本として ENU を再生成する。併記 ENU は水平 0.10 m、鉛直 0.30 m、heading 1.0 deg の閾値で検証し、超過時は warning を出す。

この方針により、route editor やログ解析で使う地理的正本は LLH に集約しつつ、走行制御系へ渡す `/active_route` では従来通り ENU pose を保持できる。

### 3.3 `rtk_gps_um982` はraw入力driverであり、融合後自己位置ではない

`/rtk_gps/fix` は `sensor_msgs/NavSatFix` としてLLH位置を持つ。`/rtk_gps/rtk_status` はRTK品質とraw headingを持つ。しかし、これらはGNSS受信機の観測値であり、LiDAR odometryやwheel odometryと融合した自己位置ではない。

`robot_console` が現在地表示として `/rtk_gps/fix` だけを正本にすると、将来 `localization_fusion` が出す推定自己位置との差が表現できない。したがって、GPS受信状態は `/rtk_gps/*`、運行上の自己位置は `localization_fusion` 系topicとして分ける必要がある。

## 4. 設計判断

### 4.1 route正本はENUとLLHの両方を持つ

`tc_route_msgs` への完全移行後は、`/active_route` を「走行用ENUだけのtopic」として閉じるのではなく、route定義の正本として拡張するべきである。

| データ | あるべき正本 | 理由 |
| --- | --- | --- |
| route全体 | `/active_route` (`tc_route_msgs/Route`) | route version、waypoint順序、停止属性、ENU pose、LLH poseを一体で管理する |
| waypointの走行位置 | `Waypoint.pose` | `route_follower` が使うENU pose。既存制御系との互換を維持する |
| waypointの地理位置 | `Waypoint.geo_pose` | OSM表示、route editor、ログ、外部連携で使うLLH pose |
| active targetの走行位置 | `/active_target` (`geometry_msgs/PoseStamped`) | 制御周期で使う現在目標。ENUのみでよい |
| active targetの地理表示 | `/route/active_target_llh` またはGUI内View | `/active_target` と `/active_route` の現在index/labelから導出する派生情報 |
| 自己位置の走行互換 | `/localization/pose_enu` | 既存ノード互換のENU pose |
| 自己位置の地理表示 | `/localization/pose_llh` | 融合後自己位置のLLH正本 |

この方針では、route全体を表す `/route/active_route_llh` は原則不要になる。`/active_route` の中にLLHが含まれるためである。必要なら、古いGUIや外部ツール向けの互換・抽出topicとして提供するに留める。

### 4.2 `Waypoint` にLLHを入れるが、二重正本にはしない

`Waypoint` にLLHを追加する場合、単に `latitude` / `longitude` を足すだけでは不十分である。ENU poseとLLH poseが同じroute点を表すこと、どの投影条件で対応していること、どちらが欠損・補完されたものかを明示する必要がある。

したがって、以下を満たす設計にする。

- `Waypoint.pose` は走行制御用のENU poseとして維持する。
- `Waypoint.geo_pose` は地理表示・編集・検証用のLLH poseとして追加する。
- `Waypoint.has_geo_pose` でLLH有無を明示する。
- `Waypoint.geo_pose_source` でCSV由来か、ENUから逆投影したものか、未確定かを明示する。
- `Route.projection` または `Route.projection_id` でENU/LLH対応条件をroute全体に持たせる。

これにより、制御系は従来どおり `Waypoint.pose` を読むだけでよく、GUIやHTML UIは `Waypoint.geo_pose` を読むだけでroute overlayを描ける。

### 4.3 `NavSatFix` だけでは自己位置LLHの正本にならない

自己位置LLHを `sensor_msgs/msg/NavSatFix` だけで表す案は採用しない。理由は以下である。

- headingが表現できない。
- RTK quality、fusion status、推定source、heading accuracyを一貫して表現できない。
- GNSS raw fixとfusion後poseの区別がtopic名以外に残りにくい。

`NavSatFix` はraw GNSS入力として維持し、運行上の自己位置LLHは専用msgで表現する。

## 5. 推奨するinterface package構成

### 5.1 interface package命名方針

LLH対応では `tc_route_msgs/Route` と `tc_route_msgs/Waypoint` を拡張する。route interface はプロジェクト接頭辞付きの `tc_route_msgs` に統一し、旧 package 名への alias や fallback は持たない。

同じ方針で、共有地理表現も `tc_geo_msgs` とする。`tc` は Tsukuba Challenge を表すプロジェクト接頭辞であり、ROSの一般的な `geo_msgs` / `geographic_msgs` 系名称との衝突を避ける。

```text
src/tc_geo_msgs/
├── package.xml
├── CMakeLists.txt
└── msg/
    ├── GeoPoint.msg
    ├── GeoPose.msg
    ├── GeoPoseWithQuality.msg
    └── MapProjection.msg

src/tc_route_msgs/
├── package.xml
├── CMakeLists.txt
├── msg/
│   ├── Waypoint.msg
│   ├── Route.msg
│   ├── ActiveTargetLlh.msg
│   └── ...
└── srv/
    ├── GetRoute.srv
    ├── UpdateRoute.srv
    ├── ReportStuck.srv
    └── ...
```

| Package | 役割 |
| --- | --- |
| `tc_geo_msgs` | routeに依存しないLLH point / pose / quality / projectionを提供する。`localization_fusion`、`geo_pose_converter`、`tc_route_msgs`、`robot_console` が共有する |
| `tc_route_msgs` | route id、version、waypoint index、停止属性、ENU pose、LLH poseを含むroute interfaceを提供する正式 package |
| `rtk_gps_um982_msgs` | UM982 driver固有のRTK状態表現を維持する。融合後poseやroute正本には使わない |

`tc_route_msgs` への移行は影響範囲が広いが、LLH field追加も同等に全依存パッケージの再ビルド・修正を必要とする。したがって、route interface package 名もこの phase で正す方針とする。

### 5.2 追加・変更するmsg一覧

推奨する変更は以下である。

| Package | Msg | 新規/変更 | 用途 |
| --- | --- | --- | --- |
| `tc_geo_msgs` | `GeoPoint` | 新規 | WGS84 LLH点 |
| `tc_geo_msgs` | `GeoPose` | 新規 | WGS84 LLH位置 + heading |
| `tc_geo_msgs` | `GeoPoseWithQuality` | 新規 | 融合後自己位置やGNSS poseの品質付きLLH |
| `tc_geo_msgs` | `MapProjection` | 新規 | ENU/LLH変換条件 |
| `tc_route_msgs` | `Waypoint` | 変更 | 既存ENU poseに加えて `geo_pose` とsource情報を持つ |
| `tc_route_msgs` | `Route` | 変更 | projection情報、route id、frame idを持つ |
| `tc_route_msgs` | `ActiveTargetLlh` | 新規任意 | GUI・ログ用の現在target LLH派生情報 |

`LlhRoute` / `LlhWaypoint` は原則追加しない。route全体のLLH情報は `Route` / `Waypoint` に含めるためである。

## 6. 共通メッセージ仕様案

### 6.1 共通規約

LLH系msgでは以下を共通規約とする。

| 項目 | 仕様 |
| --- | --- |
| 測地系 | WGS84 |
| latitude | degree。北緯を正、範囲 `[-90.0, 90.0]` |
| longitude | degree。東経を正、範囲 `[-180.0, 180.0]` |
| altitude | meter。原則としてWGS84楕円体高。MSL高度を使う場合は別fieldを追加するまで混用しない |
| heading_deg | 真北基準、時計回り正、範囲 `[0.0, 360.0)` |
| yaw_enu_rad | ENU yawが必要な場合のみ使用。東向きx軸基準、反時計回り正 |
| stamp | 原則として計測または推定対象時刻。単なるpublish時刻ではない |
| header.frame_id | LLHでは固定的に `earth` を推奨する。車体取付位置を表したい場合は別field `child_frame_id` を使う |
| freshness | msgには入れない。購読側が `header.stamp` と現在時刻から計算する |

`heading_deg` と `yaw_enu_rad` の変換は以下である。

```text
yaw_enu_rad = pi / 2 - radians(heading_deg)
heading_deg = degrees(pi / 2 - yaw_enu_rad) を [0, 360) に正規化
```

`rtk_gps_um982/heading` は既にENU quaternionであるため、これを使う場合はraw headingから再変換しない。GUIで北基準headingを表示する場合は `RtkStatus.heading_deg` または `GeoPose.heading_deg` を使う。

### 6.2 `tc_geo_msgs/msg/GeoPoint.msg`

```ros
# WGS84 LLH point.
# latitude/longitude are degrees. altitude is WGS84 ellipsoid height [m].
float64 latitude
float64 longitude
float64 altitude
bool has_altitude
```

`has_altitude=false` の場合、`altitude` は表示や制御判断に使わない。CSVに高度がないrouteでも同じmsgを使えるようにするためである。

### 6.3 `tc_geo_msgs/msg/GeoPose.msg`

```ros
std_msgs/Header header

tc_geo_msgs/GeoPoint point

# Heading: true north clockwise [deg]. [0, 360).
float32 heading_deg
bool has_heading

# Optional ENU yaw for debug or conversion validation.
# East-zero, counter-clockwise positive [rad].
float32 yaw_enu_rad
bool has_yaw_enu

# Frame attached to this pose, normally base_link, gps_link, or route_waypoint.
string child_frame_id
```

正本の向き表現は `heading_deg` とする。`yaw_enu_rad` はENU変換検証やdebug用途であり、両方を入れる場合はpublisherが整合を保証する。

### 6.4 `tc_geo_msgs/msg/MapProjection.msg`

```ros
std_msgs/Header header

uint8 PROJECTION_UNKNOWN=0
uint8 PROJECTION_LOCAL_TANGENT_PLANE=1
uint8 PROJECTION_UTM=2
uint8 projection_type

string projection_id
string datum
string map_frame_id
string earth_frame_id

float64 origin_latitude
float64 origin_longitude
float64 origin_altitude
float32 map_yaw_offset_rad

# Optional UTM metadata. Empty/zero when projection_type is not UTM.
string utm_zone
bool utm_north
```

`projection_id` は、route、自己位置、ログが同じENU/LLH変換条件に基づくことを確認するための識別子である。GUIは `projection_id` が不一致の場合に警告できる。

### 6.5 `tc_geo_msgs/msg/GeoPoseWithQuality.msg`

```ros
std_msgs/Header header

tc_geo_msgs/GeoPose pose

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_GNSS=1
uint8 SOURCE_GNSS_LIDAR_FUSION=2
uint8 SOURCE_GNSS_WHEEL_FUSION=3
uint8 SOURCE_GNSS_LIDAR_WHEEL_FUSION=4
uint8 SOURCE_SIMULATION=5
uint8 SOURCE_REPLAY=6
uint8 source

uint8 FIX_UNKNOWN=0
uint8 FIX_NONE=1
uint8 FIX_STANDALONE=2
uint8 FIX_DGPS=3
uint8 FIX_RTK_FLOAT=4
uint8 FIX_RTK_FIX=5
uint8 fix_quality

uint8 FUSION_UNKNOWN=0
uint8 FUSION_INITIALIZING=1
uint8 FUSION_DEGRADED=2
uint8 FUSION_OK=3
uint8 FUSION_ERROR=4
uint8 fusion_status

float32 horizontal_accuracy_m
float32 vertical_accuracy_m
float32 heading_accuracy_deg

uint8 num_satellites
float32 hdop
float32 correction_age_s
uint32 rtcm_bytes_received

string status_text
```

`fix_quality` はGNSS品質、`fusion_status` は融合推定器としての状態を表す。RTK FixでもLiDAR融合が破綻していれば `FUSION_DEGRADED` または `FUSION_ERROR` にできる。

## 7. `tc_route_msgs` 変更仕様案

### 7.1 `tc_route_msgs/msg/Waypoint.msg`

現行fieldは維持し、LLH fieldとsource情報を追加する。

```ros
# Waypoint in an active route.
int32 index
string label

# Driving/control pose in map/local ENU.
geometry_msgs/Pose pose
bool has_pose_enu

# Geographic pose for map display, route editing, logging, and validation.
tc_geo_msgs/GeoPose geo_pose
bool has_geo_pose

uint8 GEO_SOURCE_UNKNOWN=0
uint8 GEO_SOURCE_ROUTE_FILE=1
uint8 GEO_SOURCE_PROJECTED_FROM_ENU=2
uint8 GEO_SOURCE_MANUAL_EDIT=3
uint8 geo_pose_source

float32 right_open
float32 left_open

bool line_stop
bool signal_stop
bool not_skip
bool segment_is_fixed
```

互換性上、既存consumerは `pose` を従来どおり読めばよい。`has_pose_enu` は将来の検証用であり、走行用 `/active_route` では原則trueとする。`has_geo_pose=false` のwaypointはOSM overlayでは欠損として扱うか、projectorで補完したうえで `GEO_SOURCE_PROJECTED_FROM_ENU` とする。

### 7.2 `tc_route_msgs/msg/Route.msg`

現行fieldは維持し、route識別子と投影条件を追加する。

```ros
std_msgs/Header header
int32 version
float32 total_distance
sensor_msgs/Image route_image

string route_id
string map_frame_id
string earth_frame_id
tc_geo_msgs/MapProjection projection

tc_route_msgs/Waypoint[] waypoints
int32 start_index
string start_waypoint_label
```

`header.frame_id` は従来どおり `map` を基本とする。`map_frame_id` は明示的な文字列としても保持し、`projection.map_frame_id` と一致させる。`earth_frame_id` は `earth` を推奨する。

`total_distance` は走行制御に使うENU route距離[m]とする。LLH上の測地距離を再計算した値ではない。

### 7.3 `tc_route_msgs/msg/ActiveTargetLlh.msg` 任意追加

active targetは制御用 `/active_target` を維持する。GUIやHTML UIがtargetのlabel、index、方位、距離を直接購読したい場合のみ、派生topicとして以下を追加する。

```ros
std_msgs/Header header

int32 route_version
int32 target_index
string target_label

tc_geo_msgs/GeoPose target_pose

float32 distance_m
float32 bearing_deg

bool is_avoidance_subgoal
string target_kind
```

このmsgはroute全体の正本ではない。`/active_target`、`/follower_state`、`/active_route`、`/localization/pose_llh` から生成できる表示用snapshotである。

## 8. 推奨topic仕様

### 8.1 route / target

| Topic | Type | Publisher | Subscriber | QoS | 用途 |
| --- | --- | --- | --- | --- | --- |
| `/active_route` | `tc_route_msgs/msg/Route` | `route_manager` | `route_follower`, `robot_console`, HTML UI backend, logger | `RELIABLE / TRANSIENT_LOCAL / depth=1` | route正本。ENU poseとLLH poseを含む |
| `/active_target` | `geometry_msgs/msg/PoseStamped` | `route_follower` | `robot_navigator`, `obstacle_monitor`, `robot_console` | `RELIABLE / VOLATILE / depth=10` | 走行制御用target ENU pose |
| `/route/active_target_llh` | `tc_route_msgs/msg/ActiveTargetLlh` | `route_follower` または `robot_console` backend | `robot_console`, HTML UI backend, logger | `RELIABLE / VOLATILE / depth=10` | 表示・ログ用target LLH派生情報 |

`/route/active_route_llh` は正式topicとしては設けない。どうしても必要な場合は、`/active_route` からLLH部分だけを抽出した互換topicとして扱う。

### 8.2 自己位置LLH

| Topic | Type | Publisher | Subscriber | QoS | 用途 |
| --- | --- | --- | --- | --- | --- |
| `/localization/pose_llh` | `tc_geo_msgs/msg/GeoPoseWithQuality` | `localization_fusion` | `robot_console`, HTML UI backend, logger | `RELIABLE / VOLATILE / depth=10` | 融合後自己位置のLLH正本 |
| `/gnss/pose_llh` | `tc_geo_msgs/msg/GeoPoseWithQuality` | `geo_pose_converter` または `rtk_gps_adapter` | `localization_fusion`, debug GUI, logger | `RELIABLE / VOLATILE / depth=10` | GNSS単独pose。融合前の観測値 |
| `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `localization_fusion` または simulator | 走行系 | `RELIABLE / VOLATILE / depth=10` | ENU自己位置 |

### 8.3 raw GPS topicとの関係

| Topic | Type | 扱い |
| --- | --- | --- |
| `/rtk_gps/fix` | `sensor_msgs/msg/NavSatFix` | raw GNSS位置。GPS受信状態とadapter入力に使う |
| `/rtk_gps/heading` | `sensor_msgs/msg/Imu` | raw dual antenna headingをENU quaternionで表したもの。変換済みorientationとして使う |
| `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/msg/RtkStatus` | RTK品質、raw heading、補正age、衛星数。GUIのGPSカードとquality生成に使う |

raw GPS topicは廃止しない。`GeoPoseWithQuality` はraw topicを置き換えるのではなく、後段が使いやすいよう正規化した派生topicである。

## 9. Publisher責務案

### 9.1 `geo_pose_converter`

`geo_pose_converter` は以下を担当する。

- `/rtk_gps/fix`、`/rtk_gps/heading`、`/rtk_gps/rtk_status` を同期または近傍時刻で結合する。
- GNSS単独の `/gnss/pose_llh` をpublishする。
- LLHをENUへ変換し、`/gnss/pose_enu` をpublishする。
- ENU/LLH変換条件を `MapProjection` として提供する。
- heading規約を一元管理する。

`geo_pose_converter` はroute versionや停止属性を知らないため、`/active_route` はpublishしない。ただし、route生成系が同じ投影条件と変換式を使えるよう、`geo_core.py` を ROS 非依存ライブラリとして提供し、projection設定を `params/default.yaml` などの共通 parameter YAML で管理する。通常の route 生成では service 呼び出しに依存せず、`route_planner` が install 済み Python ライブラリとして `geo_pose_converter.geo_core` を import する。

### 9.2 `route_planner`

`route_planner` は以下を担当する。

- 実コース用 LLH-only CSV から `latitude`, `longitude`, `altitude`, `heading_deg` を読み込み、`geo_pose_converter.geo_core` で ENU pose と quaternion を生成する。
- simulation / Gazebo 用 ENU-only CSV では `x,y,z,q1..q4` をそのまま使用し、LLH field は持たせない。
- LLH と ENU が併記された CSV は warning を出し、LLH を正本として採用する。併記 ENU は整合性検証にのみ使う。
- CSVにLLHがあるwaypointは `has_geo_pose=true`, `geo_pose_source=GEO_SOURCE_ROUTE_FILE` とする。
- CSVにLLHがないwaypointは ENU-only route として `has_geo_pose=false` とし、必要な表示変換は projector 側で派生させる。
- `Route.projection` を埋めた `tc_route_msgs/Route` をservice responseとして返す。
- `projection_config_path` で `geo_pose_converter` の共通 parameter YAML を参照し、route_planner 固有の `origin_*` 設定を持たない。

### 9.3 `route_manager`

`route_manager` は以下を担当する。

- `route_planner` から受け取った `Route` のENU pose、LLH pose、projection、waypoint indexを保持する。
- `/active_route` をTRANSIENT_LOCALでpublishする。
- route更新、skip、replan後もLLH metadataを失わない。
- `Route.version` と `Route.projection.projection_id` をログへ出す。

現状の `WaypointLite` はindexとLLH metadataを保持しないため、route正本化するには拡張が必要である。`core_route_to_ros()` でも `wp.index` を明示的に設定するべきである。

### 9.4 `route_follower`

`route_follower` は走行判断には従来どおり `Waypoint.pose` を使う。LLH fieldは制御には使わない。

必要であれば、以下を追加する。

- 現在target index/labelと `/active_route.waypoints[index].geo_pose` から `/route/active_target_llh` をpublishする。
- 回避subgoalなどroute外targetでは、ENU targetを `geo_pose_converter` の逆投影機能でLLH化し、`is_avoidance_subgoal=true` とする。

### 9.5 `localization_fusion`

`localization_fusion` は以下を担当する。

- GNSS由来ENU pose、LiDAR odometry、wheel odometryを融合する。
- 融合後 ENU 自己位置として `/localization/pose_enu` をpublishする。
- 正本自己位置として `/localization/pose_llh` をpublishする。
- 必要に応じて `/localization/pose_enu` と `/localization/status` をpublishする。

## 10. `robot_console` 仕様への影響

`robot_console` の正式画面仕様は、以下の正本入力を前提に具体化する。

| GUI表示 | 正本topic | fallback |
| --- | --- | --- |
| 現在地マーカー | `/localization/pose_llh` | `/localization/pose_enu` を逆投影した暫定LLH、または `/rtk_gps/fix` |
| GPS受信状態 | `/rtk_gps/rtk_status`, `/rtk_gps/fix` | なし。未受信として表示 |
| route overlay | `/active_route.waypoints[].geo_pose` | `/active_route.route_image` または `/active_route.waypoints[].pose` のENU簡易表示 |
| active target | `/route/active_target_llh` または `/active_route` + `/follower_state` から生成したView | `/active_target` を逆投影した暫定LLH |
| 自己位置品質 | `/localization/pose_llh` のquality + `/rtk_gps/rtk_status` | `/localization/pose_enu` freshnessのみ |
| raw GNSSとの差分 | `/localization/pose_llh` と `/gnss/pose_llh` | `/rtk_gps/fix` と `/localization/pose_enu` 逆投影 |

これにより、自己位置・センサ情報タブとHTML遠隔観測UIは、以下を具体表示できる。

- OSM地図上の融合後自己位置、GNSS raw位置、active route、active target
- RTK state、衛星数、HDOP、correction age、RTCM bytes更新状況
- fusion status、source、水平精度、heading精度
- route waypoint index/label、signal_stop/line_stop/not_skip/segment_is_fixed
- active targetまでの距離、方位、target種別
- `/localization/pose_llh` と `/gnss/pose_llh` の位置差
- `Route.projection.projection_id` と自己位置側projectionの一致/不一致

初期実装で `/localization/pose_llh` が未提供の場合でも、画面仕様はLLH正本を前提に書くべきである。fallbackは互換層の仕様として扱い、画面そのものの正本仕様にしない。

## 11. 移行手順案

### Phase A: interface定義

- `tc_geo_msgs` を追加し、`GeoPoint`, `GeoPose`, `GeoPoseWithQuality`, `MapProjection` を定義する。
- `tc_route_msgs/Waypoint` に `geo_pose`, `has_geo_pose`, `geo_pose_source`, `has_pose_enu` を追加する。
- `tc_route_msgs/Route` に `route_id`, `map_frame_id`, `earth_frame_id`, `projection` を追加する。
- 必要なら `tc_route_msgs/ActiveTargetLlh` を追加する。
- msgコメントに測地系、単位、heading規約、quality enum、projectionの意味を明記する。

### Phase B: route生成・管理系のLLH保持

- `route_planner` の `WaypointRecord` に `altitude`, `heading_deg`, `has_*` を追加する。
- `WaypointRecord -> tc_route_msgs/Waypoint` 変換でLLH fieldを埋める。
- `write_waypoints_to_csv()` がLLHを落とさないようにする。
- `route_manager` の `WaypointLite` / `RouteModel` にindex、geo_pose、projectionを持たせる。
- `core_route_to_ros()` と `ros_route_to_core()` でindexとLLH metadataを保持する。

### Phase C: localization / projection実装

- `geo_pose_converter` または暫定adapterで `/gnss/pose_llh` と `/gnss/pose_enu` をpublishする。
- `/localization/pose_enu` とprojectionがある場合は、暫定 `/localization/pose_llh` をpublishする。
- `localization_fusion` 実装後は `/localization/pose_llh` を正本publishする。

### Phase D: GUI / HTML UI切替

- `robot_console` とHTML UIは `/active_route` のLLH fieldをroute overlay正本として使う。
- target表示は `/route/active_target_llh` があれば購読し、なければ `/active_route` + `/follower_state` から生成する。
- `/route/active_route_llh` への依存を持たせない。

## 12. 受け入れ条件

LLH系topic/msg仕様の実装完了条件は以下とする。

- `/active_route` の全waypointで、ENU poseとLLH poseの有無が明示される。
- route CSVにLLHがあるwaypointは `geo_pose_source=GEO_SOURCE_ROUTE_FILE` になる。
- LLHがないwaypointは逆投影されるか、`has_geo_pose=false` としてGUIに欠損表示される。
- `Route.projection.projection_id` がpublishされ、GUIで表示できる。
- `/active_route.version` は従来どおりroute更新ごとに管理される。
- `/active_target` はENU poseのまま維持され、既存 `robot_navigator` と `obstacle_monitor` の入力型を変えない。
- `/localization/pose_llh` で、融合後または暫定自己位置の緯度・経度・heading・qualityを取得できる。GNSS/localization由来の高度がある場合は `GeoPoint.has_altitude=true` で高度も保持する。
- `robot_console` はOSM地図表示に必要な座標変換を内部に持たず、公開topicのLLH fieldを表示できる。
- raw GNSS位置と融合後自己位置が画面上で区別される。
- heading表示が真北基準CWで統一され、ENU yawとの混同がない。

## 13. 未決事項

| 項目 | 推奨または確認内容 | 実装blocker |
| --- | --- | --- |
| 地理系package名 | `tc_geo_msgs` を推奨候補とする。短縮名が必要ならチーム内で再検討する | Phase Aではblocker |
| route interface package名 | `tc_route_msgs` に完全移行する。旧 package 名は残さない | 高 |
| `header.frame_id` | LLH msgは `earth` 推奨。既存REP運用やTF設計と合わせて最終確認する | 低 |
| 高度の扱い | 走行用ENUでは高度を使わず `z=0.0` を標準とする。GNSS/localization由来のLLH高度は `GeoPoint.has_altitude=true` で保持し、routeやtargetの高度未指定・ENU逆投影由来は `has_altitude=false` とする | 中 |
| route CSVの高度・heading | 現行CSVはLLH altitude/heading_degを正規化していない。route編集仕様と合わせて拡張する | Phase Bでblocker |
| `/route/active_target_llh` のpublisher | `route_follower` がpublishするか、`robot_console` backendがView生成するかを決める | 中 |
| 逆投影に必要な原点管理 | `geo_pose_converter` のparamsまたは `MapProjection` で一元管理する必要がある | Phase Cではblocker |

## 14. 実装影響と確認観点

### 14.1 package依存関係

`tc_geo_msgs` を新規追加する場合、少なくとも以下の依存追加が必要になる。

| 対象 | 追加・変更内容 | 理由 |
| --- | --- | --- |
| `tc_geo_msgs/package.xml` | `ament_cmake`, `rosidl_default_generators`, `std_msgs`, `rosidl_default_runtime` | `GeoPose` 系msg生成に必要 |
| `tc_geo_msgs/CMakeLists.txt` | `rosidl_generate_interfaces()` で `GeoPoint.msg`, `GeoPose.msg`, `GeoPoseWithQuality.msg`, `MapProjection.msg` を登録 | interface生成 |
| `tc_route_msgs/package.xml` | `tc_geo_msgs` のbuild/exec依存を追加 | `Waypoint` / `Route` が地理系msgを参照する |
| `tc_route_msgs/CMakeLists.txt` | `find_package(tc_geo_msgs REQUIRED)` と `rosidl_generate_interfaces()` の依存へ追加 | route msg生成 |
| `route_planner`, `route_manager`, `robot_console` | `package.xml` / `setup.py` の依存を確認 | 新fieldを読み書き・表示する |

`route_follower` / `robot_navigator` はLLH fieldを制御に使わない。ただし `route_follower` は `tc_route_msgs/Route` 型を購読するため、msg変更に伴う再ビルドは必要である。

### 14.2 互換性リスク

| リスク | 内容 | 対策 |
| --- | --- | --- |
| ENU/LLH不整合 | 同一waypointに対するENUとLLHが異なる原点・回転で生成される | `geo_pose_converter` の共通 projection 設定を参照し、併記CSVはroute生成時に整合検査を行う |
| heading二重変換 | `/rtk_gps/heading` のENU quaternionと `RtkStatus.heading_deg` を混在させる | raw headingからENU yawへ変換する場所を `geo_pose_converter` に限定する |
| route metadata欠落 | `route_manager` の内部モデルでLLHやindexが失われる | `WaypointLite` / `RouteModel` を正本情報を落とさない形へ拡張する |
| GUI fallback固定化 | ENU `/localization/pose_enu` 表示が通常地図表示の正本として残る | 画面仕様ではLLH topicを表示正本とし、ENU表示はデバッグ・fallback扱いにする |

### 14.3 テスト観点

実装時は以下を確認する。

- `GeoPose` のheading変換で、heading 0 degが北向き、90 degが東向きとして表示される。
- `RtkStatus.STATE_RTK_FIX` が `GeoPoseWithQuality.FIX_RTK_FIX` へ対応する。
- `/active_route.version` と `Route.projection.projection_id` がroute更新後も保持される。
- `/active_route` がTRANSIENT_LOCALで、GUI後起動でもENU/LLH両方を受信できる。
- route CSVにLLHがあるwaypointは `GEO_SOURCE_ROUTE_FILE` になる。
- route_managerを経由してもwaypoint index、LLH、projectionが失われない。
- `/localization/pose_llh` と `/gnss/pose_llh` のsource/fusion/fix qualityが画面上で区別できる。
- `/active_target` は従来どおりENU `PoseStamped` として `robot_navigator` が購読できる。

## 15. 現実装を変更してでもあるべき姿へ戻すべき点

今回のLLH設計以外にも、次の点は実装を変更してでも整えるべきである。

| 対象 | 現状 | あるべき姿 |
| --- | --- | --- |
| `route_planner.write_waypoints_to_csv()` | LLHを読み込めるが書き戻しheaderに `latitude` / `longitude` がない | route編集・再保存でLLHを落とさない |
| `route_manager.WaypointLite` | labelとpose中心で、indexやLLH metadataを保持しない | `Route` の正本情報を落とさない内部モデルにする |
| `route_manager.core_route_to_ros()` | `Waypoint.index` を明示設定していない | 配列順だけに頼らず、msg fieldとしてindexを維持する |
| `route_follower` 内部Waypoint | indexを保持せず、coreの配列indexで進捗を扱う | 制御上は配列indexでよいが、表示・target LLH生成ではmsg indexとの対応を保持できるようにする |
| route画像 `route_image` | route msg内に埋め込まれており、地理routeとは別表現になっている | route overlayの正本はwaypoint LLHとし、画像は補助表示・互換表示に限定する |
| GUIの地図変換責務 | `/localization/pose_enu` と `/active_route` からGUI側でLLHを推測しがち | GUIは公開LLH fieldを表示し、測地変換ロジックを持たない |
| package命名 | route interface と地理系 interface の接頭辞を統一する | `tc_route_msgs` と `tc_geo_msgs` として接頭辞をそろえる |

## 16. 結論

`tc_route_msgs` への完全移行後は、route全体のあるべき姿は「ENU走行poseとLLH地理poseを同じ `/active_route` に保持する正本route」である。これにより、routeのversion、waypoint順序、停止属性、ENU pose、LLH pose、projection条件が一体で管理され、GUI・HTML UI・ログ・route editorが同じroute正本を参照できる。

一方、走行中の `/active_target` と `/localization/pose_enu` は、既存制御系の入力としてENUのまま維持する。LLH targetは必要に応じて `/route/active_target_llh` として派生生成する。raw GPS topicは品質監視とadapter入力として維持し、融合後自己位置 `/localization/pose_llh` とは明確に分離する。
