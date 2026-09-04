# geo_pose_converter パッケージ README

## 概要
`geo_pose_converter` は、LLH(WGS84 緯度・経度・高度) と map frame 上の ENU pose を相互変換し、走行系 ENU topic と表示系 LLH topic を分離して扱うためのパッケージです。

主な用途は次の通りです。

- GNSS センサ系 topic を `tc_geo_msgs` と ENU pose へ変換する。
- `/localization/pose_enu`、`/active_route`、`/active_target` を表示用 LLH topic へ投影する。
- LLH 自己位置を OpenStreetMap 上で確認する。
- route CSV の `latitude,longitude,heading_deg` をもとに、走行用2D ENU pose と表示用LLH poseを相互変換する。

## ノード
### `geo_pose_converter_node`
GNSS driver 由来 topic を購読し、GNSS 単独の LLH pose と ENU pose を publish します。

既定の主な topic は次の通りです。

| 方向 | Topic | 型 | 説明 |
| --- | --- | --- | --- |
| Subscribe | `/rtk_gps/fix` | `sensor_msgs/msg/NavSatFix` | GNSS 測位結果。 |
| Subscribe | `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/msg/RtkStatus` | fix quality、衛星数、heading など。 |
| Publish | `/gnss/pose_llh` | `tc_geo_msgs/msg/GeoPoseWithQuality` | GNSS 単独 LLH pose。 |
| Publish | `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | localization_fusion 実装前の暫定 ENU pose。 |
| Publish | `/geo/map_projection` | `tc_geo_msgs/msg/MapProjection` | ENU/LLH 変換条件。 |

localization_fusion 実装後は、`/gnss/pose_enu` を fusion 入力、`/localization/pose_enu` を fusion 後の走行系自己位置として扱う想定です。

### `route_geo_projector_node`
ENU topic を表示用 LLH topic へ変換します。

| 方向 | Topic | 型 | 説明 |
| --- | --- | --- | --- |
| Subscribe | `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 走行系自己位置。 |
| Subscribe | `/active_route` | `tc_route_msgs/msg/Route` | route 正本。`Route.projection` と waypoint LLH を使用する。 |
| Subscribe | `/active_target` | `geometry_msgs/msg/PoseStamped` | 走行系 active target。 |
| Publish | `/localization/pose_llh` | `tc_geo_msgs/msg/GeoPoseWithQuality` | GUI、HTML UI、OSM 表示用の自己位置。 |
| Publish | `/route/active_target_llh` | `tc_route_msgs/msg/ActiveTargetLlh` | 表示用 active target。 |

`Route.projection` を受信すると、その projection を使って `/localization/pose_enu` を LLH に戻します。ENU pose は走行用2D座標のため、逆投影で作ったLLH高度は有効扱いせず `GeoPoint.has_altitude=false` とします。route CSV 生成時と `Route.projection` の原点がずれると、表示位置もずれるため注意してください。

### `llh_osm_viewer_node`
`/localization/pose_llh`、`/active_route`、`/route/active_target_llh` を購読し、ローカル HTTP サーバで OpenStreetMap ビューアを提供します。自己位置は赤い二等辺三角形で表示され、三角形の向きは `heading_deg` を表します。active route は `Waypoint.geo_pose` を持つ点だけを青い線と点で重畳し、active target は橙色の点で重畳します。

このノードは `geo_pose_converter` の診断・統合確認用ビューアです。正式な `robot_console` HTML遠隔観測UIでは、運行サマリ、センサ画像、node health、読み取り専用Snapshot APIを `robot_console` 側で扱う想定です。

起動例:

```bash
source install/setup.bash
ros2 run geo_pose_converter llh_osm_viewer_node --ros-args \
  -p pose_llh_topic:=/localization/pose_llh \
  -p http_host:=127.0.0.1 \
  -p http_port:=18765 \
  -p open_browser:=false
```

ブラウザで次を開きます。

```text
http://127.0.0.1:18765/
```

最新 pose の JSON は次で確認できます。

```bash
curl http://127.0.0.1:18765/pose
```

自己位置、route、active target、pose鮮度状態をまとめた viewer 状態は次で確認できます。

```bash
curl http://127.0.0.1:18765/state
```

`pose_status` は `OK`、`STALE`、`LOST`、`NO_DATA` のいずれかです。これはHTTPレスポンス生成時刻と最後に `/localization/pose_llh` を受信した壁時計時刻の差から判定します。地図上の自己位置マーカー色は通信状態で変更せず、状態表示欄でのみ示します。

Leaflet と OpenStreetMap タイルは CDN/外部タイルを利用するため、地図表示にはネットワーク接続が必要です。

## launch
`geo_pose_converter.launch.py` は `geo_pose_converter_node` と `route_geo_projector_node` を起動します。`enable_llh_osm_viewer:=true` を指定すると、診断用の `llh_osm_viewer_node` も同時に起動します。

```bash
source install/setup.bash
ros2 launch geo_pose_converter geo_pose_converter.launch.py \
  config:=$(ros2 pkg prefix geo_pose_converter)/share/geo_pose_converter/params/default.yaml
```

シミュレーションなど GNSS 入力が無い構成では、`geo_pose_converter_node` を無効にし、`route_geo_projector_node` だけを起動できます。

```bash
ros2 launch geo_pose_converter geo_pose_converter.launch.py \
  enable_geo_pose_converter:=false
```

OSM viewer も同時に起動する例:

```bash
ros2 launch geo_pose_converter geo_pose_converter.launch.py \
  enable_geo_pose_converter:=false \
  enable_llh_osm_viewer:=true \
  llh_osm_viewer_port:=18765 \
  llh_osm_viewer_open_browser:=false
```

## パラメータ
`params/default.yaml` では、projection 設定を `/**` の共通パラメータとして定義しています。`geo_pose_converter_node` と `route_geo_projector_node` は同じ ENU/LLH 変換条件を使う必要があります。

| 名称 | 既定値 | 説明 |
| --- | --- | --- |
| `projection_id` | `tokyo_station` | 投影条件識別子。 |
| `datum` | `WGS84` | 測地系。 |
| `map_frame_id` | `map` | ENU pose の frame 名。 |
| `earth_frame_id` | `earth` | LLH pose の frame 名。 |
| `origin_latitude` | `35.681382` | ENU/LLH 変換原点の緯度。 |
| `origin_longitude` | `139.766084` | ENU/LLH 変換原点の経度。 |
| `origin_altitude` | `3.86` | ENU/LLH 変換原点の高度[m]。 |
| `map_yaw_offset_rad` | `0.0` | map x/y と ENU east/north の回転オフセット[rad]。 |
| `pose_llh_topic` | `/localization/pose_llh` | `llh_osm_viewer_node` が表示する自己位置LLH topic。 |
| `active_route_topic` | `/active_route` | `llh_osm_viewer_node` がroute overlayに使うtopic。 |
| `active_target_llh_topic` | `/route/active_target_llh` | `llh_osm_viewer_node` がactive target overlayに使うtopic。 |
| `stale_timeout_s` | `1.0` | viewer上でposeをSTALE扱いにする受信停止時間[s]。 |
| `lost_timeout_s` | `3.0` | viewer上でposeをLOST扱いにする受信停止時間[s]。 |

## 動作確認例
route stack のシミュレーション実行中に、別プロセスで以下を起動すると OSM 上の自己位置を確認できます。

```bash
source install/setup.bash
ros2 run geo_pose_converter route_geo_projector_node --ros-args \
  --params-file $(ros2 pkg prefix geo_pose_converter)/share/geo_pose_converter/params/default.yaml
ros2 run geo_pose_converter llh_osm_viewer_node --ros-args \
  -p open_browser:=false \
  -p http_port:=18765
```

確認観点:

- `/localization/pose_llh` が `tc_geo_msgs/msg/GeoPoseWithQuality` として publish される。
- `http://127.0.0.1:18765/pose` が `null` ではなく、route 近傍の緯度経度を返す。
- OSM 上に赤い二等辺三角形が表示され、走行に合わせて移動する。
- `/active_route` に `Waypoint.geo_pose` が含まれる場合、青いroute polylineとwaypoint点が表示される。
- `/route/active_target_llh` がpublishされる場合、橙色のactive target点が表示される。
- `http://127.0.0.1:18765/state` の `pose_status` が受信停止時間に応じて `OK` / `STALE` / `LOST` へ変わる。
