# geo_pose_converter 詳細設計書

## 1. 文書目的・対象範囲

本書は `geo_pose_converter` パッケージの詳細設計を定義する。対象ノードは `geo_pose_converter_node` と `route_geo_projector_node` である。

本書では、走行系で扱う自己位置と、GUI / HTML UI / OSM 表示で扱う自己位置を明確に分ける。走行系の自己位置は `map` frame の ENU pose を正とする。表示系の自己位置、route、active target は WGS84 LLH を正とし、GUI / HTML UI / OSM 表示 / ログ / 外部連携で使用する。

`localization_fusion` 実装後の本来的な構成では、`geo_pose_converter_node` は GNSS のセンサ系 topic を購読して `localization_fusion` への入力となる `/gnss/pose_enu` を publish する。`route_geo_projector_node` は ENU 座標系の topic を購読し、LLH 座標系の表示用 topic へ変換して publish する。具体的には、`localization_fusion` が publish する `/localization/pose_enu` を `/localization/pose_llh` に変換し、GUI 表示に活用する。

Phase 2 では旧 `/amcl_pose` topic 名を廃止し、走行系自己位置 topic を `/localization/pose_enu` に統一する。`localization_fusion` 実装前の当面は、GNSS ENU 変換または simulation が `/localization/pose_enu` を publish し、走行系と表示用 projector が同じ ENU 自己位置を購読する。

## 2. 背景・要求・スコープ

GUI と HTML 遠隔観測 UI では、GPS 受信状態、自己位置 LLH、route LLH、active target LLH を一貫して表示する必要がある。一方、走行制御系は `map` frame の ENU 座標で統一する必要があり、`route_follower` と `robot_navigator` は `/active_route`、`/active_target`、走行系自己位置 topic を ENU pose として扱う。

本パッケージは、LLH と ENU の変換、および表示用 LLH topic の生成を担う。本パッケージは走行制御指令を生成しない。`route_follower`、`robot_navigator`、`ypspur` 系 node の制御 topic は変更しない。また、複数センサを統合して走行系へ渡す最終自己位置を決める責務は持たない。融合後 ENU 自己位置の生成は将来の `localization_fusion` が担い、`geo_pose_converter` はその前段または周辺の座標変換 adapter として動作する。

### 2.1 自己位置 topic の役割

| 用途 | topic | 型 | 座標系 | 位置づけ |
| --- | --- | --- | --- | --- |
| 走行制御用自己位置 | `/localization/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | `map` ENU | 走行系が購読する ENU 自己位置 topic。Phase 2 で旧 `/amcl_pose` から移行済み |
| 表示用 LLH 自己位置 | `/localization/pose_llh` | `tc_geo_msgs/GeoPoseWithQuality` | WGS84 LLH | GUI、HTML UI、OSM 表示、ログ用。走行制御入力ではない |
| GNSS 単独 LLH | `/gnss/pose_llh` | `tc_geo_msgs/GeoPoseWithQuality` | WGS84 LLH | GNSS 単独解、GPS 受信状態、診断、fallback 用 |
| GNSS 単独 ENU | `/gnss/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | `map` ENU | `localization_fusion` への GNSS absolute pose 入力 |

`/localization/pose_enu` は AMCL 由来名を含まない正式な ENU 自己位置 topic である。走行制御はこの topic を正とし、LLH 表示は `/localization/pose_llh` などの表示系 topic へ分離する。

### 2.2 phase ごとの位置づけ

| フェーズ | GNSS センサ系入力あり | GNSS センサ系入力なし | `route_geo_projector_node` の自己位置 ENU 入力 | 備考 |
| --- | --- | --- | --- | --- |
| localization_fusion 実装前（Phase 2 以降） | `geo_pose_converter_node` は GNSS を ENU に変換し、必要に応じて `/localization/pose_enu` として publish する | `geo_pose_converter_node` は起動しない。simulation または別 localizer が `/localization/pose_enu` を publish する | `/localization/pose_enu` を購読し、表示用 LLH へ逆変換する | 旧 `/amcl_pose` は使用しない。走行系も `/localization/pose_enu` を使う |
| localization_fusion 実装後 | `geo_pose_converter_node` は GNSS を ENU に変換し、`/gnss/pose_enu` として publish する。`localization_fusion` が `/localization/pose_enu` を publish する | simulation / replay / localization_fusion が `/localization/pose_enu` を publish する | `/localization/pose_enu` を購読し、`/localization/pose_llh` を publish する | GNSS 単独値は fusion 入力または診断用。走行系は `/localization/pose_enu` を使う |

したがって、`geo_pose_converter_node` が直接走行系自己位置を publish するのは `localization_fusion` 実装前の暫定構成である。`localization_fusion` 実装後は、`geo_pose_converter_node` は `/gnss/pose_enu` を publish し、融合後の `/localization/pose_enu` は `localization_fusion` が publish する。

### 2.3 他パッケージとの役割分担

| パッケージ | 責務 | `geo_pose_converter` との関係 |
| --- | --- | --- |
| `rtk_gps_um982` | GNSS receiver から raw fix、heading、RTK status を取得して ROS topic 化する | `geo_pose_converter_node` の入力元。測位デバイス固有の処理はここに閉じる |
| `tc_geo_msgs` | LLH point / pose / quality / projection の共通 message を提供する | `geo_pose_converter` が publish / subscribe する地理系 message 定義 |
| `tc_route_msgs` | route、waypoint、active target LLH など route 系 interface を提供する | `route_geo_projector_node` が `/active_route` と `/route/active_target_llh` で利用する |
| `route_planner` | CSV/YAML から route 正本を生成し、LLH-only CSV から ENU pose を生成して `Route` に格納する | `geo_core.py` をライブラリ import し、`Route.projection` と `Waypoint.geo_pose` を生成する |
| `route_manager` | route の受理、再配信、SHIFT/SKIP/再計画結果の管理を行う | `/active_route` の publisher。LLH/projection field を保持して再配信する |
| `route_follower` | `/active_route` の ENU waypoint と ENU 自己位置を使い、走行制御用 `/active_target` を publish する | `route_geo_projector_node` に active target ENU を提供する。LLH は制御に使わない |
| `robot_navigator` | ENU 自己位置と `/active_target` の ENU pose から自律速度指令を生成する | `geo_pose_converter` とは直接接続しない。LLH topic は速度制御に使わない |
| `localization_fusion` | GNSS ENU、LiDAR odometry、wheel odometry などを統合し、融合後 ENU 自己位置を publish する | `/gnss/pose_enu` を入力として使い、`/localization/pose_enu` を publish する |
| `robot_console` / HTML UI | 状態監視、GPS状態表示、route/target 可視化を行う | `/localization/pose_llh`、`/gnss/pose_llh`、`/active_route`、`/route/active_target_llh` を購読する利用側 |

## 3. 全体構成・アーキテクチャ

```text
src/geo_pose_converter/
├── geo_pose_converter/
│   ├── geo_core.py
│   ├── message_utils.py
│   ├── geo_pose_converter_node.py
│   └── route_geo_projector_node.py
├── launch/
│   └── geo_pose_converter.launch.py
├── params/
│   └── default.yaml
├── tests/
│   └── test_geo_core.py
└── docs/
    └── geo_pose_converter_詳細設計書.md
```

`geo_core.py` は ROS 非依存の座標変換ロジックを持つ。ROS message 生成は `message_utils.py` に寄せる。各 node は publisher / subscriber / parameter 管理に責務を限定する。`geo_core.py` は install 後に他パッケージから通常の Python ライブラリとして import できる公開ユーティリティでもあり、`route_planner` は LLH-only route CSV から ENU pose を生成するためにこれを利用する。

### 3.1 パッケージ内責務分担

| 構成要素 | 責務 | 非責務 |
| --- | --- | --- |
| `geo_core.py` | WGS84 LLH、ECEF、map ENU の相互変換、heading/yaw 変換、bearing 算出 | ROS topic、message、parameter、logger を扱わない |
| `message_utils.py` | `geo_core.py` の値を `tc_geo_msgs` / `tc_route_msgs` / `geometry_msgs` に詰める | 座標変換式そのもの、topic publish 判断を持たない |
| `geo_pose_converter_node.py` | GNSS raw topic を GNSS 単独 LLH / ENU pose に正規化し、投影条件を publish する | fusion 済み ENU 自己位置の採用判断、route target 生成、走行制御を行わない |
| `route_geo_projector_node.py` | ENU pose topic を LLH pose topic へ変換し、route / target の表示用 LLH 派生情報を生成する | route 生成、route 再計画、follower 制御、自己位置 fusion、走行系自己位置 publish を行わない |

### 3.2 ノード間の責務境界

`geo_pose_converter_node` は GNSS 観測を地理系共通表現へ変換する adapter である。入力は GNSS driver topic に限定し、GNSS 単独の LLH pose と ENU pose を生成する。`localization_fusion` 実装後の本来的な出力は `/gnss/pose_llh` と `/gnss/pose_enu` であり、`/gnss/pose_enu` は `localization_fusion` の入力となる。

`localization_fusion` 実装前は、GNSS ENU pose を走行系へ直接渡す暫定構成を許容する。この場合の出力 topic は `/localization/pose_enu` とし、旧 `/amcl_pose` への remap は行わない。

`route_geo_projector_node` は ENU pose を LLH pose へ変換する projector である。本来的には `/localization/pose_enu` を購読して `/localization/pose_llh` を publish する。また、`/active_route` の `Waypoint.geo_pose` と `/active_target` の ENU pose から `/route/active_target_llh` を生成する。自己位置は距離・方位表示のためだけに使い、走行制御や route 更新判断には使わない。

`route_geo_projector_node` と `route_planner_node` の直接の起動順依存は持たせない。`route_planner_node` は `/get_route` service provider であり、projector が実際に比較・利用する正本は `route_manager` が publish する `/active_route.projection` である。`/active_route` は Transient Local QoS で配信されるため、projector が後から起動した場合も最新 route を受信できる。projector が先に起動して `/localization/pose_enu` を先に受信した場合は、route 到着までは起動時 parameter の projection で `/localization/pose_llh` を生成するため、統合 launch では `route_planner` と `geo_pose_converter` 系 node へ同じ `projection_config_path` を渡す必要がある。

## 4. パッケージ構成・ファイル配置

| ファイル | 役割 |
| --- | --- |
| `geo_core.py` | WGS84 ECEF、LLH/ENU、heading/yaw 変換、ROS 2 parameter YAML からの `ProjectionConfig` 読み込み |
| `message_utils.py` | `tc_geo_msgs` と `tc_route_msgs` の message 生成補助 |
| `geo_pose_converter_node.py` | GNSS raw topic から GNSS 単独 LLH / ENU pose を生成し、投影条件を publish する |
| `route_geo_projector_node.py` | ENU pose topic を LLH pose topic へ変換し、route / active target の LLH 派生情報を生成する |
| `params/default.yaml` | 投影原点、frame、topic の既定値 |
| `launch/geo_pose_converter.launch.py` | ノード起動用 launch。phase ごとの topic remap を明示する |

## 5. 外部インタフェース仕様

### 5.1 `geo_pose_converter_node`

この node は GNSS driver 依存の topic を、プロジェクト共通の地理 pose 表現へ変換する。`/gnss/pose_llh` と `/gnss/pose_enu` は GNSS 単独値であり、`localization_fusion` 実装後の融合後自己位置ではない。GUI は GPS 受信状態の詳細表示には `/gnss/pose_llh` を参照してよいが、通常の自己位置表示では `/localization/pose_llh` を優先する。

| 種別 | 論理名 | 型 | QoS | 意味 |
| --- | --- | --- | --- | --- |
| Subscribe | `/rtk_gps/fix` | `sensor_msgs/NavSatFix` | `RELIABLE / depth=10` | GNSS LLH 位置 |
| Subscribe | `/rtk_gps/heading` | `sensor_msgs/Imu` | `RELIABLE / depth=10` | GNSS heading raw。初期実装では保持のみ |
| Subscribe | `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/RtkStatus` | `RELIABLE / depth=10` | RTK 品質、heading、衛星数 |
| Publish | `/gnss/pose_llh` | `tc_geo_msgs/GeoPoseWithQuality` | `RELIABLE / depth=10` | GNSS 単独 LLH。GPS 受信状態、診断、fallback 用 |
| Publish | `/gnss/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | `RELIABLE / depth=10` | GNSS LLH を map ENU に投影した pose。`localization_fusion` 入力 |
| Publish | `/geo/map_projection` | `tc_geo_msgs/MapProjection` | `RELIABLE / depth=10` | 投影条件 |

`localization_fusion` 実装前の暫定運用では、`/gnss/pose_enu` 相当の出力を launch remap により `/localization/pose_enu` として publish してよい。ただし、これは `localization_fusion` なしで統合動作確認を進めるための暫定構成であり、本来的な GNSS 単独 ENU topic 名は `/gnss/pose_enu` である。

### 5.2 `route_geo_projector_node`

この node は ENU pose topic を LLH pose topic へ変換する。自己位置表示では `/localization/pose_enu` を `/localization/pose_llh` へ変換する。route / active target 表示では `/active_route` と `/active_target` から `/route/active_target_llh` を生成する。`/route/active_target_llh` は GUI、HTML UI、ログ用であり、走行制御に使う `/active_target` を置き換えない。

| 種別 | 論理名 | 型 | QoS | 意味 |
| --- | --- | --- | --- | --- |
| Subscribe | `/localization/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | `RELIABLE / depth=10` | 表示用 LLH へ変換する ENU 自己位置 |
| Publish | `/localization/pose_llh` | `tc_geo_msgs/GeoPoseWithQuality` | `RELIABLE / depth=10` | GUI / HTML UI / OSM 表示用の LLH 自己位置 |
| Subscribe | `/active_route` | `tc_route_msgs/Route` | `RELIABLE / TRANSIENT_LOCAL / depth=1` | route 正本。ENU pose と LLH pose を含む |
| Subscribe | `/active_target` | `geometry_msgs/PoseStamped` | `RELIABLE / depth=10` | 走行制御用 active target ENU |
| Subscribe | `/follower_state` | `tc_route_msgs/FollowerState` | `RELIABLE / depth=10` | active waypoint index / label |
| Publish | `/route/active_target_llh` | `tc_route_msgs/ActiveTargetLlh` | `RELIABLE / depth=10` | 表示・ログ用 active target LLH |

初期実装や診断用途では、`/gnss/pose_llh` を fallback として利用してよい。ただし本来的な変換経路は ENU pose topic から LLH pose topic への変換である。

### 5.3 phase ごとの topic remap

| フェーズ | `geo_pose_converter_node` の ENU 出力 | `route_geo_projector_node` の ENU 入力 | 走行系自己位置入力 |
| --- | --- | --- | --- |
| localization_fusion 実装前（Phase 2 以降） | `/localization/pose_enu` | `/localization/pose_enu` | `/localization/pose_enu` |
| localization_fusion 実装後 | `/gnss/pose_enu` | `/localization/pose_enu` | `/localization/pose_enu` |

GNSS センサ系 topic の入力がない simulation 統合動作確認では、`geo_pose_converter_node` は起動しない。simulation または別の localizer が `/localization/pose_enu` を publish し、`route_geo_projector_node` は同 topic を購読して LLH 表示 topic を生成する。

## 6. パラメータ・設定仕様

| Parameter | 型 | 既定値 | 用途 |
| --- | --- | --- | --- |
| `projection_id` | string | `tokyo_station` | 投影条件識別子。既定値は開発チーム内の共通仕様に合わせた東京駅原点を表す |
| `datum` | string | `WGS84` | 測地系 |
| `map_frame_id` | string | `map` | ENU pose frame |
| `earth_frame_id` | string | `earth` | LLH pose frame |
| `child_frame_id` | string | `gps_link` | GNSS pose の child frame。`geo_pose_converter_node` のみで使う |
| `origin_latitude` | double | `35.681382` | LLH/ENU 変換原点の緯度。既定値は東京駅 |
| `origin_longitude` | double | `139.766084` | LLH/ENU 変換原点の経度。既定値は東京駅 |
| `origin_altitude` | double | `3.86` | LLH/ENU 変換原点の高さ [m]。既定値は東京駅付近の標高 |
| `map_yaw_offset_rad` | double | `0.0` | ENU 軸から map 軸への回転 |
| `pose_enu_topic` | string | `/localization/pose_enu` | `route_geo_projector_node` が LLH へ変換する ENU 自己位置 topic |
| `pose_llh_topic` | string | `/localization/pose_llh` | `route_geo_projector_node` が publish する LLH 自己位置 topic |

`origin_latitude`、`origin_longitude`、`origin_altitude` は ENU 座標と LLH 座標を相互変換するための投影原点である。`geo_pose_converter_node` と `route_geo_projector_node` は同じ map frame 上の ENU 座標を扱うため、同一の `ProjectionConfig` を使わなければならない。`params/default.yaml` では `/**` の ROS 2 wildcard parameter に投影条件を定義し、両 node が同じ値を受け取る構成にする。node 別の `origin_*` 定義は持たせない。`route_geo_projector` には ROS 2 parameter file の target node として認識させるために空の `ros__parameters` のみを置く。launch や運用用 YAML で上書きする場合も、投影条件は共通定義として 1 箇所で管理する。

将来追加する統合 launch / bringup package では、`projection_config_path` を top-level launch 引数として 1 つだけ受け取り、`route_planner`、`geo_pose_converter_node`、`route_geo_projector_node` へ同じ YAML を渡す。個別 node の launch 引数や profile metadata に `origin_*` を分散定義してはならない。

## 7. データモデル・内部状態

`ProjectionConfig` は ROS 非依存 dataclass として投影条件を保持する。`LlhPoint` は WGS84 LLH、`EnuPoint` は map frame 上の ENU 相当座標を表す。`ProjectionConfig` は `geo_pose_converter_node`、`route_geo_projector_node`、route 生成側で同一値を使う前提であり、値がずれると GNSS ENU、走行系 ENU、表示 LLH、active target LLH が互いに整合しない。

`geo_pose_converter_node` は最新の `RtkStatus` と `Imu` を保持する。heading は `RtkStatus` を受信していれば有効とし、`heading_deg=0.0` を真北として有効扱いする。

`route_geo_projector_node` は最新の ENU 自己位置、route、active target、follower state を保持する。ENU 自己位置は `/localization/pose_enu` を購読する。LLH 自己位置は保持した ENU 自己位置を `ProjectionConfig` で逆変換して生成する。

## 8. 処理フロー・状態遷移

### 8.1 GNSS ENU pose 生成

1. `NavSatFix` を受信する。
2. latest `RtkStatus` から heading、fix quality、衛星数、HDOP を取得する。
3. GNSS 単独値として `/gnss/pose_llh` を publish する。
4. 同じ LLH を `ProjectionConfig` で map ENU へ変換し、GNSS ENU pose を publish する。
5. `localization_fusion` 実装後は GNSS ENU pose を `/gnss/pose_enu` として publish し、`localization_fusion` の入力とする。
6. `localization_fusion` 実装前は暫定的に、GNSS ENU pose を `/localization/pose_enu` として publish し、走行系や projector の入力とする。

### 8.2 自己位置 LLH 生成

1. `route_geo_projector_node` が ENU 自己位置 topic を受信する。
2. 受信した ENU pose の `header.frame_id` と有効 projection の `map_frame_id` を比較し、不一致なら warning を出す。
3. 受信した ENU pose を `ProjectionConfig` で LLH へ逆変換する。
4. 変換結果を `/localization/pose_llh` として publish する。
5. `/localization/pose_llh` は GUI / HTML UI / OSM 表示 / ログ用であり、走行制御へは渡さない。

### 8.3 Active target LLH 生成

1. `/active_route` を受信し、projection と waypoint LLH を保持する。
2. `/active_route.projection` と起動時 parameter から得た projection を比較する。`projection_id`、datum、frame、origin、yaw offset のいずれかが不一致なら error ログを出す。変換には `Route.projection` を優先して使う。
3. `/follower_state` から active waypoint index / label を保持する。
4. `/active_target` 更新時、`header.frame_id` と有効 projection の `map_frame_id` を比較し、不一致なら warning を出す。
5. route waypoint LLH があれば target LLH として採用する。
6. route waypoint LLH がなければ `/active_target` ENU pose を projection で LLH へ変換する。
7. ENU 自己位置から生成した LLH 自己位置を用いて距離・方位を計算し、`/route/active_target_llh` を publish する。

## 9. 主要アルゴリズム・判定ロジック

LLH/ENU 変換は WGS84 楕円体から ECEF へ変換し、原点 ECEF との差分を East/North/Up へ射影する。`map_yaw_offset_rad` により ENU east/north 軸と map x/y 軸の回転差を扱う。本プロジェクトの走行用 ENU pose は2D座標として扱い、publishする `Pose.position.z` は原則 `0.0` とする。GNSS由来の高度はLLH系topicで保持し、ENU逆投影だけで生成したLLH poseは `GeoPoint.has_altitude=false` とする。

heading は真北 0 度・時計回り正、ENU yaw は東 0 rad・反時計回り正とする。変換式は `heading = 90deg - yaw_enu` を正規化したものである。

## 10. QoS・並行性・タイミング設計

初期実装は single-thread executor 前提で排他制御を持たない。`/active_route` は transient local、それ以外は volatile stream とする。`/geo/map_projection` は 1 Hz で再 publish する。

## 11. 起動・終了・launch 設計

`geo_pose_converter.launch.py` は `geo_pose_converter_node` と `route_geo_projector_node` を起動し、`params/default.yaml` を読み込む。Phase 2 以降の launch 既定値は `gnss_pose_enu_topic=/localization/pose_enu`、`pose_enu_topic=/localization/pose_enu` とし、`localization_fusion` 実装前の統合確認で走行系と projector が同じ ENU 自己位置を使える構成にする。`localization_fusion` 実装後は `gnss_pose_enu_topic=/gnss/pose_enu`、`pose_enu_topic=/localization/pose_enu` とする。

`enable_llh_osm_viewer` launch 引数を `true` にした場合は、診断用の `llh_osm_viewer_node` も同時に起動する。viewer は `pose_llh_topic`、`/active_route`、`active_target_llh_topic` を購読し、HTTPビューアを提供する。HTTP host、port、ブラウザ自動起動は `llh_osm_viewer_host`、`llh_osm_viewer_port`、`llh_osm_viewer_open_browser` で指定する。viewerは統合確認用であり、正式な `robot_console` HTML遠隔観測UIを置き換えない。

GNSS センサ系 topic の入力がない simulation 統合動作確認では、`enable_geo_pose_converter=false` として `geo_pose_converter_node` を起動しない。simulation または別 localizer が `/localization/pose_enu` を publish し、`route_geo_projector_node` も同 topic を購読する。

`localization_fusion` 実装後は、実機・シミュレーションの統合 launch で `geo_pose_converter_node`、`localization_fusion`、`route_geo_projector_node` の接続を明示する。走行系は `/localization/pose_enu` を購読する。

## 12. エラー処理・ログ・診断

`params/default.yaml` には東京駅原点の具体値を設定するため、既定設定でも LLH/ENU 変換は定義済みである。ただし、実コースや実 map が東京駅原点以外で作られている場合は、launch または運用用 parameter YAML の共通定義で投影原点を上書きする必要がある。route が未受信、または active target が未受信の場合、`route_geo_projector_node` は active target LLH を publish しない。

`route_geo_projector_node` は `/active_route` 受信時に、起動時 parameter 由来の projection と `Route.projection` を比較する。これは `route_planner` が LLH-only route CSV から ENU pose を生成した条件と、projector が ENU pose を LLH へ戻す条件が一致しているかを検出するためである。不一致時は error ログを出す。ただし、route 表示と active target LLH 生成では `/active_route` に埋め込まれた `Route.projection` を優先する。

ENU 自己位置 topic が未受信の場合、`route_geo_projector_node` は `/localization/pose_llh` を publish しない。初期実装や診断用途で `/gnss/pose_llh` を fallback 表示する場合でも、それは GUI/ログ表示のための補助であり、走行系自己位置の採用判断ではない。

## 13. UI・可視化仕様

GUI は通常、自己位置表示には `/localization/pose_llh` を使用する。GPS 受信状況の詳細表示には `/gnss/pose_llh` または `/rtk_gps/rtk_status` を併用する。HTML UI は `/active_route` と `/route/active_target_llh` を利用して route と active target を地図上に描画する。

走行系の `/localization/pose_enu` は GUI 上では ENU 自己位置のデバッグ表示には使えるが、OSM 上の通常表示では `/localization/pose_llh` を使う。

`llh_osm_viewer_node` は `geo_pose_converter` の診断・統合確認用可視化ノードである。表示対象は以下とする。

- `/localization/pose_llh`: 赤い二等辺三角形で自己位置とheadingを表示する。
- `/active_route`: `Waypoint.has_geo_pose=true` の waypoint を青いroute polylineと点で表示する。LLHを持たないwaypointは表示対象外として数を `skipped_waypoints` に含める。
- `/route/active_target_llh`: 橙色の点でactive targetを表示する。
- `/state`: HTTP JSON APIとして、自己位置、route、active target、`pose_status` を返す。
- `/pose`: 後方互換の簡易HTTP JSON APIとして、自己位置のみを返す。

`pose_status` は最後に `/localization/pose_llh` を受信した壁時計時刻からの経過時間で `OK` / `STALE` / `LOST` / `NO_DATA` を判定する。通信状態による地図マーカー色変更は行わず、状態表示欄でのみ示す。地図描画は Leaflet CDN と OpenStreetMap 外部タイルを利用し、ネットワーク接続がある運用を前提とする。オフラインタイルキャッシュや正式な遠隔観測UIのSnapshot APIは `robot_console` 側の将来実装範囲とする。

## 14. 依存関係・ビルド設定

`ament_python` package とする。依存は `rclpy`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`tc_geo_msgs`、`tc_route_msgs`、`rtk_gps_um982_msgs`、`launch`、`launch_ros`、`ament_index_python` である。

## 15. テスト計画・受け入れ条件

- `geo_core.py` の LLH/ENU 往復変換、heading/yaw 変換、bearing 計算の pytest が成功する。
- `colcon build --packages-select geo_pose_converter` が成功する。
- GNSS 入力あり、localization_fusion 実装前の暫定運用では、`geo_pose_converter_node` の ENU 出力を `/localization/pose_enu` として publish し、走行系と `route_geo_projector_node` が同じ ENU pose を購読できることを確認する。
- GNSS 入力なし simulation では、`geo_pose_converter_node` を起動せず、simulation または別 localizer が publish する ENU 自己位置 topic から `route_geo_projector_node` が `/localization/pose_llh` を生成できることを確認する。
- `localization_fusion` 実装後は、`geo_pose_converter_node` が `/gnss/pose_enu` を publish し、`localization_fusion` が `/localization/pose_enu` を publish し、`route_geo_projector_node` が `/localization/pose_llh` を publish できることを確認する。
- `params/default.yaml` で投影条件が `/**` の共通 parameter として 1 箇所に定義され、node 別に異なる `origin_*` を設定できる構造になっていないことを確認する。
- `route_planner` が `geo_pose_converter.geo_core` を install 後に import し、LLH-only route CSV から ENU pose を生成できることを確認する。
- `/active_route` に LLH が含まれる route で `/route/active_target_llh` の緯度経度が waypoint LLH と一致することを確認する。
- `llh_osm_viewer_node` のJSON生成テストで、pose、route、active target、`pose_status` が期待通り生成されることを確認する。
- `enable_llh_osm_viewer:=true` を指定した launch で viewer が起動対象に含まれることを確認する。
- 自律走行確認では `/active_target` と route follower の ENU 制御が変更前と同等に動作することを確認する。

## 16. 互換性・移行・影響範囲

Phase 2 で走行系の現在自己位置入力は `/localization/pose_enu` へ移行済みであり、旧 `/amcl_pose` topic 名は使用しない。`/localization/pose_enu` は ENU 走行制御用 topic として維持し、LLH 表示 topic とは分離する。

`/localization/pose_llh` は GUI / HTML UI / OSM 表示用 topic であり、走行系の ENU 自己位置 topic を置き換えない。`/gnss/pose_llh` は GPS 受信状態、GNSS 単独解、fusion 入力の妥当性確認に使う。

移行は以下の順で行う。

1. Phase 2 で、走行系、simulation、`route_geo_projector_node` の ENU 自己位置 topic を `/localization/pose_enu` へ統一する。
2. localization_fusion 実装前は、GNSS 入力がある場合に `geo_pose_converter_node` の ENU 出力を暫定的に `/localization/pose_enu` として使う。GNSS 入力がない simulation では `geo_pose_converter_node` を起動せず、simulation 側の `/localization/pose_enu` を使う。
3. `localization_fusion` 実装後は、`geo_pose_converter_node` が `/gnss/pose_enu` を publish し、`localization_fusion` が `/localization/pose_enu` を publish する。
4. `route_geo_projector_node` は `/localization/pose_enu` を `/localization/pose_llh` へ変換し、GUI / HTML UI / OSM 表示へ提供する。

## 17. 未決事項・今後の拡張

- `localization_fusion` の `/localization/pose_enu`、`/localization/pose_llh` の publish 責務と同期条件を詳細設計で確定する。
- `RtkStatus` と `NavSatFix` の timestamp 差分許容値を parameter 化する。
- HTML UI backend の topic bridge 仕様を別途定義する。

## 18. 改版履歴

| 日付 | 版 | 変更概要 |
| --- | --- | --- |
| 2026-06-03 | 1.8 | `llh_osm_viewer_node` の route/active target overlay、stale表示、launch引数、正式HTML UIとの差分を追記 |
| 2026-06-02 | 1.7 | `route_geo_projector_node` の projection mismatch 検出、起動順、統合 launch での `projection_config_path` 一本化方針を追記 |
| 2026-06-01 | 1.6 | `geo_core.py` を route_planner から参照する共通変換ライブラリとして明記し、projection YAML 読み込み責務を追加 |
| 2026-05-29 | 1.5 | Phase 2 として旧 `/amcl_pose` を廃止し、走行系自己位置 topic を `/localization/pose_enu` へ統一 |
| 2026-05-29 | 1.4 | LLH/ENU 変換原点を `/**` の共通 parameter として一本化し、東京駅原点の既定値を定義 |
| 2026-05-29 | 1.3 | `geo_pose_converter_node` と `route_geo_projector_node` の本来的役割、localization_fusion 前後の topic remap、旧 `/amcl_pose` 廃止前提の移行手順を整理 |
| 2026-05-29 | 1.2 | 走行系 ENU と表示系 LLH の自己位置 topic を分離し、旧 `/amcl_pose` 互換 alias と `/localization/pose_enu` 移行方針を明記 |
| 2026-05-29 | 1.1 | `localization_fusion` 実装後の正本関係、他パッケージとの役割分担、2 ノードの責務境界を追記 |
| 2026-05-28 | 1.0 | 初版。GNSS pose 変換と active target LLH 生成の詳細設計を定義 |
