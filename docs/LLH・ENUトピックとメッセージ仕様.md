# LLH・ENUトピックとメッセージ仕様

## 座標の使い分け

走行制御はmap上の2D ENU姿勢、地図表示・経路保存はWGS84 LLHを使う。
LLH高度の有効性は `GeoPoint.has_altitude` で表し、ENUのz=0から実高度を復元したとは扱わない。
headingは真北0°・時計回り、ENU yawは東0 rad・反時計回り。
mapに回転を与える場合は `map_yaw_offset_rad` を位置と方位の双方へ適用する。

## トピックと配信責務

| トピック | 型 | 共通構成の配信元・用途 |
| --- | --- | --- |
| `/gnss/pose_llh` | tc_geo_msgs/GeoPoseWithQuality | geo_pose_converter、GNSS単独診断 |
| `/gnss/pose_enu` | geometry_msgs/PoseWithCovarianceStamped | geo_pose_converter、GNSS単独比較 |
| `/localization/pose_enu` | geometry_msgs/PoseWithCovarianceStamped | gnss_lio_fusion、走行用車軸姿勢 |
| `/localization/pose_llh` | tc_geo_msgs/GeoPoseWithQuality | route_geo_projector、融合位置の地図表示 |
| `/geo/map_projection` | tc_geo_msgs/MapProjection | geo_pose_converter、投影条件 |
| `/active_route` | tc_route_msgs/Route | route_manager、経路の正本 |
| `/active_target` | geometry_msgs/PoseStamped | route_follower、制御目標 |
| `/route/active_target_llh` | tc_route_msgs/ActiveTargetLlh | route_geo_projector、目標の表示 |

融合へのGNSS入力はNavSatFix/RtkStatusであり、`/gnss/pose_enu`を入力にしない。
単体geo_pose_converter.launchはGNSS単独ENUを `/localization/pose_enu` へ出す既定である。
融合と併用する場合は `gnss_pose_enu_topic:=/gnss/pose_enu` を指定して競合を避ける。

## 経路の正本と投影条件

実コースCSVはlatitude・longitude・heading_degを正本とし、altitudeは空欄を標準とする。
route_plannerが共通projectionからWaypoint.poseを生成し、Route.projectionへ条件を格納する。
LLHとENUを併記した行はLLHを優先し、ENUとの差を検査する。
詳しい列・閾値は[route_planner](../src/route_planner/README.md)を参照する。

route_geo_projectorはRoute.projectionを受信したらそれを優先する。
起動時設定との不一致はログへ出すが、設定を自動で修復しない。
ENU逆変換で生成したLLHは `has_altitude=false` とする。

## 定義と確認

型の正本は[tc_geo_msgs](../src/tc_geo_msgs/docs/tc_geo_msgs_詳細設計書.md)、
[tc_route_msgs](../src/tc_route_msgs/README.md)のmsg/srv定義である。
QoS、ノードの入出力と起動方法は[geo_pose_converter](../src/geo_pose_converter/docs/geo_pose_converter_詳細設計書.md)を参照する。
frame名だけで原点一致を判断せず、projection ID・原点・datum・map回転を照合する。
GNSS単独値と融合値、アンテナ位置と車軸位置を精度評価で混同しない。
