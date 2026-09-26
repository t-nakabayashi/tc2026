# 経路全体（配信の中核メッセージ）
# - header.frame_id は "map" を必須とする。
# - version は route_manager が配信するルートの世代管理用。
# - route_image は経路可視化の埋込画像（encoding: "bgr8" 推奨）。
# - projection は pose と geo_pose の対応関係を示す。
# - waypoints は index 順で格納する（0..N-1）。
#
# 配信様式: latch
# 配信元: route_manager
# 備考: 後発の購読者へ最新ルートを届けるため TRANSIENT_LOCAL で配信する。
#       更新が無い間は配信されないため、死活監視には使わない。

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
