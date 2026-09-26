# Waypoint（経路中の1点の「実体」）
#
# 配信様式: 埋め込み用（Route.waypoints の配列要素。単独では配信しない）
#
# - index: 当該 Route における連番（0起算）。同一labelでも再訪で複数エントリが存在し得る。
# - label: 地図上の地点識別子（文字列）。同一点を再訪しても同じlabelを用いる。
# - pose: 走行制御用の位置・姿勢（geometry_msgs/Pose）。yaw 専用フィールドは用いない。
# - geo_pose: 地図表示・ログ・route編集用のWGS84 LLH位置姿勢。
# - geo_pose_source: geo_pose の由来。route file由来か、ENUからの逆投影かを区別する。
# - right_open/left_open: 路側開放度など、走行判断に寄与するスカラー（拡張属性）。
# - line_stop/signal_stop/not_skip: 停止・スキップに関する制御フラグ。
# - segment_is_fixed: TrueならこのWaypointへ向かう直前区間が固定ルート由来であることを示す。

int32 index
string label
geometry_msgs/Pose pose
bool has_pose_enu

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
