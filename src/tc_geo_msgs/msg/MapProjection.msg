# ENU/LLH projection parameters shared by route and localization components.
#
# 配信様式: stream (1 Hz, /geo/map_projection)／埋め込み用 (Route.projection)
# 配信元: geo_pose_converter (geo_pose_converter_node)
# QoS 例外: RELIABLE / depth 10。同ノードの他 publisher と同じ QoS を共用する。
#

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

string utm_zone
bool utm_north
