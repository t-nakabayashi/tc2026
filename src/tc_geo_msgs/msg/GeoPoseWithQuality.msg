# Geographic pose with GNSS/fusion quality for monitoring and logging.
#
# 配信様式: stream (10 Hz, 上流の GNSS 出力レートに追随)
# 配信元: geo_pose_converter (geo_pose_converter_node / route_geo_projector_node)

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
