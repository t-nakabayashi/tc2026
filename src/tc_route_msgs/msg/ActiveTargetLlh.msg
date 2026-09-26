# Current active target in WGS84 LLH for GUI, HTML UI, and logging.
# /active_target remains the ENU control target.
#
# 配信様式: stream (20 Hz, 上流 /active_target の配信レートに追随)
# 配信元: geo_pose_converter (route_geo_projector_node)

std_msgs/Header header

int32 route_version
int32 target_index
string target_label

tc_geo_msgs/GeoPose target_pose

float32 distance_m
float32 bearing_deg

bool is_avoidance_subgoal
string target_kind
