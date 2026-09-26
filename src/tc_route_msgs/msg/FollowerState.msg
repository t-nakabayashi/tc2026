# tc_route_msgs/msg/FollowerState.msg
# route_follower ノードが現在の追従状態を通知する。
# Phase2にて拡張された最終仕様。
#
# 配信様式: stream (20 Hz, route_follower の control_rate_hz)
# 配信元: route_follower

std_msgs/Header header

int32 route_version
string state
int32 active_waypoint_index
string active_waypoint_label

float32 active_target_distance_m
float32 segment_length_m

int32 avoidance_attempt_count
string last_stagnation_reason

bool front_blocked
float32 front_clearance_m
float32 left_offset_m
float32 right_offset_m
