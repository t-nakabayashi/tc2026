# route_manager 観点のルート状態の概要（監視・UI向け）
#
# 配信様式: stream (1 Hz, route_manager の state_publish_rate_hz)
# 配信元: route_manager
# 備考: 状態変化時は即時配信し、変化が無い間も上記レートで再送する。

std_msgs/Header header
int32 route_version
int32 total_waypoints
int32 current_index
string current_label

uint8 STATUS_UNKNOWN=0
uint8 STATUS_IDLE=1
uint8 STATUS_RUNNING=2
uint8 STATUS_UPDATING_ROUTE=3
uint8 STATUS_HOLDING=4
uint8 STATUS_COMPLETED=5
uint8 STATUS_ERROR=6

uint8 status
string message



