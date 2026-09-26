# tc_route_msgs/msg/ObstacleAvoidanceHint.msg
# obstacle_monitor が出力する障害物検知ヒント
#
# 配信様式: stream (40 Hz, 上流 /scan の配信レートに追随)
# 配信元: obstacle_monitor
# 備考: robot_console からの手動 override も同じ topic へ配信する。

std_msgs/Header header

bool front_blocked
float32 front_clearance_m
float32 left_offset_m
float32 right_offset_m
