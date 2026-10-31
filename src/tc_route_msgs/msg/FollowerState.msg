# tc_route_msgs/msg/FollowerState.msg
# route_follower ノードが現在の追従状態を通知する。
#
# 配信様式: stream (20 Hz, route_follower の control_rate_hz)
# 配信元: route_follower
#
# 本メッセージは追従状態の通知であり、車輪の実測速度ゼロを保証しない。
# 姿勢は /localization/pose_enu、停止属性は active_route の Waypoint から取得する。

std_msgs/Header header

# 進捗情報
int32 route_version              # ルートバージョン番号
string state                     # FollowerStatus の名称
int32 active_waypoint_index      # 現在追従しているウェイポイントのインデックス
string active_waypoint_label     # 現在追従しているウェイポイントのラベル

# 距離情報
float32 active_target_distance_m  # 現在地とアクティブターゲットのユークリッド距離 [m]
float32 segment_length_m          # 直前ウェイポイントから現在ウェイポイントまでの距離 [m]

# 回避・滞留
int32 avoidance_attempt_count    # 現在ウェイポイントでの回避試行回数
string last_stagnation_reason    # 直近の滞留理由ラベル

# 障害物ヒント
bool front_blocked               # ObstacleAvoidanceHint の多数決による前方遮蔽判定
float32 front_clearance_m        # ObstacleAvoidanceHint から得た前方余裕距離 [m]
float32 left_offset_m            # ObstacleAvoidanceHint サンプルの左オフセット中央値 [m]
float32 right_offset_m           # ObstacleAvoidanceHint サンプルの右オフセット中央値 [m]
