# tc_route_msgs/msg/AvoidanceAdvice.msg
# AI エージェントから得た回避の助言。route_follower が回避サブゴール生成の入力に使う。
#
# 配信様式: event (助言の確定時のみ。要求が無い間は配信されない)
# 配信元: avoidance_advisor
# 備考: 死活監視には使わない。助言は提案値であり、採用時は route_follower 側で
#       waypoint の開放幅・実測空き幅・大域上限によりクランプする。
#       obstacle_class と rationale は記録・事後分析用で、制御の分岐には使わない。

std_msgs/Header header

# 推奨行動。UNKNOWN は助言が成立しなかったことを表し、従来回避へ退避する。
uint8 ACTION_UNKNOWN=0
uint8 ACTION_AVOID=1
uint8 ACTION_WAIT=2
uint8 action

# 回避方向。ACTION_AVOID で SIDE_NONE の場合は従来回避の左右選択に委ねる。
int8 SIDE_RIGHT=-1
int8 SIDE_NONE=0
int8 SIDE_LEFT=1
int8 side

float32 offset_m        # 助言された横オフセット [m]
float32 wait_sec        # ACTION_WAIT で待機する時間 [s]
bool is_moving          # 前方障害物が移動していると判断したか
float32 confidence      # 助言の確信度 [0.0, 1.0]

string obstacle_class   # 障害物の種類（情報項目）
string rationale        # 判断根拠（情報項目）
string request_id       # 要求と助言、保存した入力画像を対応付ける識別子
