# MID-360 と topURG から作った水平スライス各層の観測結果。
#
# 配信様式: stream (10 Hz, lidar_layer_slicer の publish_rate_hz)
# 配信元: lidar_layer_slicer
# 備考: 前方閉塞判定には使わない。停止判定は obstacle_monitor が topURG のみから行う。
#       本メッセージは AI 助言の入力と回避幅の上限算出に使う観測値である。
#
# header.stamp は層の生成に使った最新フレームの観測時刻、frame_id は base_link とする。
std_msgs/Header header

ObstacleLayer[] layers

float32 forward_range_m    # 各層の評価に使った前方距離の上限 [m]
float32 robot_width_m      # 空き幅の判定に使った車体全幅 [m]
float32 side_margin_m      # 車体幅へ加えた左右の余裕 [m]
int32 accumulated_frames   # 点群層の生成に使った累積フレーム数
float32 accumulated_span_s # 累積した最古フレームから最新フレームまでの時間 [s]
