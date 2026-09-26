# UM982 受信機の測位品質と補正状態。
#
# 配信様式: stream (10 Hz, rtk_gps_um982 の output_rate_hz)
# 配信元: rtk_gps_um982 (driver_node)
# QoS 例外: RELIABLE / depth 10。driver 実装の既定であり、後段はこれに合わせる。

std_msgs/Header header

# RTK state (numeric form for C++ consumers + raw string from library)
uint8 STATE_UNKNOWN    = 0
uint8 STATE_STANDALONE = 1
uint8 STATE_DGPS       = 2
uint8 STATE_RTK_FLOAT  = 3
uint8 STATE_RTK_FIX    = 4
uint8 rtk_state
string rtk_state_raw

# 衛星・精度
uint8   num_satellites
float32 hdop

# Dual-antenna heading (deg, 真北基準 CW)
float32 heading_deg
float32 heading_stddev_deg
float32 pitch_deg
float32 pitch_stddev_deg

# Baseline
float32 baseline_length_m

# RTCM 補正
float32 correction_age_s
uint32  rtcm_bytes_received

# 参照: 生 fix の経度緯度高度
float64 latitude
float64 longitude
float32 altitude
