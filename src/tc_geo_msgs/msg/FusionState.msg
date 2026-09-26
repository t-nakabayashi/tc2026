# GNSS・LIO 融合の状態。表示・記録に加えて制御判断の入力にもなる。
#
# 配信様式: stream (50 Hz 通常時 / 1 Hz 初期化待ち時, gnss_lio_fusion の tick)
# 配信元: gnss_lio_fusion (fusion_node)
# 備考: 融合内部の全診断量は /diagnostics と出力ログへ回し、本メッセージは
#       購読側が判断に用いる値だけを運ぶ。

std_msgs/Header header

# 融合モード。
# 水平基準の確定待ち。姿勢推定は未確定。
string MODE_WAIT_GRAVITY_ALIGNMENT=WAIT_GRAVITY_ALIGNMENT
# 初期 fix 待ち。姿勢推定は未確定。
string MODE_WAIT_INITIAL_FIX=WAIT_INITIAL_FIX
# GNSS と LIO の双方が有効。
string MODE_GPS_LIO=GPS_LIO
# GNSS が途絶し LIO のみで推定している。
string MODE_LIO_PRIORITY=LIO_PRIORITY
# LIO が異常。
string MODE_LIO_FAULT=LIO_FAULT
# GNSS と車輪オドメトリで推定している。
string MODE_GPS_WHEEL=GPS_WHEEL
# 車輪オドメトリのみで推定している。
string MODE_WHEEL_PRIORITY=WHEEL_PRIORITY
string mode

# 姿勢・不確かさが確定しているか。MODE_WAIT_* の間は false となり、
# yaw_rad / heading_sigma_deg / position_sigma_m は未定義とする。
bool has_estimate

# 推定方位 [rad]。ENU の East 基準、反時計回り正。
float64 yaw_rad

# 推定不確かさ。実誤差ではなくフィルタの推定値である。
float32 heading_sigma_deg
float32 position_sigma_m

# アンテナ基線長の監視値。
float32 baseline_reference_m
float32 baseline_sigma_m
bool baseline_ready

# 融合品質から決まる速度上限 [m/s]。下流の走行制御が参照する。
float32 speed_limit_mps

# 本推定が対象とした観測時刻 [s]。購読側が自身の観測と対応付けるために使う。
# シミュレーション時は sim time となるため、header.stamp とは一致しないことがある。
float64 estimate_stamp_s

# 未確定・縮退の理由。正常時は空文字。
string reason
