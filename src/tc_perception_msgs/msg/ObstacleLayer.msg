# 水平スライス 1 層の観測結果。
#
# 配信様式: 埋め込み用（ObstacleLayers.layers の配列要素。単独では配信しない）
#
# 座標は base_link 系で、x が前方、y が左を正とする。高さは床面基準で表す。
# 層は観測センサごとに生成元が異なり、topURG の層は 2D scan、それ以外は
# MID-360 の点群を高さで切り出したものである。

string name
string source            # scan / point_cloud
float32 height_m         # 床面からの層の中心高さ [m]
float32 thickness_m      # 層の厚さ [m]（中心から上下へ half を取る）
int32 point_count        # 層に含まれた点数。0 は未観測であり、障害物なしを意味しない

# 直進帯（車体幅＋余裕）内の最近傍距離 [m]。該当点が無い場合は inf。
# 横移動量の判定は forward_range_m までで行うが、この距離には上限を設けない。
float32 nearest_x_m

# 車体を横へ寄せたときに前方帯が空く最小の横移動量 [m]。
# 0 は現在位置のまま通行可能、inf は通行可能な横移動量が見つからないことを表す。
float32 required_left_offset_m
float32 required_right_offset_m

# 前方帯が空いた状態を保てる最大の横移動量 [m]。回避幅の上限に使う。
# required が inf の場合は 0 とする。
float32 available_left_offset_m
float32 available_right_offset_m
