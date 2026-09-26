# ミッションのメタ情報（静的・準静的設定を通知）
# - 例：スタート/ゴール/必須通過群のラベル列、運用パラメータ概要など
#
# 配信様式: latch
# 配信元: route_manager
# 備考: ルート確定時に一度だけ配信する。死活監視には使わない。

std_msgs/Header header
string start_label
string goal_label
string[] checkpoint_labels
string description

