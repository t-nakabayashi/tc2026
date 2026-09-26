# route_manager が外部へ公開する状態情報（Phase2仕様）
#
# 配信様式: stream (1 Hz, route_manager の state_publish_rate_hz)
# 配信元: route_manager
# 備考: 状態変化時は即時配信し、変化が無い間も上記レートで再送する。

std_msgs/Header header
string state        # "idle", "running", "updating_route", "holding"
string decision     # "none", "update", "replan_first", "shift", "skip", "failed"
string last_cause   # HOLDING時のみ滞留理由（"front_blocked", "no_hint", など）
uint32 route_version
