# カメラ画像へ重畳するための認識結果。画像そのものは含まず、検出データと判定結果だけを運ぶ。
#
# 配信様式: event (認識処理が 1 フレームを評価するごと)
# 配信元: traffic_signal_recognizer / road_blockage_detector
# 備考: 認識処理は生画像より低いレートで動作するため、公称レートは定めない。
#
# 認識ノードは重畳画像を配信せず、本メッセージのみを配信する。重畳は表示側
# (robot_console の ConsoleCore) が行い、Qt UI と HTML UI で同一の描画結果を共有する。
#
# header は判定根拠となった元画像フレームの stamp/frame_id をそのまま引き継ぐ。
# これにより生画像トピックのどのフレームに対する結果かを一意に特定できる。
# 認識処理は生画像より低いレートで動作し得るため、全フレームに結果が対応するとは限らない。
std_msgs/Header header

# 認識の種別。表示側はこの値でパネル内の描き分けと判定チップの対応付けを行う。
# 既定値: traffic_signal / road_blockage
string source

OverlayDetection[] detections

# 判定結果。値の意味は source ごとに異なる (信号認識は sig_recog と同じ値、
# 経路封鎖は 0=封鎖なし / 1=封鎖あり)。制御経路へ流す /sig_recog・/road_blocked とは
# 別に本メッセージへも載せることで、検出枠と判定が必ず同一フレームの評価結果になる。
int32 decision

# 判定結果の表示文字列 (GO / STOP / 封鎖あり など)。
string decision_text

# 判定を行えなかった場合の理由 (no_pose / suppressed など)。正常時は空文字。
string status_note
