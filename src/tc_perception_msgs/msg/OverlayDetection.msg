# 画像へ重畳する検出 1 件。
#
# 配信様式: 埋め込み用（PerceptionOverlay.detections の配列要素。単独では配信しない）
#
# 座標は判定根拠となった元画像のピクセル系で、中心座標と大きさで表す
# (vision_msgs/BoundingBox2D と同じ表現)。

float32 center_x
float32 center_y
float32 size_x
float32 size_y

# 画面表示用のラベル。class id から class name への解決は検出ノード側で行い、
# 表示側 (robot_console) が class 定義を持たなくて済むようにする。
string label
float32 score

# 判定に採用された検出か。採用外の検出も観測できるよう残し、表示側で描き分ける。
bool adopted
