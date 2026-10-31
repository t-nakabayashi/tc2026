# 回避幅の算出結果

[waypointの回避幅を求める手順](../docs/waypointの回避幅を求める手順.md)の成果物のうち、
文字と図だけで現地の写真を含まないものを置く。区間の説明HTMLは写真を含むため
ここには置かない。

| ファイル | 内容 |
|---|---|
| `tsukuba2026_waypoints.csv` | 回避幅を書き込んだ経路。`tools/build_route.py` の出力から `right_is_open` と `left_is_open` の2列だけが異なる |
| `tsukuba2026_route_widths.png` | 全3,704点の回避幅を、ENU俯瞰図と距離方向の帯で1枚にした図 |

## 由来

2026年10月4日の手動走行ログ、そのログから作ったグラフSLAM地図、そのログから作った
3,704点の経路に対して、手順の1から8を適用した。区間定義は
[params/tsukuba2026_regions.csv](../params/tsukuba2026_regions.csv)である。

## 検証した範囲

- `tools/build_route.py` を同じ入力で再実行した出力との差分が、幅2列だけであることを
  確認した。
- 20 m間隔の映像から人が判断した値と突き合わせた。
- 模擬環境で経路を読み込ませ、走行が破綻しないこと、幅が0より大きい区間で横移動が
  命令され、幅が0の区間で命令されないことを確認した。

**実機の走行では確認していない。** 値は提案であり、衝突余裕の保証ではない。
本番として使う前に利用者が確認する。確認後の配置先は
`src/route_planner/routes/<コース名>/` とする。

## 作り直す

図は次で再生成できる。入力を変えたときは作り直す。

```bash
python3 src/route_survey/tools/render_route_widths.py \
  --waypoints-enu <経路>/waypoints_enu.json \
  --route src/route_survey/results/tsukuba2026_waypoints.csv \
  --regions src/route_survey/params/tsukuba2026_regions.csv \
  --output src/route_survey/results/tsukuba2026_route_widths.png
```
