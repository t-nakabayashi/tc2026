#!/usr/bin/env python3
"""区間ごとに、左右へ出せる横移動量を境界の種類に応じて求める.

境界の種類は利用者が映像で判断して区間表に与える。種類ごとに測り方が異なる。

| 種類 | 測り方 | 根拠 |
|---|---|---|
| continuous | 近い側の検出に1次式を当てはめ、その値を使う | 生垣・壁・フェンスは連続している |
| cone | 同じ当てはめに追加の余裕を引く | ロードコーンの設置は直線から外れ、外側はコース外である |
| discrete | 区間内の最小値を使う | 柱や車止めは間を通れるが、最も近いものに合わせる |
| none | 上限値を使う | 探索範囲に境界が無い |
| open_wide | 遠い側の検出から求め、広い区間の上限で頭打ちにする | 近い検出は置かれた障害物であり路端ではない |
| crossing | 0 とする | 横断中に横移動しない |

広場にコーンを疎に置いて通路を示す区間では、近い側の上限が既定では狭すぎる。
区間表の `near_limit_m` 列で区間ごとに上書きする。

観測から求めた値が映像の判断と大きく食い違う区間では、区間表の `left_override_m` /
`right_override_m` に利用者の判断値を与える。URGは最も近い鉛直物で走査を打ち切るため、
通り抜けられるボラードや疎な植栽を連続した境界として扱い、値を過小に見積もることがある。
上書きした側は `basis` に その旨を残す。

出力は提案値であり、測量値でも衝突余裕の保証値でもない。利用者が映像と突き合わせて
確認したうえで経路へ反映する。
"""
import argparse
import csv
from pathlib import Path

import numpy as np

SIDES = (('left', 'urg_left_m'), ('right', 'urg_right_m'))


def far_boundary(value: np.ndarray, far_floor: float) -> tuple[float, int] | None:
    """障害物が多い区間で、遠い側の検出から路端の位置を求める.

    置かれた障害物は経路の近くに現れるため、近い検出を使うと路端を過小に見積もる。
    一定距離より遠い検出だけを残し、その中央値を路端とする。

    Args:
        value (np.ndarray): 境界までの距離 [m]。未検出は NaN.
        far_floor (float): 路端として扱う最小距離 [m].

    Returns:
        tuple[float, int] | None: 路端の位置と採用点数。点数が足りなければ None.
    """
    found = value[(~np.isnan(value))&(value >= far_floor)]
    if len(found) < 8:
        return None
    return float(np.median(found)), int(len(found))


def fit_boundary(distance: np.ndarray, value: np.ndarray, near_limit: float,
                 degree: int) -> tuple[float, float, int] | None:
    """近い側の検出だけに1次式を当てはめ、区間の代表値と残差を求める.

    コーン列では、コーンの隙間から遠くの物が見えるため分布が二峰になる。
    近い側だけを残さないと、隙間の値に引かれて代表値が大きく出る。

    Args:
        distance (np.ndarray): 断面の弧長 [m].
        value (np.ndarray): 境界までの距離 [m]。未検出は NaN.
        near_limit (float): 近い側として残す上限 [m].
        degree (int): 当てはめる多項式の次数.

    Returns:
        tuple[float, float, int] | None: 代表値、残差の四分位範囲、採用点数.
            点数が足りない場合は None.
    """
    usable = (~np.isnan(value))&(value <= near_limit)
    x, y = distance[usable], value[usable]
    if len(x) < 8:
        return None
    keep = np.ones(len(x), bool)
    for _ in range(3):
        if keep.sum() < 5:
            return None
        coefficients = np.polyfit(x[keep], y[keep], degree)
        residual = y-np.polyval(coefficients, x)
        center = np.median(residual[keep])
        scale = max(.05, 1.4826*float(np.median(np.abs(residual[keep]-center))))
        keep = np.abs(residual-center) < 2.5*scale
    fitted = np.polyval(coefficients, x[keep])
    spread = float(np.percentile(residual[keep], 75)-np.percentile(residual[keep], 25))
    return float(np.median(fitted)), spread, int(keep.sum())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cross-sections', type=Path, required=True)
    parser.add_argument('--regions', type=Path, required=True,
                        help='start_m,end_m,name,left_type,right_type を持つCSV')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--body-margin', type=float, default=.4,
                        help='車体半幅と余裕の合計 [m]')
    parser.add_argument('--cone-margin', type=float, default=.25,
                        help='コーン列に追加で引く余裕 [m]')
    parser.add_argument('--near-limit', type=float, default=1.8,
                        help='近い側として残す上限 [m]。区間表の near_limit_m 列で上書きできる')
    parser.add_argument('--max-curvature', type=float, default=2.,
                        help='これを超える曲率の断面を除く [deg/m]')
    parser.add_argument('--cap', type=float, default=2.5, help='境界が無い区間の上限 [m]')
    parser.add_argument('--wide-cap', type=float, default=3.,
                        help='障害物が多い広い区間の上限 [m]')
    parser.add_argument('--far-floor', type=float, default=2.,
                        help='障害物が多い区間で路端として扱う最小距離 [m]')
    args = parser.parse_args()

    rows = [r for r in csv.DictReader(args.cross_sections.open())
            if float(r['curvature_deg_per_m']) <= args.max_curvature]
    distance = np.array([float(r['distance_m']) for r in rows])
    columns = {key: np.array([float(r[key]) if r[key] else np.nan for r in rows])
               for _, key in SIDES}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['start_m', 'end_m', 'name', 'side', 'type', 'samples',
                         'boundary_m', 'residual_iqr_m', 'offset_m', 'basis'])
        print(f'{"区間":>12} {"種別":<20} 側 {"種類":<10} {"境界":>6} {"残差":>6} {"採用値":>7}')
        for region in csv.DictReader(args.regions.open()):
            start, end = float(region['start_m']), float(region['end_m'])
            inside = (distance >= start)&(distance <= end)
            for side, key in SIDES:
                kind = region[f'{side}_type']
                values = columns[key][inside]
                boundary = spread = None
                count = int((~np.isnan(values)).sum())
                near_limit = float(region.get('near_limit_m') or args.near_limit)
                override = region.get(f'{side}_override_m')
                if override:
                    writer.writerow([f'{start:.0f}', f'{end:.0f}', region['name'], side, kind,
                                     count, '', '', f'{float(override):.1f}', '映像判断で上書き'])
                    print(f'{start:5.0f}-{end:5.0f} {region["name"][:20]:<20} '
                          f'{"左" if side == "left" else "右"} {kind:<10} '
                          f'{"-":>6} {"-":>6} {float(override):7.1f}')
                    continue
                if kind in ('continuous', 'cone'):
                    fitted = fit_boundary(distance[inside], values, near_limit, 1)
                    if fitted is None:
                        offset, basis = 0., '当てはめ不成立'
                    else:
                        boundary, spread, count = fitted
                        margin = args.body_margin+(args.cone_margin if kind == 'cone' else 0.)
                        offset, basis = max(0., boundary-margin), '当てはめ'
                elif kind == 'discrete':
                    found = values[~np.isnan(values)]
                    if len(found) < 5:
                        offset, basis = 0., '点数不足'
                    else:
                        boundary = float(np.percentile(found, 5))
                        offset, basis = max(0., boundary-args.body_margin), '下側5%点'
                elif kind == 'open_wide':
                    far = far_boundary(values, args.far_floor)
                    if far is None:
                        offset, basis = args.wide_cap, '広い区間の上限'
                    else:
                        boundary, count = far
                        offset = min(max(0., boundary-args.body_margin), args.wide_cap)
                        basis = '遠い側の検出'
                elif kind == 'crossing':
                    offset, basis = 0., '横断部'
                else:
                    offset, basis = args.cap, '上限'
                writer.writerow([
                    f'{start:.0f}', f'{end:.0f}', region['name'], side, kind, count,
                    '' if boundary is None else f'{boundary:.2f}',
                    '' if spread is None else f'{spread:.2f}', f'{offset:.1f}', basis])
                print(f'{start:5.0f}-{end:5.0f} {region["name"][:20]:<20} '
                      f'{"左" if side == "left" else "右"} {kind:<10} '
                      f'{"-" if boundary is None else f"{boundary:6.2f}"} '
                      f'{"-" if spread is None else f"{spread:6.2f}"} {offset:7.1f}')
    print(f'\nDONE -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
