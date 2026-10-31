#!/usr/bin/env python3
"""区間がどのような場所かを、写真と説明でまとめた1枚のHTMLにする.

区間の名前だけでは現地の様子が分からず、幅の値を後から見直すときに判断材料が無い。
区間ごとに代表するコマ、観測メモ、求めた幅と根拠を並べる。

写真は現地の様子を含むため、出力はリポジトリの外へ置く。画像はHTMLへ埋め込み、
1ファイルだけで閲覧できるようにする。
"""
import argparse
import base64
import csv
import html
import json
from pathlib import Path


def load_frames(directory: Path) -> list[dict]:
    """抽出済みのコマを、距離順に並べて返す.

    Args:
        directory (Path): frames.json を含むディレクトリ.

    Returns:
        list[dict]: コマの情報を距離順に並べたもの.
    """
    rows = json.loads((directory/'frames.json').read_text())
    rows.sort(key=lambda r: float(r['target_distance_m']))
    return rows


def pick_frame(frames: list[dict], start: float, end: float) -> tuple[dict | None, bool]:
    """区間の中央に最も近いコマを選ぶ.

    横断歩道の手前のように区間が短いとコマが1つも入らない。その場合は最寄りのコマを
    区間外と明示して使う。

    Args:
        frames (list[dict]): コマの情報.
        start (float): 区間の開始距離 [m].
        end (float): 区間の終了距離 [m].

    Returns:
        tuple[dict | None, bool]: 選んだコマと、それが区間の外かどうか.
    """
    middle = (start+end)/2
    inside = [f for f in frames if start <= float(f['target_distance_m']) < end]
    if inside:
        return min(inside, key=lambda f: abs(float(f['target_distance_m'])-middle)), False
    if not frames:
        return None, False
    return min(frames, key=lambda f: abs(float(f['target_distance_m'])-middle)), True


def embed(path: Path) -> str:
    """画像をHTMLへ埋め込める形式にする.

    Args:
        path (Path): 画像のパス.

    Returns:
        str: data URL.
    """
    return 'data:image/jpeg;base64,'+base64.b64encode(path.read_bytes()).decode()


def load_widths(path: Path) -> dict[tuple[float, float], dict[str, dict]]:
    """区間ごと・左右ごとの幅と根拠を引けるようにする.

    Args:
        path (Path): region_widths.csv のパス.

    Returns:
        dict: (開始, 終了) から左右それぞれの値への対応.
    """
    out: dict[tuple[float, float], dict[str, dict]] = {}
    for row in csv.DictReader(path.open()):
        key = (float(row['start_m']), float(row['end_m']))
        out.setdefault(key, {})[row['side']] = row
    return out


def load_notes(path: Path) -> list[tuple[float, str]]:
    """映像から人が書いた観測メモを距離つきで返す.

    Args:
        path (Path): 映像判断のCSV.

    Returns:
        list[tuple[float, str]]: 距離 [m] とメモ.
    """
    return [(float(r['distance_m']), r['note'])
            for r in csv.DictReader(path.open()) if r.get('note')]


def section(number: int, region: dict, chosen: tuple, frames_dir: Path,
            widths: dict, notes: list[tuple[float, str]]) -> str:
    """1区間分のHTMLを作る.

    Args:
        number (int): 区間の番号.
        region (dict): 区間の定義.
        chosen (tuple): 代表するコマと、それが区間の外かどうか.
        frames_dir (Path): コマの画像があるディレクトリ.
        widths (dict): 左右の幅と根拠.
        notes (list[tuple[float, str]]): 区間内の観測メモ.

    Returns:
        str: 区間1つ分のHTML.
    """
    start, end = float(region['start_m']), float(region['end_m'])
    frame, outside = chosen
    note = '（区間外の最寄り）' if outside else ''
    picture = ('<p class="missing">コマがありません</p>' if frame is None else
               f'<img src="{embed(frames_dir/frame["name"])}" '
               f'alt="{html.escape(region["name"])}">'
               f'<p class="caption">{float(frame["target_distance_m"]):.0f} m 地点{note}'
               f'（手前 {float(frame["lead_m"]):.0f} m から撮影）</p>')
    rows = ''
    for side, japanese in [('left', '左'), ('right', '右')]:
        value = widths.get(side)
        if value is None:
            continue
        boundary = value['boundary_m'] or '-'
        rows += (f'<tr><td>{japanese}</td><td>{region[side+"_type"]}</td>'
                 f'<td>{value["offset_m"]} m</td><td>{boundary}</td>'
                 f'<td>{value["basis"]}</td><td>{value["samples"]}</td></tr>')
    note_items = ''.join(f'<li>{distance:.0f} m: {html.escape(text)}</li>'
                         for distance, text in notes)
    return f'''<section>
  <h2>{number}. {html.escape(region['name'])}
    <span class="range">{start:.0f} – {end:.0f} m</span></h2>
  <div class="body">
    <div class="photo">{picture}</div>
    <div class="text">
      <p class="description">{html.escape(region.get('description', ''))}</p>
      <table>
        <tr><th>側</th><th>境界の種類</th><th>回避幅</th><th>路端まで</th>
            <th>根拠</th><th>標本数</th></tr>
        {rows}
      </table>
      <details open><summary>映像からの観測メモ（{len(notes)} 件）</summary>
        <ul>{note_items}</ul></details>
    </div>
  </div>
</section>'''


def main() -> None:
    """区間定義・コマ・幅・観測メモを1枚のHTMLにまとめる.

    Returns:
        None
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--regions', type=Path, required=True,
                        help='説明つきの区間定義CSV')
    parser.add_argument('--frames', type=Path, required=True,
                        help='frames.json を含むディレクトリ')
    parser.add_argument('--region-widths', type=Path, required=True,
                        help='区間ごとの幅 region_widths.csv')
    parser.add_argument('--notes', type=Path, default=None,
                        help='映像判断のCSV。あれば区間ごとに並べる')
    parser.add_argument('--output', type=Path, required=True,
                        help='出力するHTML。リポジトリの外を指定する')
    parser.add_argument('--title', default='つくばチャレンジ2026 コース区間の説明')
    args = parser.parse_args()

    regions = sorted(csv.DictReader(args.regions.open()), key=lambda r: float(r['start_m']))
    frames = load_frames(args.frames)
    widths = load_widths(args.region_widths)
    notes = load_notes(args.notes) if args.notes else []
    sections = []
    for number, region in enumerate(regions, start=1):
        start, end = float(region['start_m']), float(region['end_m'])
        inside = [(distance, text) for distance, text in notes if start <= distance < end]
        sections.append(section(number, region, pick_frame(frames, start, end), args.frames,
                                widths.get((start, end), {}), inside))
    total = float(regions[-1]['end_m'])
    document = f'''<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(args.title)}</title>
<style>
 body {{ font-family: sans-serif; margin: 0 auto; max-width: 1100px; padding: 16px;
         color: #222; line-height: 1.6; }}
 h1 {{ font-size: 20px; border-bottom: 2px solid #333; padding-bottom: 8px; }}
 section {{ border: 1px solid #ddd; border-radius: 6px; margin: 16px 0; padding: 12px; }}
 h2 {{ font-size: 16px; margin: 0 0 8px; }}
 .range {{ font-weight: normal; color: #666; margin-left: 12px; font-size: 14px; }}
 .body {{ display: flex; gap: 16px; flex-wrap: wrap; }}
 .photo {{ flex: 0 0 420px; }}
 .photo img {{ width: 100%; border: 1px solid #ccc; border-radius: 4px; }}
 .text {{ flex: 1 1 380px; }}
 .caption, .missing {{ color: #666; font-size: 13px; margin: 4px 0 0; }}
 .description {{ margin: 0 0 10px; }}
 table {{ border-collapse: collapse; font-size: 13px; width: 100%; }}
 th, td {{ border: 1px solid #ddd; padding: 4px 8px; text-align: left; }}
 th {{ background: #f4f4f4; }}
 details {{ margin-top: 10px; font-size: 13px; }}
 ul {{ margin: 6px 0; padding-left: 20px; }}
 .lead {{ background: #f8f8f8; border-left: 4px solid #999; padding: 8px 12px; }}
 @page {{ size: A4; margin: 12mm; }}
 @media print {{
   body {{ max-width: none; padding: 0; font-size: 11px; }}
   h1 {{ font-size: 16px; }}
   h2 {{ font-size: 13px; }}
   section {{ break-inside: avoid; page-break-inside: avoid; }}
   .photo {{ flex: 0 0 300px; }}
   table, details {{ font-size: 10px; }}
 }}
</style></head><body>
<h1>{html.escape(args.title)}</h1>
<p class="lead">全{len(regions)}区間、{total:.0f} m。写真は手動走行時の前方カメラから、
区間の中央付近を選んだ1コマである。回避幅は経路CSVへ書き込んだ値、
路端までの距離はURGの水平断面から求めた値である。
現地の通行可否や当日の設置物を保証しない。</p>
{''.join(sections)}
</body></html>'''
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(document, encoding='utf-8')
    size = args.output.stat().st_size/1024**2
    print(f'出力: {args.output}（{len(regions)} 区間, {size:.1f} MB）')


if __name__ == '__main__':
    main()
