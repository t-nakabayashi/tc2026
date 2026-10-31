"""経路CSVを指定ラベルで2つへ分け、route_plannerの固定ブロック2つ分として配置する.

選択課題Aに対応するルートと対応しないルートを、同じ waypoint の行を複製せずに
定義するために分ける。対応しないルートは2つの固定ブロックを連結し、対応する
ルートは同じ2ファイルの間に可変ブロックを挟む。

分割点の waypoint は前半の末尾に入る。`route_planner` はブロック末尾の姿勢を
直前点からの方位で上書きするため、停止点を分割点にすると停止時の向きが変わる。
停止点が指定された場合は拒否する。
"""
import argparse
import csv
from pathlib import Path
from typing import List


def split_rows(rows: List[dict], split_label: str) -> tuple:
    """指定ラベルまでを前半、その次の行からを後半として分ける.

    Args:
        rows (List[dict]): 経路CSVの全行.
        split_label (str): 前半の末尾にする waypoint のラベル.

    Returns:
        tuple: (前半の行, 後半の行).
    """

    matched = [i for i, row in enumerate(rows) if row["label"] == split_label]
    if len(matched) != 1:
        raise ValueError(
            f"分割ラベル '{split_label}' が {len(matched)} 件見つかりました。1件である必要があります。"
        )
    index = matched[0]
    row = rows[index]
    if row["line_is_stop"] != "0" or row["signal_is_stop"] != "0":
        raise ValueError(
            f"分割ラベル '{split_label}' は停止点です。ブロック末尾では姿勢が再計算されるため、"
            "停止点は分割点にできません。"
        )
    if index == 0 or index == len(rows) - 1:
        raise ValueError(f"分割ラベル '{split_label}' が経路の端にあります。")
    return rows[: index + 1], rows[index + 1 :]


def main() -> None:
    """コマンドラインから経路CSVを分割する."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="分割する経路CSV")
    parser.add_argument("--split-label", required=True, help="前半の末尾にするラベル")
    parser.add_argument("--output", type=Path, required=True, help="書き出し先ディレクトリ")
    parser.add_argument("--names", nargs=2, default=["part1", "part2"], help="出力ファイル名")
    args = parser.parse_args()

    with args.source.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = list(reader)
    if fields is None:
        raise ValueError(f"{args.source} にヘッダがありません。")

    head, tail = split_rows(rows, args.split_label)
    args.output.mkdir(parents=True, exist_ok=True)
    for name, part in zip(args.names, (head, tail)):
        path = args.output / f"{name}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            writer.writerows(part)
        print(f"{path}: {len(part)} 点")


if __name__ == "__main__":
    main()
