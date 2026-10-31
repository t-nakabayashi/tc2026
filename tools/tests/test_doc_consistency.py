"""文書と実装の整合性を CI で検査する。

`tools/check_docs.py` の検査を pytest から呼ぶ。ソースを変更して文書が追随していない
場合に、PR の時点で落ちるようにするための入口である。

決着期限の超過は警告として扱い、テストは落とさない。期限を延ばすか恒久文書へ反映するかは
人が判断する事項であり、無関係な変更の PR を止める理由にならない。
"""

from pathlib import Path
import subprocess
import sys

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import check_docs  # noqa: E402  sys.path 設定後に import する必要がある


def _today() -> str:
    """検討資料の期限判定に使う基準日を返す.

    Returns:
        str: YYYY-MM-DD 形式の当日.
    """
    return subprocess.run(
        ['date', '+%Y-%m-%d'], capture_output=True, text=True, check=True
    ).stdout.strip()


def test_documents_are_consistent_with_implementation() -> None:
    """文書と実装の乖離が無いことを確認する。

    検出するのはリンク切れ、廃止トピック名の残存、パラメータ表の乖離、必須文書の欠落、
    記述方針ブロックの不一致、検討資料の必須ヘッダ欠落である。
    """
    report = check_docs.run_checks(check_docs.repo_root(), _today())

    for warning in report.warnings:
        print(f'警告: {warning}')

    assert not report.failures, '\n'.join(['文書と実装の乖離:', *report.failures])
