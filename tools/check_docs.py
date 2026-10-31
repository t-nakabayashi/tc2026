#!/usr/bin/env python3
"""文書と実装の乖離を検出する.

ソースを変更したのに文書が追随していない状態を、CI で機械的に見つけるための検査である。
`tools/tests/test_doc_consistency.py` から呼ばれ、pytest の一部として実行される。

検査は次の 7 つとする。

1. リンク・アンカー切れ       文書の移動・改名の取りこぼし
2. トピック名の残存           廃止・改名されたトピック名が文書に残っている
3. パラメータ名の残存         パラメータ表が実装から乖離している
4. パッケージ必須文書の欠落   README と詳細設計書の欠落
5. 記述方針の同期             CLAUDE.md と AGENTS.md の規則ブロックの不一致
6. 検討資料の決着期限         docs/notes/ の期限超過（報告のみ）
7. 章番号参照の切れ           コードが指す設計書の章・節が実在しない

除外は `tools/doc_check_allowlist.yaml` に理由付きで書く。無条件の無視は作らない。

使い方:
    python3 tools/check_docs.py [--warn-only]

`--warn-only` を付けると、検出しても終了コード 0 を返す。
"""

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

import check_doc_links

# 走査から除外するディレクトリ。submodule、colcon 生成物、外部資料、検討資料を含む。
# 検討資料は現状追従の義務を負わないため、名前の整合性検査の対象にしない。
EXCLUDED_DIRS = check_doc_links.EXCLUDED_DIRS | {"notes"}

# 文書中で backtick に囲まれたトピック風のトークン。
DOC_TOPIC_PATTERN = re.compile(r"`(/[a-z][a-z0-9_]*(?:/[a-z0-9_]+)*)`")

# ソース中の「<文書名>.md 6.3節」「<文書名>.md 10章」形式の参照。
# backtick や読点が間に入る書き方を許し、文書名・番号・単位を取り出す。
SOURCE_DOC_SECTION_PATTERN = re.compile(
    r"([A-Za-z0-9_]+)\.md`?\s*([0-9]+(?:\.[0-9]+)?)\s*(節|章)"
)

# 文書の見出しのうち、先頭が章・節番号であるもの。章見出しは「## 6. 章題」のように
# 番号の後ろにピリオドを置くため、末尾のピリオドを番号に含めずに取り出す。
DOC_HEADING_NUMBER_PATTERN = re.compile(r"^#{1,6}\s+([0-9]+(?:\.[0-9]+)?)\.?\s+\S")

# コード中の文字列リテラルに現れるトピック・サービス名。相対名（先頭の `/` 無し、
# 単一セグメント）も拾う。launch remap を前提に相対名で宣言する実装が多く、これを
# 拾わないと文書側の絶対名が誤検出になる。
CODE_TOPIC_PATTERN = re.compile(r"['\"](/?[a-z][a-z0-9_]*(?:/[a-z0-9_]+)*)['\"]")

# yaml / msg / launch / C++ に直接書かれたトピック名。
PLAIN_TOPIC_PATTERN = re.compile(r"(/[a-z][a-z0-9_]*(?:/[a-z0-9_]+)*)")

# `declare_parameter('name'` と、文書のパラメータ表に現れる backtick 付きの名前。
DECLARE_PARAM_PATTERN = re.compile(r"declare_parameter\(\s*['\"]([a-z_][a-z0-9_.]*)['\"]")

# launch 引数。文書のパラメータ表は ROS パラメータと launch 引数を区別せずに並べる
# ことがあるため、両方を宣言済みとして扱う。
LAUNCH_ARG_PATTERN = re.compile(r"DeclareLaunchArgument\(\s*['\"]([a-z_][a-z0-9_.]*)['\"]")

# declare_parameters (複数形) は (名前, 既定値) のタプル列で宣言する。タプル形式は
# 他の用途にも現れるため、declare_parameters を含むファイルに限って適用する。
PARAM_TUPLE_PATTERN = re.compile(r"\(\s*['\"]([a-z_][a-z0-9_.]+)['\"]\s*,")
DOC_PARAM_ROW_PATTERN = re.compile(r"^\|\s*`([a-z_][a-z0-9_.]*)`\s*\|")

# パラメータ表と判定する条件。見出しとヘッダ行の両方が揃う表だけを対象にする。
# ファイル名・ノード名・内部状態の一覧も同じ表形式で書かれるため、片方だけでは絞れない。
PARAM_HEADING_PATTERN = re.compile(r"^#{1,6}\s.*パラメータ")
PARAM_TABLE_HEADER_PATTERN = re.compile(r"既定値|デフォルト")
HEADING_PATTERN = re.compile(r"^#{1,6}\s")

# CLAUDE.md と AGENTS.md で同期する範囲。
RULES_BLOCK_PATTERN = re.compile(
    r"<!-- doc-writing-rules:begin -->\n(.*?)\n<!-- doc-writing-rules:end -->", re.S
)

# 検討資料の必須ヘッダ。
NOTE_HEADER_KEYS = ("状態", "作成日", "対象コミット", "決着期限")


@dataclass
class Report:
    """検査結果をまとめる.

    Attributes:
        failures (list[str]): 修正が必要な指摘.
        warnings (list[str]): 報告にとどめる指摘.
    """

    failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def repo_root() -> Path:
    """リポジトリルートを求める.

    Returns:
        Path: リポジトリルートの絶対パス.
    """
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        check=True,
    )
    return Path(result.stdout.strip()).resolve()


def is_excluded(path: Path, root: Path) -> bool:
    """走査対象外のパスかを判定する.

    Args:
        path (Path): 判定対象のパス.
        root (Path): リポジトリルート.

    Returns:
        bool: 除外対象なら True.
    """
    try:
        parts = path.relative_to(root).parts
    except ValueError:
        return True
    return any(part in EXCLUDED_DIRS for part in parts)


def load_allowlist(root: Path) -> dict[str, set[str]]:
    """除外リストを読み込む.

    Args:
        root (Path): リポジトリルート.

    Returns:
        dict[str, set[str]]: 区分ごとの除外名の集合.
    """
    path = root / "tools/doc_check_allowlist.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return {
        key: {entry["name"] for entry in (value or [])}
        for key, value in data.items()
    }


def iter_docs(root: Path) -> list[Path]:
    """検査対象の Markdown を集める.

    Args:
        root (Path): リポジトリルート.

    Returns:
        list[Path]: 検査対象のファイル一覧.
    """
    return sorted(
        p for p in root.rglob("*.md")
        if not is_excluded(p, root) and p.is_file() and not p.is_symlink()
    )


def iter_sources(root: Path) -> list[Path]:
    """検査対象のソースを集める.

    Args:
        root (Path): リポジトリルート.

    Returns:
        list[Path]: 検査対象のファイル一覧.
    """
    patterns = ("*.py", "*.yaml", "*.yml", "*.msg", "*.srv", "*.cpp", "*.hpp", "*.h")
    files: list[Path] = []
    for pattern in patterns:
        files.extend(
            p for p in root.rglob(pattern)
            if not is_excluded(p, root) and p.is_file() and not p.is_symlink()
        )
    return sorted(files)


def collect_code_topics(sources: list[Path]) -> set[str]:
    """コードに現れるトピック名を集める.

    先頭の `/` の有無は起動時の namespace 解決で変わるため、比較では落とす。

    Args:
        sources (list[Path]): 走査対象のソース.

    Returns:
        set[str]: 先頭の `/` を除いたトピック名の集合.
    """
    names: set[str] = set()
    for path in sources:
        text = path.read_text(encoding="utf-8", errors="ignore")
        names |= set(CODE_TOPIC_PATTERN.findall(text))
        if path.suffix in (".yaml", ".yml", ".msg", ".srv", ".cpp", ".hpp", ".h"):
            names |= set(PLAIN_TOPIC_PATTERN.findall(text))
    return {name.lstrip("/") for name in names}



def _is_known_topic(doc_name: str, known: set[str]) -> bool:
    """文書中のトピック名がコード側の宣言と対応するかを判定する.

    文書は名前空間まで解決した完全名（`/rtk_gps/time_sync`）を書くが、コードは
    起動時の remap を前提に相対名（`time_sync`）で宣言する。名前空間を静的に解決
    できないため、末尾から区切りごとに切った部分名のいずれかが一致すれば対応ありと
    みなす。`/a/b/c` なら `a/b/c`、`b/c`、`c` を順に照合する。

    Args:
        doc_name (str): 文書中のトピック名.
        known (set[str]): コードに現れた名前の集合（先頭の `/` は除いてある）.

    Returns:
        bool: 対応する宣言があれば True.
    """
    segments = doc_name.lstrip("/").split("/")
    return any("/".join(segments[i:]) in known for i in range(len(segments)))


def check_topics(docs: list[Path], sources: list[Path], root: Path,
                 allowlist: dict[str, set[str]], report: Report) -> None:
    """文書中のトピック名がコードに存在するかを検査する.

    Args:
        docs (list[Path]): 検査対象の文書.
        sources (list[Path]): 走査対象のソース.
        root (Path): リポジトリルート.
        allowlist (dict[str, set[str]]): 除外名.
        report (Report): 結果の記録先.
    """
    known = collect_code_topics(sources)
    allowed = (allowlist.get("topics", set()) | allowlist.get("filesystem_paths", set())
               | allowlist.get("remapped_topics", set()))
    for path in docs:
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            for name in DOC_TOPIC_PATTERN.findall(line):
                if name in allowed or _is_known_topic(name, known):
                    continue
                report.failures.append(
                    f"{path.relative_to(root)}:{lineno}: "
                    f"コードに存在しないトピック名 -> {name}"
                )


def check_parameters(root: Path, allowlist: dict[str, set[str]], report: Report) -> None:
    """パッケージ文書のパラメータ表が declare_parameter と対応するかを検査する.

    パッケージ単位で突き合わせる。文書のパラメータ表に書かれた名前が、そのパッケージの
    どの `declare_parameter` にも無い場合を検出する。

    Args:
        root (Path): リポジトリルート.
        allowlist (dict[str, set[str]]): 除外名.
        report (Report): 結果の記録先.
    """
    allowed = allowlist.get("parameters", set())
    for package in sorted((root / "src").iterdir()):
        if not (package / "package.xml").exists() or is_excluded(package, root):
            continue
        declared: set[str] = set()
        for source in package.rglob("*.py"):
            if is_excluded(source, root) or source.is_symlink():
                continue
            text = source.read_text(encoding="utf-8", errors="ignore")
            declared |= set(DECLARE_PARAM_PATTERN.findall(text))
            declared |= set(LAUNCH_ARG_PATTERN.findall(text))
            if "declare_parameters(" in text:
                declared |= set(PARAM_TUPLE_PATTERN.findall(text))
        if not declared:
            continue
        docs = [package / "README.md", *package.glob("docs/*.md")]
        for path in docs:
            if not path.exists() or is_excluded(path, root):
                continue
            in_param_section = False
            in_param_table = False
            for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                if HEADING_PATTERN.match(line):
                    in_param_section = bool(PARAM_HEADING_PATTERN.match(line))
                    in_param_table = False
                    continue
                if not in_param_section:
                    continue
                if line.startswith("|"):
                    if not in_param_table:
                        # 表の先頭行。既定値の列を持つものだけをパラメータ表と見なす。
                        in_param_table = bool(PARAM_TABLE_HEADER_PATTERN.search(line))
                        continue
                else:
                    in_param_table = False
                    continue
                matched = DOC_PARAM_ROW_PATTERN.match(line)
                if matched is None:
                    continue
                name = matched.group(1)
                if name in allowed or name in declared:
                    continue
                report.failures.append(
                    f"{path.relative_to(root)}:{lineno}: "
                    f"declare_parameter に無いパラメータ名 -> {name}"
                )


def check_required_docs(root: Path, allowlist: dict[str, set[str]], report: Report) -> None:
    """パッケージごとの必須文書が存在するかを検査する.

    Args:
        root (Path): リポジトリルート.
        allowlist (dict[str, set[str]]): 除外名. `pending_design_docs` を参照する.
        report (Report): 結果の記録先.
    """
    pending = allowlist.get("pending_design_docs", set())
    for package in sorted((root / "src").iterdir()):
        if not (package / "package.xml").exists() or is_excluded(package, root):
            continue
        name = package.name
        if not (package / "README.md").exists():
            report.failures.append(f"src/{name}: README.md が無い")
        # ノードを持つパッケージには詳細設計書を要求する。
        has_node = any(
            "rclpy.init" in source.read_text(encoding="utf-8", errors="ignore")
            or "rclcpp::init" in source.read_text(encoding="utf-8", errors="ignore")
            for source in list(package.rglob("*.py")) + list(package.rglob("*.cpp"))
            if not is_excluded(source, root) and not source.is_symlink()
        )
        if not has_node:
            continue
        design_docs = [
            doc for doc in package.glob("docs/*.md")
            if "設計" in doc.name or "design" in doc.name
        ]
        if design_docs:
            continue
        if name in pending:
            report.warnings.append(
                f"src/{name}: ノードを持つが詳細設計書が無い。"
                "allowlist の pending_design_docs に理由を記載済み"
            )
            continue
        report.failures.append(f"src/{name}: ノードを持つが詳細設計書が無い")


def check_rules_sync(root: Path, report: Report) -> None:
    """CLAUDE.md と AGENTS.md の記述方針ブロックが一致するかを検査する.

    Args:
        root (Path): リポジトリルート.
        report (Report): 結果の記録先.
    """
    blocks = {}
    for name in ("CLAUDE.md", "AGENTS.md"):
        path = root / name
        if not path.exists():
            report.failures.append(f"{name}: 存在しない")
            return
        matched = RULES_BLOCK_PATTERN.search(path.read_text(encoding="utf-8"))
        if matched is None:
            report.failures.append(f"{name}: doc-writing-rules マーカーが無い")
            return
        blocks[name] = matched.group(1)
    if blocks["CLAUDE.md"] != blocks["AGENTS.md"]:
        report.failures.append(
            "CLAUDE.md と AGENTS.md の doc-writing-rules ブロックが一致しない。"
            "片方だけを編集していないか確認する"
        )


def check_notes(root: Path, today: str, report: Report) -> None:
    """検討資料の必須ヘッダと決着期限を検査する.

    期限超過は報告にとどめる。期限を延ばす判断は人が行う。

    Args:
        root (Path): リポジトリルート.
        today (str): 基準日 (YYYY-MM-DD).
        report (Report): 結果の記録先.
    """
    notes_dir = root / "docs/notes"
    if not notes_dir.exists():
        return
    for path in sorted(notes_dir.glob("*.md")):
        text = path.read_text(encoding="utf-8")
        header = dict(
            (line.split(":", 1)[0].strip(), line.split(":", 1)[1].strip())
            for line in text.splitlines()
            if ":" in line and line.split(":", 1)[0].strip() in NOTE_HEADER_KEYS
        )
        missing = [key for key in NOTE_HEADER_KEYS if key not in header]
        if missing:
            report.failures.append(
                f"{path.relative_to(root)}: 必須ヘッダが無い -> {', '.join(missing)}"
            )
            continue
        if not re.match(r"^\d{4}-\d{2}-\d{2}$", path.name[:10]):
            report.failures.append(
                f"{path.relative_to(root)}: ファイル名の先頭が YYYY-MM-DD ではない"
            )
        if header["決着期限"] < today:
            report.warnings.append(
                f"{path.relative_to(root)}: 決着期限 {header['決着期限']} を過ぎている。"
                "恒久文書へ反映して削除するか、期限を書き換える"
            )


def check_permanent_docs_do_not_link_notes(docs: list[Path], root: Path,
                                           report: Report) -> None:
    """恒久文書から検討資料へのリンクが無いことを検査する.

    Args:
        docs (list[Path]): 検査対象の文書.
        root (Path): リポジトリルート.
        report (Report): 結果の記録先.
    """
    for path in docs:
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            for target in check_doc_links.LINK_PATTERN.findall(line):
                if "docs/notes/" in target or target.startswith("notes/"):
                    report.failures.append(
                        f"{path.relative_to(root)}:{lineno}: "
                        f"恒久文書から検討資料へリンクしている -> {target}"
                    )


def check_doc_section_references(sources: list[Path], root: Path, report: Report) -> None:
    """ソースから参照されている文書の章・節が実在することを検査する.

    コードコメントが `<文書名>.md 6.3節` の形で設計書の章番号を指す箇所がある。
    文書側で章立てを変更すると参照が無言で壊れるため、見出しの実在を照合する。

    Args:
        sources (list[Path]): 検査対象のソース.
        root (Path): リポジトリルート.
        report (Report): 結果の記録先.
    """
    all_docs = iter_docs(root)
    heading_cache: dict[Path, set[str]] = {}
    resolve_cache: dict[str, list[Path]] = {}

    def resolve(stem: str) -> list[Path]:
        """参照された文書名から候補を引く.

        コードは `robot_console_gui_screen_function_design.md` を
        `screen_function_design.md` のように前方を省略して参照する。末尾一致で
        候補を探し、一意に定まる場合だけ採用する。
        """
        if stem not in resolve_cache:
            target = f"{stem}.md"
            resolve_cache[stem] = [
                path for path in all_docs
                if path.name == target or path.name.endswith(f"_{target}")
            ]
        return resolve_cache[stem]

    def headings(path: Path) -> set[str]:
        if path not in heading_cache:
            found: set[str] = set()
            for line in path.read_text(encoding="utf-8").splitlines():
                match = DOC_HEADING_NUMBER_PATTERN.match(line)
                if match is not None:
                    found.add(match.group(1))
            heading_cache[path] = found
        return heading_cache[path]

    for source in sources:
        text = source.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), 1):
            for stem, number, unit in SOURCE_DOC_SECTION_PATTERN.findall(line):
                candidates = resolve(stem)
                if not candidates:
                    # 文書名そのものの誤りはリンク検査の対象外であり、ここでも報告する。
                    report.failures.append(
                        f"{source.relative_to(root)}:{lineno}: "
                        f"参照先の文書が無い -> {stem}.md"
                    )
                    continue
                if len(candidates) > 1:
                    names = ", ".join(sorted(path.name for path in candidates))
                    report.failures.append(
                        f"{source.relative_to(root)}:{lineno}: "
                        f"参照先の文書名が一意に定まらない -> {stem}.md ({names})"
                    )
                    continue
                path = candidates[0]
                if number not in headings(path):
                    report.failures.append(
                        f"{source.relative_to(root)}:{lineno}: "
                        f"{path.relative_to(root)} に {number}{unit} が無い"
                    )


def run_checks(root: Path, today: str) -> Report:
    """すべての検査を実行する.

    Args:
        root (Path): リポジトリルート.
        today (str): 検討資料の期限判定に使う基準日.

    Returns:
        Report: 検査結果.
    """
    report = Report()
    allowlist = load_allowlist(root)
    docs = iter_docs(root)
    sources = iter_sources(root)

    link_errors = []
    anchor_cache: dict[Path, set[str]] = {}
    for path in check_doc_links.collect_markdown_files(root, []):
        link_errors.extend(check_doc_links.check_file(path, root, anchor_cache))
    report.failures.extend(link_errors)

    check_topics(docs, sources, root, allowlist, report)
    check_parameters(root, allowlist, report)
    check_required_docs(root, allowlist, report)
    check_rules_sync(root, report)
    check_notes(root, today, report)
    check_permanent_docs_do_not_link_notes(docs, root, report)
    check_doc_section_references(sources, root, report)
    return report


def main(argv: list[str]) -> int:
    """エントリポイント.

    Args:
        argv (list[str]): コマンドライン引数（プログラム名を除く）.

    Returns:
        int: 終了コード。指摘があれば 1.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--warn-only", action="store_true",
                        help="指摘があっても終了コード 0 を返す")
    args = parser.parse_args(argv)

    root = repo_root()
    today = subprocess.run(
        ["date", "+%Y-%m-%d"], capture_output=True, text=True, check=True
    ).stdout.strip()
    report = run_checks(root, today)

    for warning in report.warnings:
        print(f"警告: {warning}")
    for failure in report.failures:
        print(f"指摘: {failure}")
    print(f"指摘 {len(report.failures)} 件 / 警告 {len(report.warnings)} 件")

    return 0 if args.warn_only or not report.failures else 1


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).parent))
    sys.exit(main(sys.argv[1:]))
