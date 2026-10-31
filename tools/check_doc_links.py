#!/usr/bin/env python3
"""Markdown 文書の相対リンクとアンカーリンクの切れを検出する.

ドキュメントの移動・改名・削除の後に実行し、参照が追随できているかを確認する。
対象はリポジトリ管理下の Markdown のうち、submodule と生成物を除いたものとする。

使い方:
    python3 tools/check_doc_links.py [対象パス ...]

対象パスを省略した場合はリポジトリ全体を走査する。リンク切れがあれば
標準出力へ一覧を出し、終了コード 1 を返す。
"""

import re
import subprocess
import sys
import unicodedata
from pathlib import Path

# 走査から除外するディレクトリ。submodule と colcon 生成物、キャッシュに加え、
# 外部から取り込んだメーカー資料 (docs/references) も対象外とする。取り込み元の
# 書式をそのまま保持する方針であり、こちらで直すべきリンク切れではない。
EXCLUDED_DIRS = {
    "references",
    "build",
    "install",
    "log",
    ".git",
    "node_modules",
    ".pytest_cache",
    "third_party",
    "FAST_LIO",
    "livox_ros_driver2",
    "livox_sdk2_vendor",
    "ypspur_ros2",
}

# `[text](target)` 形式のリンク。画像記法 `![]()` も同じ書式で解決できるため区別しない。
LINK_PATTERN = re.compile(r"\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")

# 見出し行。ATX 形式のみを対象とする。
HEADING_PATTERN = re.compile(r"^(#{1,6})\s+(.*?)\s*#*$")

# GitHub のアンカー生成で除去される記号。英数字・アンダースコア・ハイフン・
# 各国語文字以外を落とす挙動に合わせる。
ANCHOR_STRIP_PATTERN = re.compile(r"[^\w\- ]", re.UNICODE)


def is_excluded(path: Path, root: Path) -> bool:
    """走査対象外のパスかを判定する.

    Args:
        path (Path): 判定対象のパス.
        root (Path): リポジトリルート.

    Returns:
        bool: 除外対象なら True.
    """
    return any(part in EXCLUDED_DIRS for part in path.relative_to(root).parts)


def collect_markdown_files(root: Path, targets: list[Path]) -> list[Path]:
    """走査対象の Markdown ファイルを収集する.

    Args:
        root (Path): リポジトリルート.
        targets (list[Path]): 走査起点。空ならリポジトリ全体.

    Returns:
        list[Path]: 走査対象の Markdown ファイル一覧.
    """
    files: list[Path] = []
    for target in targets or [root]:
        if target.is_file():
            files.append(target)
            continue
        files.extend(target.rglob("*.md"))

    collected: set[Path] = set()
    for path in files:
        # colcon --symlink-install が作る install/ 配下の symlink は、解決後のパスが
        # 除外対象に見えなくなる。走査で見つけた位置と解決後の位置の両方で判定する。
        if is_excluded(path, root):
            continue
        resolved = path.resolve()
        if is_excluded(resolved, root):
            continue
        # 改名・削除の直後は install/ 配下に解決先を失った symlink が残る。
        if not resolved.is_file():
            continue
        collected.add(resolved)
    return sorted(collected)


def to_anchor(heading_text: str) -> str:
    """見出し文字列から GitHub 互換のアンカー名を生成する.

    Args:
        heading_text (str): 見出しの本文.

    Returns:
        str: アンカー名.
    """
    # 見出し中のリンクは表示テキストのみがアンカーに反映される。
    text = LINK_PATTERN.sub(lambda m: m.group(0)[1 : m.group(0).index("](")], heading_text)
    text = re.sub(r"[`*_~]", "", text)
    text = unicodedata.normalize("NFC", text).strip().lower()
    text = ANCHOR_STRIP_PATTERN.sub("", text)
    return text.replace(" ", "-")


def collect_anchors(path: Path) -> set[str]:
    """Markdown 内の見出しアンカーを収集する.

    同名見出しが複数ある場合、GitHub は 2 つ目以降へ連番を付ける。その規則も再現する。

    Args:
        path (Path): 対象ファイル.

    Returns:
        set[str]: アンカー名の集合.
    """
    anchors: set[str] = set()
    counts: dict[str, int] = {}
    in_code_block = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith("```"):
            in_code_block = not in_code_block
            continue
        if in_code_block:
            continue
        matched = HEADING_PATTERN.match(line)
        if matched is None:
            continue
        base = to_anchor(matched.group(2))
        if not base:
            continue
        index = counts.get(base, 0)
        anchors.add(base if index == 0 else f"{base}-{index}")
        counts[base] = index + 1
    return anchors


def iter_links(path: Path) -> list[tuple[int, str]]:
    """Markdown 内のリンク先を行番号付きで取り出す.

    Args:
        path (Path): 対象ファイル.

    Returns:
        list[tuple[int, str]]: (行番号, リンク先) の一覧.
    """
    links: list[tuple[int, str]] = []
    in_code_block = False
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if line.lstrip().startswith("```"):
            in_code_block = not in_code_block
            continue
        if in_code_block:
            continue
        links.extend((lineno, m.group(1)) for m in LINK_PATTERN.finditer(line))
    return links


def check_file(path: Path, root: Path, anchor_cache: dict[Path, set[str]]) -> list[str]:
    """1 ファイル分のリンクを検査する.

    Args:
        path (Path): 対象ファイル.
        root (Path): リポジトリルート.
        anchor_cache (dict[Path, set[str]]): 解析済みアンカーのキャッシュ.

    Returns:
        list[str]: リンク切れの説明文一覧.
    """
    errors: list[str] = []
    for lineno, target in iter_links(path):
        # 外部 URL、メール、プロトコル相対は検査対象外。
        if re.match(r"^[a-zA-Z][a-zA-Z0-9+.\-]*:", target) or target.startswith("//"):
            continue
        file_part, _, anchor = target.partition("#")
        if file_part:
            resolved = (path.parent / file_part).resolve()
            if not resolved.exists():
                errors.append(f"{path.relative_to(root)}:{lineno}: 参照先が無い -> {target}")
                continue
        else:
            resolved = path
        if not anchor or resolved.suffix != ".md" or not resolved.is_file():
            continue
        if resolved not in anchor_cache:
            anchor_cache[resolved] = collect_anchors(resolved)
        if to_anchor(anchor) not in anchor_cache[resolved]:
            errors.append(f"{path.relative_to(root)}:{lineno}: アンカーが無い -> {target}")
    return errors


def find_root() -> Path:
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


def main(argv: list[str]) -> int:
    """エントリポイント.

    Args:
        argv (list[str]): コマンドライン引数（プログラム名を除く）.

    Returns:
        int: 終了コード。リンク切れがあれば 1.
    """
    root = find_root()
    targets = [Path(a).resolve() for a in argv]
    anchor_cache: dict[Path, set[str]] = {}
    errors: list[str] = []
    files = collect_markdown_files(root, targets)
    for path in files:
        errors.extend(check_file(path, root, anchor_cache))

    print(f"走査したファイル数: {len(files)}")
    if errors:
        print(f"リンク切れ: {len(errors)} 件")
        for error in errors:
            print(f"  {error}")
        return 1
    print("リンク切れ: 0 件")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
