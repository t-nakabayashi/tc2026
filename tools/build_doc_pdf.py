"""Markdown 文書をA5縦・日本語PDFへ変換する。依存: reportlab。

現地でオフライン閲覧する用途を想定する。PDF は生成物であり Git 管理対象にしないため、
既定の出力先は Git 管理外の log/pdf/ とする。

PDF 化は任意の運用のため reportlab は requirements.txt に含めない。使う場合は
`python3 -m pip install reportlab` で個別に導入する。

使い方:
    python3 tools/build_doc_pdf.py [入力Markdown] [-o 出力PDF]
"""
import argparse
from pathlib import Path
from xml.sax.saxutils import escape
import re
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.pagesizes import A5
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = REPO_ROOT / 'docs/走行システムの機能と制約.md'

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('input', nargs='?', type=Path, default=DEFAULT_INPUT,
                    help='変換する Markdown。既定は docs/走行システムの機能と制約.md')
parser.add_argument('-o', '--output', type=Path, default=None,
                    help='出力 PDF。既定は log/pdf/<入力名>.pdf')
args = parser.parse_args()
source = args.input.resolve()
title = source.stem

pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
body = ParagraphStyle('body', fontName='HeiseiKakuGo-W5', fontSize=10, leading=16,
                      wordWrap='CJK', spaceAfter=6)
heading = ParagraphStyle('heading', parent=body, fontSize=14, leading=21,
                         spaceBefore=14, spaceAfter=9, keepWithNext=True, textColor=colors.HexColor('#153f5a'))
small = ParagraphStyle('small', parent=body, fontSize=8, leading=12)
def rich(text):
    text=escape(text)
    text=re.sub(r'\[([^\]]+)\]\((https://[^)]+)\)', r'<link href="\2" color="#12699b">\1</link>',text)
    text=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'\1',text)
    text=re.sub(r'\*\*([^*]+)\*\*',r'<b>\1</b>',text)
    return text.replace('`','')
def footer(canvas, doc):
    canvas.setFont('HeiseiKakuGo-W5',8)
    canvas.drawString(30,18,f'TC2026 / {title}')
    canvas.drawRightString(A5[0]-30,18,str(doc.page))
flow=[];lines=source.read_text(encoding='utf-8').splitlines();i=0
while i<len(lines):
    line=lines[i];i+=1
    if not line.strip():continue
    if line.startswith('|'):
        rows=[line]
        while i<len(lines) and lines[i].startswith('|'):rows.append(lines[i]);i+=1
        cells=[[Paragraph(rich(c.strip()),small) for c in r.strip('|').split('|')] for r in rows if not re.match(r'^\|[\s:|\-]+$',r)]
        t=Table(cells,colWidths=[(A5[0]-60)*.29,(A5[0]-60)*.71],repeatRows=1,hAlign='LEFT')
        t.setStyle(TableStyle([('VALIGN',(0,0),(-1,-1),'TOP'),('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e5eef4')),('GRID',(0,0),(-1,-1),.3,colors.HexColor('#c1ccd3')),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]));flow.extend([t,Spacer(1,8)]);continue
    if line.startswith('#'):
        flow.append(Paragraph(rich(line.lstrip('# ')),heading));continue
    if line.startswith('- '):
        line='・'+line[2:]
    else:
        # Markdownのソフト改行を段落内でつなぎ、紙幅に合わせて折り返す。
        paragraph=[line]
        while i<len(lines) and lines[i].strip() and not lines[i].startswith(('#','|','- ')):
            paragraph.append(lines[i]);i+=1
        line=' '.join(paragraph)
    flow.append(Paragraph(rich(line),body))
output=args.output or REPO_ROOT/'log/pdf'/f'{title}.pdf';output.parent.mkdir(parents=True,exist_ok=True)
SimpleDocTemplate(str(output),pagesize=A5,rightMargin=30,leftMargin=30,topMargin=25,bottomMargin=35,
                  title=title,author='tc2026 repository').build(flow,onFirstPage=footer,onLaterPages=footer)
print(output)
