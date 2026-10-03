from pathlib import Path
import re
from xml.sax.saxutils import escape
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.colors import HexColor
from reportlab.lib.pagesizes import A4
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer

ROOT = Path(__file__).resolve().parents[3]
pdfmetrics.registerFont(TTFont('JP', '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'))
pdfmetrics.registerFont(TTFont('Latin', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
styles = {
    'body': ParagraphStyle('body', fontName='JP', fontSize=10.5, leading=15, spaceAfter=5, wordWrap='CJK'),
    'title': ParagraphStyle('title', fontName='JP', fontSize=21, leading=30, spaceAfter=18, textColor=HexColor('#163b55')),
    'h2': ParagraphStyle('h2', fontName='JP', fontSize=14, leading=20, spaceBefore=15, spaceAfter=8, keepWithNext=True, textColor=HexColor('#163b55')),
    'h3': ParagraphStyle('h3', fontName='JP', fontSize=12, leading=18, spaceBefore=10, spaceAfter=7, keepWithNext=True, textColor=HexColor('#963b27')),
}
def markup(text):
    text = text.replace('`', '')
    return ''.join('<font name="Latin">'+escape(x)+'</font>' if all(ord(c) < 0x400 or 0x2190 <= ord(c) <= 0x21ff for c in x) else escape(x)
                   for x in re.split(r'([\x20-\x7e\u00a0-\u03ff\u2190-\u21ff]+)', text) if x)
story=[]
for block in (Path(__file__).parent/'README.md').read_text().split('\n\n'):
    if not block.strip(): continue
    if block.startswith('# '): key='title'; text=block[2:]
    elif block.startswith('## '): key='h2'; text=block[3:]
    elif block.startswith('### '): key='h3'; text=block[4:]
    else: key='body'; text=block
    if key=='body' and re.match(r'(?:- |\d+\. )', text):
        items=re.split(r'\n(?=(?:- |\d+\. ))', text)
    else: items=[text]
    for item in items:
        story.append(Paragraph(markup(item.replace('\n', '')),styles[key]))

def page(canvas, doc):
    canvas.setStrokeColor(HexColor('#c8d8e2'))
    canvas.line(36, 32, A4[0]-36, 32)
    canvas.setFont('Latin',8)
    canvas.setFillColor(HexColor('#526675'))
    canvas.drawString(36,20,'Localization review | 2026-09-27 | 6b34e8f')
    canvas.drawRightString(A4[0]-36,20,str(doc.page))

out=ROOT/'output/pdf/localization_review_20260927.pdf'
SimpleDocTemplate(str(out),pagesize=A4,rightMargin=36,leftMargin=36,topMargin=36,bottomMargin=44,
                  title='自律走行の位置推定評価',author='System review').build(story,onFirstPage=page,onLaterPages=page)
print(out)
