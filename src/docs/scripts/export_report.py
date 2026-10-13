#!/usr/bin/env python3
"""Export the Vietnamese Markdown report to PDF with ReportLab and system fonts."""

from html import escape
from pathlib import Path
import re

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image, KeepTogether, Paragraph, Preformatted, SimpleDocTemplate,
    Spacer, Table, TableStyle,
)


ROOT = Path(__file__).resolve().parents[1]
WIDTH = A4[0] - 88
BLUE = colors.HexColor('#1d4e78')


def register_fonts():
    base = Path('/usr/share/fonts/truetype/dejavu')
    mono = Path('/usr/share/fonts/truetype/liberation2')
    for name, path in [
        ('Body', base / 'DejaVuSans.ttf'),
        ('BodyBold', base / 'DejaVuSans-Bold.ttf'),
        ('BodyItalic', base / 'DejaVuSans-Oblique.ttf'),
        ('BodyBoldItalic', base / 'DejaVuSans-BoldOblique.ttf'),
        ('Code', mono / 'LiberationMono-Regular.ttf'),
    ]:
        pdfmetrics.registerFont(TTFont(name, str(path)))
    pdfmetrics.registerFontFamily(
        'Body', normal='Body', bold='BodyBold', italic='BodyItalic',
        boldItalic='BodyBoldItalic')


def inline(text):
    """Render only the inline Markdown used by this report, escaping source HTML."""
    pattern = r'`([^`]+)`|\[([^\]]+)\]\(([^)]+)\)|\*\*([^*]+)\*\*|\*([^*]+)\*'
    output = []
    start = 0
    for match in re.finditer(pattern, text):
        output.append(escape(text[start:match.start()]))
        code, label, url, bold, italic = match.groups()
        if code is not None:
            output.append('<font name="Code">' + escape(code) + '</font>')
        elif label is not None:
            output.append('<link href="' + escape(url, quote=True) +
                          '" color="#1d4e78">' + escape(label) + '</link>')
        elif bold is not None:
            output.append('<b>' + inline(bold) + '</b>')
        else:
            output.append('<i>' + inline(italic) + '</i>')
        start = match.end()
    output.append(escape(text[start:]))
    return ''.join(output)


class Report(SimpleDocTemplate):
    def afterFlowable(self, flowable):
        if isinstance(flowable, Paragraph) and flowable.style.name in ('H1', 'H2'):
            title = flowable.getPlainText()
            anchor = 'section-' + str(self.seq.nextf('section'))
            self.canv.bookmarkPage(anchor)
            self.canv.addOutlineEntry(title, anchor,
                                      level=0 if flowable.style.name == 'H1' else 1)


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor('#cbd5e1'))
    canvas.line(44, 36, A4[0] - 44, 36)
    canvas.setFont('Body', 8)
    canvas.setFillColor(colors.HexColor('#64748b'))
    canvas.drawString(44, 23, 'TurtleBot4 • ROS 2 Humble • Cập nhật ' + doc.update_date)
    canvas.drawRightString(A4[0] - 44, 23, str(doc.page))
    canvas.restoreState()


def export():
    register_fonts()
    body = ParagraphStyle(
        'Body', fontName='Body', fontSize=9.4, leading=14.3,
        spaceAfter=8, splitLongWords=True)
    cell = ParagraphStyle('Cell', parent=body, fontSize=8, leading=11.7, spaceAfter=0)
    caption = ParagraphStyle(
        'Caption', parent=body, fontSize=8.5, leading=12,
        textColor=colors.HexColor('#475569'), spaceAfter=12)
    headings = {
        1: ParagraphStyle('Title', parent=body, fontName='BodyBold', fontSize=19,
                          leading=25, textColor=BLUE, spaceAfter=16, keepWithNext=True),
        2: ParagraphStyle('H1', parent=body, fontName='BodyBold', fontSize=14,
                          leading=19, textColor=BLUE, spaceBefore=14,
                          spaceAfter=9, keepWithNext=True),
        3: ParagraphStyle('H2', parent=body, fontName='BodyBold', fontSize=11,
                          leading=15, textColor=BLUE, spaceBefore=10,
                          spaceAfter=7, keepWithNext=True),
    }
    code_style = ParagraphStyle('CodeBlock', fontName='Code', fontSize=7.4, leading=10.5)
    lines = (ROOT / 'bao_cao_turtlebot4.md').read_text().splitlines()
    story = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue
        if line.startswith('```'):
            i += 1
            block = []
            while i < len(lines) and not lines[i].strip().startswith('```'):
                block.append(lines[i])
                i += 1
            code = Preformatted('\n'.join(block), code_style)
            box = Table([[code]], colWidths=[WIDTH], hAlign='LEFT')
            box.setStyle(TableStyle([
                ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor('#f1f5f9')),
                ('LEFTPADDING', (0, 0), (-1, -1), 10),
                ('RIGHTPADDING', (0, 0), (-1, -1), 10),
                ('TOPPADDING', (0, 0), (-1, -1), 8),
                ('BOTTOMPADDING', (0, 0), (-1, -1), 8),
            ]))
            story.extend([box, Spacer(1, 10)])
            i += 1
            continue
        heading = re.match(r'^(#{1,3}) (.+)$', line)
        if heading:
            story.append(Paragraph(inline(heading[2]), headings[len(heading[1])]))
            i += 1
            continue
        if line.startswith('|'):
            rows = []
            while i < len(lines) and lines[i].strip().startswith('|'):
                values = [v.strip() for v in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-{3,}:?', v) for v in values):
                    rows.append(values)
                i += 1
            columns = len(rows[0])
            if columns == 2:
                fractions = [0.42, 0.58]
            elif columns == 3:
                fractions = [0.29, 0.27, 0.44]
            elif columns == 4:
                fractions = [0.15, 0.28, 0.28, 0.29]
            else:
                fractions = [0.15, 0.16, 0.24, 0.21, 0.24]
            rendered = []
            for n, row in enumerate(rows):
                rendered.append([Paragraph(
                    '<b>' + inline(value) + '</b>' if n == 0 else inline(value), cell)
                    for value in row])
            table = Table(rendered, colWidths=[WIDTH * f for f in fractions],
                          repeatRows=1, hAlign='LEFT')
            table.setStyle(TableStyle([
                ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#e2edf7')),
                ('ROWBACKGROUNDS', (0, 1), (-1, -1),
                 [colors.white, colors.HexColor('#f8fafc')]),
                ('GRID', (0, 0), (-1, -1), 0.35, colors.HexColor('#cbd5e1')),
                ('VALIGN', (0, 0), (-1, -1), 'TOP'),
                ('LEFTPADDING', (0, 0), (-1, -1), 6),
                ('RIGHTPADDING', (0, 0), (-1, -1), 6),
                ('TOPPADDING', (0, 0), (-1, -1), 6),
                ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ]))
            story.extend([table, Spacer(1, 10)])
            continue
        picture = re.fullmatch(r'!\[([^\]]*)\]\(([^)]+)\)', line)
        if picture:
            image = Image(str(ROOT / picture[2]))
            scale = min(WIDTH / image.imageWidth, 355 / image.imageHeight)
            image.drawWidth = image.imageWidth * scale
            image.drawHeight = image.imageHeight * scale
            image.hAlign = 'CENTER'
            group = [image, Spacer(1, 6)]
            i += 1
            while i < len(lines) and not lines[i].strip():
                i += 1
            if i < len(lines) and lines[i].strip().startswith('*Hình'):
                group.append(Paragraph(inline(lines[i].strip()), caption))
                i += 1
            story.append(KeepTogether(group))
            continue
        bullet = re.match(r'^(\d+\.|-) (.+)$', line)
        if bullet:
            content = bullet[2]
            i += 1
            while i < len(lines) and lines[i].startswith('   '):
                content += ' ' + lines[i].strip()
                i += 1
            style = ParagraphStyle('List', parent=body, leftIndent=14,
                                   firstLineIndent=-12, spaceAfter=5)
            marker = '•' if bullet[1] == '-' else bullet[1]
            story.append(Paragraph(escape(marker) + ' ' + inline(content), style))
            continue
        paragraph = [line]
        i += 1
        while i < len(lines) and lines[i].strip() and not re.match(
                r'^(#|\||```|!\[|\d+\. |\- )', lines[i].strip()):
            paragraph.append(lines[i].strip())
            i += 1
        story.append(Paragraph(inline(' '.join(paragraph)), body))

    output = ROOT / 'bao_cao_turtlebot4.pdf'
    document = Report(str(output), pagesize=A4, leftMargin=44, rightMargin=44,
                      topMargin=42, bottomMargin=48,
                      title='Mô phỏng, xây dựng bản đồ và điều hướng TurtleBot4',
                      author='TurtleBot4 workspace', pageCompression=1)
    updated = re.search(r'Cập nhật ngày (\d{2}/\d{2}/\d{4})', '\n'.join(lines))
    document.update_date = updated[1] if updated else '—'
    document.build(story, onFirstPage=footer, onLaterPages=footer)
    print(output)


if __name__ == '__main__':
    export()
