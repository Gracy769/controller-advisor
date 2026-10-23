"""Render the editable Markdown research report as a readable PDF."""

from pathlib import Path
import html
import re

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, HRFlowable


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "report.md"
TARGET = ROOT / "output" / "pdf" / "controller_advisor_research_report.pdf"
TARGET.parent.mkdir(parents=True, exist_ok=True)


def markup(raw: str) -> str:
    value = html.escape(raw)
    value = re.sub(r"\*\*(.*?)\*\*", r"<b>\1</b>", value)
    value = re.sub(r"\*(.*?)\*", r"<i>\1</i>", value)
    return value


styles = getSampleStyleSheet()
title = ParagraphStyle("ReportTitle", parent=styles["Title"], fontName="Helvetica-Bold",
                       fontSize=17, leading=21, textColor=colors.HexColor("#17365D"),
                       alignment=TA_LEFT, spaceAfter=12)
subtitle = ParagraphStyle("ReportSubtitle", parent=styles["Normal"], fontSize=9,
                          leading=13, textColor=colors.HexColor("#52667C"), spaceAfter=10)
heading = ParagraphStyle("ReportHeading", parent=styles["Heading2"], fontName="Helvetica-Bold",
                         fontSize=11, leading=14, textColor=colors.HexColor("#17365D"),
                         spaceBefore=9, spaceAfter=4, keepWithNext=True)
body = ParagraphStyle("ReportBody", parent=styles["BodyText"], fontName="Helvetica",
                      fontSize=8.8, leading=12.8, spaceAfter=5.5, alignment=TA_LEFT)
bullet = ParagraphStyle("ReportBullet", parent=body, leftIndent=13, firstLineIndent=-8,
                        spaceAfter=3)
reference = ParagraphStyle("ReportReference", parent=body, leftIndent=18,
                           firstLineIndent=-18, fontSize=8.2, leading=11, spaceAfter=4)


def decorate_page(canvas, doc):
    canvas.saveState()
    width, height = A4
    canvas.setStrokeColor(colors.HexColor("#CBD5E1"))
    canvas.line(19 * mm, height - 19 * mm, width - 19 * mm, height - 19 * mm)
    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(colors.HexColor("#52667C"))
    canvas.drawString(19 * mm, 14 * mm, "Controller Advisor | BTech control systems research proposal")
    canvas.drawRightString(width - 19 * mm, 14 * mm, f"{doc.page}")
    canvas.restoreState()


story = []
lines = SOURCE.read_text(encoding="utf-8").splitlines()
buffer = []


def flush():
    if buffer:
        story.append(Paragraph(markup(" ".join(buffer)), body))
        buffer.clear()


for line in lines:
    stripped = line.strip()
    if not stripped:
        flush()
        continue
    if stripped.startswith("# "):
        flush()
        story.append(Paragraph(markup(stripped[2:]), title))
        story.append(HRFlowable(width="100%", thickness=1.3,
                                color=colors.HexColor("#2C6DA4"), spaceAfter=9))
    elif stripped.startswith("## "):
        flush()
        story.append(Paragraph(markup(stripped[3:]), heading))
    elif stripped.startswith("- "):
        flush()
        story.append(Paragraph("&#8226; " + markup(stripped[2:]), bullet))
    elif re.match(r"^\d+\. ", stripped):
        flush()
        story.append(Paragraph(markup(stripped), bullet))
    elif re.match(r"^\[\d+\] ", stripped):
        flush()
        story.append(Paragraph(markup(stripped), reference))
    elif stripped.startswith("**Simulation-only"):
        flush()
        story.append(Paragraph(markup(stripped), subtitle))
    else:
        buffer.append(stripped)
flush()

doc = SimpleDocTemplate(str(TARGET), pagesize=A4, rightMargin=19 * mm,
                        leftMargin=19 * mm, topMargin=24 * mm, bottomMargin=22 * mm,
                        title="Requirement-Based Controller Recommendation for Higher-Order SISO Systems",
                        author="BTech Control Systems Research Project")
doc.build(story, onFirstPage=decorate_page, onLaterPages=decorate_page)
print(TARGET)
