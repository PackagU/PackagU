#!/usr/bin/env python3
"""인쇄용 ArUco 시트(A4 가로, 실제 크기 PDF) + 부착 예시 + 인식 화면 미리보기(합성) 생성. 데스크톱 전용.

    python3 apps/pickup_real/make_marker_sheet.py [--out logs/pickup_real/marker_sheet]

PDF 는 배율 100%(용지 맞춤 끄기)로 인쇄하고, 마커 한 변이 설정값(기본 80 mm)인지 자로 확인한다.
"""
import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from vision import MarkerEstimator, marker_image, render_plane, sheet_texture  # noqa: E402

FONT = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"


def write_pdf(cfg, path):
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.units import mm
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.pdfgen import canvas

    pdfmetrics.registerFont(TTFont("Nanum", FONT))
    m = cfg["marker"]
    W, H = landscape(A4)
    c = canvas.Canvas(str(path), pagesize=(W, H))
    c.setTitle("PackagU parcel pickup marker sheet")
    side = float(m["size_m"]) * 1000.0
    cx0, cy0 = W / 2, H / 2
    for i, x in zip(m["ids"], m["center_x_m"]):
        bits = marker_image(m["dictionary"], i, 6)  # 4x4 + 테두리 1칸 = 6x6 칸, 1칸 = 1픽셀
        cell = side / 6.0
        left = cx0 + (float(x) * 1000.0 - side / 2) * mm
        top = cy0 + side / 2 * mm
        c.setFillColorRGB(0, 0, 0)
        for r in range(6):
            for k in range(6):
                if bits[r, k] < 128:
                    c.rect(left + k * cell * mm, top - (r + 1) * cell * mm, cell * mm, cell * mm, stroke=0, fill=1)
        c.setFont("Nanum", 9)
        c.drawCentredString(left + side / 2 * mm, top - side * mm - 5 * mm, "id %d" % i)
    # 중심선(= 포켓 중심선)
    c.setDash(4, 3)
    c.setStrokeColorRGB(0.85, 0.1, 0.1)
    c.setLineWidth(0.8)
    c.line(cx0, 8 * mm, cx0, H - 8 * mm)
    c.setDash()
    c.setFillColorRGB(0.85, 0.1, 0.1)
    c.setFont("Nanum", 11)
    c.drawCentredString(cx0, 12 * mm, "▼ 이 빨간 선을 팔레트 포켓(포크 구멍) 중앙에 맞춰 수직으로 붙이세요 ▼")
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Nanum", 10)
    c.drawString(12 * mm, H - 12 * mm, "PackagU 택배 집기 표식 · %s · id %s · 마커 한 변 %.0f mm · 중심 간격 %.0f mm"
                 % (m["dictionary"], "/".join(str(i) for i in m["ids"]), side,
                    (m["center_x_m"][1] - m["center_x_m"][0]) * 1000))
    c.drawString(12 * mm, H - 18 * mm, "배율 100%(용지 맞춤 끄기)로 인쇄 → 아래 50 mm 자와 마커 한 변을 자로 확인. 접히거나 구겨지지 않게 평평한 면에.")
    # 50 mm 자
    x0, y0 = 12 * mm, 22 * mm
    c.setLineWidth(1)
    c.line(x0, y0, x0 + 50 * mm, y0)
    for k in range(6):
        c.line(x0 + k * 10 * mm, y0, x0 + k * 10 * mm, y0 + (3 if k % 5 else 5) * mm)
    c.drawString(x0, y0 - 5 * mm, "0")
    c.drawString(x0 + 47 * mm, y0 - 5 * mm, "50 mm")
    c.showPage()
    c.save()


def put_text(img, text, xy, size=22, color=(20, 20, 20), anchor="la"):
    from PIL import Image, ImageDraw, ImageFont

    pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ImageDraw.Draw(pil).text(xy, text, font=ImageFont.truetype(FONT, size), fill=color[::-1], anchor=anchor)
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def front_texture(cfg, dims):
    """택배 앞면 + 팔레트 정면, 1 px = 1 mm. S 원점(시트 중심) 픽셀 위치도 돌려준다."""
    box_w, box_h = dims["box_w"], dims["box_h"]
    pal_w, pal_h, pocket_w, pocket_h, deck = dims["pal_w"], dims["pal_h"], dims["pocket_w"], dims["pocket_h"], dims["deck"]
    Wt, Ht = pal_w + 120, box_h + pal_h + 60
    tex = np.full((Ht, Wt, 3), (205, 205, 200), np.uint8)
    floor_v = Ht - 30
    cx = Wt // 2
    # 팔레트
    cv2.rectangle(tex, (cx - pal_w // 2, floor_v - pal_h), (cx + pal_w // 2, floor_v), (60, 110, 160), -1)
    cv2.rectangle(tex, (cx - pocket_w // 2, floor_v - pocket_h), (cx + pocket_w // 2, floor_v - 0), (35, 35, 35), -1)
    cv2.rectangle(tex, (cx - pal_w // 2, floor_v - pal_h), (cx + pal_w // 2, floor_v - pal_h + deck), (45, 90, 140), -1)
    # 택배 상자
    top = floor_v - pal_h - box_h
    cv2.rectangle(tex, (cx - box_w // 2, top), (cx + box_w // 2, floor_v - pal_h), (95, 150, 200), -1)
    cv2.line(tex, (cx - box_w // 2, top + 40), (cx + box_w // 2, top + 40), (80, 130, 180), 2)
    # 시트(A4 가로) — 상자 앞면 가운데보다 약간 위
    sheet, (ou, ov) = sheet_texture(cfg, px_per_m=1000)
    sh, sw = sheet.shape
    sv0 = top + 30
    su0 = cx - sw // 2
    tex[sv0:sv0 + sh, su0:su0 + sw] = cv2.cvtColor(sheet, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(tex, (su0, sv0), (su0 + sw, sv0 + sh), (120, 120, 120), 1)
    cv2.line(tex, (cx, sv0 + 4), (cx, floor_v), (40, 40, 220), 2)
    origin = (su0 + ou, sv0 + ov)
    return tex, origin, floor_v


def mockup(cfg, dims, est, out_dir):
    tex, origin, floor_v = front_texture(cfg, dims)
    # 왼쪽: 정면 예시(치수 표기)
    scale = 1.4
    front = cv2.resize(tex, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    pad = np.full((front.shape[0] + 160, front.shape[1] + 40, 3), 255, np.uint8)
    pad[90:90 + front.shape[0], 20:20 + front.shape[1]] = front
    put_text(pad, "부착 예시 (정면, 치수는 가정값)", (20, 20), 30)
    put_text(pad, "A4 시트를 택배 앞면에, 빨간 중심선을 포켓 중앙에", (20, 58), 22, (60, 60, 60))
    sheet_center_h = (floor_v - origin[1]) / 1000.0
    put_text(pad, "포켓(포크 구멍) 폭 %d mm · 포크 폭 %d mm [가정]" % (dims["pocket_w"], dims["fork_w"]),
             (20, pad.shape[0] - 52), 20, (60, 60, 60))
    put_text(pad, "시트 중심 높이 ≈ %.2f m (바닥 기준)" % sheet_center_h, (20, pad.shape[0] - 24), 20, (60, 60, 60))
    # 오른쪽: 팔 카메라가 보는 합성 화면 + 인식 결과
    pose = (-0.70, 0.045, math.radians(5.0), 0.55 - sheet_center_h, math.radians(22.0))
    view = render_plane(tex, 1000, origin, est.K, pose, background=170, noise=2.5)
    r = est.estimate(view)
    lines = ["[synthetic preview] arm camera front_view"]
    if r["ok"]:
        c = r["cam"]
        lines += ["cam lateral %+.1f cm  heading %+.1f deg" % (c["y"] * 100, math.degrees(c["heading"])),
                  "distance to sheet %.2f m  reproj %.2f px" % (-c["x"], r["reprojection_px"])]
    shown = est.annotate(view, r, lines)
    panel = np.full((pad.shape[0], 700, 3), 255, np.uint8)
    panel[90:90 + 480, 30:30 + 640] = shown
    put_text(panel, "인식 화면 미리보기 (합성 영상)", (30, 20), 30)
    put_text(panel, "참값: 카메라 좌우 +4.5 cm, 5° 틀어짐, 시트까지 0.70 m", (30, 58), 20, (60, 60, 60))
    if r["ok"]:
        put_text(panel, "인식: 좌우 %+.1f cm, %+.1f°, %.2f m" % (r["cam"]["y"] * 100, math.degrees(r["cam"]["heading"]),
                                                          -r["cam"]["x"]), (30, 590), 22, (0, 120, 0))
    put_text(panel, "초록 사각형 = 인식한 마커, 축 = 시트 자세", (30, 625), 20, (60, 60, 60))
    canvas = np.hstack([pad, panel[:pad.shape[0]] if panel.shape[0] >= pad.shape[0] else
                        np.vstack([panel, np.full((pad.shape[0] - panel.shape[0], 700, 3), 255, np.uint8)])])
    cv2.imwrite(str(out_dir / "marker_mockup.png"), canvas)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    ap.add_argument("--out", default=str(HERE.parents[1] / "logs" / "pickup_real" / "marker_sheet"))
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    write_pdf(cfg, out / "marker_sheet_A4.pdf")
    sheet, _ = sheet_texture(cfg, px_per_m=4000)
    cv2.imwrite(str(out / "marker_sheet_preview.png"), sheet)
    dims = {"box_w": 300, "box_h": 400, "pal_w": 400, "pal_h": 120, "pocket_w": 320, "pocket_h": 100, "deck": 20,
            "fork_w": 140}
    r = mockup(cfg, dims, MarkerEstimator(cfg), out)
    print("wrote", out / "marker_sheet_A4.pdf", out / "marker_mockup.png", "preview_ok", r["ok"])


if __name__ == "__main__":
    main()
