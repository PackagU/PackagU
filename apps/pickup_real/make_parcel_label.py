#!/usr/bin/env python3
"""송장 라벨 출력용 PNG — parcel_target.read_labels 가 읽는 굵은 블록 숫자(글자 폭:높이 7:12, 획 = 높이 0.15).
A4 세로 300 dpi 한 장에 라벨 2개. 프린터에서 "실제 크기(100 %)"로 출력해 상자 로봇 쪽 면(정면 자세 카메라에 보이는
윗부분)에 붙인다. 라벨 둘레에 골판지가 보여야 라벨로 인식한다(흰 종이를 상자보다 크게 붙이지 않는다).

    python3 apps/pickup_real/make_parcel_label.py 402 310 --out logs/labels
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from parcel_target import DIGIT_SEGMENTS

DPI = 300
MM = DPI / 25.4


def strokes(d, w, h, t):
    seg = {'a': (0, h / 2 - t / 2, w, t), 'd': (0, -h / 2 + t / 2, w, t), 'g': (0, 0, w, t),
           'f': (-w / 2 + t / 2, h / 4, t, h / 2), 'b': (w / 2 - t / 2, h / 4, t, h / 2),
           'e': (-w / 2 + t / 2, -h / 4, t, h / 2), 'c': (w / 2 - t / 2, -h / 4, t, h / 2)}
    return [seg[s] for s in DIGIT_SEGMENTS[d]]


def draw_label(img, x0, y0, text, w_mm=168.0):
    h_mm = w_mm * 18 / 28                     # 라벨 비율 28:18 (판독기 기본값)
    gh = h_mm * 0.667                          # 글자 높이(계획서 12/18)
    gw, t = gh * 7 / 12, gh * 0.15
    W, H = int(w_mm * MM), int(h_mm * MM)
    cv2.rectangle(img, (x0, y0), (x0 + W, y0 + H), 255, -1)
    cv2.rectangle(img, (x0, y0), (x0 + W, y0 + H), 0, 2)      # 자를 선(얇게)
    pitch = gw * 1.45
    for i, ch in enumerate(text):
        cx = x0 + W / 2 + (i - 1) * pitch * MM
        cy = y0 + H / 2
        for u, v, du, dv in strokes(ch, gw, gh, t):
            a = (int(cx + (u - du / 2) * MM), int(cy - (v + dv / 2) * MM))
            b = (int(cx + (u + du / 2) * MM), int(cy - (v - dv / 2) * MM))
            cv2.rectangle(img, a, b, 0, -1)
    cv2.putText(img, "PackagU %s  (print 100%%, %.0f x %.0f mm)" % (text, w_mm, h_mm), (x0, y0 + H + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 120, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("labels", nargs="+")
    ap.add_argument("--out", default="logs/labels")
    ap.add_argument("--width-mm", type=float, default=168.0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for i in range(0, len(a.labels), 2):
        page = np.full((int(297 * MM), int(210 * MM)), 255, np.uint8)
        for k, text in enumerate(a.labels[i:i + 2]):
            if len(text) != 3 or not text.isdigit():
                raise SystemExit("세 자리 숫자만: %s" % text)
            draw_label(page, int(21 * MM), int((20 + k * 140) * MM), text, a.width_mm)
        path = out / ("labels_%s.png" % "_".join(a.labels[i:i + 2]))
        cv2.imwrite(str(path), page)
        print(path)


if __name__ == "__main__":
    main()
