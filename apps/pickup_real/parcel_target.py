"""집을 팔레트 고르기 — 팔레트가 여러 개 보일 때 "택배가 올라간 쪽"(지금) 또는 "송장 숫자가 맞는 쪽"(시연)을 고른다.

입력은 기존 검정 표식 쌍 가설(autonomy.tape_hypotheses)과 같은 프레임의 컬러 영상이다. 처음 한 번 이 근거로 목표를
정하고, 그 뒤에는 기존 목표 연속성(실제 이동 피드백으로 예측한 자세와 일치하는 쌍만 인정)으로 같은 팔레트를 따라간다.
숫자 모드에서는 따라가는 동안 같은 팔레트 위에서 다른 숫자가 읽히면 그 프레임을 거부한다.

택배(상자) 판정: 표식 쌍 위쪽(윗판 너머) 띠에서 골판지 색(주황 갈색, 채도 높음) 화소 비율.
  2026-10-31 실측: 상자 H 4~14·S 90~140·V 55~120, 나무 바닥 S 15~30·V 150~200, 검정 윗판 V < 60.
송장 숫자: 골판지 위 흰 라벨 → 네 모서리 원근 보정 → 굵은 블록 숫자 3개를 7획 자리 채움으로 판독
  (7획 블록 숫자 판독기(read_label) 과 같은 방식을 Jetson 호스트 Python 3.8 용으로 옮김. 라벨 글자는
  계획서 규격의 굵은 블록 숫자 — 글자 폭:높이 7:12, 획 = 높이의 0.15 — 여야 한다. 일반 글꼴은 판독을 보장하지 않는다).
장치 접근이나 이동 명령은 하지 않는다.
"""
import math

import cv2
import numpy as np

CARDBOARD_HSV = [((0, 60, 35), (25, 255, 230)), ((170, 60, 35), (180, 255, 230))]
LABEL_WHITE_HSV = [((0, 0, 165), (180, 60, 255))]
GLYPH_DARK_V = 95
DIGIT_SEGMENTS = {'0': 'abcdef', '1': 'bc', '2': 'abged', '3': 'abgcd', '4': 'fgbc', '5': 'afgcd',
                  '6': 'afgedc', '7': 'abc', '8': 'abcdefg', '9': 'abcfgd'}
PATTERN = {''.join(sorted(v)): d for d, v in DIGIT_SEGMENTS.items()}
REGION = {'a': (.28, .72, .0, .13), 'd': (.28, .72, .87, 1.0), 'g': (.28, .72, .44, .56),
          'f': (.0, .16, .14, .38), 'b': (.84, 1.0, .14, .38), 'e': (.0, .16, .62, .86), 'c': (.84, 1.0, .62, .86)}
TEMPLATE_SIZE = (24, 36)
ON_FILL = 0.3

DEFAULTS = {"min_parcel_frac": 0.06, "min_parcel_px": 120, "max_other_frac": 0.02, "other_ratio": 4.0,
            "label_aspect": 28.0 / 18.0, "min_label_conf": 0.15}


def hsv_mask(bgr, ranges):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return m


def pair_box(left, right, shape):
    """표식 쌍 위쪽 띠(상자가 보일 자리): 두 표식 사이 가운데 80 % 열, 표식 줄 위로 쌍 간격 1.2 배까지."""
    h, w = shape[:2]
    (ul, vl), (ur, vr) = left, right
    span = max(1.0, ur - ul)
    x0, x1 = int(max(0, ul + 0.1 * span)), int(min(w, ur - 0.1 * span))
    top_line = min(vl, vr)
    y1 = int(max(0, top_line - 0.15 * span))
    y0 = int(max(0, top_line - 1.2 * span))
    return x0, y0, x1, y1


def parcel_score(bgr, left, right, board=None):
    board = hsv_mask(bgr, CARDBOARD_HSV) if board is None else board
    x0, y0, x1, y1 = pair_box(left, right, bgr.shape)
    if x1 - x0 < 4 or y1 - y0 < 2:
        return {"frac": 0.0, "px": 0, "box": [x0, y0, x1, y1], "why": "no image area above the markers"}
    roi = board[y0:y1, x0:x1]
    n = int(np.count_nonzero(roi))
    return {"frac": round(n / float(roi.size), 4), "px": n, "box": [x0, y0, x1, y1]}


# ---------------------------------------------------------------- labels
def _normalise_glyph(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    crop = mask[ys.min():ys.max() + 1, xs.min():xs.max() + 1].astype(np.float32) / 255.0
    return cv2.resize(crop, TEMPLATE_SIZE, interpolation=cv2.INTER_AREA)


SEG_SHIFT = {'g': (0.0, .12), 'e': (.14, 0.0), 'f': (.14, 0.0), 'b': (.14, 0.0), 'c': (.14, 0.0)}


def classify_glyph(mask, aspect=None):
    """(숫자 또는 None, 확신도 0~1). 7획 자리별 채움으로 판정, 가는 막대(w/h<0.35)는 '1'."""
    g = _normalise_glyph(mask)
    if g is None:
        return None, 0.0
    if aspect is not None and aspect < .35:
        return '1', round(max(0.0, min(1.0, (float(g.mean()) - .5) * 2)), 3)
    w, h = TEMPLATE_SIZE
    fills = {}
    for seg, (x0, x1, y0, y1) in REGION.items():
        # 손으로 그린 블록 숫자: 가운데 가로획이 위아래로, 옆 세로획이 안쪽으로 조금 비켜도 같은 획으로 본다
        # (획 자리를 SEG_SHIFT 만큼 옮겨 가며 가장 잘 채워진 값). 192호 2026-11-01 매직 라벨 "4"·"2".
        dx, dy = SEG_SHIFT.get(seg, (0.0, 0.0))
        best = 0.0
        for sx in np.linspace(-dx, dx, 3) if dx else (0.0,):
            for sy in np.linspace(-dy, dy, 5) if dy else (0.0,):
                ax0, ax1 = min(max(x0 + sx, 0.0), 1.0), min(max(x1 + sx, 0.0), 1.0)
                ay0, ay1 = min(max(y0 + sy, 0.0), 1.0), min(max(y1 + sy, 0.0), 1.0)
                ya, yb = int(round(ay0 * h)), max(int(round(ay1 * h)), int(round(ay0 * h)) + 1)
                xa, xb = int(round(ax0 * w)), max(int(round(ax1 * w)), int(round(ax0 * w)) + 1)
                cell = g[ya:yb, xa:xb]
                if cell.size:
                    best = max(best, float(cell.mean()))
        fills[seg] = best
    on = ''.join(sorted(s for s, f in fills.items() if f >= ON_FILL))
    conf = min(((f - ON_FILL) / (1 - ON_FILL) if f >= ON_FILL else (ON_FILL - f) / ON_FILL) for f in fills.values())
    return PATTERN.get(on), round(conf, 3)


def _order(pts):
    pts = np.asarray(pts, np.float32).reshape(4, 2)
    s, d = pts.sum(1), np.diff(pts, axis=1).ravel()
    return np.float32([pts[np.argmin(s)], pts[np.argmin(d)], pts[np.argmax(s)], pts[np.argmax(d)]])


def read_labels(bgr, board=None, aspect=DEFAULTS["label_aspect"], warp_h=180, min_area=60, min_conf=0.15):
    """화면의 모든 송장 라벨 → [{"text" 또는 None, "u", "v", "bbox", "why"}]. 골판지 위 흰 사각형만 라벨로 본다."""
    white = cv2.morphologyEx(hsv_mask(bgr, LABEL_WHITE_HSV), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    board = hsv_mask(bgr, CARDBOARD_HSV) if board is None else board
    n, lab, stats, cen = cv2.connectedComponentsWithStats(white)
    H, W = white.shape
    ring_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area or w < 12 or h < 8 or w > .6 * W or h > .6 * H:
            continue
        xa, ya, xb, yb = max(0, x - 4), max(0, y - 4), min(W, x + w + 4), min(H, y + h + 4)
        blob = (lab[ya:yb, xa:xb] == i).astype(np.uint8)
        ring = cv2.dilate(blob, ring_kernel) & (1 - blob)
        if not ring.any() or (board[ya:yb, xa:xb][ring > 0] > 0).mean() < .15:
            continue   # 흰 테이프·벽 등 골판지 위가 아닌 흰 면
        contours, _ = cv2.findContours((lab == i).astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = max(contours, key=cv2.contourArea)
        approx = cv2.approxPolyDP(c, .03 * cv2.arcLength(c, True), True)
        quad = _order(approx if len(approx) == 4 else cv2.boxPoints(cv2.minAreaRect(c)))
        ww = int(round(warp_h * aspect))
        M = cv2.getPerspectiveTransform(quad, np.float32([[0, 0], [ww - 1, 0], [ww - 1, warp_h - 1], [0, warp_h - 1]]))
        warped = cv2.warpPerspective(bgr, M, (ww, warp_h), flags=cv2.INTER_LINEAR)
        v = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)[:, :, 2]
        dark = (v < GLYPH_DARK_V).astype(np.uint8) * 255
        m = int(.04 * warp_h)
        dark[:m, :], dark[-m:, :], dark[:, :m], dark[:, -m:] = 0, 0, 0, 0
        k, gl, gs, gc = cv2.connectedComponentsWithStats(dark, connectivity=8)
        glyphs = []
        for j in range(1, k):
            gx, gy, gw, gh, ga = gs[j]
            if gh < .4 * warp_h or ga < 30:
                continue
            digit, conf = classify_glyph((gl == j).astype(np.uint8) * 255, aspect=gw / max(gh, 1))
            glyphs.append((float(gc[j][0]), digit, conf))
        glyphs.sort()
        rec = {"u": float(cen[i][0]), "v": float(cen[i][1]), "bbox": [int(x), int(y), int(w), int(h)], "text": None}
        if len(glyphs) != 3:
            rec["why"] = "%d glyphs (need 3)" % len(glyphs)
        elif any(d is None or c < min_conf for _, d, c in glyphs):
            rec["why"] = "a glyph is unclear"
        else:
            rec["text"] = "".join(d for _, d, _ in glyphs)
            rec["confidence"] = min(c for _, _, c in glyphs)
        out.append(rec)
    if not any(r.get("text") for r in out):
        # 흰 사각형으로 못 읽으면 글자 묶음으로 다시(종이가 상자보다 크거나 밝은 물체와 붙은 경우)
        out.extend(read_glyph_rows(bgr))
    return out


def read_glyph_rows(bgr, min_h=18, max_h=260, min_conf=0.12):
    """흰 종이 경계 없이 글자 묶음으로 라벨 찾기(보조): 밝은 바탕 위 검정 글자 3개가 한 줄로 나란한 것.
    라벨 종이가 상자보다 크거나 주변의 밝은 물체와 붙어 흰 사각형이 안 갈라질 때(192호 2026-11-01) 쓴다."""
    v = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
    dark = (v < GLYPH_DARK_V).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(dark, connectivity=8)
    H, W = dark.shape
    cand = []
    for j in range(1, n):
        x, y, w, h, a = st[j]
        if not (min_h <= h <= max_h) or h < .9 * w or w < 3 or a < 25 or x == 0 or y == 0 or x + w >= W or y + h >= H:
            continue
        m = int(round(.35 * h))
        xa, ya, xb, yb = max(0, x - m), max(0, y - m), min(W, x + w + m), min(H, y + h + m)
        around = v[ya:yb, xa:xb][lab[ya:yb, xa:xb] != j]
        if around.size == 0 or np.median(around) < 150:
            continue   # 밝은 종이 위가 아님
        cand.append((x, y, w, h, j))
    cand.sort()
    out, used = [], set()
    for i, a in enumerate(cand):
        if a[4] in used:
            continue
        row = [a]
        for b in cand[i + 1:]:
            last = row[-1]
            if (abs(b[3] - a[3]) <= .3 * a[3] and abs((b[1] + b[3] / 2) - (a[1] + a[3] / 2)) <= .3 * a[3]
                    and 0 <= b[0] - (last[0] + last[2]) <= 1.0 * a[3]):
                row.append(b)
            if len(row) == 3:
                break
        if len(row) != 3:
            continue
        digits, confs = [], []
        for x, y, w, h, j in row:
            d, c = classify_glyph((lab[y:y + h, x:x + w] == j).astype(np.uint8) * 255, aspect=w / max(h, 1))
            digits.append(d)
            confs.append(c)
        if any(d is None for d in digits) or min(confs) < min_conf:
            continue
        used.update(r[4] for r in row)
        x0, y0 = row[0][0], min(r[1] for r in row)
        x1, y1 = row[-1][0] + row[-1][2], max(r[1] + r[3] for r in row)
        out.append({"text": "".join(digits), "u": (x0 + x1) / 2.0, "v": (y0 + y1) / 2.0,
                    "bbox": [int(x0), int(y0), int(x1 - x0), int(y1 - y0)], "confidence": min(confs), "mode": "glyph_row"})
    return out


def label_on_pair(label, left, right):
    """라벨이 이 표식 쌍 팔레트 위의 택배에 붙었나: 라벨 가운데 열이 두 표식 사이(±10 %), 표식 줄보다 위."""
    span = max(1.0, right[0] - left[0])
    return left[0] - 0.1 * span <= label["u"] <= right[0] + 0.1 * span and label["v"] < max(left[1], right[1])


# ---------------------------------------------------------------- per-frame evidence and choice
def scene_evidence(bgr, pairs, want_labels=False):
    """pairs: [(left_uv, right_uv)] → 프레임 근거 {"pairs": [...], "labels": [...]} (카메라 서버가 프레임마다 붙인다)."""
    board = hsv_mask(bgr, CARDBOARD_HSV)
    labels = read_labels(bgr, board) if want_labels else []
    out = []
    for left, right in pairs:
        rec = {"left": [round(float(left[0]), 2), round(float(left[1]), 2)],
               "right": [round(float(right[0]), 2), round(float(right[1]), 2)]}
        rec["parcel"] = parcel_score(bgr, left, right, board)
        rec["labels"] = [lb["text"] for lb in labels if lb.get("text") and label_on_pair(lb, left, right)]
        out.append(rec)
    return {"pairs": out, "labels": [{k: lb.get(k) for k in ("text", "u", "v", "why")} for lb in labels]}


def _match(evidence, hyp):
    """가설(표식 쌍 화소)에 해당하는 근거 줄."""
    t = hyp.get("tape") or {}
    for rec in evidence.get("pairs", []):
        if (abs(rec["left"][0] - t.get("left", [1e9])[0]) < 0.6 and abs(rec["right"][0] - t.get("right", [1e9])[0]) < 0.6
                and abs(rec["left"][1] - t["left"][1]) < 0.6):
            return rec
    return None


def choose(hypotheses, evidence, context, limits=None):
    """hypotheses 중 목표 하나. 반환 (가설 또는 None, 이유, 근거 요약).
    context: {"parcel": true} → 상자 있는 쪽이 하나여야 한다. {"label": "402"} → 그 숫자가 읽힌 쪽이 하나여야 한다."""
    lim = dict(DEFAULTS, **(limits or {}))
    if not evidence:
        return None, "no scene evidence for target choice", None
    rows = []
    for h in hypotheses:
        rec = _match(evidence, h)
        if rec is None:
            return None, "scene evidence does not match marker pairs", None
        rows.append((h, rec))
    summary = [{"left_u": r["left"][0], "right_u": r["right"][0], "parcel_frac": r["parcel"]["frac"],
                "labels": r["labels"]} for _, r in rows]
    if context.get("label"):
        want = str(context["label"])
        hits = [(h, r) for h, r in rows if want in r["labels"]]
        if len(hits) != 1:
            seen = sorted({t for _, r in rows for t in r["labels"]})
            return None, "target label %s on %d pallets (read: %s)" % (want, len(hits), ",".join(seen) or "none"), summary
        h, r = hits[0]
        return dict(h, target_evidence={"label": want, "parcel_frac": r["parcel"]["frac"]}), None, summary
    if context.get("parcel"):
        rows.sort(key=lambda hr: -hr[1]["parcel"]["frac"])
        best, rest = rows[0], rows[1:]
        bf, bp = best[1]["parcel"]["frac"], best[1]["parcel"]["px"]
        if bf < lim["min_parcel_frac"] or bp < lim["min_parcel_px"]:
            return None, "no parcel above any pallet (best %.3f, %d px)" % (bf, bp), summary
        for _, r in rest:
            of = r["parcel"]["frac"]
            if of > lim["max_other_frac"] and of * lim["other_ratio"] > bf:
                return None, "parcel colour above more than one pallet (%.3f vs %.3f)" % (bf, of), summary
        return dict(best[0], target_evidence={"parcel_frac": bf, "parcel_px": bp}), None, summary
    return None, "target context needs parcel or label", summary


def label_conflict(hyp, evidence, want):
    """따라가는 팔레트 위에서 다른 숫자가 읽혔으면 그 숫자(아니면 None). 안 읽히면 None(가까우면 라벨이 화면 밖)."""
    rec = _match(evidence or {}, hyp) if evidence else None
    if not rec:
        return None
    other = [t for t in rec["labels"] if t != str(want)]
    return other[0] if other and str(want) not in rec["labels"] else None


# ---------------------------------------------------------------- bearing (정면 자세에서 읽은 숫자 → 보기 자세의 팔레트)
def pixel_bearing_deg(K, u, v=None, pitch=0.0):
    """화소 → 카메라 기준 수평 방향(도, 오른쪽 +). pitch(아래 +)가 있으면 수평면으로 돌려 계산."""
    xn = (float(u) - K[0, 2]) / K[0, 0]
    yn = 0.0 if v is None else (float(v) - K[1, 2]) / K[1, 1]
    return math.degrees(math.atan2(xn, math.cos(pitch) - yn * math.sin(pitch)))


def hypothesis_bearing_deg(estimator, hyp):
    """표식 쌍 가설의 두 중심 가운데 화소 방향(보기 자세 pitch 반영)."""
    t = hyp["tape"]
    u = 0.5 * (t["left"][0] + t["right"][0])
    v = 0.5 * (t["left"][1] + t["right"][1])
    return pixel_bearing_deg(estimator.K, u, v, float(hyp["cam"]["pitch_down"]))


def choose_by_bearing(estimator, hypotheses, bearing_deg, tol_deg=8.0):
    """정면 자세에서 송장 숫자가 보인 방향에 가장 가까운 팔레트. 허용 안에 하나만 있어야 한다."""
    rows = sorted(((abs(hypothesis_bearing_deg(estimator, h) - bearing_deg), h) for h in hypotheses if h.get("cam")),
                  key=lambda r: r[0])
    summary = [{"left_u": h["tape"]["left"][0], "right_u": h["tape"]["right"][0], "bearing_off_deg": round(d, 2)}
               for d, h in rows]
    if not rows or rows[0][0] > tol_deg:
        return None, "no pallet within %.0f deg of label bearing %.1f" % (tol_deg, bearing_deg), summary
    if len(rows) > 1 and rows[1][0] <= tol_deg:
        return None, "two pallets within %.0f deg of label bearing" % tol_deg, summary
    d, h = rows[0]
    return dict(h, target_evidence={"label_bearing_deg": round(bearing_deg, 2), "bearing_off_deg": round(d, 2)}), None, summary
