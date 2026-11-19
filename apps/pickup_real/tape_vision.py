"""검정 표식 2개 모드 — 받침 앞면 좌우 다리에 붙인 검정 사각형(흰 테이프 바탕 위 무광 검은 테이프)으로 받침 자세를 구한다.

구멍 모드(pallet_vision.PalletEstimator)와 따로 있다. 이 모드는 실패해도 구멍 모드로 넘어가지 않는다.

검출
  영상 → 주변보다 크게 어두운 덩어리 → 사각형 모양(채움 비율·가로세로 비·크기)·안쪽이 충분히 검정
  → 마주 보는 두 변 바깥이 흰색(흰 테두리) → 두 후보의 상대 배치(크기 비, 잇는 선 기울기, 간격/크기 비)
  → 조건을 만족하는 쌍이 정확히 하나일 때만 유효. 하나만 보임·쌍 여럿·화면 끝 잘림·너무 작음은 이유와 함께 실패.
  표식 중심 = 네 변에 직선을 맞춘 네 꼭짓점의 대각선 교점(원근에서 물리 중심이 비치는 영상점).

자세 계산(닫힌 꼴, 반복 최적화 없음)
  pitch 를 정하면 두 중심의 영상 광선이 "표식 중심 높이" 수평면과 만나는 점이 정해진다. 두 점 사이 거리가
  실측 중심 간격(tape.center_spacing_m)과 같아지는 pitch 를 이분법으로 찾는다(거리는 pitch 에 단조 감소 → 해 하나).
  두 점을 잇는 선 = 받침 앞면 방향(yaw), 가운데 = 받침 중심. 카메라 높이·roll 은 설정값(실측)이다.
  측정 4개(두 중심의 u, v)·미지수 4개(앞뒤·좌우·yaw·pitch)라 재투영 오차는 항상 0 → 그것으로 맞고 틀림을 판단하지 않는다.

yaw 는 두 중심의 영상 높이 차이에서만 나온다(1 m 에서 1° ≈ 1 px). 그래서
  - 매 프레임 화소 잡음(tape.center_sigma_px)에 대한 yaw 민감도를 계산해 tape.max_heading_sigma_deg 를 넘으면 거부
  - 카메라 roll 1° 는 yaw 약 2~3° 로 번진다(heading_per_roll 로 기록). roll 은 설정값이며 현장 확인 대상
  - 좌우 위치(받침 중심 방향)는 yaw 오차에 거의 영향받지 않는다: 정렬 결과 로봇은 받침 중심을 지나는 선 위에 서고,
    yaw 오차는 진입 각도 오차로 남는다.

출력 형식은 vision.MarkerEstimator·PalletEstimator 와 같다(cam: 포켓 좌표계 P 의 카메라 자세). 정답 좌표는 쓰지 않는다.
"""
import math

import cv2
import numpy as np

from vision import intrinsics

MODE = "tape"
TEACH_FILE = "teach_tape.json"   # 구멍 모드 teach.json 과 분리(옛 구멍용 길이·극성이 섞이지 않게)
TEACH_VERSION = 1
HOLE_KEYS = ("row_length_m", "hole_pitch_m", "hole_polarity")


# ===================================================================== small geometry helpers
def _intersect(p, u, q, w):
    A = np.array([u, -w], float).T
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    t = np.linalg.solve(A, np.asarray(q, float) - np.asarray(p, float))
    return np.asarray(p, float) + np.asarray(u, float) * t[0]


def order_quad(pts):
    """네 점을 중심 기준 각도로 정렬(영상 y 아래 방향: 왼위·오른위·오른아래·왼아래)."""
    pts = np.asarray(pts, float).reshape(4, 2)
    c = pts.mean(axis=0)
    return pts[np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))]


def quad_center(quad):
    """사각형 영상의 대각선 교점 = 평면 사각형 물리 중심의 투영(원근 보존)."""
    q = np.asarray(quad, float)
    return _intersect(q[0], q[2] - q[0], q[1], q[3] - q[1])


def _bilinear(img, pts):
    H, W = img.shape
    x = np.clip(pts[:, 0], 0, W - 1.001)
    y = np.clip(pts[:, 1], 0, H - 1.001)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x - x0, y - y0
    return (img[y0, x0] * (1 - fx) * (1 - fy) + img[y0, x0 + 1] * fx * (1 - fy)
            + img[y0 + 1, x0] * (1 - fx) * fy + img[y0 + 1, x0 + 1] * fx * fy)


def refine_corners(gray_f, quad, inner_level, reach=3.0, min_step=30.0):
    """각 변을 가로지르는 밝기 단면에서 검정↔바깥 밝기의 중간값을 지나는 점(서브픽셀 변 위치)을 여러 개 찾아
    직선을 맞추고, 이웃 변 직선의 교점을 꼭짓점으로 한다. 이진화 경계(정수 좌표)보다 정밀하고, 원근으로
    사다리꼴이 된 사각형도 변마다 따로 맞춘다. 실패하면 (원래 꼭짓점, False)."""
    quad = np.asarray(quad, float)
    c = quad.mean(axis=0)
    ts = np.arange(-reach, reach + 1e-9, 0.25)
    tail = ts >= reach - 1.0
    lines = []
    for k in range(4):
        a, b = quad[k], quad[(k + 1) % 4]
        d = b - a
        L = float(np.hypot(*d))
        if L < 4.0:
            return quad, False
        u = d / L
        n = np.array([-u[1], u[0]])
        if ((a + b) / 2 - c) @ n < 0:
            n = -n                                   # 바깥 방향
        # 이 변의 단면 전부를 한 번에 표본(Jetson 에서 작은 numpy 호출이 느림)
        ss = np.linspace(0.2, 0.8, max(3, int(0.6 * L)))
        base = a[None, :] + ss[:, None] * d[None, :]                       # (S, 2)
        pts = base[:, None, :] + ts[None, :, None] * n[None, None, :]      # (S, T, 2)
        prof = _bilinear(gray_f, pts.reshape(-1, 2)).reshape(len(ss), len(ts))
        outside = np.median(prof[:, tail], axis=1)
        level = 0.5 * (inner_level + outside)[:, None]
        up = (prof[:, :-1] < level) & (prof[:, 1:] >= level)
        score = np.where(up, np.abs(ts[:-1])[None, :], np.inf)
        j = np.argmin(score, axis=1)
        rows = np.arange(len(ss))
        good = up[rows, j] & (outside - inner_level >= min_step)
        if good.sum() < 3:
            return quad, False
        jj, rr = j[good], rows[good]
        p0, p1 = prof[rr, jj], prof[rr, jj + 1]
        t0 = ts[jj] + (level[rr, 0] - p0) / np.maximum(p1 - p0, 1e-6) * 0.25
        edge = base[rr] + t0[:, None] * n[None, :]
        vx, vy, x0, y0 = cv2.fitLine(edge.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        lines.append((np.array([x0, y0], float), np.array([vx, vy], float)))
    out = []
    for k in range(4):
        (p, u), (q, w) = lines[k - 1], lines[k]
        corner = _intersect(p, u, q, w)
        if corner is None or np.linalg.norm(corner - quad[k]) > 2.5:
            return quad, False
        out.append(corner)
    return np.array(out), True


# ===================================================================== detection
def _side_bands(gray, rect, gap, band):
    """회전 사각형 네 변 바깥 띠의 밝기 중앙값 [+a1, -a1, +a2, -a2] (영상 밖이면 nan)."""
    (cx, cy), (w, h), ang = rect
    t = math.radians(ang)
    a1 = np.array([math.cos(t), math.sin(t)])     # w 방향
    a2 = np.array([-math.sin(t), math.cos(t)])    # h 방향
    H, W = gray.shape
    out = []
    ts = np.arange(gap, gap + band + 0.01, 1.0)
    for axis, half, other, other_half in ((a1, w / 2, a2, h / 2), (a2, h / 2, a1, w / 2)):
        ss = np.linspace(-0.7 * other_half, 0.7 * other_half, max(3, int(1.4 * other_half)))
        T, S = np.meshgrid(ts, ss)
        off = other[None, None, :] * S[..., None]
        for sign in (1.0, -1.0):
            pts = np.array([cx, cy]) + axis * sign * (half + T[..., None]) + off
            xi = np.rint(pts[..., 0]).astype(int).ravel()
            yi = np.rint(pts[..., 1]).astype(int).ravel()
            inside = (xi >= 0) & (xi < W) & (yi >= 0) & (yi < H)
            if inside.mean() < 0.8:
                out.append(float("nan"))
            else:
                out.append(float(np.median(gray[yi[inside], xi[inside]])))
    return out   # [+w side, -w side, +h side, -h side]


def find_candidates(gray, t):
    """검정 사각형 후보들. 반환 (good, cut, rejected_counts)."""
    H, W = gray.shape
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    blur_f = blur.astype(np.float32)
    block = int(t.get("block_px", 51)) | 1
    local = cv2.blur(blur.astype(np.float32), (block, block))
    dark = (blur.astype(np.float32) < local - float(t.get("dark_offset", 45))) & (blur <= int(t.get("black_max", 110)))
    mask = cv2.morphologyEx(dark.astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # 대칭(짝수 커널은 1 px 밀림)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    min_area, max_area = float(t.get("min_area_px", 60)), float(t.get("max_area_px", 6000))
    margin = int(t.get("border_margin_px", 3))
    good, cut = [], []
    rejected = {"size": 0, "shape": 0, "not_black": 0, "no_white_border": 0}
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area or area > max_area:
            rejected["size"] += area >= 0.5 * min_area and area <= 2 * max_area   # 작은 잡점은 세지 않는다
            continue
        roi = (lab[y:y + h, x:x + w] == i).astype(np.uint8)
        cs, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnt = max(cs, key=cv2.contourArea).reshape(-1, 2).astype(float) + [x, y]
        rect = cv2.minAreaRect(cnt.astype(np.float32))
        rw, rh = rect[1]
        short, long_ = min(rw, rh), max(rw, rh)
        fill = area / max(rw * rh, 1.0)
        if short < float(t.get("min_short_px", 5)) or long_ / max(short, 1e-6) > float(t.get("max_aspect", 4.0)) \
                or fill < float(t.get("min_fill", 0.72)):
            rejected["shape"] += 1
            continue
        touches = x <= margin or y <= margin or x + w >= W - margin or y + h >= H - margin
        inner = cv2.erode(roi, np.ones((3, 3), np.uint8)) if short >= 7 else roi
        inner_med = float(np.median(gray[y:y + h, x:x + w][inner > 0])) if inner.any() else 255.0
        if inner_med > float(t.get("black_max", 110)):
            rejected["not_black"] += 1
            continue
        band = max(2.0, 0.35 * short)
        sides = _side_bands(gray, rect, float(t.get("band_gap_px", 2)), band)
        need = float(t.get("min_border_contrast", 100))
        pairs = [(sides[0], sides[1]), (sides[2], sides[3])]
        white = [min(a, b) - inner_med for a, b in pairs if not (math.isnan(a) or math.isnan(b))]
        cand = {"area": float(area), "rect": rect, "contour": cnt, "inner": inner_med, "sides": sides,
                "border_contrast": max(white) if white else float("nan"), "bbox": (int(x), int(y), int(w), int(h)),
                "centroid": np.array(cent[i], float), "aspect": long_ / max(short, 1e-6)}
        if touches:
            # 영상 끝에 걸린 덩어리는 모양·중심을 제대로 볼 수 없다 → 보이는 변이 흰 테두리처럼 밝으면 잘린 표식 후보로 기록
            seen = [s - inner_med for s in sides if not math.isnan(s)]
            if seen and min(seen) >= float(t.get("min_side_contrast", 35)) and max(seen) >= need:
                cand["edge"] = ("left" if x <= margin else "right" if x + w >= W - margin
                                else "top" if y <= margin else "bottom")
                cut.append(cand)
            continue
        # 흰 테두리: 마주 보는 두 변 바깥이 모두 밝아야 하고, 그 밝기가 주변 장면 평균보다도 밝아야 한다(바닥 위 옹이·
        # 그림자는 바깥이 바닥 밝기 그대로라 탈락). 고립: 네 변 어느 쪽도 검정과 이어진 어두운 면이면 안 된다
        # (가구 아래 그림자·바닥 틈은 한쪽 변이 어둡다, 10/31 실제 프레임)
        cx_i, cy_i = int(round(cent[i][0])), int(round(cent[i][1]))
        scene = float(local[min(max(cy_i, 0), H - 1), min(max(cx_i, 0), W - 1)])
        excess = [min(a, b) - scene for a, b in pairs if not (math.isnan(a) or math.isnan(b))]
        cand["white_excess"] = max(excess) if excess else float("nan")
        isolated = all(not math.isnan(s) and s - inner_med >= float(t.get("min_side_contrast", 35)) for s in sides)
        best = int(np.nanargmax([min(a, b) for a, b in pairs])) if white else None
        if (not white or max(white) < need or not isolated
                or (best is not None and excess and min(pairs[best]) - scene < float(t.get("min_white_excess", 20)))):
            rejected["no_white_border"] += 1
            continue
        # 초기 사각형은 최소 외접 회전 사각형(작은 덩어리에서 다각형 근사보다 안정), 변은 밝기로 서브픽셀 보정
        quad0 = order_quad(cv2.boxPoints(rect))
        quad, refined = refine_corners(blur_f, quad0, inner_med)
        centre = quad_center(quad)
        if centre is None:
            rejected["shape"] += 1
            continue
        # 대각선 교점이 덩어리 무게중심과 크게 다르면 깨끗한 사각형이 아니다(가림·반사·이어 붙은 그림자)
        if np.linalg.norm(centre - cand["centroid"]) > max(1.5, 0.15 * short):
            rejected["shape"] += 1
            continue
        cand.update(quad=quad, centre=centre, refined=refined, short=float(short), long=float(long_))
        good.append(cand)
    return good, cut, rejected


def pair_markers(good, cut, t, width):
    """좌우 표식 쌍 하나를 고른다. 반환 (pair 또는 None, 이유, edge)."""
    pairs = []
    lo_r, hi_r = t.get("spacing_to_size", [3.0, 30.0])
    for i in range(len(good)):
        for j in range(len(good)):
            a, b = good[i], good[j]
            if a["centre"][0] >= b["centre"][0]:
                continue
            d = b["centre"] - a["centre"]
            sep = float(np.hypot(*d))
            ratio = a["area"] / b["area"]
            size = math.sqrt(0.5 * (a["area"] + b["area"]))
            tilt = abs(math.degrees(math.atan2(d[1], d[0])))
            if sep < float(t.get("min_spacing_px", 40)):
                continue
            # 같은 크기 테이프라 넓이 비는 원근(좌우 거리 차)만큼만 다르다. 30° 비스듬해도 약 1.8 배
            if not (1.0 / float(t.get("max_area_ratio", 2.0)) <= ratio <= float(t.get("max_area_ratio", 2.0))):
                continue
            if abs(math.log(a["aspect"] / b["aspect"])) > math.log(float(t.get("max_aspect_ratio_diff", 1.6))):
                continue
            # 같은 높이 표식이라 잇는 선은 거의 수평이다(yaw 20° 에서 약 9°). 과하게 강제하지 않고 20° 까지
            if tilt > float(t.get("max_pair_tilt_deg", 20)):
                continue
            if not (float(lo_r) <= sep / size <= float(hi_r)):
                continue
            pairs.append((a, b))
    if len(pairs) == 1:
        return pairs[0], None, None
    if len(pairs) > 1:
        return None, "ambiguous: %d candidate marker pairs" % len(pairs), None
    if len(good) == 1 and cut:
        # 보이는 표식이 하나면, 그와 같은 줄(잇는 선 기울기 허용치 안)에 있는 잘린 덩어리만 짝 후보로 본다.
        # 다른 줄의 가장자리 덩어리(상자·바닥 무늬)를 "잘림"으로 보고하면 "하나만 보임"이 가려진다
        g = good[0]
        tan_t = math.tan(math.radians(float(t.get("max_pair_tilt_deg", 20))))
        cut = [c for c in cut if abs(c["centroid"][1] - g["centre"][1])
               <= tan_t * abs(c["centroid"][0] - g["centre"][0]) + g["short"]]
    if cut:
        e = cut[0]["edge"]
        return None, "marker cut at image edge (%s)" % e, e
    if len(good) == 1:
        side = "left" if good[0]["centre"][0] < width / 2 else "right"
        return None, "only one marker visible (%s half)" % side, None
    if len(good) == 0:
        return None, "no black marker with white border", None
    return None, "markers do not form a left/right pair (%d candidates)" % len(good), None


# ===================================================================== pose
def camera_axes(pitch, roll):
    """수평 기준 W(x 오른쪽, y 아래, z 앞 수평) → 카메라 광학 좌표 회전 행 [x_c; y_c; z_c].
    pallet_vision.camera_rotation 과 같은 pitch(아래 +)·roll 규약."""
    x0 = np.array([1.0, 0.0, 0.0])
    y0 = np.array([0.0, math.cos(pitch), -math.sin(pitch)])
    z0 = np.array([0.0, math.sin(pitch), math.cos(pitch)])
    if roll:
        x0, y0 = math.cos(roll) * x0 + math.sin(roll) * y0, -math.sin(roll) * x0 + math.cos(roll) * y0
    return np.vstack([x0, y0, z0])


def normalise(K, D, uv):
    pts = np.asarray(uv, float).reshape(-1, 1, 2)
    if D is not None and np.any(np.abs(D) > 0):
        return cv2.undistortPoints(pts, K, D).reshape(-1, 2)
    return np.column_stack([(pts[:, 0, 0] - K[0, 2]) / K[0, 0], (pts[:, 0, 1] - K[1, 2]) / K[1, 1]])


def _floor_points(rays, pitch, roll, dh):
    """camera_axes(pitch, roll) 의 전치로 광선을 수평 기준으로 돌린 것과 같은 식을 스칼라로 푼다(Jetson 에서 빠름).
    광선 (xn, yn, 1) → W: (a, b·cos p + sin p, cos p - b·sin p), a = xn·cos r - yn·sin r, b = xn·sin r + yn·cos r."""
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    out = []
    for xn, yn in rays:
        a = xn * cr - yn * sr
        b = xn * sr + yn * cr
        down = b * cp + sp
        if down <= 1e-6:
            return None   # 이 pitch 에서는 광선이 표식 높이 평면 위쪽(수평선 위)을 본다
        s = dh / down
        out.append(np.array([a * s, (cp - b * sp) * s]))   # (오른쪽, 앞)
    return out


def solve_pose(rays, spacing, dh, roll, hint=None):
    """두 중심의 정규 좌표 광선 → 카메라 자세(P 좌표). 해가 없으면 (None, 이유). hint = 가까운 pitch(빠른 구간 찾기)."""
    if dh <= 0.02:
        return None, "camera height above marker centre too small (%.3f m)" % dh

    def gap(p):
        pts = _floor_points(rays, p, roll, dh)
        return None if pts is None else math.hypot(*(pts[1] - pts[0])) - spacing

    lo = hi = None
    if hint is not None:
        a, b = hint - math.radians(2.0), hint + math.radians(2.0)
        ga, gb = gap(a), gap(b)
        if ga is not None and gb is not None and ga > 0 >= gb:
            lo, hi = a, b
    if lo is None:
        grid = np.radians(np.arange(0.5, 85.01, 1.0))
        vals = [gap(p) for p in grid]
        for k in range(len(grid) - 1):
            if vals[k] is not None and vals[k + 1] is not None and vals[k] > 0 >= vals[k + 1]:
                lo, hi = grid[k], grid[k + 1]
                break
    if lo is None:
        return None, "no pitch makes the two centres %.3f m apart" % spacing
    for _ in range(45):
        mid = 0.5 * (lo + hi)
        if gap(mid) > 0:
            lo = mid
        else:
            hi = mid
    pitch = 0.5 * (lo + hi)
    L, Rr = _floor_points(rays, pitch, roll, dh)
    e = (Rr - L) / np.linalg.norm(Rr - L)       # 받침 앞면 오른쪽 방향(카메라 수평 좌표)
    n = np.array([e[1], -e[0]])                  # 앞면에서 카메라 쪽
    M = 0.5 * (L + Rr)                           # 받침 중심(표식 중심 높이)
    cam = {"x": float(M @ n), "y": float(M @ e), "heading": float(math.atan2(-e[1], e[0])), "pitch_down": float(pitch),
           "height_above_sheet": float(dh), "range": float(math.hypot(float(np.linalg.norm(M)), dh))}
    return cam, None


# ===================================================================== estimator
class TapeEstimator:
    """한 프레임 → 받침 앞면(두 검정 표식 중심) 대비 카메라 자세. MarkerEstimator 와 같은 인터페이스."""

    mode = MODE

    def __init__(self, cfg, focal_scale=1.0, root=None):
        self.cfg = cfg
        self.t = cfg["tape"]
        self.cam_cfg = cfg["camera"]
        self.K, self.D, self.k_source = intrinsics(self.cam_cfg, focal_scale, root)
        self.spacing = float(self.t["center_spacing_m"])
        self.dh = float(self.t["camera_height_m"]) - float(self.t["marker_center_height_m"])
        self.roll = math.radians(float(self.t.get("camera_roll_deg", 0.0)))
        self.last = None   # camera_server 호환(이 모드는 직전 해를 쓰지 않는다)

    # ------------------------------------------------------------------ features
    def features(self, gray):
        good, cut, rejected = find_candidates(gray, self.t)
        pair, reason, edge = pair_markers(good, cut, self.t, gray.shape[1])
        out = {"candidates": good, "cut": cut, "rejected": rejected, "pair": pair}
        if reason:
            out["reason"] = reason
            if edge:
                out["edge"] = edge
            return out
        a, b = pair
        min_short = float(self.t.get("min_marker_short_px", 7))
        if min(a["short"], b["short"]) < min_short:
            out["reason"] = "markers too small (%.1f px < %.0f px) - too far" % (min(a["short"], b["short"]), min_short)
        return out

    def solve_centres(self, uv_left, uv_right, K=None, hint=None):
        rays = normalise(self.K if K is None else K, self.D, [uv_left, uv_right])
        return solve_pose(rays, self.spacing, self.dh, self.roll, hint)

    def sensitivity(self, uv_left, uv_right, base):
        """화소 0.5 px·roll 0.5° 흔들었을 때 자세 변화 → 1 px 당 민감도와 roll 1° 당 heading 변화."""
        uv = np.array([uv_left, uv_right], float)
        d_head, d_y, d_x = [], [], []
        hint = base["pitch_down"]
        for k in range(4):
            q = uv.copy()
            q[k // 2, k % 2] += 0.5
            cam, _ = self.solve_centres(q[0], q[1], hint=hint)
            if cam is None:
                return None
            d_head.append((cam["heading"] - base["heading"]) / 0.5)
            d_y.append((cam["y"] - base["y"]) / 0.5)
            d_x.append((cam["x"] - base["x"]) / 0.5)
        rays = normalise(self.K, self.D, uv)
        cam_r, _ = solve_pose(rays, self.spacing, self.dh, self.roll + math.radians(0.5), hint)
        sig = float(self.t.get("center_sigma_px", 0.3))
        return {"heading_sigma_deg": math.degrees(math.sqrt(sum(v * v for v in d_head))) * sig,
                "y_sigma_m": math.sqrt(sum(v * v for v in d_y)) * sig,
                "x_sigma_m": math.sqrt(sum(v * v for v in d_x)) * sig,
                "heading_per_roll": None if cam_r is None else (cam_r["heading"] - base["heading"]) / math.radians(0.5)}

    def estimate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        return self.estimate_features(self.features(gray))

    def estimate_features(self, feats):
        pair = feats.get("pair")
        result = {"ok": False, "ids": [], "corners": {}, "mode": MODE, "reprojection_px": 0.0,
                  "features": {"candidates": len(feats.get("candidates", [])), "cut": len(feats.get("cut", [])),
                               "rejected": feats.get("rejected")}}
        result["_feats"] = feats
        if feats.get("reason"):
            result["reason"] = feats["reason"]
            return result
        a, b = pair
        tape = {"left": [round(float(v), 2) for v in a["centre"]], "right": [round(float(v), 2) for v in b["centre"]],
                "spacing_px": round(float(np.linalg.norm(b["centre"] - a["centre"])), 1),
                "size_px": [[round(a["long"], 1), round(a["short"], 1)], [round(b["long"], 1), round(b["short"], 1)]],
                "border_contrast": [round(a["border_contrast"], 1), round(b["border_contrast"], 1)]}
        result["tape"] = tape
        cam, why = self.solve_centres(a["centre"], b["centre"])
        if cam is None:
            result["reason"] = "pose: " + why
            return result
        result["cam"] = cam
        sens = self.sensitivity(a["centre"], b["centre"], cam)
        if sens is None:
            result["reason"] = "pose: sensitivity solve failed"
            return result
        tape.update({k: (None if v is None else round(v, 4)) for k, v in sens.items()})
        if not (0.2 < -cam["x"] < 3.0) or abs(cam["heading"]) > math.radians(45) \
                or not (math.radians(2) < cam["pitch_down"] < math.radians(80)):
            result["reason"] = "implausible pose x=%.2f heading=%.1f pitch=%.1f" % (
                cam["x"], math.degrees(cam["heading"]), math.degrees(cam["pitch_down"]))
            return result
        if sens["heading_sigma_deg"] > float(self.t.get("max_heading_sigma_deg", 1.0)):
            # 각도를 안정적으로 못 구하는 배치(멀거나 두 표식이 너무 붙어 보임) → 0° 로 채우지 않고 거부
            result["reason"] = "yaw poorly observable (+-%.2f deg per %.1f px)" % (
                sens["heading_sigma_deg"], float(self.t.get("center_sigma_px", 0.3)))
            return result
        result["ok"] = True
        return result

    # ------------------------------------------------------------------ drawing
    def annotate(self, frame, est, lines=()):
        out = frame.copy()
        feats = est.get("_feats") or {}
        for c in feats.get("cut", []):
            x, y, w, h = c["bbox"]
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 1)
        pair = feats.get("pair")
        for c in feats.get("candidates", []):
            if pair and (c is pair[0] or c is pair[1]):
                continue
            q = np.round(c["quad"]).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [q], True, (0, 165, 255), 1)
        if pair:
            colour = (0, 255, 0) if est.get("ok") else (0, 165, 255)
            for c, name in zip(pair, ("L", "R")):
                q = np.round(c["quad"]).astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(out, [q], True, colour, 2)
                u, v = c["centre"]
                cv2.drawMarker(out, (int(round(u)), int(round(v))), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
                cv2.putText(out, name, (int(u) - 6, int(v) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            a, b = pair[0]["centre"], pair[1]["centre"]
            cv2.line(out, (int(round(a[0])), int(round(a[1]))), (int(round(b[0])), int(round(b[1]))), (255, 0, 255), 1)
            m = (a + b) / 2
            cv2.drawMarker(out, (int(round(m[0])), int(round(m[1]))), (0, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        cv2.line(out, (out.shape[1] // 2, 0), (out.shape[1] // 2, out.shape[0]), (80, 80, 80), 1)
        y = 22
        for text in list(lines):
            cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
            cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            y += 22
        status = "mode TAPE  %s" % ("VALID" if est.get("ok") else "INVALID")
        cv2.putText(out, status, (out.shape[1] - 230, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
        cv2.putText(out, status, (out.shape[1] - 230, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if est.get("ok") else (0, 0, 255), 1)
        if not est.get("ok"):
            cv2.putText(out, str(est.get("reason", ""))[:64], (8, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)
        return out


# ===================================================================== calibration file (teach_tape.json)
def teach_problem(teach, cfg):
    """보정 기록(teach)을 이 모드에서 써도 되는지. 문제 없으면 None, 있으면 이유."""
    if not teach:
        return "no teach file"
    if teach.get("vision_mode") != MODE:
        return "teach is not for tape mode (%s) - run calibrate again" % (teach.get("vision_mode") or "old hole/marker teach")
    bad = [k for k in HOLE_KEYS if teach.get(k) is not None]
    if bad:
        return "teach has hole-mode keys %s" % bad
    t = cfg["tape"]
    for key, tol in (("center_spacing_m", 0.0005), ("camera_height_m", 0.002), ("marker_center_height_m", 0.002),
                     ("camera_roll_deg", 0.05)):
        if key not in teach or abs(float(teach[key]) - float(t.get(key, 0.0))) > tol:
            return "config tape.%s changed since calibrate (teach %s, config %s)" % (key, teach.get(key), t.get(key))
    if int(teach.get("teach_version", 0)) != TEACH_VERSION:
        return "teach version %s != %d" % (teach.get("teach_version"), TEACH_VERSION)
    return None


def teach_fields(cfg):
    """teach_tape.json 에 함께 남겨 다음 로딩 때 설정과 대조할 값."""
    t = cfg["tape"]
    return {"vision_mode": MODE, "teach_version": TEACH_VERSION,
            "center_spacing_m": float(t["center_spacing_m"]), "camera_height_m": float(t["camera_height_m"]),
            "marker_center_height_m": float(t["marker_center_height_m"]),
            "camera_roll_deg": float(t.get("camera_roll_deg", 0.0))}


def fit_focal(cfg, centres, d, lo=0.80, hi=1.20):
    """보정 사진 3장(후진 전·후진 후·전진 후)의 표식 중심(중앙값)으로, 카메라 이동이 odom 이동 d(후진, 전진) 와 맞는
    초점 배율을 찾는다. 중심 간격은 실측값 그대로 둔다(다시 추정하지 않는다).
    반환 (scale, rms_m, cams) 또는 (None, 이유, None)."""
    d1, d2 = (float(d), float(d)) if np.isscalar(d) else (float(d[0]), float(d[1]))
    est = TapeEstimator(cfg)
    best = None
    for k in np.arange(lo, hi + 1e-9, 0.002):
        K = est.K.copy()
        K[0, 0] *= k
        K[1, 1] *= k
        cams = []
        for left, right in centres:
            cam, _ = est.solve_centres(left, right, K)
            if cam is None:
                break
            cams.append(cam)
        if len(cams) != len(centres):
            continue
        c = [np.array([q["x"], q["y"]]) for q in cams]
        err = (np.linalg.norm(c[1] - c[0]) - d1) ** 2 + (np.linalg.norm(c[2] - c[1]) - d2) ** 2
        if best is None or err < best[0]:
            best = (err, float(k), cams)
    if best is None:
        return None, "no focal scale solves all three photos", None
    rms = math.sqrt(best[0] / 2)
    return best[1], rms, best[2]


def focal_from_distance(cfg, centre, distance, lo=0.70, hi=1.30):
    """줄자로 잰 카메라 렌즈 ↔ 받침 앞면 수평 거리(m) 한 번으로 초점 배율을 정한다(12 cm 이동 맞춤보다 조건이 좋음).
    반환 (scale, 그 배율에서 본 거리) 또는 (None, 이유)."""
    est = TapeEstimator(cfg)
    best = None
    for k in np.arange(lo, hi + 1e-9, 0.001):
        K = est.K.copy()
        K[0, 0] *= k
        K[1, 1] *= k
        cam, _ = est.solve_centres(centre[0], centre[1], K)
        if cam is None:
            continue
        err = abs(-cam["x"] - float(distance))
        if best is None or err < best[0]:
            best = (err, float(k), -cam["x"])
    if best is None or best[0] > 0.005:
        return None, "no focal scale reproduces %.3f m" % float(distance)
    return best[1], best[2]
