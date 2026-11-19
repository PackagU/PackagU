"""마커 없는 받침(팔레트) 인식 — 구멍 뚫린 알루미늄 앵글의 구멍 줄로 받침 앞면을 찾는다.

받침 앞면: 두 세로 기둥(중심 간격 post_spacing_m) + 그 사이 가로대(구멍 줄 높이 row_height_m). 포크는 가로대 아래로 들어간다.
영상 → 어두운 작은 둥근 점(구멍) → 일정 간격으로 이어진 가로 줄(가로대) + 세로 줄(기둥) → 기둥 중심과 가로대 구멍 줄의
교점 2개 + 가로대 구멍 전체 → 카메라 높이(줄자)를 알고 카메라 (좌우, 거리, heading, pitch) 를 푼다.

출력 형식은 vision.MarkerEstimator 와 같다(cam: 포켓 좌표계 P 의 카메라 자세). 정답 좌표는 쓰지 않는다.
"""
import math

import cv2
import numpy as np

from vision import intrinsics


def detect_holes(gray, block=15, offset=25, min_area=4, max_area=90, max_side=14, bright=False):
    """구멍 후보 중심들. bright=False 는 주변보다 어두운 점, True 는 밝은 점(낮에 구멍 너머 바닥이 밝게 비침, 10/31)."""
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    mode = cv2.THRESH_BINARY if bright else cv2.THRESH_BINARY_INV
    th = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, mode, block, -offset if bright else offset)
    n, _, stats, cent = cv2.connectedComponentsWithStats(th, 8)
    keep = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if min_area <= a <= max_area and w <= max_side and h <= max_side and max(w, h) <= 2.6 * min(w, h):
            keep.append(cent[i])
    return np.array(keep, float).reshape(-1, 2)


def _runs(points, direction, origin, max_gap):
    """직선 위 점들을 직선 방향으로 정렬해 간격이 max_gap 이하로 이어진 묶음들로 나눈다."""
    s = (points - origin) @ direction
    order = np.argsort(s)
    runs, cur = [], [order[0]]
    for a, b in zip(order[:-1], order[1:]):
        if s[b] - s[a] <= max_gap:
            cur.append(b)
        else:
            runs.append(cur)
            cur = [b]
    runs.append(cur)
    return [np.array(r) for r in runs]


def find_rows(points, angle_ok, min_count, max_gap=24.0, tol=1.8, tries=350, max_rows=3, seed=0):
    """RANSAC 으로 구멍 줄(간격 max_gap 이하로 이어진 묶음)을 찾는다. 쌍은 가까운 이웃끼리만 뽑아 빠르다.

    angle_ok(deg 0..180) → bool. 반환: [{idx, point, dir, angle_deg, span, ends}]
    """
    rng = np.random.default_rng(seed)
    n = len(points)
    if n < min_count:
        return []
    d2 = ((points[:, None, :] - points[None, :, :]) ** 2).sum(-1)
    neigh = [np.where((d2[i] > 36) & (d2[i] < (max_gap * 1.3) ** 2))[0] for i in range(n)]
    free = np.ones(n, bool)
    rows = []
    for _ in range(max_rows):
        cand = np.where(free)[0]
        seeds = [i for i in cand if len(neigh[i])]
        if len(cand) < min_count or not seeds:
            break
        best = None
        for _ in range(tries):
            i = seeds[rng.integers(len(seeds))]
            j = neigh[i][rng.integers(len(neigh[i]))]
            if not free[j]:
                continue
            d = points[j] - points[i]
            ang = math.degrees(math.atan2(d[1], d[0])) % 180.0
            if not angle_ok(ang):
                continue
            u = d / math.hypot(*d)
            nrm = np.array([-u[1], u[0]])
            inl = cand[np.abs((points[cand] - points[i]) @ nrm) < tol]
            if len(inl) < min_count or (best is not None and len(inl) <= len(best)):
                continue
            for run in _runs(points[inl], u, points[i], max_gap):
                if best is None or len(run) > len(best):
                    best = inl[run]
        if best is None or len(best) < min_count:
            break
        P = points[best].astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(P, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        u = np.array([vx, vy], float)
        if u[0] < 0 or (abs(u[0]) < 1e-6 and u[1] < 0):
            u = -u
        s = (points[best] - np.array([x0, y0])) @ u
        rows.append({"idx": best, "point": np.array([x0, y0], float), "dir": u,
                     "angle_deg": math.degrees(math.atan2(u[1], u[0])), "span": float(np.ptp(s)),
                     "ends": (np.array([x0, y0]) + u * s.min(), np.array([x0, y0]) + u * s.max())})
        free[best] = False
    return rows


def estimate_pitch(gaps, lo=4.0, hi=25.0):
    """구멍 사이 간격들 → 기본 주기(px). 빠진 구멍(2배·3배 간격)에 강하다. 약수(반 주기) 대신 큰 주기를 고른다."""
    g = np.asarray([x for x in gaps if x > 2.5], float)
    if len(g) < 5:
        return None
    cands = np.arange(lo, hi, 0.05)
    score = np.array([np.mean(np.cos(2 * np.pi * g / c) * (g <= 3.5 * c)) for c in cands])
    good = cands[score >= 0.9 * score.max()]
    if score.max() < 0.5:
        return None
    best = good.max()
    near = (np.abs(g / best - np.round(g / best)) < 0.25) & (np.round(g / best) == 1)
    return float(np.median(g[near])) if near.sum() >= 3 else float(best)


def lattice_step(gap, pitch, tol=0.38, max_missing=3):
    k = gap / pitch
    return round(k) >= 1 and round(k) <= 1 + max_missing and abs(k - round(k)) <= tol


def intersect(p, u, q, w):
    A = np.array([u, -w]).T
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    t = np.linalg.solve(A, q - p)
    return p + u * t[0]


def camera_rotation(heading, pitch, roll=0.0):
    """S(시트/받침: X 오른쪽, Y 위, Z 로봇 쪽) → 카메라 광학 좌표 회전. vision.render_plane 과 같은 규약."""
    fwd = np.array([-math.sin(heading) * math.cos(pitch), -math.sin(pitch), -math.cos(heading) * math.cos(pitch)])
    right = np.cross(fwd, np.array([0.0, 1.0, 0.0]))
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    if roll:
        right, down = math.cos(roll) * right + math.sin(roll) * down, -math.sin(roll) * right + math.cos(roll) * down
    return np.vstack([right, down, fwd])


def project(K, params, height, roll, pts):
    cx, cz, heading, pitch = params
    R = camera_rotation(heading, pitch, roll)
    C = np.array([cx, height, cz])
    cam = (R @ (np.asarray(pts, float) - C).T).T
    uv = (K @ cam.T).T
    return uv[:, :2] / uv[:, 2:3], cam[:, 2]


class PalletEstimator:
    """한 프레임 → 받침 앞면 대비 카메라 자세. MarkerEstimator 와 같은 인터페이스."""

    def __init__(self, cfg, focal_scale=1.0, root=None):
        self.cfg = cfg
        self.p = cfg["pallet"]
        self.cam_cfg = cfg["camera"]
        self.K, self.D, self.k_source = intrinsics(self.cam_cfg, focal_scale, root)
        self.height = float(self.p["camera_height_m"])
        self.roll = math.radians(float(self.p.get("camera_roll_deg", 0.0)))
        self.row_h = float(self.p["row_height_m"])
        self.last = None
        # 구멍 극성: dark(밤, 10/30) / bright(낮, 구멍 너머 바닥이 밝음 — 큰 타원 구멍만 보여 간격이 2배) / auto.
        # 극성이 바뀌면 구멍 간격도 바뀌므로 한 보정·실행 안에서는 고정한다(calibrate 가 teach 에 기록)
        self.polarity = str(self.p.get("hole_polarity", "auto"))
        self.bright = self.polarity == "bright"   # auto 일 때는 마지막으로 통한 쪽

    def features(self, gray):
        """구멍이 어둡게(10/30 밤) 또는 밝게(10/31 낮, 구멍 너머 바닥) 보인다. 지난번 통한 쪽부터 해 보고,
        안 되면 반대쪽. 둘 다 안 되면 가장자리 정보(edge)가 있는 이유를 우선 돌려준다(탐색 회전 방향)."""
        if self.polarity in ("dark", "bright"):
            order = [self.polarity == "bright"]
        else:
            order = [self.bright, not self.bright]
        tried = []
        for bright in order:
            f = self._features(gray, bright)
            if not f.get("reason"):
                self.bright = bright
                f["bright_holes"] = bright
                return f
            tried.append(f)
        edged = [f for f in tried if f.get("edge")]
        return edged[0] if edged else tried[0]

    def _features(self, gray, bright):
        p = self.p
        # 가까워지면 구멍이 커진다(0.5 m 에서 약 12 px). 밝은 구멍은 타원이라 크기 상한을 넉넉히
        holes = detect_holes(gray, max_area=int(p.get("hole_max_area_px", 260)), max_side=int(p.get("hole_max_side_px", 24)),
                             bright=bright)
        # 화면 위쪽 일부(상자·윗판)는 뺀다. 1 m 쯤 떨어지면 가로대가 화면 위 1/4 안으로 올라온다(10/31 실기, y≈108)
        top = float(p.get("ignore_top_frac", 0.10))
        lower = holes[holes[:, 1] > gray.shape[0] * top] if len(holes) else holes
        out = {"holes": lower, "bar": None, "posts": []}
        if len(lower) < int(p["min_bar_holes"]):
            out["reason"] = "too few hole-like spots (%d)" % len(lower)
            return out
        bars = [r for r in find_rows(lower, lambda a: a <= 35 or a >= 145, int(p["min_bar_holes"]),
                                     max_gap=float(p.get("bar_max_gap_px", 40.0)), tries=500, max_rows=3)
                if r["span"] >= float(p["min_bar_span_px"])]
        if not bars:
            out["reason"] = "no perforated bar row"
            return out
        # 가장 긴 줄 중에서 가장 아래(카메라에 가까운) 것 = 가로대 앞면 구멍 줄
        longest = max(r["span"] for r in bars)
        cands = [r for r in bars if r["span"] >= 0.7 * longest]
        bar = max(cands, key=lambda r: r["point"][1] + r["dir"][1] / max(abs(r["dir"][0]), 1e-6) * (320 - r["point"][0]))
        out["bar"] = bar
        W = gray.shape[1]
        margin = float(p.get("border_margin_px", 18))
        e0, e1 = bar["ends"]
        if e0[0] < margin or e1[0] > W - margin:
            out["edge"] = "left" if e0[0] < margin else "right"
            out["reason"] = "bar row touches image border (%s) - pallet not fully in view" % out["edge"]
            return out
        # 구멍 간격(주기) p 와, 간격이 p 의 정수배로 이어지는 구간(가로대 구멍만) → 끝점
        svals = np.sort((lower[bar["idx"]] - bar["point"]) @ bar["dir"])
        gaps = np.diff(svals)
        pitch = estimate_pitch(gaps)
        if pitch is None:
            out["reason"] = "hole pitch not found"
            return out
        mid = int(np.argmin(np.abs(svals - np.median(svals))))
        lo = hi = mid
        while lo > 0 and lattice_step(svals[lo] - svals[lo - 1], pitch):
            lo -= 1
        while hi < len(svals) - 1 and lattice_step(svals[hi + 1] - svals[hi], pitch):
            hi += 1
        n = int(round((svals[hi] - svals[lo]) / pitch)) + 1
        if n < int(p["min_bar_holes"]):
            out["reason"] = "regular hole run too short (%d)" % n
            return out
        # 중심 = 구간 구멍 위치의 중앙값(끝 구멍이 들쭉날쭉해도 덜 흔들림), 끝점은 중심 ± (n-1)/2·간격
        centre = float(np.median(svals[lo:hi + 1]))
        half_px = (n - 1) / 2.0 * pitch
        a = bar["point"] + bar["dir"] * (centre - half_px)
        b = bar["point"] + bar["dir"] * (centre + half_px)
        out["posts"] = [{"uv": a}, {"uv": b}]
        out["pitch_px"], out["count"] = float(pitch), n
        return out

    def solve(self, feats):
        K = self.K
        uL, uR = feats["posts"][0]["uv"], feats["posts"][1]["uv"]
        bar_pts = feats["holes"][feats["bar"]["idx"]]
        # 물리 폭 = 구멍 줄 양 끝 구멍 사이 길이(row_length_m, calibrate 가 맞춤). 개수 × 간격은 낮에 작은 구멍이
        # 보였다 말았다 하면 개수가 2배로 바뀌어 거리가 2배로 뒤집혔다(10/31 실기). 끝 구멍 픽셀 위치는 그대로라 길이로 쓴다
        W = self.p.get("row_length_m")
        half = float(W) / 2.0 if W else (feats["count"] - 1) * float(self.p["hole_pitch_m"]) / 2.0
        model = np.array([[-half, self.row_h, 0.0], [half, self.row_h, 0.0]])
        f = K[0, 0]
        width = abs(uR[0] - uL[0])
        Z = f * 2 * half / max(width, 1.0)
        mid = (uL + uR) / 2
        hr = max(self.height - self.row_h, 0.05)
        horiz = math.sqrt(max(Z * Z - hr * hr, 0.01))
        pitch0 = math.atan2(hr, horiz) - math.atan2(mid[1] - K[1, 2], f)
        bearing = math.atan2(mid[0] - K[0, 2], f)
        guess = np.array([horiz * math.sin(bearing), horiz * math.cos(bearing), 0.0, pitch0], float)
        # 매 프레임 닫힌 꼴 추정에서 시작한다. 직전 해(warm start)는 둘째 후보로만 쓴다:
        # 보정으로 카메라 높이가 바뀐 뒤 옛 해에서 시작하면 엉뚱한 해로 수렴했다(시뮬 10/30, 좌우 흔들림 117 mm).
        starts = [guess] + ([np.array(self.last, float)] if self.last is not None else [])

        def residual(q):
            uv, depth = project(K, q, self.height, self.roll, model)
            r = [uv[0, 0] - uL[0], uv[0, 1] - uL[1], uv[1, 0] - uR[0], uv[1, 1] - uR[1]]
            d = uv[1] - uv[0]
            n = np.array([-d[1], d[0]]) / max(np.linalg.norm(d), 1e-6)
            r.extend(list(((bar_pts - uv[0]) @ n) * 0.5))
            if np.any(depth <= 0.05):
                r = [1e3] * len(r)
            return np.array(r)

        def gauss_newton(params):
            for _ in range(30):
                r0 = residual(params)
                J = np.zeros((len(r0), 4))
                for k in range(4):
                    dq = params.copy()
                    dq[k] += 1e-4
                    J[:, k] = (residual(dq) - r0) / 1e-4
                step = np.linalg.lstsq(J, -r0, rcond=None)[0]
                params = params + np.clip(step, -0.2, 0.2)
                if np.linalg.norm(step) < 1e-7:
                    break
            return params, float(np.sum(residual(params) ** 2))

        params = min((gauss_newton(q) for q in starts), key=lambda t: t[1])[0]
        r = residual(params)
        rms = float(np.sqrt(np.mean(r[:4] ** 2)))
        line_rms = float(np.sqrt(np.mean((r[4:] / 0.5) ** 2))) if len(r) > 4 else 0.0
        return params, rms, line_rms

    def estimate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        return self.estimate_features(self.features(gray))

    def estimate_features(self, feats):
        """features() 결과로 자세만 푼다. 구멍 검출은 카메라 높이·구멍 간격과 무관해 보정 격자 탐색에서 재사용한다."""
        result = {"ok": False, "ids": [], "corners": {}, "features": {
            "holes": len(feats["holes"]), "bar_holes": 0 if feats["bar"] is None else len(feats["bar"]["idx"]),
            "posts": [q["uv"].round(1).tolist() for q in feats["posts"]]}}
        result["_feats"] = feats
        if feats.get("reason"):
            result["reason"] = feats["reason"]
            return result
        solved = self.solve(feats)
        if solved is None:
            result["reason"] = "pose solve failed"
            return result
        (cx, cz, heading, pitch), rms, line_rms = solved
        result["features"]["bar_line_rms_px"] = round(line_rms, 2)
        cam = {"x": float(-cz), "y": float(-cx), "heading": float(heading), "pitch_down": float(pitch),
               "height_above_sheet": self.height - self.row_h, "range": float(math.hypot(cx, cz))}
        result["cam"] = cam
        result["reprojection_px"] = max(rms, line_rms)
        if not (0.15 < -cam["x"] < 3.0) or abs(heading) > math.radians(60) or not (0 < pitch < math.radians(85)):
            result["reason"] = "implausible pose x=%.2f heading=%.1f pitch=%.1f" % (cam["x"], math.degrees(heading),
                                                                                   math.degrees(pitch))
            self.last = None
            return result
        if line_rms > float(self.cam_cfg["max_reprojection_px"]):
            result["reason"] = "bar holes off the fitted line (%.2f px)" % line_rms
            return result
        self.last = [cx, cz, heading, pitch]
        result["ok"] = True
        return result

    def annotate(self, frame, est, lines=()):
        out = frame.copy()
        feats = est.get("_feats") or {}
        holes = feats.get("holes", np.zeros((0, 2)))
        for p in holes:
            cv2.circle(out, (int(p[0]), int(p[1])), 2, (0, 0, 200), 1)
        bar = feats.get("bar")
        if bar is not None:
            for p in holes[bar["idx"]]:
                cv2.circle(out, (int(p[0]), int(p[1])), 3, (0, 255, 0), 1)
            a, b = bar["ends"]
            cv2.line(out, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (0, 255, 0), 1)
        for q in feats.get("posts", []):
            u, v = q["uv"]
            cv2.drawMarker(out, (int(u), int(v)), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
        if est.get("ok") and len(feats.get("posts", [])) == 2:
            mid = (feats["posts"][0]["uv"] + feats["posts"][1]["uv"]) / 2
            cv2.drawMarker(out, (int(mid[0]), int(mid[1])), (0, 255, 255), cv2.MARKER_TILTED_CROSS, 16, 2)
        cv2.line(out, (out.shape[1] // 2, 0), (out.shape[1] // 2, out.shape[0]), (80, 80, 80), 1)
        y = 22
        for text in lines:
            cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
            cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            y += 22
        if not est.get("ok"):
            cv2.putText(out, str(est.get("reason", ""))[:60], (8, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)
        return out
