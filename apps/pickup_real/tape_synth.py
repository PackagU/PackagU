"""오프라인 시험용 합성 장면 — 받침 앞면(다리·흰 띠·검정 표식·구멍 가로대·윗판·상자)과 나뭇결·틈·옹이 무늬를
주어진 카메라 자세(roll 포함)에서 바늘구멍 모델로 렌더링한다. 검출기에는 이 영상만 넘긴다(정답 자세는 평가에만).

좌표: 받침 앞면 S(X 로봇에서 볼 때 오른쪽, Y 위, 원점 = 두 표식 중심의 가운데). 카메라 자세는 포켓 좌표 P
(x 받침 안쪽, y 왼쪽)의 (x, y, heading, 표식 중심 위 카메라 높이, pitch 아래 +) — vision.render_plane 과 같은 규약.
"""
import math

import cv2
import numpy as np

from pallet_vision import camera_rotation

PPM = 2000.0                 # 텍스처 1 m 당 화소(0.5 mm)
X_RANGE = (-0.36, 0.36)
Y_RANGE = (-0.25, 0.30)      # 표식 중심 높이 기준


def face_texture(spacing=0.303, marker=(0.030, 0.020), centre_h=0.06, leg_w=0.035, band_h=0.05, seed=0,
                 hide=(), decoys=(), clutter=True):
    """hide: "left"/"right" 표식 가림(흰 띠만 남김). decoys: [(X, Y, w, h, 흰 테두리 여부)] 추가 검정 사각형."""
    rng = np.random.default_rng(seed)
    W = int((X_RANGE[1] - X_RANGE[0]) * PPM)
    H = int((Y_RANGE[1] - Y_RANGE[0]) * PPM)
    ou, ov = -X_RANGE[0] * PPM, Y_RANGE[1] * PPM

    def px(X, Y):
        return int(round(ou + X * PPM)), int(round(ov - Y * PPM))

    def rect(X0, Y0, X1, Y1, value):
        a, b = px(X0, Y1), px(X1, Y0)
        cv2.rectangle(tex, a, (b[0] - 1, b[1] - 1), int(value), -1)   # cv2 는 끝점 포함 → 한 칸 줄여 중심 보존

    yy, xx = np.mgrid[0:H, 0:W]
    tex = (150 + 18 * np.sin(xx / 37.0 + 3 * np.sin(yy / 230.0)) + rng.normal(0, 6, (H, W))).clip(0, 255).astype(np.uint8)
    if clutter:
        for _ in range(9):     # 바닥 틈(가늘고 긴 어두운 선)
            y0 = rng.uniform(0, H)
            cv2.line(tex, (0, int(y0)), (W, int(y0 + rng.uniform(-60, 60))), int(rng.uniform(40, 70)), int(rng.integers(2, 4)))
        for _ in range(14):    # 옹이(흰 테두리 없는 어두운 덩어리)
            c = (int(rng.uniform(0, W)), int(rng.uniform(0, H)))
            cv2.ellipse(tex, c, (int(rng.uniform(8, 30)), int(rng.uniform(4, 12))), float(rng.uniform(0, 180)), 0, 360,
                        int(rng.uniform(25, 60)), -1)
    rect(-0.15, 0.115, 0.15, 0.30, 125)                   # 상자
    rect(-0.20, 0.10, 0.20, 0.115, 238)                   # 윗판
    half = spacing / 2
    rect(-half + leg_w / 2, -0.0125, half - leg_w / 2, 0.0125, 190)   # 가로대
    for k in range(int((spacing - leg_w) / 0.010)):
        X = -half + leg_w / 2 + 0.005 + 0.010 * k
        cv2.circle(tex, px(X, 0.0), int(0.0025 * PPM), 45, -1)        # 가로대 구멍(작은 어두운 점)
    for side, sx in (("left", -1), ("right", 1)):
        X0 = sx * half
        rect(X0 - leg_w / 2, -centre_h, X0 + leg_w / 2, 0.10, 185)    # 다리
        for Y in (0.045, 0.075, -0.040):
            cv2.circle(tex, px(X0, Y), int(0.003 * PPM), 70, -1)       # 다리 구멍
        rect(X0 - leg_w / 2, -band_h / 2, X0 + leg_w / 2, band_h / 2, 245)   # 흰 테이프 바탕
        if side not in hide:
            rect(X0 - marker[0] / 2, -marker[1] / 2, X0 + marker[0] / 2, marker[1] / 2, 18)
    for X, Y, w, h, border in decoys:
        if border:
            rect(X - w / 2 - 0.008, Y - h / 2 - 0.008, X + w / 2 + 0.008, Y + h / 2 + 0.008, 245)
        rect(X - w / 2, Y - h / 2, X + w / 2, Y + h / 2, 18)
    return tex, (ou, ov)


def render(tex, origin, K, pose, roll_deg=0.0, size=(640, 480), background=150):
    """2배 크기로 그린 뒤 넓이 평균으로 줄인다(계단 현상 없이 서브픽셀 위치 보존)."""
    x, y, heading, dh, pitch = pose
    R = camera_rotation(heading, pitch, math.radians(roll_deg))
    C = np.array([-y, dh, -x])
    Ht, Wt = tex.shape[:2]
    ou, ov = origin
    src = np.array([[0, 0], [Wt, 0], [Wt, Ht], [0, Ht]], np.float32)
    S = np.array([[(u - ou) / PPM, (ov - v) / PPM, 0.0] for u, v in src])
    cam = (R @ (S - C).T).T
    if np.any(cam[:, 2] <= 0.05):
        return np.full((size[1], size[0], 3), background, np.uint8)
    K2 = np.asarray(K, float).copy()
    K2[0, 0] *= 2
    K2[1, 1] *= 2
    K2[0, 2] = 2 * K2[0, 2] + 0.5
    K2[1, 2] = 2 * K2[1, 2] + 0.5
    uv = (K2 @ cam.T).T
    uv = (uv[:, :2] / uv[:, 2:3]).astype(np.float32)
    Hm = cv2.getPerspectiveTransform(src, uv)
    big = cv2.warpPerspective(tex, Hm, (2 * size[0], 2 * size[1]), flags=cv2.INTER_LINEAR, borderValue=background)
    img = cv2.resize(big, size, interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def light(img, gain=1.0, gamma=1.0, noise=0.0, shadow=None, jpeg=None, rng=None):
    """밝기 변화: 이득·감마·잡음·그림자(x0, y0, x1, y1, 곱) · JPEG 압축."""
    f = img.astype(np.float32) / 255.0
    f = np.power(np.clip(f * gain, 0, 1), gamma)
    if shadow is not None:
        x0, y0, x1, y1, k = shadow
        f[y0:y1, x0:x1] *= k
    out = f * 255.0
    if noise:
        out = out + (rng or np.random.default_rng(0)).normal(0, noise, out.shape)
    out = np.clip(out, 0, 255).astype(np.uint8)
    if jpeg:
        ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg)])
        out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return out
