"""ArUco 시트 인식 → 카메라 자세. OpenCV 4.5(데스크톱, 옛 API)·4.10(Jetson venv, 새 API) 둘 다 지원.

시뮬레이터 정답 없음. 입력은 카메라 영상과 설정(마커 치수·카메라 내부값)뿐이다.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

from geometry import camera_in_pocket, sheet_points


def aruco_dictionary(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_detector(name):
    aruco = cv2.aruco
    dictionary = aruco_dictionary(name)
    if hasattr(aruco, "ArucoDetector"):
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        detector = aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return lambda image: aruco.detectMarkers(image, dictionary, parameters=params)


def marker_image(name, marker_id, pixels):
    aruco = cv2.aruco
    dictionary = aruco_dictionary(name)
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(dictionary, int(marker_id), int(pixels))
    return aruco.drawMarker(dictionary, int(marker_id), int(pixels))


def intrinsics(cam_cfg, focal_scale=1.0, root=None):
    """카메라 내부값. 보정 파일이 있으면 그것, 없으면 수평 화각 가정값에서 만든다."""
    path = cam_cfg.get("calibration_file") or ""
    if path:
        p = Path(path)
        if root is not None and not p.is_absolute():
            p = Path(root) / p
        data = json.loads(p.read_text())
        K = np.array(data["k"], float).reshape(3, 3)
        D = np.array(data.get("d", [0, 0, 0, 0, 0]), float)
        source = "calibration:" + p.name
    else:
        w, h = float(cam_cfg["width"]), float(cam_cfg["height"])
        f = (w / 2.0) / math.tan(math.radians(float(cam_cfg["hfov_deg"])) / 2.0)
        K = np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], float)
        D = np.zeros(5)
        source = "hfov %.1f deg [assumed]" % float(cam_cfg["hfov_deg"])
    K = K.copy()
    K[0, 0] *= focal_scale
    K[1, 1] *= focal_scale
    return K, D, source + ("" if focal_scale == 1.0 else " x%.4f(teach)" % focal_scale)


class MarkerEstimator:
    """한 프레임 → 시트(=포켓 중심선) 대비 카메라 자세. 두 마커가 다 보여야 유효(require_both)."""

    def __init__(self, cfg, focal_scale=1.0, root=None):
        self.marker_cfg = cfg["marker"]
        self.cam_cfg = cfg["camera"]
        self.detect = make_detector(self.marker_cfg["dictionary"])
        self.points = sheet_points(self.marker_cfg)
        self.K, self.D, self.k_source = intrinsics(self.cam_cfg, focal_scale, root)

    def estimate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.detect(gray)
        found = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}
        wanted = [i for i in self.points if i in found]
        result = {"ids": sorted(found), "corners": {i: found[i].tolist() for i in wanted}, "ok": False}
        need = len(self.points) if self.marker_cfg.get("require_both", True) else 1
        if len(wanted) < need:
            result["reason"] = "markers %s not all visible" % sorted(self.points)
            return result
        obj = np.concatenate([self.points[i] for i in wanted]).astype(np.float64)
        img = np.concatenate([found[i] for i in wanted]).astype(np.float64)
        ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj, img, self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
        if not ok or len(rvecs) == 0:
            result["reason"] = "PnP failed"
            return result
        errors = np.asarray(errors, float).reshape(-1)
        order = np.argsort(errors)
        best = int(order[0])
        rvec, tvec = rvecs[best], tvecs[best]
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, self.D, rvec.copy(), tvec.copy())
        projected, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
        reproj = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img) ** 2, axis=1))))
        R = cv2.Rodrigues(rvec)[0]
        cam = camera_in_pocket(R, tvec)
        if len(order) > 1:
            alt = camera_in_pocket(cv2.Rodrigues(rvecs[int(order[1])])[0], tvecs[int(order[1])])
            ratio = float(errors[order[1]] / max(errors[best], 1e-6))
            cam["alt_heading"] = alt["heading"]
            cam["ambiguity_ratio"] = ratio
            # 두 해의 오차가 비슷한데 방향이 크게 다르면 평면 모호성 → 거부
            if ratio < 1.5 and abs(alt["heading"] - cam["heading"]) > math.radians(3.0):
                result.update(reason="planar ambiguity (ratio %.2f)" % ratio, cam=cam)
                return result
        if reproj > float(self.cam_cfg["max_reprojection_px"]):
            result.update(reason="reprojection %.2f px" % reproj, cam=cam)
            return result
        result.update(ok=True, cam=cam, reprojection_px=reproj, rvec=rvec.reshape(3).tolist(),
                      tvec=tvec.reshape(3).tolist())
        return result

    def annotate(self, frame, est, lines=()):
        out = frame.copy()
        for i, c in est.get("corners", {}).items():
            pts = np.array(c, np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [pts], True, (0, 255, 0) if est.get("ok") else (0, 165, 255), 2)
            cv2.putText(out, "id%d" % i, tuple(int(v) for v in c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        if est.get("ok"):
            cv2.drawFrameAxes(out, self.K, self.D, np.array(est["rvec"]), np.array(est["tvec"]), 0.05, 2)
            mid = (out.shape[1] // 2, 0)
            cv2.line(out, mid, (mid[0], out.shape[0]), (80, 80, 80), 1)
        y = 22
        for text in lines:
            cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
            cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            y += 22
        if not est.get("ok"):
            cv2.putText(out, est.get("reason", "")[:60], (8, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)
        return out


def sheet_texture(cfg, px_per_m=2000):
    """A4 가로 시트(마커 2개) 영상과 S 원점(시트 중심) 픽셀 위치."""
    m = cfg["marker"]
    W, H = int(0.297 * px_per_m), int(0.210 * px_per_m)
    sheet = np.full((H, W), 255, np.uint8)
    side = int(round(float(m["size_m"]) * px_per_m))
    for i, cx in zip(m["ids"], m["center_x_m"]):
        tile = marker_image(m["dictionary"], i, side)
        u0 = int(round(W / 2 + cx * px_per_m - side / 2))
        v0 = int(round(H / 2 - side / 2))
        sheet[v0:v0 + side, u0:u0 + side] = tile
    return sheet, (W / 2.0, H / 2.0)


def render_plane(texture, px_per_m, origin_px, K, cam_pose_in_pocket, size=(640, 480), background=150,
                 noise=0.0, rng=None):
    """오프라인 시험·미리보기: 시트 면(S 의 Z=0)에 붙은 영상을 주어진 카메라 자세에서 본 합성 영상(바늘구멍 모델).

    texture 의 (u, v) 픽셀 ↔ S (X, Y) = ((u - ou) / px_per_m, (ov - v) / px_per_m).
    cam_pose_in_pocket = (x, y, heading, height_above_sheet_centre, pitch_down).
    """
    Ht, Wt = texture.shape[:2]
    ou, ov = origin_px
    x, y, heading, height, pitch = cam_pose_in_pocket
    C = np.array([-y, height, -x])  # S 좌표의 카메라 중심 (P x = -Z, y = -X)
    fwd = np.array([-math.sin(heading) * math.cos(pitch), -math.sin(pitch), -math.cos(heading) * math.cos(pitch)])
    right = np.cross(fwd, np.array([0.0, 1.0, 0.0]))
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    Rcs = np.vstack([right, down, fwd])  # S → 카메라 광학 좌표
    t = -Rcs @ C
    src = np.array([[0, 0], [Wt, 0], [Wt, Ht], [0, Ht]], np.float32)
    corners_s = np.array([[(u - ou) / px_per_m, (ov - v) / px_per_m, 0.0] for u, v in src])
    cam = (Rcs @ corners_s.T).T + t
    channels = 3 if texture.ndim == 3 else 1
    if np.any(cam[:, 2] <= 0.05):
        return np.full((size[1], size[0], 3), background, np.uint8)
    uv = (K @ cam.T).T
    uv = (uv[:, :2] / uv[:, 2:3]).astype(np.float32)
    Hm = cv2.getPerspectiveTransform(src, uv)
    border = (background,) * channels
    img = cv2.warpPerspective(texture, Hm, size, flags=cv2.INTER_AREA, borderValue=border)
    if channels == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if noise > 0:
        rng = rng or np.random.default_rng(0)
        img = np.clip(img.astype(float) + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


def render_sheet_view(cfg, K, cam_pose_in_pocket, size=(640, 480), background=150, noise=0.0, rng=None):
    sheet, origin = sheet_texture(cfg)
    return render_plane(sheet, 2000, origin, K, cam_pose_in_pocket, size, background, noise, rng)
