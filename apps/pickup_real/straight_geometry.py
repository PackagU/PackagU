"""직진 전용 접근의 영상 기하 — 표식 두 중심(화소) → 로봇 기준 두 표식 위치·팔레트 중심 좌우·앞면 기울기.

왜 따로 두나(2026-10-31 실측): 두 표식의 세로 화소 차는 "앞면 yaw" 와 "카메라 roll" 이 거의 같은 영상을 만든다
(보기 자세 B 에서 roll 1° ≈ yaw 2.45°). 설정 roll 0° 로 풀면 사람이 정면으로 맞춘 배치가 yaw −11°·포크 좌우 +23 cm
로 계산됐다. 반면 각 표식 광선과 표식 높이 평면의 교점(로봇 기준 위치)과 그 가운데의 좌우 값은 roll 가정에 거의
무관했다(같은 영상에서 roll 0~5° 사이 +1.4~−0.3 cm). 그래서
  - 사람이 정면으로 맞춘 기준 자세에서 "앞면 기울기 = 0" 이 되는 roll 을 역산해 기준으로 기록하고(teach 파일은 그대로)
  - 접근 중에는 그 roll 로 같은 계산을 반복해 기준 대비 변화(좌우·기울기·거리)로 판단한다.

로봇 기준 좌표: x 앞, y 왼쪽, 원점 = 바퀴 축 가운데. 장착 mount = {forward, left, yaw(0)}.
장치 접근이나 이동 명령은 하지 않는다(numpy·OpenCV 만).
"""
import math

import numpy as np

from tape_vision import _floor_points, camera_axes, normalise, solve_pose


def markers_in_robot(est, centres, roll, mount):
    """centres {"left": [u, v], "right": [u, v]} → dict(pitch, L, R, mid, lateral, face_yaw) 또는 (None, 이유).

    face_yaw: 오른쪽 표식이 더 멀면 + (로봇이 앞면에 수직이면 0). lateral: 팔레트 중심(두 표식 가운데)의 로봇 기준 y.
    """
    rays = normalise(est.K, est.D, [centres["left"], centres["right"]])
    cam, why = solve_pose(rays, est.spacing, est.dh, roll)
    if cam is None:
        return None, why
    pts = _floor_points(rays, cam["pitch_down"], roll, est.dh)
    if pts is None:
        return None, "rays above marker plane"
    yaw = float(mount.get("yaw", 0.0))
    if abs(yaw) > 1e-9:
        return None, "straight mode assumes camera mount yaw 0"
    fwd, left = float(mount["forward"]), float(mount["left"])
    L = np.array([fwd + pts[0][1], left - pts[0][0]])
    R = np.array([fwd + pts[1][1], left - pts[1][0]])
    mid = 0.5 * (L + R)
    face_yaw = math.atan2(R[0] - L[0], L[1] - R[1])
    return {"pitch": float(cam["pitch_down"]), "L": L, "R": R, "mid": mid, "lateral": float(mid[1]),
            "forward": float(mid[0]), "face_yaw": float(face_yaw), "spacing_m": float(np.linalg.norm(L - R))}, None


def roll_for_square_face(est, centres, mount, lo_deg=-12.0, hi_deg=12.0):
    """사람이 정면으로 맞춘 기준 자세: 앞면 기울기 0 이 되는 카메라 roll(rad). 범위 안에 해가 없으면 (None, 이유)."""
    def f(r):
        m, _ = markers_in_robot(est, centres, r, mount)
        return None if m is None else m["face_yaw"]

    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    flo, fhi = f(lo), f(hi)
    if flo is None or fhi is None:
        return None, "pose solve failed at roll limits"
    if flo * fhi > 0:
        return None, "no roll in %.0f..%.0f deg makes the face square (%.1f..%.1f deg)" % (
            lo_deg, hi_deg, math.degrees(flo), math.degrees(fhi))
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        fm = f(mid)
        if fm is None:
            return None, "pose solve failed during roll search"
        if (fm > 0) == (flo > 0):
            lo, flo = mid, fm
        else:
            hi = mid
    return 0.5 * (lo + hi), None


def project_robot_point(est, point, pitch, roll, mount):
    """로봇 기준 표식 높이의 점 (x, y) → 화소 (u, v). 카메라 뒤면 None."""
    right = float(mount["left"]) - float(point[1])
    forward = float(point[0]) - float(mount["forward"])
    W = np.array([right, est.dh, forward])
    c = camera_axes(pitch, roll) @ W
    if c[2] <= 1e-3:
        return None
    return (float(est.K[0, 0] * c[0] / c[2] + est.K[0, 2]), float(est.K[1, 1] * c[1] / c[2] + est.K[1, 2]))


def visible_after(est, ref, travel, roll, mount, size=(640, 480), margin_side=30, margin_bottom=24):
    """기준 관측(ref: markers_in_robot 결과)에서 travel 만큼 곧게 간 뒤 두 표식 화소와 여유. 둘 다 여유 안이면 True."""
    shift = np.array([travel, 0.0])
    uv = [project_robot_point(est, p - shift, ref["pitch"], roll, mount) for p in (ref["L"], ref["R"])]
    if any(p is None for p in uv):
        return False, uv
    w, h = size
    ok = all(margin_side <= u <= w - margin_side and margin_side <= v <= h - margin_bottom for u, v in uv)
    return ok, uv
