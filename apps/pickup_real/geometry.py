"""택배 집기 기하 — ArUco 시트 점, PnP 결과 → 바닥 좌표계 자세, 차동 구동 이동 계획.

좌표계
- 시트 S: 원점 = 두 마커 중심을 잇는 선의 가운데(시트 중심선 위). X = 로봇에서 볼 때 오른쪽, Y = 위,
  Z = 시트에서 로봇 쪽. 인쇄한 마커를 정면에서 본 그대로다.
- 포켓 P(바닥, 위에서 본 오른손 좌표): 원점 = 시트 중심선이 시트 면과 만나는 곳. x = 팔레트 안쪽(포크가 들어가는
  방향), y = 정렬된 로봇의 왼쪽. 정렬된 로봇은 heading 0, x 는 음수.
- 로봇 R: x 전방, y 왼쪽, 원점 = 회전 중심(두 바퀴 축 가운데).

numpy 만 쓴다(Jetson 호스트 Python 3.8, 데스크톱 시험 공용).
"""
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def marker_corners(center_x, size):
    """ArUco 모서리 순서(왼위·오른위·오른아래·왼아래), 시트 S 좌표."""
    h = size / 2.0
    return np.array([[center_x - h, h, 0.0], [center_x + h, h, 0.0],
                     [center_x + h, -h, 0.0], [center_x - h, -h, 0.0]], float)


def sheet_points(marker_cfg):
    size = float(marker_cfg["size_m"])
    return {int(i): marker_corners(float(x), size)
            for i, x in zip(marker_cfg["ids"], marker_cfg["center_x_m"])}


def camera_in_pocket(R, t):
    """solvePnP 결과(p_cam = R p_S + t) → 카메라 자세 (x, y, heading) in P + 진단값.

    시트가 바닥에 수직으로 서 있다고 본다. 카메라의 기울기(pitch·roll)는 결과에서 빠진다.
    """
    R = np.asarray(R, float)
    t = np.asarray(t, float).reshape(3)
    centre = -R.T @ t                     # 카메라 중심, S 좌표
    axis = R.T @ np.array([0.0, 0.0, 1.0])  # 광축 방향, S 좌표
    x, y = -centre[2], -centre[0]
    heading = math.atan2(-axis[0], -axis[2])
    pitch_down = math.atan2(-axis[1], math.hypot(axis[0], axis[2]))
    return {"x": float(x), "y": float(y), "heading": float(heading),
            "height_above_sheet": float(centre[1]), "pitch_down": float(pitch_down),
            "range": float(np.linalg.norm(t))}


def robot_from_camera(cam, mount):
    """카메라 자세(P) + 카메라 장착(R: forward, left, yaw) → 로봇 회전 중심 자세 (x, y, th) in P."""
    th = wrap(cam["heading"] - mount["yaw"])
    fx, fy = mount["forward"], mount["left"]
    x = cam["x"] - (fx * math.cos(th) - fy * math.sin(th))
    y = cam["y"] - (fx * math.sin(th) + fy * math.cos(th))
    return (x, y, th)


def teach_mount(cam, forward, target_y):
    """정렬된 자세(th = 0, y = target_y)에서 본 카메라 → 카메라 left·yaw, predock 로봇 x."""
    return {"forward": float(forward), "left": float(cam["y"] - target_y), "yaw": float(cam["heading"]),
            "predock_x": float(cam["x"] - forward)}


def fork_errors(pose, target):
    """포크 기준 오차: lateral(+ = 포크가 포켓 중심보다 왼쪽), yaw, 거리(+ = 목표보다 앞쪽)."""
    x, y, th = pose
    xt, yt, tht = target
    return {"lateral": y - yt, "yaw": wrap(th - tht), "along": x - xt}


def plan_moves(pose, target, tol, max_path_angle, max_back):
    """차동 구동: 제자리 회전 + 직진만으로 pose → target(heading 0) 으로 가는 계획.

    옆 오차가 허용치 안이면 회전 → 거리 맞춤. 밖이면 비스듬히(각도 <= max_path_angle) 가서 고친다.
    앞 거리가 모자라면(목표에 너무 가깝거나 지나침) 먼저 정면으로 돌고 뒤로 물러난다.
    반환: (moves, note). moves = [("turn", rad) | ("straight", m)]. 불가능하면 moves=None.
    """
    x, y, th = pose
    xt, yt, tht = target
    ex, ey = xt - x, yt - y
    moves = []
    if abs(ey) <= tol["lateral"]:
        if abs(wrap(tht - th)) > tol["yaw"]:
            moves.append(("turn", wrap(tht - th)))
        if abs(ex) > tol["distance"]:
            moves.append(("straight", ex))
        return moves, "heading/distance only"
    need = abs(ey) / math.tan(max_path_angle)
    note = "diagonal"
    if ex < need:
        back = need - ex
        if back > max_back:
            return None, "needs %.2f m reverse (> %.2f m allowed)" % (back, max_back)
        if abs(wrap(tht - th)) > math.radians(0.5):
            moves.append(("turn", wrap(tht - th)))
            th = tht
        moves.append(("straight", -back))
        x -= back
        ex = xt - x
        note = "reverse %.2f m then diagonal" % back
    beta = math.atan2(ey, ex)
    moves.append(("turn", wrap(beta - th)))
    moves.append(("straight", math.hypot(ex, ey)))
    moves.append(("turn", wrap(tht - beta)))
    return [m for m in moves if not (m[0] == "turn" and abs(m[1]) < math.radians(0.3))
            and not (m[0] == "straight" and abs(m[1]) < 0.005)], note


def simulate_moves(pose, moves):
    """오프라인 시험용: 계획을 이상적으로 실행한 결과 자세."""
    x, y, th = pose
    for kind, value in moves:
        if kind == "turn":
            th = wrap(th + value)
        else:
            x += value * math.cos(th)
            y += value * math.sin(th)
    return (x, y, th)
