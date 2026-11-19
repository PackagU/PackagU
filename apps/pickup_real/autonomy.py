"""집기 목표 연속성·시야 여유 계산. 장치 접근이나 이동 명령은 하지 않는다."""
import itertools
import math

import numpy as np

from geometry import robot_from_camera, wrap


def tape_hypotheses(estimator, result):
    """기존 검출 규칙을 통과하는 각 쌍을 별도 자세 가설로 평가한다."""
    from tape_vision import pair_markers
    feats = result.get("_feats") or {}
    if not str(result.get("reason", "")).startswith("ambiguous:"):
        return [result] if result.get("ok") else []
    hypotheses = []
    for a, b in itertools.combinations(feats.get("candidates", []), 2):
        pair, why, _ = pair_markers([a, b], [], estimator.t, int(estimator.cam_cfg["width"]))
        if why:
            continue
        selected = dict(feats, pair=pair)
        selected.pop("reason", None)
        selected.pop("edge", None)
        candidate = estimator.estimate_features(selected)
        candidate["stamp"] = result["stamp"]
        if candidate.get("ok"):
            hypotheses.append(candidate)
    return hypotheses


def select_target(estimator, result, context, teach):
    """초기 운영자 목표 선택, 이후 odom 예측과 일치하는 동일 목표만 허용."""
    if not context:
        return result
    if not isinstance(context, dict):
        raise ValueError("target selection must be an object")
    candidates = tape_hypotheses(estimator, result)
    expected = context.get("expected_robot")
    if expected is not None:
        if len(expected) != 3 or not all(math.isfinite(float(v)) for v in expected):
            raise ValueError("invalid expected target pose")
        if not teach or "mount" not in teach:
            return dict(result, ok=False, reason="selected target needs valid teach")
        scale = float(teach.get("position_scale", 1.0))
        accepted = []
        for candidate in candidates:
            cam = candidate["cam"]
            pose = robot_from_camera(dict(cam, x=cam["x"] * scale, y=cam["y"] * scale), teach["mount"])
            if (math.hypot(pose[0] - expected[0], pose[1] - expected[1]) <= 0.12
                    and abs(wrap(pose[2] - expected[2])) <= math.radians(8)):
                accepted.append(candidate)
        if len(accepted) != 1:
            if not accepted and context.get("allow_partial"):
                partial = single_endpoint_pose(estimator, result, context, teach)
                if partial is not None:
                    return partial
            if not candidates:
                return result
            return dict(result, ok=False, reason="selected target disagrees with odometry (%d matches)" % len(accepted))
        if context.get("label"):
            # 숫자 모드: 따라가는 팔레트 위에서 다른 송장 숫자가 읽히면 그 프레임은 쓰지 않는다
            from parcel_target import label_conflict
            other = label_conflict(accepted[0], result.get("_scene"), context["label"])
            if other:
                return dict(result, ok=False, reason="tracked pallet shows label %s, not %s" % (other, context["label"]))
        return accepted[0]
    if context.get("bearing_deg") is not None:
        # 숫자 모드 처음 목표: 정면 자세에서 송장 숫자가 보인 방향의 팔레트(보기 자세에서는 라벨이 화면 밖일 수 있다)
        from parcel_target import choose_by_bearing
        chosen, why, summary = choose_by_bearing(estimator, candidates, float(context["bearing_deg"]),
                                                 float(context.get("bearing_tol_deg", 8.0)))
        if chosen is None:
            return dict(result, ok=False, reason="target: " + why, target_summary=summary)
        return dict(chosen, target_summary=summary)
    if context.get("parcel") or context.get("label"):
        # 처음 목표: 택배(상자)가 올라간 팔레트 또는 송장 숫자가 맞는 팔레트. 근거는 카메라 서버가 프레임마다 붙인 _scene
        from parcel_target import choose
        chosen, why, summary = choose(candidates, result.get("_scene"), context)
        if chosen is None:
            return dict(result, ok=False, reason="target: " + why, target_summary=summary)
        return dict(chosen, target_summary=summary)
    side = context.get("side")
    if side not in ("left", "right"):
        raise ValueError("target side must be left or right")
    if not candidates:
        return result
    candidates.sort(key=lambda r: sum(r["tape"][k][0] for k in ("left", "right")))
    selected = candidates[0 if side == "left" else -1]
    # 목표 표식 줄과 무관한 바닥·상자 그림자를 목표 잘림으로 오인하지 않는다.
    a, b = selected["_feats"]["pair"]
    slope = (b["centre"][1] - a["centre"][1]) / (b["centre"][0] - a["centre"][0])
    for cut in (result.get("_feats") or {}).get("cut", []):
        if cut.get("edge") != side:
            continue
        x, y = cut["centroid"]
        expected_y = a["centre"][1] + slope * (x - a["centre"][0])
        if abs(y - expected_y) <= 2 * max(a["short"], b["short"]) + 4:
            return dict(result, ok=False, reason="selected target cut at image edge (%s)" % side)
    return selected


def single_endpoint_pose(estimator, result, context, teach):
    """이전에 확정한 정적 목표·odom yaw·고정 pitch로 보이는 끝점 하나의 위치를 계산."""
    from tape_vision import normalise, _floor_points
    if not teach or "pitch_down" not in context:
        return None
    pitch = float(context["pitch_down"])
    pose = context["expected_robot"]
    mount = teach["mount"]
    heading = wrap(float(pose[2]) + float(mount["yaw"]))
    camera_x = pose[0] + mount["forward"] * math.cos(pose[2]) - mount["left"] * math.sin(pose[2])
    camera_y = pose[1] + mount["forward"] * math.sin(pose[2]) + mount["left"] * math.cos(pose[2])
    scale = float(teach.get("position_scale", 1.0))
    if abs(scale - 1.0) > 1e-6:  # 원근과 별도 거리 배율을 혼합하지 않는다.
        return None
    width, height = int(estimator.cam_cfg["width"]), int(estimator.cam_cfg["height"])
    cp, sp, ch, sh = math.cos(pitch), math.sin(pitch), math.cos(heading), math.sin(heading)
    projected = []
    marker_y = [estimator.spacing / 2, -estimator.spacing / 2]  # 화면 왼쪽/오른쪽 표식
    for y in marker_y:
        dx, dy = -camera_x, y - camera_y
        right = dx * sh - dy * ch
        down = -sp * (dx * ch + dy * sh) + estimator.dh * cp
        depth = cp * (dx * ch + dy * sh) + estimator.dh * sp
        if depth <= 0:
            return None
        cr, sr = math.cos(estimator.roll), math.sin(estimator.roll)
        u = estimator.K[0, 0] * (cr * right + sr * down) / depth + estimator.K[0, 2]
        v = estimator.K[1, 1] * (-sr * right + cr * down) / depth + estimator.K[1, 2]
        projected.append(np.array([u, v]))
    matches = []
    for index, predicted_uv in enumerate(projected):
        other = projected[1 - index]
        # 보이지 않는 쪽이 실제로 화면 가장자리 근처여야 한다. 가운데 가림을 무시하지 않는다.
        if 40 < other[0] < width - 40 and 40 < other[1] < height - 40:
            continue
        for candidate in (result.get("_feats") or {}).get("candidates", []):
            if candidate["short"] < float(estimator.t.get("min_marker_short_px", 6)):
                continue
            if np.linalg.norm(candidate["centre"] - predicted_uv) <= 40:
                matches.append((index, candidate))
    if len(matches) != 1:
        return None
    index, candidate = matches[0]
    points = _floor_points(normalise(estimator.K, estimator.D, [candidate["centre"]]), pitch, estimator.roll, estimator.dh)
    if points is None:
        return None
    right, forward = points[0]
    dx = forward * ch + right * sh
    dy = forward * sh - right * ch
    cam = {"x": float(-dx), "y": float(marker_y[index] - dy), "heading": heading,
           "pitch_down": pitch, "height_above_sheet": estimator.dh}
    measured = robot_from_camera(cam, mount)
    if math.hypot(measured[0] - pose[0], measured[1] - pose[1]) > 0.05:
        return None
    return {"ok": True, "stamp": result["stamp"], "cam": cam, "mode": "tape",
            "measurement_mode": "single_endpoint+odometry", "reprojection_px": 0.0,
            "tape": {"observed_marker": "left" if index == 0 else "right"}, "_feats": result.get("_feats")}


def recovery_hint(fresh, n, now, max_age):
    """새 프레임 n개가 같은 좌/우 잘림을 보일 때만 작은 회전 복구를 제안."""
    if len(fresh) < n or now - fresh[-1][0] > max_age:
        return None
    edges = []
    for _, result in fresh[-n:]:
        reason = str(result.get("reason", ""))
        feats = result.get("_feats") or {}
        if not feats.get("candidates"):
            return None  # 화면 끝의 바닥 틈만으로 목표를 추측하지 않는다.
        edge = feats.get("edge")
        if reason.startswith("selected target cut at image edge"):
            edge = "left" if "(left)" in reason else "right" if "(right)" in reason else None
        if edge not in ("left", "right"):
            return None
        edges.append(edge)
    if len(set(edges)) != 1:
        return None
    return {"kind": "edge", "edge": edges[0], "fresh_frames": n}


def visible_look_x(cfg, teach, pitch, current_x):
    """정렬 완료 자세에서 두 표식이 화면 여유 안에 드는 관찰 거리(가까워지는 변경 없음)."""
    from vision import intrinsics
    K, _, _ = intrinsics(cfg["camera"], float(teach.get("focal_scale", 1.0)))
    width, height = int(cfg["camera"]["width"]), int(cfg["camera"]["height"])
    margin = float(cfg["align"].get("visibility_margin_px", 64))
    spacing = float(cfg["tape"]["center_spacing_m"])
    dh = float(cfg["tape"]["camera_height_m"]) - float(cfg["tape"]["marker_center_height_m"])
    mount = teach["mount"]
    heading = float(mount["yaw"])
    roll = math.radians(float(cfg["tape"].get("camera_roll_deg", 0.0)))
    cp, sp = math.cos(pitch), math.sin(pitch)
    ch, sh = math.cos(heading), math.sin(heading)
    # 원근 정렬은 삽입 최대 거리 안에서만 허용한다.
    goal_x = float(teach["predock_x"]) + float(teach["gap_m"]) + float(cfg["insert"]["depth_m"])
    max_range = float(cfg["insert"].get("max_total_m", 0.9)) - goal_x
    for distance in np.arange(-current_x, max_range + 0.001, 0.02):
        camera_x = -distance + float(mount["forward"])
        camera_y = float(teach["target_y"]) + float(mount["left"])
        pixels = []
        for marker_y in (-spacing / 2, spacing / 2):
            dx, dy = -camera_x, marker_y - camera_y
            right = dx * sh - dy * ch
            down = -sp * (dx * ch + dy * sh) + dh * cp
            depth = cp * (dx * ch + dy * sh) + dh * sp
            if depth <= 0:
                break
            xr = math.cos(roll) * right + math.sin(roll) * down
            yr = -math.sin(roll) * right + math.cos(roll) * down
            pixels.append((K[0, 0] * xr / depth + K[0, 2], K[1, 1] * yr / depth + K[1, 2]))
        if len(pixels) == 2 and all(margin <= u <= width - margin and margin <= v <= height - margin for u, v in pixels):
            return -float(distance)
    raise ValueError("two markers need a view point beyond the allowed insertion distance")
