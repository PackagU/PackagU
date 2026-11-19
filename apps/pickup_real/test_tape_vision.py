#!/usr/bin/env python3
"""검정 표식 2개 모드 오프라인 시험(데스크톱·Jetson, 장치 없음).

합성 장면(tape_synth: 다리·흰 띠·검정 표식·구멍 가로대·나뭇결·틈·옹이)을 실제로 렌더링해 검출기에 영상만 넘긴다.
정답 자세는 평가에만 쓴다. 실제 카메라 프레임 폴더가 로컬에 있으면 그것도 본다(영상은 저장소에 올리지 않는다).

    python3 apps/pickup_real/test_tape_vision.py
    python3 apps/pickup_real/test_tape_vision.py --real-ok DIR --real-neg DIR [DIR ...]
"""
import argparse
import copy
import glob
import json
import math
import sys
import tempfile
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import camera_server  # noqa: E402
import tape_synth as ts  # noqa: E402
import tape_vision as tv  # noqa: E402
from pallet_vision import camera_rotation  # noqa: E402

PITCH = math.radians(32)
LOCAL_FRAMES = HERE.parents[1] / "logs" / "pickup_real" / "marker_tape_20261031"


def load_cfg():
    cfg = yaml.safe_load((HERE / "config.yaml").read_text())
    cfg["vision"]["mode"] = "tape"
    cfg["camera"]["calibration_file"] = ""
    return cfg


def dh_of(cfg):
    return float(cfg["tape"]["camera_height_m"]) - float(cfg["tape"]["marker_center_height_m"])


def project_marker(K, pose, X, roll_deg=0.0):
    x, y, h, dh, p = pose
    R = camera_rotation(h, p, math.radians(roll_deg))
    pc = R @ (np.array([X, 0.0, 0.0]) - np.array([-y, dh, -x]))
    uv = K @ pc
    return uv[:2] / uv[2]


def errors(r, pose):
    c = r["cam"]
    return {"dx_mm": round((c["x"] - pose[0]) * 1000, 2), "dy_mm": round((c["y"] - pose[1]) * 1000, 2),
            "dh_deg": round(math.degrees(c["heading"] - pose[2]), 3)}


def within(e, pose, dx=0.006, dy=0.005, dh=0.35):
    return abs(e["dx_mm"]) <= (dx + 0.01 * abs(pose[0])) * 1000 and abs(e["dy_mm"]) <= dy * 1000 \
        and abs(e["dh_deg"]) <= dh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real-ok", default=str(LOCAL_FRAMES / "live_static1"),
                    help="두 표식이 보이는 실제 프레임 폴더(정지 상태). 없으면 건너뜀")
    ap.add_argument("--real-neg", nargs="*", default=[str(LOCAL_FRAMES / "jetson_looks"),
                                                       str(LOCAL_FRAMES / "jetson_looks_1030")],
                    help="표식이 없는 실제 프레임 폴더들(전부 거부돼야 함)")
    ap.add_argument("--out", default=str(HERE.parents[1] / "logs" / "pickup_real" / "tape_offline_test.json"))
    args = ap.parse_args()
    cfg = load_cfg()
    est = tv.TapeEstimator(cfg)
    K = est.K
    dh = dh_of(cfg)
    sp = float(cfg["tape"]["center_spacing_m"])   # 합성 받침도 설정 간격으로 그린다(설정값이 바뀌어도 시험이 따라감)
    tex, org = ts.face_texture(seed=3, spacing=sp)
    results = []

    def record(name, ok, **info):
        results.append(dict(case=name, ok=bool(ok), **info))

    def look(pose, texture=None, roll=0.0, estimator=None, **lighting):
        img = ts.render(texture if texture is not None else tex, org, K, pose[:3] + (dh, PITCH), roll)
        if lighting:
            img = ts.light(img, **lighting)
        return (estimator or est).estimate(img)

    # 1. 자세 격자 — 거리·좌우·각도(영상만 보고 푼 값과 정답 비교)
    grid = [(-0.45, 0.0, 0.0), (-0.60, 0.08, 0.0), (-0.60, -0.08, math.radians(-8)), (-0.75, 0.0, 0.0),
            (-0.75, 0.12, math.radians(5)), (-0.75, -0.12, math.radians(-5)), (-0.90, 0.05, math.radians(10)),
            (-0.90, -0.05, math.radians(-12)), (-1.10, 0.0, 0.0)]
    for pose in grid:
        r = look(pose)
        e = errors(r, pose) if r.get("ok") else None
        record("pose x%.2f y%+.2f h%+.0f" % (pose[0], pose[1], math.degrees(pose[2])),
               r.get("ok") and within(e, pose), err=e, reason=r.get("reason"),
               yaw_sd=(r.get("tape") or {}).get("heading_sigma_deg"))

    # 2. 표식 하나 가림 / 전부 가림 — 반대쪽을 추측하지 않고 실패
    #    둘 다 가리면 화면 가장자리의 다른 어두운 덩어리가 "잘린 표식"으로 보고될 수 있다(짝 비교 대상이 없음) → 둘 다 거부로 인정
    for hide, expect in ((("left",), ("only one",)), (("right",), ("only one",)),
                         (("left", "right"), ("no black marker", "cut at image edge"))):
        t2, o2 = ts.face_texture(seed=3, spacing=sp, hide=hide)
        r = look((-0.75, 0.0, 0.0), texture=t2)
        record("hidden %s" % "+".join(hide), (not r.get("ok")) and any(e in str(r.get("reason")) for e in expect),
               reason=r.get("reason"))

    # 3. 화면 끝 잘림 — 오른쪽 표식 중심이 영상 오른쪽 끝(u≈637)에 오도록 좌우 위치를 찾는다(y + = 카메라가 왼쪽 → 받침이 오른쪽)
    lo, hi = 0.0, 0.6
    for _ in range(40):
        mid = (lo + hi) / 2
        u = project_marker(K, (-0.75, mid, 0.0, dh, PITCH), cfg["tape"]["center_spacing_m"] / 2)[0]
        lo, hi = (lo, mid) if u > 637 else (mid, hi)
    r = look((-0.75, hi, 0.0))
    record("cut at right edge", (not r.get("ok")) and ("cut" in str(r.get("reason")) or "only one" in str(r.get("reason"))),
           reason=r.get("reason"), lateral_m=round(lo, 3))

    # 4. 바닥 무늬·흰 테두리 없는 검정 사각형(옹이·그림자 흉내) — 무시하고 정확
    decoys = [(-0.10, -0.15, 0.030, 0.020, False), (0.12, -0.20, 0.040, 0.020, False), (0.0, 0.05, 0.03, 0.02, False)]
    t4, _ = ts.face_texture(seed=5, spacing=sp, decoys=decoys)
    pose = (-0.75, 0.04, math.radians(4))
    r = look(pose, texture=t4)
    record("floor clutter + borderless black rects", r.get("ok") and within(errors(r, pose), pose),
           err=errors(r, pose) if r.get("ok") else None, reason=r.get("reason"))

    # 5. 흰 테두리 있는 가짜 쌍(같은 크기·같은 높이) — 쌍이 둘이면 구분 불가로 실패
    t5, _ = ts.face_texture(seed=5, spacing=sp, decoys=[(-0.10, -0.17, 0.030, 0.020, True), (0.20, -0.17, 0.030, 0.020, True)])
    r = look((-0.75, 0.0, 0.0), texture=t5)
    record("second white-bordered pair -> ambiguous", (not r.get("ok")) and "ambiguous" in str(r.get("reason")),
           reason=r.get("reason"))

    # 6. 밝기 변화: 그늘(이득 0.45)·과노출(1.6)·감마·한쪽 표식 그림자·잡음+JPEG
    pose = (-0.75, -0.05, math.radians(-3))
    ul = project_marker(K, pose[:3] + (dh, PITCH), -cfg["tape"]["center_spacing_m"] / 2)
    sh = (int(ul[0]) - 40, int(ul[1]) - 40, int(ul[0]) + 40, int(ul[1]) + 40, 0.5)
    for name, kw in (("shade gain 0.45", {"gain": 0.45}), ("sun gain 1.6", {"gain": 1.6}),
                     ("gamma 1.8", {"gamma": 1.8}), ("shadow on left marker", {"shadow": sh}),
                     ("noise 4 + jpeg 70", {"noise": 4.0, "jpeg": 70})):
        r = look(pose, **kw)
        record("light: " + name, r.get("ok") and within(errors(r, pose), pose, dh=0.5),
               err=errors(r, pose) if r.get("ok") else None, reason=r.get("reason"))

    # 7. 너무 멂 — 거부(작음 또는 yaw 불안정 또는 미검출)
    r = look((-2.3, 0.0, 0.0))
    record("too far 2.3 m refused", not r.get("ok"), reason=r.get("reason"))

    # 8. 카메라 roll 1° 가 설정과 다를 때 yaw 오차 = heading_per_roll 로 예측되는지, 설정을 맞추면 사라지는지
    pose = (-0.75, 0.0, 0.0)
    r = look(pose, roll=1.0)
    e = errors(r, pose) if r.get("ok") else None
    pred = (r.get("tape") or {}).get("heading_per_roll")
    good = bool(r.get("ok") and pred and abs(abs(e["dh_deg"]) - abs(pred)) <= 0.3 * abs(pred))
    cfg_roll = copy.deepcopy(cfg)
    cfg_roll["tape"]["camera_roll_deg"] = 1.0
    r2 = look(pose, roll=1.0, estimator=tv.TapeEstimator(cfg_roll))
    e2 = errors(r2, pose) if r2.get("ok") else None
    record("roll 1 deg unmodelled -> yaw error predicted; modelled -> removed",
           good and e2 is not None and abs(e2["dh_deg"]) <= 0.1, unmodelled=e, predicted_deg_per_deg=pred, modelled=e2)

    # 9. 중심 간격은 설정값을 그대로 쓴다(자동 재추정 없음): 실제 간격 0.32 m 받침을 설정 0.32 로 보면 정확,
    #    설정 0.303 으로 보면 거리가 틀린다(=간격을 영상에서 다시 맞추지 않는다)
    cfg_s, cfg_w = copy.deepcopy(cfg), copy.deepcopy(cfg)
    cfg_s["tape"]["center_spacing_m"] = 0.32
    cfg_w["tape"]["center_spacing_m"] = 0.303
    t9, _ = ts.face_texture(seed=3, spacing=0.32)
    pose = (-0.75, 0.0, 0.0)
    ra, rb = (look(pose, texture=t9, estimator=tv.TapeEstimator(cfg_s)),
              look(pose, texture=t9, estimator=tv.TapeEstimator(cfg_w)))
    ea = errors(ra, pose) if ra.get("ok") else None
    eb = errors(rb, pose) if rb.get("ok") else None
    record("spacing from config (no re-estimation)", ea is not None and within(ea, pose) and eb is not None
           and abs(eb["dx_mm"]) > 20, with_correct_spacing=ea, with_wrong_spacing=eb)

    # 10. 초점 배율 맞춤: 실제 화각이 가정보다 4 % 넓은 카메라로 12 cm 후진·전진 → 배율 복원, 간격은 그대로.
    #     12 cm 이동은 거리 오차 2 mm 가 배율 약 2~3 % 로 커진다(조건이 나쁨) → 3.5 % 안이면 통과, 값은 그대로 기록
    K_true = tv.intrinsics(dict(cfg["camera"], hfov_deg=cfg["camera"]["hfov_deg"] * 1.04))[0]
    centres = []
    for x in (-0.95, -1.07, -0.95):
        img = ts.render(tex, org, K_true, (x, 0.02, math.radians(2), dh, PITCH))
        rr = est.estimate(img)
        centres.append((rr["tape"]["left"], rr["tape"]["right"]) if rr.get("ok") else None)
    if all(centres):
        k, rms, cams = tv.fit_focal(cfg, centres, 0.12)
        truth = K_true[0, 0] / K[0, 0]
        record("focal fit from 12 cm motion", k is not None and abs(k - truth) < 0.035 and rms < 0.004,
               focal_scale=k, truth=round(truth, 4), rms_m=rms,
               headings_deg=[round(math.degrees(c["heading"]), 2) for c in cams] if cams else None)
    else:
        record("focal fit from 12 cm motion", False, reason="calibration photos not recognised")

    # 11. 보정 파일 호환: 옛 구멍용 teach·간격 바뀐 teach 거부, 새 모드용만 사용
    old = {"created": "2026-10-30 23:12:00", "camera_height_m": 0.48, "hole_pitch_m": 0.010, "focal_scale": 1.0,
           "mount": {"forward": 0.15, "left": 0.0, "yaw": 0.0}, "predock_x": -0.56, "target_y": 0.0, "gap_m": 0.30}
    good_t = dict(old, **tv.teach_fields(cfg))
    good_t.pop("hole_pitch_m")
    moved = dict(good_t, center_spacing_m=0.305)
    probs = [tv.teach_problem(old, cfg), tv.teach_problem(moved, cfg), tv.teach_problem(good_t, cfg),
             tv.teach_problem(dict(good_t, row_length_m=0.31), cfg)]
    record("teach compatibility", probs[0] and probs[1] and probs[2] is None and probs[3], problems=probs)
    with tempfile.TemporaryDirectory() as tmp:
        tp = Path(tmp) / tv.TEACH_FILE
        tp.write_text(json.dumps(dict(old, focal_scale=1.3, row_length_m=0.31, hole_polarity="bright")))
        svc = camera_server.CameraService(cfg, lambda: None, tp, tmp)
        rejected = svc.teach is None and svc.teach_error and svc.focal_scale == 1.0
        tp.write_text(json.dumps(dict(good_t, focal_scale=1.05)))
        svc.reload_teach()
        record("camera server ignores old teach, loads tape teach",
               rejected and svc.teach is not None and abs(svc.focal_scale - 1.05) < 1e-9 and svc.teach_error is None,
               first_error=None if not rejected else "rejected")

    # 12. 한 번 보기(중앙값): 정지면 통과·중심 기록, 프레임마다 튀면 거부, 영상 끊김·오래됨 거부
    def source_from(poses):
        frames = [ts.render(tex, org, K, p[:3] + (dh, PITCH)) for p in poses]
        state = {"i": 0}

        def src():
            time.sleep(1 / 30.0)
            state["i"] += 1
            return frames[state["i"] % len(frames)]
        return src

    with tempfile.TemporaryDirectory() as tmp:
        for name, poses, expect_ok in (("steady", [(-0.75, 0.0, 0.0)], True),
                                       ("jumping", [(-0.75, 0.0, 0.0), (-0.75, 0.03, 0.0)], False)):
            svc = camera_server.CameraService(cfg, source_from(poses), Path(tmp) / "none.json", tmp)
            th = threading.Thread(target=svc.run_forever, daemon=True)
            th.start()
            res = svc.look(n=8, timeout=6, tag=name)
            svc.stop()
            th.join(2)
            okk = bool(res.get("ok")) == expect_ok and (("tape_centres_px" in res) if expect_ok
                                                         else "unstable" in str(res.get("reason")))
            record("look %s" % name, okk, reason=res.get("reason"), centres=res.get("tape_centres_px"),
                   jump=res.get("centre_jump_px"))
        svc = camera_server.CameraService(cfg, lambda: (time.sleep(0.05), None)[1], Path(tmp) / "none.json", tmp)
        th = threading.Thread(target=svc.run_forever, daemon=True)
        th.start()
        res = svc.look(n=4, timeout=1.0, tag="lost")
        svc.stop()
        th.join(2)
        record("look image loss", (not res.get("ok")) and "image loss" in str(res.get("reason")), reason=res.get("reason"))

    # 13. 실제 프레임(로컬에만 있음)
    ok_dir = Path(args.real_ok)
    files = sorted(glob.glob(str(ok_dir / "*.jpg")) + glob.glob(str(ok_dir / "*.png")))
    if files:
        res = [est.estimate(cv2.imread(f)) for f in files]
        good = [r for r in res if r.get("ok")]
        px = np.array([r["tape"]["left"] + r["tape"]["right"] for r in good]) if good else np.zeros((0, 4))
        spread = float((px.max(axis=0) - px.min(axis=0)).max()) if len(px) else None
        heads = [math.degrees(r["cam"]["heading"]) for r in good]
        record("real frames with tape (%s)" % ok_dir.name, len(good) >= 0.9 * len(res) and spread is not None and spread <= 1.5,
               recognised="%d/%d" % (len(good), len(res)), centre_spread_px=spread,
               heading_range_deg=[round(min(heads), 2), round(max(heads), 2)] if heads else None,
               failures=sorted({r.get("reason") for r in res if not r.get("ok")}))
    else:
        record("real frames with tape", True, skipped="no local frames at %s" % ok_dir)
    neg = []
    for d in args.real_neg:
        neg += sorted(glob.glob(str(Path(d) / "*.jpg")) + glob.glob(str(Path(d) / "*_raw.png")))
    if neg:
        bad = [Path(f).name for f in neg if est.estimate(cv2.imread(f)).get("ok")]
        record("real frames without tape all refused", not bad, frames=len(neg), false_ok=bad)
    else:
        record("real frames without tape", True, skipped="no local frames")

    for r in results:
        print(("PASS " if r["ok"] else "FAIL ") + json.dumps(r, ensure_ascii=False, default=str)[:400])
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(results, indent=1, ensure_ascii=False, default=str))
    n = sum(r["ok"] for r in results)
    print("%d/%d PASS" % (n, len(results)))
    return 0 if n == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
