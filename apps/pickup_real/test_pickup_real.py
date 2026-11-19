#!/usr/bin/env python3
"""택배 집기 실행기 오프라인 시험(데스크톱, 장치 없음).

가짜 세계: 합성 카메라(가정 화각보다 4 % 넓음) + 바퀴 거리 -3 %·회전 +3 %·주행 중 헤딩 흔들림 + 카메라 장착 편차.
제어기는 합성 영상만 받는다. 정답 자세는 아래 평가(evaluate)만 읽는다.

    python3 apps/pickup_real/test_pickup_real.py            # 전체
    python3 apps/pickup_real/test_pickup_real.py -k angled  # 일부
"""
import argparse
import json
import math
import shutil
import sys
import tempfile
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import geometry  # noqa: E402
import pickup_real  # noqa: E402

FORK_TIP = 0.26
PREDOCK_X = -0.80      # 정답: teach 때 로봇 회전 중심 (시트 면 기준)
GAP = 0.25             # 정답: predock 포크 끝 ↔ 포켓 입구
ENTRANCE_X = PREDOCK_X + FORK_TIP + GAP

CASES = {
    "front": (-1.10, 0.00, 0.0),
    "left": (-1.10, 0.12, 0.0),
    "right": (-1.10, -0.12, 0.0),
    "angled": (-1.15, 0.06, math.radians(12)),
    "near": (-0.86, -0.05, math.radians(-6)),
    "far": (-1.45, 0.05, math.radians(3)),
}
FAULTS = ["image_loss", "image_stale", "occluded", "wheel_blocked"]


def make_args(mode, tmp, extra=()):
    args = pickup_real.parse([mode, "--fake", "--teach", str(tmp / "teach.json"), "--yes-lift", *extra])
    return args


def run_mode(cfg, world, mode, tmp, extra=(), fault_after_insert_segments=None, fault=None):
    pickup_real.LOG_ROOT = tmp / "logs"
    pickup_real.STATE_FILE = tmp / "logs" / "state.json"
    args = make_args(mode, tmp, extra)
    args.world = world
    run_dir = tmp / "logs" / ("%s_%d" % (mode, int(time.monotonic() * 1000)))
    p = pickup_real.build(args, cfg, run_dir)
    service = p.cam.service
    if fault:
        orig = world.straight
        count = {"n": 0}

        def straight(d, speed):
            if fault == "wheel_blocked" and d > 0 and p.summary.get("stage", "").startswith("INSERT"):
                # 카메라로 확인하는 첫 삽입 구간에서 막힘 → odom/vision 불일치로 멈춰야 한다.
                # 마지막(카메라 없는) 구간의 막힘은 전방 초음파 또는 상승 전 사람 확인만 잡는다(10/31 기록)
                world.blocked_after = 0.0
            out = orig(d, speed)
            if fault != "wheel_blocked" and d > 0 and p.summary.get("stage", "").startswith("INSERT"):
                count["n"] += 1
                if count["n"] == fault_after_insert_segments:
                    if fault in ("image_loss", "image_stale"):
                        service.fault = fault
                    elif fault == "occluded":
                        world.occluded = True
            return out
        world.straight = straight
    ok = p.run()
    service.stop()
    return ok, p.summary


def evaluate(world):
    """정답 기준 포크 오차(평가 전용). 상승 순간의 자세."""
    x, y, th = getattr(world, "pose_at_lift", None) or world.pose
    tip_x = x + FORK_TIP * math.cos(th)
    return {"lateral_mm": round(y * 1000 + FORK_TIP * math.sin(th) * 1000, 1), "yaw_deg": round(math.degrees(th), 2),
            "depth_mm": round((tip_x - ENTRANCE_X) * 1000, 1)}


class World(pickup_real.FakeWorld):
    occluded = False

    def frame(self):
        if self.occluded and self.arm_view:
            time.sleep(1 / 15.0)
            import numpy as np
            return np.full((480, 640, 3), 120, np.uint8)
        return super().frame()


# ---------------------------------------------------------------- 검정 표식 모드(받침 앞면 = x 0, 포크 입구)
TAPE_CASES = {
    "front": (-1.00, 0.00, 0.0),
    "left": (-1.00, 0.10, 0.0),
    "right": (-1.00, -0.10, 0.0),
    "angled": (-1.05, 0.05, math.radians(8)),
    "near": (-0.88, -0.04, math.radians(-5)),
    "far": (-1.20, 0.03, math.radians(3)),
}
TAPE_FAULTS = ["image_loss", "image_stale", "one_marker_hidden", "wheel_blocked"]


class TapeWorld(pickup_real.FakeWorld):
    """받침 앞면의 검정 표식 2개를 렌더링. 장착은 실기 설정 근처(앞 0.34 m, 높이 0.48 m), 아래 38°
    (실제 설정 blind_final_m 0.5 까지 표식이 화면에 남는 각도 — 32° 면 0.62 m 부터 안 보임. 실물 한계는 현장 확인),
    화각은 가정보다 4 % 넓고 바퀴는 3 % 덜 가고 주행 중 헤딩이 흔들린다(FakeWorld). 제어기는 영상만 받는다."""
    occluded = False

    def __init__(self, cfg, start, seed=1):
        t = cfg["tape"]
        dh = float(t["camera_height_m"]) - float(t["marker_center_height_m"])
        super().__init__(cfg, start, seed=seed, mount={"forward": float(cfg["robot"]["camera_forward_m"]),
                                                       "left": float(cfg["robot"]["camera_left_m"]),
                                                       "yaw": 0.0, "height": dh, "pitch": math.radians(38)})
        import tape_synth
        self.synth = tape_synth
        self.tex = tape_synth.face_texture(spacing=float(t["center_spacing_m"]), seed=seed)
        self.tex_hidden = tape_synth.face_texture(spacing=float(t["center_spacing_m"]), seed=seed, hide=("left",))

    def frame(self):
        import numpy as np
        time.sleep(1 / 15.0)
        if not self.arm_view:
            return np.full((480, 640, 3), 90, np.uint8)
        tex, org = self.tex_hidden if self.occluded else self.tex
        img = self.synth.render(tex, org, self.K_true, self.camera_pose())
        return self.synth.light(img, noise=2.0, rng=self.rng)


def tape_cfg(cfg):
    c = json.loads(json.dumps(cfg))
    c["vision"]["mode"] = "tape"
    c["camera"]["calibration_file"] = ""
    return c   # 삽입·정렬 허용치는 실제 설정 그대로


def tape_evaluate(world, cfg):
    x, y, th = getattr(world, "pose_at_lift", None) or world.pose
    tip = float(cfg["robot"]["fork_tip_forward_m"])
    return {"lateral_mm": round((y + tip * math.sin(th)) * 1000, 1), "yaw_deg": round(math.degrees(th), 2),
            "depth_mm": round((x + tip * math.cos(th)) * 1000, 1)}


def run_tape(cfg0, tmp, seed, only=""):
    cfg = tape_cfg(cfg0)
    teach_path = tmp / "teach_tape.json"
    results, failures = [], []
    depth = float(cfg["insert"]["depth_m"]) * 1000

    def staged(world, fault=None, tape_distance=True):
        """현장 순서대로 단계별 실행: calibrate → align → insert → lift (lift 는 가짜 장치라 --yes-lift).
        tape_distance: 현장 절차처럼 calibrate 에 줄자로 잰 렌즈↔앞면 수평 거리를 준다(정답 + 5 mm 오차)."""
        if teach_path.exists():
            teach_path.unlink()
        steps = []
        measured = -world.camera_pose()[0] + 0.005
        for mode in ("calibrate", "align", "insert", "lift"):
            extra = ["--teach", str(teach_path)]
            if mode == "calibrate" and tape_distance:
                extra += ["--camera-distance", "%.3f" % measured]
            ok, s = run_mode(cfg, world, mode, tmp, extra=extra,
                             fault=fault if mode == "insert" else None, fault_after_insert_segments=1)
            steps.append({"mode": mode, "ok": ok, "reason": s.get("reason", ""), "calibration": {
                k: s.get("calibration", {}).get(k) for k in ("focal_scale", "focal_scale_by_motion", "fit_rms_m")}
                if mode == "calibrate" else None, "aligned": s.get("aligned"), "inserted": s.get("inserted")})
            if not ok:
                break
        return steps

    for index, (name, start) in enumerate(TAPE_CASES.items()):
        if only and only not in name:
            continue
        w = TapeWorld(cfg, start, seed=seed + 31 + index * 7)
        steps = staged(w)
        ev = tape_evaluate(w, cfg)
        good = (all(s["ok"] for s in steps) and len(steps) == 4 and w.lifted and abs(ev["lateral_mm"]) <= 15
                and abs(ev["yaw_deg"]) <= 2.0 and abs(ev["depth_mm"] - depth) <= 25)
        results.append({"case": "tape_" + name, "ok": good, "truth": ev, "steps": steps})
        if not good:
            failures.append("tape_" + name)
    if not only or "no_tape" in only:
        # 줄자 거리 없이(12 cm 이동 맞춤만, 약 ±3 %): 성공하거나, 실패해도 상승 없이 멈춰야 한다(잘못 넣고 들지 않음)
        w = TapeWorld(cfg, TAPE_CASES["front"], seed=seed + 5)
        steps = staged(w, tape_distance=False)
        ev = tape_evaluate(w, cfg)
        inserted_ok = abs(ev["lateral_mm"]) <= 15 and abs(ev["yaw_deg"]) <= 2.0 and abs(ev["depth_mm"] - depth) <= 25
        good = (w.lifted and inserted_ok) or (not w.lifted and not steps[-1]["ok"])
        results.append({"case": "tape_front_motion_focal_only (succeed or stop safely)", "ok": good, "truth": ev,
                        "lifted": w.lifted, "stopped": None if steps[-1]["ok"] else steps[-1]["reason"],
                        "focal": steps[0].get("calibration")})
        if not good:
            failures.append("tape_front_motion_focal_only")
    for fault in TAPE_FAULTS:
        if only and only not in fault:
            continue
        w = TapeWorld(cfg, TAPE_CASES["front"], seed=seed + 99)
        inject = "occluded" if fault == "one_marker_hidden" else fault
        steps = staged(w, fault=inject)
        expect = {"image_loss": "image loss", "image_stale": "frozen", "one_marker_hidden": "only one marker",
                  "wheel_blocked": "disagree"}[fault]
        last = steps[-1]
        good = (not last["ok"]) and last["mode"] == "insert" and (not w.lifted) and expect in last["reason"]
        results.append({"case": "tape_fault_" + fault, "ok": good, "lifted": w.lifted, "reason": last["reason"],
                        "stopped_at": last["mode"]})
        if not good:
            failures.append("tape_fault_" + fault)
    if not only or "teach" in only:
        # 보정 없이·옛 구멍용 teach 로는 움직이지 않는다
        for label, content in (("no_teach", None), ("old_hole_teach", {
                "created": "2026-10-30", "camera_height_m": 0.48, "hole_pitch_m": 0.010, "focal_scale": 1.0,
                "mount": {"forward": 0.15, "left": 0.0, "yaw": 0.0}, "predock_x": -0.56, "target_y": 0.0, "gap_m": 0.30})):
            if teach_path.exists():
                teach_path.unlink()
            if content:
                teach_path.write_text(json.dumps(content))
            w = TapeWorld(cfg, TAPE_CASES["front"], seed=seed)
            ok, s = run_mode(cfg, w, "align", tmp, extra=("--teach", str(teach_path)))
            good = (not ok) and not w.commands and not w.lifted
            results.append({"case": "tape_%s_refuses" % label, "ok": good, "reason": s.get("reason", "")})
            if not good:
                failures.append("tape_%s_refuses" % label)
    return results, failures


def teach(cfg, tmp, seed, camera_distance=True):
    w = World(cfg, (PREDOCK_X, 0.0, 0.0), seed=seed)
    cam_dist = None
    if camera_distance:
        cx, cy, ch, _, _ = w.camera_pose()
        cam_dist = -cx  # 줄자로 잰 렌즈 ↔ 시트 수평 거리(정답, 실물에서는 줄자)
    extra = ["--gap", str(GAP)] + (["--camera-distance", "%.3f" % cam_dist] if cam_dist else [])
    ok, s = run_mode(cfg, w, "teach", tmp, extra)
    assert ok, s
    return w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="")
    ap.add_argument("--seed", type=int, default=20261030)
    ap.add_argument("--out", default=str(HERE.parents[1] / "logs" / "pickup_real" / "offline_test.json"))
    ap.add_argument("--vision", choices=["all", "marker", "tape"], default="all",
                    help="marker = ArUco 시트 가짜 세계, tape = 검정 표식 2개 가짜 세계(받침 구멍 모드는 Gazebo 시뮬)")
    args = ap.parse_args()
    cfg0 = yaml.safe_load((HERE / "config.yaml").read_text())
    cfg = json.loads(json.dumps(cfg0))
    cfg["vision"]["mode"] = "marker"   # 가짜 세계는 ArUco 시트만 그린다. 받침(pallet) 모드는 Gazebo 시뮬로 시험
    tmp = Path(tempfile.mkdtemp(prefix="pickup_real_test_"))
    results, failures = [], []
    try:
        if args.vision in ("all", "tape"):
            r, f = run_tape(cfg0, tmp, args.seed, args.k)
            results += r
            failures += f
        if args.vision == "tape":
            raise StopIteration
        # 0. 기하 단위 시험
        for pose in [(-1.0, 0.1, 0.2), (-0.81, 0.02, -0.1), (-0.79, -0.08, 0.0), (-0.6, 0.15, 0.3)]:
            moves, note = geometry.plan_moves(pose, (-0.8, 0.0, 0.0), {"lateral": 0.005, "yaw": 0.01, "distance": 0.01},
                                              math.radians(35), 0.6)
            end = geometry.simulate_moves(pose, moves)
            assert abs(end[0] + 0.8) < 1e-6 and abs(end[1]) < 1e-6 and abs(end[2]) < 1e-6, (pose, moves, end)
            turns = [abs(v) for k, v in moves if k == "turn"]
            assert max(turns + [0]) <= math.radians(100), moves
        results.append({"case": "geometry_plan", "ok": True})
        teach(cfg, tmp, args.seed)
        t = json.loads((tmp / "teach.json").read_text())
        results.append({"case": "teach", "ok": True, "focal_scale": round(t["focal_scale"], 4),
                        "mount_left_mm": round(t["mount"]["left"] * 1000, 1),
                        "mount_yaw_deg": round(math.degrees(t["mount"]["yaw"]), 2)})
        for index, (name, start) in enumerate(CASES.items()):
            if args.k and args.k not in name:
                continue
            w = World(cfg, start, seed=args.seed + 7 + index * 13)  # 경우별 고정 seed(-k 와 무관하게 재현)
            ok, s = run_mode(cfg, w, "run", tmp)
            ev = evaluate(w)
            # 깊이 ±25 mm: look point(predock 뒤 0.15 m)부터 마지막 0.5 m 를 카메라 없이 가고, 가짜 세계 바퀴는 3 % 덜 간다.
            # 포크가 가로대 아래로 0.15 ± 0.025 m 들어가면 받침 윗판 아래에 있다(10/31 결정, 실기 깊이 확인 필요)
            good = (ok and w.lifted and abs(ev["lateral_mm"]) <= 15 and abs(ev["yaw_deg"]) <= 2.0
                    and abs(ev["depth_mm"] - cfg["insert"]["depth_m"] * 1000) <= 25)
            results.append({"case": name, "ok": good, "run_ok": ok, "reason": s.get("reason", ""), "truth": ev,
                            "looks": s["looks"], "realigns": s["realigns"], "aligned": s.get("aligned")})
            if not good:
                failures.append(name)
        for fault in FAULTS:
            if args.k and args.k not in fault:
                continue
            w = World(cfg, CASES["front"], seed=args.seed + 99)
            ok, s = run_mode(cfg, w, "run", tmp, fault=fault, fault_after_insert_segments=1)
            # 정렬 실패 같은 다른 이유로 멈춘 것은 통과가 아니다: 넣은 고장 때문에 멈췄는지 이유로 확인
            expect = {"image_loss": "image loss", "image_stale": "frozen", "occluded": "not seen",
                      "wheel_blocked": "disagree"}[fault]
            good = (not ok) and (not w.lifted) and expect in s.get("reason", "")
            results.append({"case": "fault_" + fault, "ok": good, "stopped": not ok, "lifted": w.lifted,
                            "reason": s.get("reason", "")})
            if not good:
                failures.append("fault_" + fault)
        if not args.k or "noteach" in args.k:
            (tmp / "teach.json").unlink()
            w = World(cfg, CASES["front"], seed=1)
            ok, s = run_mode(cfg, w, "run", tmp)
            good = (not ok) and not w.commands and not w.lifted
            results.append({"case": "no_teach_refuses", "ok": good, "reason": s.get("reason", "")})
            if not good:
                failures.append("no_teach_refuses")
    except StopIteration:
        pass
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    for r in results:
        print(("PASS " if r["ok"] else "FAIL ") + json.dumps(r, ensure_ascii=False, default=str))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(results, indent=1, ensure_ascii=False, default=str))
    print("%d/%d PASS" % (sum(r["ok"] for r in results), len(results)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
